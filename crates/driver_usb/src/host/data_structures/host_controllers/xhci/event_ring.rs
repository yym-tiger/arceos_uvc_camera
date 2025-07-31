use core::sync::atomic::{fence, Ordering};
use tock_registers::interfaces::Readable;

pub use super::ring::{Ring, TrbData};
use crate::abstractions::dma::DMA;
use crate::abstractions::OSAbstractions;
use crate::err::*;
use log::{debug, info, warn};
use tock_registers::interfaces::Writeable;
use tock_registers::register_structs;
use tock_registers::registers::{ReadOnly, ReadWrite, WriteOnly};
use xhci::extended_capabilities::hci_extended_power_management::Data;
use xhci::ring::trb::event::{CompletionCode, TransferEvent};
use xhci::ring::trb::{self, event::Allowed};
use crate::host::data_structures::host_controllers::xhci::TAG;

register_structs! {
    EventRingSte {
        (0x000 => addr_low: ReadWrite<u32>),
        (0x004 => addr_high: ReadWrite<u32>),
        (0x008 => size: ReadWrite<u16>),
        (0x00A => _reserved),
        (0x010 => @END),
    }
}

pub struct EventRing<O>
where
    O: OSAbstractions,
{
    pub ring: Ring<O>,
    pub ste: DMA<[EventRingSte], O::DMA>,
}

impl<O> EventRing<O>
where
    O: OSAbstractions,
{
    pub fn new(os: O) -> Result<Self> {
        let a = os.dma_alloc();
        // 根据XHCI规范，某些硬件对事件环段大小有限制
        // 但是对于USB摄像头这样的高带宽设备，需要较大的事件环
        const EVENT_RING_SIZE: usize = 256; // 使用256个TRB，平衡性能和兼容性
        
        let mut ring = EventRing {
            ste: DMA::zeroed(1, 64, a),
            ring: Ring::new(os, EVENT_RING_SIZE, false)?,
        };
        ring.ring.cycle = true;
        ring.ste[0].addr_low.set(ring.ring.register() as u32);
        ring.ste[0]
            .addr_high
            .set((ring.ring.register() as u64 >> 32) as u32);
        ring.ste[0].size.set(ring.ring.trbs.len() as u16);
        
        log::info!("[XHCI EventRing] Created event ring with {} TRBs at 0x{:X}", 
                   EVENT_RING_SIZE, ring.ring.register());

        Ok(ring)
    }

    /// Returns the base address of the first event ring segment.
    pub fn segment_base_address(&self) -> u64 {
        if let Some(ste_entry) = self.ste.get(0) { // Ensure ste is not empty
            let low = ste_entry.addr_low.get();
            let high = ste_entry.addr_high.get();
            ((high as u64) << 32) | (low as u64)
        } else {
            0 // Or handle error appropriately, e.g., panic or return Option<u64>
        }
    }

    /// 完成一次循环返回 true
    pub fn next(&mut self) -> Option<(Allowed, bool)> {
        let (data_ref, flag) = self.ring.current_data();
        let data = unsafe {
            let mut out = [0u32; 4];
            for i in 0..out.len() {
                out[i] = (data_ref.as_ptr() as *const u32).offset(i as _).read_volatile();
            }
            out
        };

        // Check for all-zero TRB which indicates we've reached uninitialized memory
        if data == [0, 0, 0, 0] {
            debug!("EventRing::next(): Found all-zero TRB at index {}, likely end of valid events", self.ring.i);
            return None;
        }
        
        let trb_type = (data[3] >> 10) & 0x3F;
        debug!(
            "EventRing::next(): Raw TRB data @ index {}: [{:08x}, {:08x}, {:08x}, {:08x}], ExpectedCycle(SW): {}",
            self.ring.i,
            data[0], data[1], data[2], data[3],
            flag
        );
        let hardware_cycle_bit = (data[3] & 1) == 1;
        debug!(
            "EventRing::next(): TRB Type = {}, ActualCycleBit(HW): {}",
            trb_type,
            hardware_cycle_bit
        );

        // Ensure full raw data is logged, potentially splitting if needed
        debug!("    Raw[0]: {:08x}", data[0]);
        debug!("    Raw[1]: {:08x}", data[1]);
        debug!("    Raw[2]: {:08x}", data[2]);
        debug!("    Raw[3]: {:08x}", data[3]);

        let mut allowed = match Allowed::try_from(data) {
            Ok(a) => a,
            Err(_) => {
                warn!(
                    "EventRing::next(): Failed to parse TRB data @ index {}: [{:08x}, {:08x}, {:08x}, {:08x}], Type={}",
                    self.ring.i,
                    data[0], data[1], data[2], data[3],
                    trb_type
                );
                return None;
            }
        };

        // Log raw data for DCI 5 transfer events
        if let Allowed::TransferEvent(te) = &allowed {
            if te.endpoint_id() == 5 { // Check for DCI 5 (our ISOC endpoint)
                warn!(
                    "[XHCI EventRing] DCI 5 (ISOC) TransferEvent RAW: [{:08X}, {:08X}, {:08X}, {:08X}]", 
                    data[0], data[1], data[2], data[3]
                );
                warn!(
                    "[XHCI EventRing] DCI 5 (ISOC) Parsed: TRBPtr={:X}, EP_ID={}, SlotID={}, CC={:?}, Len={}", 
                    te.trb_pointer(), te.endpoint_id(), te.slot_id(), te.completion_code(), te.trb_transfer_length()
                );
            } else if te.endpoint_id() == 0 { // Existing log for DCI 0
                // Use warn level and potentially split if lines are too long
                warn!("Detected DCI 0 Transfer Event!");
                warn!("    Raw data: [{:08x}, {:08x}, {:08x}, {:08x}]", data[0], data[1], data[2], data[3]);
                warn!("    Parsed TRBPtr: {:X}, Parsed EP_ID: {}, Parsed SlotID: {}, Parsed CC: {:?}", 
                      te.trb_pointer(), te.endpoint_id(), te.slot_id(), te.completion_code());
            }
        }

        if flag != hardware_cycle_bit {
            let temp_allowed_for_log = Allowed::try_from(data);
            warn!(
                "EventRing::next(): Cycle bit MISMATCH @ index {}. SW_Expected_Cycle: {}, HW_Actual_TRB_Cycle: {}. TRB Data: [{:08x}, {:08x}, {:08x}, {:08x}]. Parsed (if possible): {:?}. Skipping event.",
                self.ring.i, flag, hardware_cycle_bit, data[0], data[1], data[2], data[3], temp_allowed_for_log.ok()
            );
            return None;
        }

        if let Allowed::TransferEvent(c) = allowed {
            if let Ok(CompletionCode::Invalid) = c.completion_code() {
                info!(
                    "EventRing::next(): Skipping TransferEvent @ index {} with CompletionCode::Invalid.",
                    self.ring.i
                );
                return None;
            }
        }

        fence(Ordering::SeqCst);
        let cycle_flipped = self.ring.inc_deque();
        info!(
            "EventRing::next(): Dequeue pointer advanced. New index: {}, New cycle: {}. Cycle flipped: {}. Processed Event: {:?}",
            self.ring.i, self.ring.cycle, cycle_flipped, allowed
        );
        info!("{TAG} [EventRing::next] After processing event, SW ERDP Index: {}, SW Cycle: {}. Consider updating HW ERDP now.", self.ring.i, self.ring.cycle);
        Some((allowed, cycle_flipped))
    }

    pub fn erdp(&self) -> u64 {
        // ERDP必须指向当前的出队位置，不是环的基地址
        let base_addr = self.ring.register() as u64;
        let current_offset = (self.ring.i * core::mem::size_of::<[u32; 4]>()) as u64;
        (base_addr + current_offset) & 0xFFFF_FFFF_FFFF_FFF0
    }
    pub fn erstba(&self) -> u64 {
        let ptr = &self.ste[0];
        ptr as *const EventRingSte as usize as u64
    }
}
