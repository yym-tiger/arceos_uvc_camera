use crate::abstractions::dma::DMA;
use crate::abstractions::{OSAbstractions, PlatformAbstractions};
use crate::host::data_structures::host_controllers::xhci::ring::Ring;
use crate::host::data_structures::host_controllers::ControllerArc;
use crate::{err::*, USBSystemConfig};
use alloc::alloc::Allocator;
use alloc::collections::{BTreeMap, BTreeSet};
use alloc::sync::Arc;
use alloc::{boxed::Box, vec::Vec};
use alloc::{format, vec};
use core::borrow::BorrowMut;
use core::num;
use log::{debug, error, info, warn, trace};
use spinlock::SpinNoIrq;
use xhci::context::Input64Byte;
pub use xhci::context::{Device, Device64Byte, DeviceHandler};
use crate::host::data_structures::host_controllers::xhci::TAG;
const NUM_EPS: usize = 32;

// 设备上下文列表，使用BTreeMap存储传输环
pub struct DeviceContextList<O>
where
    O: PlatformAbstractions,
{
    config: Arc<SpinNoIrq<USBSystemConfig<O>>>,
    pub dcbaa: DMA<[u64; 256], O::DMA>,
    pub device_out_context_list: Vec<DMA<Device64Byte, O::DMA>>,
    pub device_input_context_list: Vec<DMA<Input64Byte, O::DMA>>,
    // 修改这里，改为BTreeMap嵌套结构
    // 第一层key是设备槽位ID，第二层key是端点DCI
    pub transfer_rings: Vec<BTreeMap<usize, Ring<O>>>,
}

impl<O> DeviceContextList<O>
where
    O: PlatformAbstractions,
{
    pub fn new(max_slots: u8, config: Arc<SpinNoIrq<USBSystemConfig<O>>>) -> Self {
        let os = config.lock().os.clone();
        let a = os.dma_alloc();

        trace!("new dcbaa");
        let mut dcbaa = DMA::new([0u64; 256], 4096, a.clone());
        trace!("new dcbaa");
        let mut out_context_list = Vec::with_capacity(max_slots as _);
        trace!("new dcbaa");
        let mut in_context_list = Vec::with_capacity(max_slots as _);
        for i in 0..max_slots as usize {
            trace!("new in/out ctx");
            let out_context = DMA::new(Device::new_64byte(), 4096, a.clone()).fill_zero();
            dcbaa[i] = out_context.addr() as u64;
            out_context_list.push(out_context);
            in_context_list.push(DMA::new(Input64Byte::new_64byte(), 4096, a.clone()).fill_zero());
        }
        // 修改这里，初始化设备槽位的BTreeMap
        let mut transfer_rings = Vec::with_capacity(max_slots as _);
        for _ in 0..transfer_rings.capacity() {
            trace!("new transfer ring map");
            transfer_rings.push(BTreeMap::new());
        }

        Self {
            dcbaa,
            device_out_context_list: out_context_list,
            device_input_context_list: in_context_list,
            transfer_rings,
            config: config.clone(),
        }
    }

    pub fn dcbaap(&self) -> usize {
        self.dcbaa.as_ptr() as _
    }

    pub fn new_slot(
        &mut self,
        slot: usize,
        hub: usize,
        port: usize,
        num_ep: usize, // cannot lesser than 0, and consider about alignment, use usize
    ) {
        if slot > self.device_out_context_list.len() {
            panic!("slot {} > max {}", slot, self.device_out_context_list.len())
        }

        let os = self.config.lock().os.clone();

        // 初始化传输环映射
        let slot_rings = &mut self.transfer_rings[slot];
        trace!("{TAG} [new_slot] Clearing transfer rings for slot {}", slot); // Log clearing
        slot_rings.clear(); // 清除之前可能存在的环

        // 预先为控制端点创建传输环(DCI=1)
        let ring = Ring::new(os.clone(), 32, true).unwrap();
        let ctrl_ring_addr = ring.register();
        trace!("{TAG} [new_slot] Creating initial control ring (DCI 1) for slot {} at address {:#X}", slot, ctrl_ring_addr); // Log creation of DCI 1 ring
        slot_rings.insert(1, ring);
        trace!("{TAG} [new_slot] Inserted control ring (DCI 1) for slot {} at address {:#X} into BTreeMap.", slot, ctrl_ring_addr);

        debug!("Initialized transfer rings for slot {}", slot);
    }

    // 获取或创建给定槽位和DCI的传输环
    pub fn get_or_create_ring(&mut self, slot_id: usize, dci: usize) -> &mut Ring<O> {
        trace!("{TAG} [get_or_create_ring] Called for slot_id: {}, dci: {}", slot_id, dci); // Log entry
        let os = self.config.lock().os.clone();
        
        // 确保槽位存在
        if slot_id >= self.transfer_rings.len() {
            error!("{TAG} [get_or_create_ring] Slot {} exceeds max slots {}", slot_id, self.transfer_rings.len());
            panic!("Slot {} exceeds max slots {}", slot_id, self.transfer_rings.len());
        }
        
        let slot_rings = &mut self.transfer_rings[slot_id];
        let contains_key = slot_rings.contains_key(&dci);
        trace!("{TAG} [get_or_create_ring] Slot {} rings map contains_key for dci {}: {}", slot_id, dci, contains_key);
        
        // 如果传输环不存在，则创建一个新的
        if !contains_key {
            trace!("{TAG} [get_or_create_ring] Creating NEW ring for slot {}, dci {}.", slot_id, dci);
            
            // 检查是否是等时传输端点 (DCI 3, 5, 7, 9 等是IN端点)
            let ring = if dci == 3 || dci == 5 || dci == 7 || dci == 9 {
                // 为等时传输创建更大的环
                trace!("{TAG} [get_or_create_ring] Creating LARGER ring for ISOCH endpoint DCI {}", dci);
                Ring::new(os.clone(), 512, true).unwrap() 
            } else {
                // 其他端点使用标准大小
                Ring::new(os.clone(), 32, true).unwrap()
            };
            
            let new_ring_addr = ring.register(); // Get address before moving
            trace!("{TAG} [get_or_create_ring]    New ring addr: {:#X}, size: {}", new_ring_addr, ring.len());
            
            // 保存ring的初始cycle state
            let initial_cycle = ring.cycle;
            
            slot_rings.insert(dci, ring);
            trace!("{TAG} [get_or_create_ring] Inserted NEW ring for slot {}, dci {}. Addr: {:#X} into BTreeMap.", slot_id, dci, new_ring_addr);
            
            // 标记需要配置端点
            warn!("{TAG} [get_or_create_ring] ⚠️ 新环已创建但端点未配置！需要调用configure_endpoint更新TR Dequeue指针");
            warn!("{TAG} [get_or_create_ring] ⚠️ 新环地址: 0x{:X}, 初始cycle: {}", new_ring_addr, initial_cycle);
        } else {
            let existing_ring = slot_rings.get(&dci).unwrap(); // 获取不可变引用来打印
            trace!("{TAG} [get_or_create_ring] Returning EXISTING ring for slot {}, dci {}. Addr: {:#X}, Cycle: {}", 
                slot_id, dci, existing_ring.register(), existing_ring.cycle);
        }
        
        // 检查端点的DCS状态（仅用于调试）
        if let Some(output_ctx) = self.device_out_context_list.get(slot_id) {
            let endpoint_handler = DeviceHandler::endpoint(&**output_ctx, dci);
            let dcs = endpoint_handler.dequeue_cycle_state();
            let ring = slot_rings.get(&dci).unwrap();
            
            trace!("{TAG} [get_or_create_ring] 端点状态检查: Ring cycle={}, Endpoint DCS={}, slot={}, dci={}", 
                  ring.cycle, dcs, slot_id, dci);
            
            // 检查dequeue指针位置
            let tr_deq_ptr = endpoint_handler.tr_dequeue_pointer();
            let tr_deq_ptr_masked = tr_deq_ptr & !1u64; // 屏蔽DCS位
            let ring_base = ring.register();
            
            // 计算硬件期望的环索引
            let hw_ring_index = if tr_deq_ptr_masked >= ring_base {
                ((tr_deq_ptr_masked - ring_base) / 16) as usize
            } else {
                0
            };
            
            trace!("{TAG} [get_or_create_ring] HW dequeue index={}, SW index={}", hw_ring_index, ring.i);
        }
        
        // 返回传输环的可变引用
        slot_rings.get_mut(&dci).unwrap()
    }

    /// Finds the slot ID associated with a given TRB address by checking transfer rings.
    pub fn find_slot_by_trb_addr(&self, trb_addr: u64) -> Option<usize> {
        trace!("{TAG} Searching for Slot ID for TRB Addr: {:#X}", trb_addr); // Log target address

        for (slot_id, slot_rings_map) in self.transfer_rings.iter().enumerate() {
            if slot_id == 0 {
                continue;
            }
            trace!("{TAG}  Checking Slot ID: {}", slot_id); // Log current slot

            for (dci, ring) in slot_rings_map.iter() {
                let ring_start_addr = ring.register();
                let ring_size_bytes = ring.len() * core::mem::size_of::<xhci::ring::trb::Link>();
                let ring_end_addr = ring_start_addr.saturating_add(ring_size_bytes as u64);

                // Log details for each ring being checked
                trace!("{TAG}    Checking DCI: {}, Ring Start: {:#X}, Ring Size: {} bytes, Ring End: {:#X}",
                       dci, ring_start_addr, ring_size_bytes, ring_end_addr);

                if trb_addr >= ring_start_addr && trb_addr < ring_end_addr {
                    trace!("{TAG}      Found match! TRB Addr {:#X} is within range for Slot {}, DCI {}", trb_addr, slot_id, dci);
                    return Some(slot_id);
                }
            }
        }
        warn!("{TAG} TRB Addr {:#X} not found in any known transfer ring.", trb_addr); // Use warn if not found after checking all
        None
    }
}

use tock_registers::interfaces::Writeable;
use tock_registers::register_structs;
use tock_registers::registers::{ReadOnly, ReadWrite, WriteOnly};

register_structs! {
    ScratchpadBufferEntry{
        (0x000 => value_low: ReadWrite<u32>),
        (0x004 => value_high: ReadWrite<u32>),
        (0x008 => @END),
    }
}

impl ScratchpadBufferEntry {
    pub fn set_addr(&mut self, addr: u64) {
        self.value_low.set(addr as u32);
        self.value_high.set((addr >> 32) as u32);
    }
}

pub struct ScratchpadBufferArray<O>
where
    O: OSAbstractions,
{
    pub entries: DMA<[ScratchpadBufferEntry], O::DMA>,
    pub pages: Vec<DMA<[u8], O::DMA>>,
}

unsafe impl<O: OSAbstractions> Sync for ScratchpadBufferArray<O> {}

impl<O> ScratchpadBufferArray<O>
where
    O: OSAbstractions,
{
    pub fn new(entries: u32, os: O) -> Self {
        let page_size = O::PAGE_SIZE;
        let align = 64;

        let mut entries: DMA<[ScratchpadBufferEntry], O::DMA> =
            DMA::zeroed(entries as usize, align, os.dma_alloc());

        let pages = entries
            .iter_mut()
            .map(|entry| {
                let dma = DMA::zeroed(page_size, align, os.dma_alloc());

                assert_eq!(dma.addr() % page_size, 0);
                entry.set_addr(dma.addr() as u64);
                dma
            })
            .collect();

        Self { entries, pages }
    }
    pub fn register(&self) -> usize {
        self.entries.addr()
    }
}
