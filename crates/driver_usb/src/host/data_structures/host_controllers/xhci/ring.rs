use core::fmt;
use core::fmt::Debug;
use core::mem;
use core::ptr::slice_from_raw_parts;
use log::{trace, warn, info};

use crate::abstractions::dma::DMA;
use crate::abstractions::OSAbstractions;
use crate::err::*;
use alloc::boxed::Box;
use alloc::slice;
use alloc::vec;
use alloc::vec::Vec;
// Already imported above
pub use xhci::ring::trb;
use xhci::ring::trb::command;
use xhci::ring::trb::event;
use xhci::ring::trb::transfer;
use xhci::ring::trb::Link;
const TRB_LEN: usize = 4;
pub type TrbData = [u32; TRB_LEN];

pub struct Ring<O: OSAbstractions> {
    link: bool,
    pub trbs: DMA<[TrbData], O::DMA>,
    pub i: usize,
    pub cycle: bool,
}

impl<O: OSAbstractions> Ring<O> {
    pub fn new(os: O, len: usize, link: bool) -> Result<Self> {
        let a = os.dma_alloc();
        let mut trbs = DMA::new_vec([0; TRB_LEN], len, 64, a);
        
        // 如果是带链接的环，初始化Link TRB
        if link && len > 1 {
            let address = trbs.addr() as u64;
            let mut link_trb = Link::new();
            link_trb.set_ring_segment_pointer(address)
                    .set_toggle_cycle()
                    .clear_cycle_bit(); // Link TRB初始cycle bit为0，因为初始PCS=1，toggle后将变为0
            
            let link_data = command::Allowed::Link(link_trb).into_raw();
            trbs[len - 1].copy_from_slice(&link_data);
            
            trace!("[Ring::new] 初始化Link TRB at index {}: addr=0x{:X}, toggle_cycle=1, cycle_bit=0", 
                   len - 1, address);
            trace!("[Ring::new] 创建新环: 大小={}, 基地址=0x{:08X}, 初始cycle=true, link={}", len, address, link);
        }
        
        // XHCI规范要求：
        // 1. 对于Transfer Ring，Producer Cycle State (PCS)初始值必须为1
        // 2. 端点上下文的DCS初始值为1
        // 3. 这样硬件才能识别新提交的TRB
        let initial_cycle = if link {
            // Transfer Ring的初始cycle必须是1
            true
        } else {
            // Event Ring的初始cycle是1
            true
        };
        
        Ok(Self {
            trbs,
            i: 0,
            cycle: initial_cycle,
            link,
        })
    }
    pub fn len(&self) -> usize {
        self.trbs.len()
    }

    fn get_trb_at_index(&self, index: usize) -> &TrbData {
        &self.trbs[index]
    }

    /// Returns the base physical address of the TRB ring buffer.
    pub fn register(&self) -> u64 {
        self.trbs.addr() as u64
    }

    pub fn enque_command(&mut self, mut trb: command::Allowed) -> usize {
        if self.cycle {
            trb.set_cycle_bit();
        } else {
            trb.clear_cycle_bit();
        }
        let addr = self.enque_trb(trb.clone().into_raw());
        trace!("[CMD] >> {:?} @{:X}", trb, addr);
        addr
    }

    pub fn enque_transfer(&mut self, mut trb: transfer::Allowed) -> usize {
        // 记录入队前的cycle状态
        let old_cycle = self.cycle;
        
        trace!("[Ring] enque_transfer: 当前ring.cycle={}, ring.i={}", self.cycle, self.i);
        if self.cycle {
            trb.set_cycle_bit();
            trace!("[Ring] enque_transfer: 设置TRB cycle bit = 1");
        } else {
            trb.clear_cycle_bit();
            trace!("[Ring] enque_transfer: 清除TRB cycle bit = 0");
        }
        let addr = self.enque_trb(trb.clone().into_raw());
        trace!("[Ring] enque_transfer: TRB写入地址=0x{:08X}", addr);
        
        // 检查是否发生了cycle翻转
        if old_cycle != self.cycle {
            info!("[Ring] 检测到cycle翻转！old={}, new={}", old_cycle, self.cycle);
        }
        
        addr
    }

    pub fn enque_trb(&mut self, mut trb: TrbData) -> usize {
        self.trbs[self.i].copy_from_slice(&trb);
        let addr = self.trbs[self.i].as_ptr() as usize;
        
        // 确保TRB写入内存
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        trace!("[Ring] enque_trb: 已写入TRB到地址=0x{:08X}, 环索引={}", addr, self.i);
        
        trace!(
            "enqueued {} @{:#X}\n{:x}\n{:x}\n{:x}\n{:x}\n------------------------------------------------",
            self.i, addr, trb[0], trb[1], trb[2], trb[3]
        );
        self.next_index();
        addr
    }

    pub fn enque_trbs(&mut self, trb: Vec<TrbData>) {
        for ele in trb {
            self.enque_trb(ele);
        }
    }

    fn next_index(&mut self) -> usize {
        self.i += 1;
        let mut need_link = false;
        let len = self.len();

        // link模式下，最后一个是Link
        if self.link && self.i >= len - 1 {
            self.i = 0;
            need_link = true;
            trace!("flip and link!")
        } else if self.i >= len {
            self.i = 0;
        }

        if need_link {
            trace!("link!");
            let address = self.trbs[0].as_ptr() as usize;
            let mut link = Link::new();
            link.set_ring_segment_pointer(address as u64)
                .set_toggle_cycle();

            if self.cycle {
                link.set_cycle_bit();
            } else {
                link.clear_cycle_bit();
            }
            let trb = command::Allowed::Link(link);
            let link_trb = trb.into_raw();
            let mut this_trb = &mut self.trbs[len - 1];
            this_trb.copy_from_slice(&link_trb);

            warn!("[Ring] next_index: 翻转环cycle位! 旧cycle={}, 新cycle={}", self.cycle, !self.cycle);
            self.cycle = !self.cycle;
        }

        self.i
    }

    /// 完成一次循环返回true
    pub fn inc_deque(&mut self) -> bool {
        let old_i = self.i;
        let old_cycle = self.cycle;
        self.i += 1;
        let mut is_cycle = false;
        let len = self.len();
        // Event Ring is always non-linked
        // if self.link { <-- This block is irrelevant for Event Ring
        // } else { <-- Logic for non-linked ring (Event Ring)
            if self.i >= len {
                self.i = 0;
                let old_cycle_val = self.cycle;
                warn!("[Ring] next_index: 翻转环cycle位! 旧cycle={}, 新cycle={}", self.cycle, !self.cycle);
            self.cycle = !self.cycle;
                is_cycle = true;
                trace!(
                    "Ring::inc_deque: Cycle bit flipped by software from {} to {} at index {} (wrapped to 0). Ring len: {}.",
                    old_cycle_val, self.cycle, old_i, len
                );
            }
        // }
        trace!(
            "Ring::inc_deque: i: {} -> {}, cycle: {} -> {}, is_cycle: {}",
            old_i, self.i, old_cycle, self.cycle, is_cycle
        );
        is_cycle
    }

    pub fn current_data(&mut self) -> (&TrbData, bool) {
        (&self.trbs[self.i], self.cycle)
    }

    pub fn get_len(&self) -> usize {
        self.trbs.len()
    }

    pub fn new_isoch(os: O, alloc: &O::DMA) -> Result<Self> {
        // 等时传输需要更大的环来避免溢出
        const ISOCH_RING_SIZE: usize = 512; // 增加到256个TRB
        // 重要：等时传输环必须使用Link TRB来正确循环
        Self::new(os, ISOCH_RING_SIZE, true) // true表示使用link TRB
    }

    /// 返回环是否使用Link TRB
    pub fn has_link(&self) -> bool {
        self.link
    }
    
    /// 创建命令环 - 命令环的初始cycle bit必须是1
    pub fn new_command_ring(os: O, len: usize) -> Result<Self> {
        let a = os.dma_alloc();
        let mut trbs = DMA::new_vec([0; TRB_LEN], len, 64, a);
        
        // 命令环总是需要Link TRB
        if len > 1 {
            let address = trbs.addr() as u64;
            let mut link_trb = Link::new();
            link_trb.set_ring_segment_pointer(address)
                    .set_toggle_cycle()
                    .set_cycle_bit(); // 初始Link TRB的cycle bit应该是1
            
            let link_data = command::Allowed::Link(link_trb).into_raw();
            trbs[len - 1].copy_from_slice(&link_data);
            
            trace!("[Ring::new_command_ring] 初始化Link TRB at index {}: addr=0x{:X}, toggle_cycle=1, cycle_bit=1", 
                   len - 1, address);
            trace!("[Ring::new_command_ring] 创建命令环: 大小={}, 基地址=0x{:08X}, 初始cycle=true", len, address);
        }
        
        // XHCI规范：命令环的初始cycle bit是1
        Ok(Self {
            trbs,
            i: 0,
            cycle: true, // 命令环初始cycle为true
            link: true,
        })
    }
}
