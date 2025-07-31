use alloc::{borrow::ToOwned, boxed::Box, sync::Arc, vec, vec::Vec, string::ToString};
use context::{DeviceContextList, ScratchpadBufferArray};
use core::{
    mem::{self, MaybeUninit},
    num::NonZeroUsize,
    ops::{Deref, DerefMut},
    sync::atomic::{fence, Ordering},
    future::Future,
    pin::Pin,
    task::{Context as TaskContext, Poll, Waker},
    time::Duration,
};
// use instant::Instant; // Not available in no_std
use event_ring::EventRing;
use log::{info, error, warn, debug, trace};
use ring::Ring;
use spinlock::SpinNoIrq;
use xhci::{
    accessor::Mapper,
    context::{DeviceHandler, EndpointState, EndpointType, Input, InputHandler, SlotHandler},
    extended_capabilities::XhciSupportedProtocol,
    ring::trb::{
        command,
        event::{self, CommandCompletion, CompletionCode, HostController},
        transfer::{self, Direction, Normal, TransferType},
    },
    ExtendedCapability,
    ring::trb::event::TransferEvent,
    registers::doorbell,
};
use alloc::format;
use alloc::collections::BTreeMap;

use crate::{
    abstractions::{dma::DMA, PlatformAbstractions},
    err::Error,
    glue::ucb::{CompleteCode, TransferEventCompleteCode, UCB},
    host::data_structures::MightBeInited,
    usb::{
        descriptors::{
            desc_configuration,
            topological_desc::{
                TopologicalUSBDescriptorConfiguration, TopologicalUSBDescriptorEndpoint,
                TopologicalUSBDescriptorFunction,
            },
            USBStandardDescriptorTypes,
        },
        operation::{Configuration, ExtraStep},
        trasnfer::{
            self,
            control::{bRequest, bmRequestType, ControlTransfer, DataTransferType},
            isoch::IsochTransfer,
        },
        urb,
        drivers::driverapi::USBSystemDriverModuleInstance,
    },
    USBSystemConfig,
};

use super::Controller;

mod context;
mod event_ring;
mod ring;

pub type RegistersBase = xhci::Registers<MemMapper>;
pub type RegistersExtList = xhci::extended_capabilities::List<MemMapper>;
pub type SupportedProtocol = XhciSupportedProtocol<MemMapper>;

const TAG: &str = "[XHCI]"; 
const ISOC_TRANSFER_TIMEOUT_MS: u64 = 1000000; // 增加等时传输超时时间到1秒

fn current_time() -> u128 {
    axhal::time::current_time_nanos() as u128
}

static mut GLOBAL_XHCI_ARC: Option<*const ()> = None;

/// 全局中断处理函数指针
static mut GLOBAL_XHCI_HANDLER: Option<fn()> = None;

/// 全局中断处理回调 - 真正的处理逻辑
static mut GLOBAL_INTERRUPT_CALLBACK: Option<Box<dyn Fn() + Send + 'static>> = None;

/// 全局中断计数器，用于调试
static INTERRUPT_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// XHCI中断处理trait
trait XHCIInterruptHandler: Send {
    fn handle_interrupt(&mut self);
}

/// 设置全局XHCI Arc指针
pub fn set_global_xhci_arc<O>(arc: Arc<SpinNoIrq<XHCI<O>>>) 
where
    O: PlatformAbstractions + 'static,
{
    unsafe {
        // 增加引用计数并存储原始指针
        let ptr = Arc::into_raw(arc) as *const ();
        GLOBAL_XHCI_ARC = Some(ptr);
        info!("{TAG} 全局XHCI Arc指针已设置: {:p}", ptr);
    }
}

/// 设置全局中断处理函数
pub fn set_global_xhci_handler(handler: fn()) {
    unsafe {
        GLOBAL_XHCI_HANDLER = Some(handler);
        info!("{TAG} 全局XHCI中断处理函数已设置");
    }
}

/// 设置全局中断处理回调（带类型信息）
pub fn set_global_interrupt_callback<O>(xhci_arc: Arc<SpinNoIrq<XHCI<O>>>) 
where
    O: PlatformAbstractions + 'static,
{
    unsafe {
        // 创建一个闭包，捕获Arc
        let callback = Box::new(move || {
            let mut xhci = xhci_arc.lock();
            // 处理中断事件
            xhci.handle_interrupt();
        });
        
        GLOBAL_INTERRUPT_CALLBACK = Some(callback);
        
        // 设置简单的中断处理函数，它会调用真正的回调
        GLOBAL_XHCI_HANDLER = Some(generic_interrupt_handler);
        
        info!("{TAG} 全局中断处理回调已设置");
    }
}

/// 通用中断处理函数 - 调用真正的回调
fn generic_interrupt_handler() {
    unsafe {
        if let Some(ref callback) = GLOBAL_INTERRUPT_CALLBACK {
            callback();
        } else {
            let count = INTERRUPT_COUNT.load(core::sync::atomic::Ordering::Relaxed);
            if count < 10 {
                error!("{TAG} 中断回调未设置!");
            }
        }
    }
}

/// XHCI中断处理函数（由系统调用）
pub fn xhci_interrupt_handler() {
    let count = INTERRUPT_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    
    unsafe {
        // 检查所有全局变量的状态
        let has_handler = GLOBAL_XHCI_HANDLER.is_some();
        let has_callback = GLOBAL_INTERRUPT_CALLBACK.is_some();
        let has_arc = GLOBAL_XHCI_ARC.is_some();
        
        if count == 0 {
            error!("{TAG} 第一次中断 - 调试信息:");
            error!("{TAG}   GLOBAL_XHCI_HANDLER: {}", if has_handler { "已设置" } else { "未设置" });
            error!("{TAG}   GLOBAL_INTERRUPT_CALLBACK: {}", if has_callback { "已设置" } else { "未设置" });
            error!("{TAG}   GLOBAL_XHCI_ARC: {}", if has_arc { "已设置" } else { "未设置" });
        }
        
        if let Some(handler) = GLOBAL_XHCI_HANDLER {
            handler();
        } else {
            // 临时解决方案：只打印前100次的错误信息
            if count < 100 {
                error!("{TAG} 收到中断 #{} 但GLOBAL_XHCI_HANDLER未设置！", count + 1);
            }
        }
    }
}



// ===== 异步传输支持 =====

/// 传输请求的唯一标识符
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TransferId {
    /// 设备槽ID
    pub slot_id: usize,
    /// 端点ID (DCI)
    pub endpoint_id: u8,
    /// TRB指针地址
    pub trb_pointer: u64,
}

impl TransferId {
    /// 创建新的传输ID
    pub fn new(slot_id: usize, endpoint_id: u8, trb_pointer: u64) -> Self {
        Self {
            slot_id,
            endpoint_id,
            trb_pointer,
        }
    }
}

/// 命令ID，用于跟踪异步命令
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CommandId {
    /// 命令TRB地址
    pub trb_pointer: u64,
}

/// 传输请求类型
#[derive(Debug, Clone)]
pub enum AsyncTransferType {
    Control(ControlTransfer),
    Interrupt(trasnfer::interrupt::InterruptTransfer),
    Isoch(IsochTransfer),
    Normal, // 普通传输
}

/// 传输请求状态
#[derive(Debug, Clone)]
pub enum TransferState {
    /// 等待提交到硬件
    Pending,
    /// 已提交到硬件，等待完成
    Submitted {
        /// 提交时间戳（纳秒）
        submitted_at: u128,
    },
    /// 传输完成
    Completed(crate::err::Result<TransferEvent>),
    /// 传输已取消
    Cancelled,
}

/// 命令状态
#[derive(Debug, Clone)]
pub enum CommandState {
    /// 等待提交
    Pending,
    /// 已提交，等待完成
    Submitted {
        /// 提交时间戳（纳秒）
        submitted_at: u128,
    },
    /// 命令完成
    Completed(Result<CommandCompletion, Error>),
}

/// 异步命令记录
pub struct AsyncCommand {
    /// 命令ID
    pub id: CommandId,
    /// 命令TRB地址
    pub trb_pointer: u64,
    /// 命令状态
    pub state: CommandState,
    /// Future唤醒器
    pub waker: Option<Waker>,
}

/// 异步传输请求
pub struct AsyncTransfer<O: PlatformAbstractions> {
    /// 传输ID
    pub id: TransferId,
    /// 设备槽ID
    pub slot_id: usize,
    /// 端点DCI
    pub endpoint_dci: u8,
    /// TRB指针
    pub trb_pointer: u64,
    /// 传输类型
    pub transfer_type: AsyncTransferType,
    /// 当前状态
    pub state: TransferState,
    /// 等待此传输完成的waker
    pub waker: Option<Waker>,
    /// 平台抽象标记
    _phantom: core::marker::PhantomData<O>,
}

/// 命令完成的Future
pub struct CommandFuture<O: PlatformAbstractions> {
    /// 命令ID
    id: CommandId,
    /// XHCI控制器的引用
    xhci: Arc<SpinNoIrq<XHCI<O>>>,
}

/// 传输完成的Future
pub struct TransferFuture<O: PlatformAbstractions> {
    /// 传输ID
    id: TransferId,
    /// 传输管理器的引用
    xhci: Arc<SpinNoIrq<XHCI<O>>>,
}

impl<O: PlatformAbstractions> Future for CommandFuture<O> {
    type Output = crate::err::Result<CommandCompletion>;

    fn poll(self: Pin<&mut Self>, cx: &mut TaskContext<'_>) -> Poll<Self::Output> {
        let mut xhci = self.xhci.lock();
        
        if let Some(command) = xhci.async_commands.get_mut(&self.id) {
            match &command.state {
                CommandState::Completed(result) => {
                    // 克隆结果，因为我们需要返回它
                    let result = result.clone();
                    // 从映射中移除命令
                    xhci.async_commands.remove(&self.id);
                    Poll::Ready(result)
                }
                CommandState::Pending | CommandState::Submitted { .. } => {
                    // 设置waker，以便在命令完成时唤醒
                    command.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            }
        } else {
            // 命令不存在，可能已经被处理了
            Poll::Ready(Err(Error::Unknown("命令不存在".to_owned())))
        }
    }
}

impl<O: PlatformAbstractions> Future for TransferFuture<O> {
    type Output = crate::err::Result<UCB<O>>;

    fn poll(self: Pin<&mut Self>, cx: &mut TaskContext<'_>) -> Poll<Self::Output> {
        let mut xhci = self.xhci.lock();
        
        if let Some(transfer) = xhci.async_transfers.get_mut(&self.id) {
            match &transfer.state {
                TransferState::Completed(result) => {
                    match result {
                        Ok(event) => {
                            let code = event.completion_code().unwrap_or(CompletionCode::Invalid);
                            let complete_code = match code {
                                CompletionCode::Success => {
                                    CompleteCode::Event(TransferEventCompleteCode::Success(Some(event.trb_pointer())))
                                }
                                CompletionCode::Stopped => {
                                    CompleteCode::Event(TransferEventCompleteCode::Halt(Some(event.trb_pointer())))
                                }
                                CompletionCode::StallError => {
                                    CompleteCode::Event(TransferEventCompleteCode::Stall(Some(event.trb_pointer())))
                                }
                                CompletionCode::BabbleDetectedError => {
                                    CompleteCode::Event(TransferEventCompleteCode::Babble(Some(event.trb_pointer())))
                                }
                                _ => CompleteCode::Event(TransferEventCompleteCode::Unknown(code as u8)),
                            };
                            Poll::Ready(Ok(UCB::new(complete_code)))
                        }
                        Err(e) => Poll::Ready(Err(e.clone())),
                    }
                }
                TransferState::Cancelled => {
                    Poll::Ready(Err(Error::Cancelled))
                }
                _ => {
                    // 保存waker以便在传输完成时唤醒
                    transfer.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            }
        } else {
            // 传输不存在
            Poll::Ready(Err(Error::InvalidSlot))
        }
    }
}

impl<O: PlatformAbstractions> Clone for AsyncTransfer<O> {
    fn clone(&self) -> Self {
        Self {
            id: self.id,
            slot_id: self.slot_id,
            endpoint_dci: self.endpoint_dci,
            trb_pointer: self.trb_pointer,
            transfer_type: self.transfer_type.clone(),
            state: self.state.clone(),
            waker: None, // Waker不能克隆
            _phantom: core::marker::PhantomData,
        }
    }
}

// 全局中断状态和当前XHCI实例指针
static mut INTERRUPT_ENABLED: bool = false;
// static mut CURRENT_XHCI_BASE: usize = 0;
const CURRENT_XHCI_BASE: usize = 0x7f0000004218;
// static mut INTERRUPT_COUNT: usize = 0; // 已在上面定义为AtomicU32
static mut PROCESSED_EVENT_COUNT: usize = 0;

// UVC等时传输统计变量
static mut ZERO_BYTE_COUNT: u32 = 0;
static mut LAST_ENDPOINT: u8 = 0;

// 等时传输帧间隔
const ISOC_FRAME_DELAY: u32 = 32;

// 简化版中断处理函数，仅处理当前实例的事件
fn handle_xhci_events<O: PlatformAbstractions + 'static>(xhci: &mut XHCI<O>) {
    // 检查是否是XHCI中断
    let usbsts = xhci.regs.operational.usbsts.read_volatile();
    if !usbsts.event_interrupt() {
        info!("{TAG} 收到中断但USBSTS.EINT=0，忽略");
        return; 
    }
    
    // 处理所有待处理事件
    let processed = xhci.process_event_ring();
    
    
    // 清除中断挂起位
    xhci.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
        im.set_0_interrupt_pending();  // 正确的方法名
    });
}

#[derive(Clone)]
pub struct MemMapper;
impl Mapper for MemMapper {
    unsafe fn map(&mut self, phys_start: usize, bytes: usize) -> NonZeroUsize {
        return NonZeroUsize::new_unchecked(phys_start);
    }
    fn unmap(&mut self, virt_start: usize, bytes: usize) {}
}
pub struct XHCI<O>
where
    O: PlatformAbstractions,
{
    config: Arc<SpinNoIrq<USBSystemConfig<O>>>,
    pub regs: RegistersBase,
    pub ext_list: Option<RegistersExtList>,
    max_slots: u8,
    max_ports: u8,
    max_irqs: u16,
    scratchpad_buf_arr: Option<ScratchpadBufferArray<O>>,
    cmd: Ring<O>,
    event: EventRing<O>,
    pub dev_ctx: DeviceContextList<O>,
    // 添加中断处理状态标志
    interrupt_enabled: bool,
    // 等时传输第一次标志
    isoc_first_transfer: bool,
    // 异步传输管理
    async_transfers: BTreeMap<TransferId, AsyncTransfer<O>>,
    // 异步命令管理
    async_commands: BTreeMap<CommandId, AsyncCommand>,
    // 自引用Arc，用于创建Future
    self_arc: Option<Arc<SpinNoIrq<XHCI<O>>>>,
    // 等时传输URB映射：使用buffer地址作为唯一键，避免多个URB使用同一端点时被覆盖
    pending_isoch_urbs: BTreeMap<u64, PendingIsochUrb<'static, O>>,
    // 端点进度跟踪，用于检测硬件卡住
    endpoint_progress: BTreeMap<(usize, usize), EndpointProgress>,
}

/// 等待中的等时URB信息
struct PendingIsochUrb<'a, O: PlatformAbstractions> {
    sender: Arc<SpinNoIrq<dyn USBSystemDriverModuleInstance<'a, O>>>,
    buffer_addr: u64,
    submitted_at: u64,
}

/// 端点进度跟踪 - 修正版：不再依赖TR Dequeue Pointer
#[derive(Debug, Clone)]
struct EndpointProgress {
    submitted_trbs: u64,        // 已提交的TRB数量
    completed_events: u64,      // 已收到的Transfer Event数量
    last_event_time: u64,       // 最后一次收到事件的时间(MFINDEX)
    last_submit_time: u64,      // 最后一次提交TRB的时间(MFINDEX)
    no_progress_count: u32,     // 连续无进展的检查次数
}

impl<O> XHCIInterruptHandler for XHCI<O>
where
    O: PlatformAbstractions + 'static,
{
    fn handle_interrupt(&mut self) {
        // 使用静态计数器避免在中断处理中频繁打印
        static mut INTERRUPT_LOG_COUNT: u32 = 0;
        
        // 读取中断状态
        let usbsts = self.regs.operational.usbsts.read_volatile();
        let iman = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
        
        // 检查是否有真正的中断
        if !usbsts.event_interrupt() && !iman.interrupt_pending() {
            return;
        }
        
        // 只记录前几次中断，避免日志系统死锁
        unsafe {
            if INTERRUPT_LOG_COUNT < 5 {
                INTERRUPT_LOG_COUNT += 1;
                trace!("{TAG} 处理中断 #{}: USBSTS.EINT={}, IMAN.IP={}", 
                      INTERRUPT_LOG_COUNT, usbsts.event_interrupt(), iman.interrupt_pending());
            }
        }
        
        // 清除中断状态位
        if usbsts.event_interrupt() {
            self.regs.operational.usbsts.update_volatile(|s| {
                s.clear_event_interrupt();
            });
        }
        
        // 处理事件环中的所有事件
        let events_processed = self.process_event_ring();
        
        // 清除IMAN的IP位
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.set_0_interrupt_pending(); // W1C
        });
        
        unsafe {
            if INTERRUPT_LOG_COUNT <= 5 {
                trace!("{TAG} 中断处理完成，处理了 {} 个事件", events_processed);
            }
        }
    }
}


impl<O> XHCI<O>
where
    O: PlatformAbstractions + 'static,
{
    /// 设置self_arc引用
    pub fn set_self_arc(&mut self, arc: Arc<SpinNoIrq<XHCI<O>>>) {
        self.self_arc = Some(arc.clone());
        
        // 设置全局XHCI Arc指针
        set_global_xhci_arc(arc.clone());
        
        // 设置全局中断处理回调
        set_global_interrupt_callback(arc);
        
        info!("{TAG} self_arc和全局XHCI实例指针已设置");
        
        // 现在可以安全地启用中断了
        if !self.interrupt_enabled {
            info!("{TAG} 现在启用中断（self_arc已设置）");
            
            // 先启用Interrupter 0的中断（如果还没启用）
            self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
                im.set_interrupt_enable();
            });
            
            // 然后启用全局XHCI中断
            self.regs.operational.usbcmd.update_volatile(|r| {
                r.set_interrupter_enable();
            });
            
            // 最后在系统中启用中断路由
            const XHCI_IRQ_NUM: usize = 48;
            axhal::irq::set_enable(XHCI_IRQ_NUM, true);
            info!("{TAG} 已在系统中启用XHCI中断(IRQ {})", XHCI_IRQ_NUM);
            
            let usbcmd_val = self.regs.operational.usbcmd.read_volatile();
            info!("{TAG} USBCMD.INTE (中断使能) 最终状态: {}", usbcmd_val.interrupter_enable());
            
            // 添加更多调试信息
            self.debug_interrupt_status();
            
            // 检查GIC中断状态
            info!("{TAG} 检查GIC中断状态...");
            
            // 延迟中断测试到init完成后
            info!("{TAG} 中断测试将在控制器完全初始化后进行");
            
            self.interrupt_enabled = true;
        }
    }
    
    // 初始化中断处理（不启用中断）
    pub fn init_interrupt_handling_without_enable(&mut self) {
        info!("{TAG} 初始化XHCI中断处理（不启用中断）");
        
        // 全局中断处理函数将在用户代码中设置
        
        const XHCI_IRQ_NUM: usize = 48;
        
    }
    
    // 清除所有挂起的事件
    pub fn clear_pending_events(&mut self) {
        info!("{TAG} 清除所有挂起的事件");
        
        // 清除USBSTS的中断标志
        let usbsts = self.regs.operational.usbsts.read_volatile();
        if usbsts.event_interrupt() {
            self.regs.operational.usbsts.update_volatile(|s| {
                s.clear_event_interrupt();
            });
            info!("{TAG} 清除了USBSTS.EINT");
        }
        
        if usbsts.port_change_detect() {
            self.regs.operational.usbsts.update_volatile(|s| {
                s.clear_port_change_detect();
            });
            info!("{TAG} 清除了USBSTS.PCD");
        }
        
        // 清除IMAN的中断挂起位
        let iman = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
        if iman.interrupt_pending() {
            self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
                im.set_0_interrupt_pending(); // W1C
            });
            info!("{TAG} 清除了IMAN.IP");
        }
        
        // 处理并丢弃事件环中的所有事件
        let mut events_cleared = 0;
        while let Some((event, _)) = self.event.next() {
            events_cleared += 1;
            // 更新ERDP
            self.update_erdp();
        }
        
        if events_cleared > 0 {
            info!("{TAG} 从事件环中清除了 {} 个事件", events_cleared);
        }
        
        // 再次确保所有中断标志都被清除
        self.regs.operational.usbsts.update_volatile(|s| {
            s.clear_event_interrupt();
            s.clear_port_change_detect();
        });
        
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.set_0_interrupt_pending();
        });
    }
    
    // 原来的init_interrupt_handling函数（已废弃）
    pub fn init_interrupt_handling(&mut self) {
        panic!("{TAG} 不应该调用这个函数，请使用init_interrupt_handling_without_enable");
    }
    
    // Linux方式处理事件环
    pub fn process_event_ring(&mut self) -> usize {

        let mut processed_events = 0;
        let max_events = 256; // 防止无限循环
        
        // 在开始处理事件前，确保EHB位被清除
        let erdp_before = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        if erdp_before.event_handler_busy() {
            warn!("{TAG} [process_event_ring] EHB=1 在处理前，尝试清除");
            self.clear_ehb();
        }
        
        // Linux不限制每次处理的事件数量，而是处理所有可用事件
        // 直到事件环为空
        while processed_events < max_events {
            // 检查EHB状态
            let erdp_reg = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
            if erdp_reg.event_handler_busy() && processed_events == 0 {
                warn!("{TAG} EHB=1但尝试处理事件");
            }
            
            if let Some((event, _cycle_wrapped)) = self.event.next() {
                processed_events += 1;
                
                // Linux方式：每处理一个事件就更新ERDP
                // 这是避免EHB问题的关键
                self.update_erdp();
                
                // Linux方式：根据事件类型分发
                match event {
                    event::Allowed::TransferEvent(te) => {
                        self.handle_transfer_event(te);
                    },
                    event::Allowed::CommandCompletion(cc) => {
                        self.handle_command_completion(cc);
                    },
                    event::Allowed::PortStatusChange(psc) => {
                        self.handle_port_status_change(psc);
                    },
                    event::Allowed::HostController(hc) => {
                        self.handle_host_controller_event(hc);
                    },
                    _ => {
                        // Linux默默忽略未知事件
                    }
                }
            } else {
                // 没有更多事件
                break;
            }
        }
        
        // Linux: 最后检查并清除中断状态
        if processed_events > 0 {
            // 清除USBSTS中的EINT位
            self.regs.operational.usbsts.update_volatile(|s| {
                s.clear_event_interrupt();
            });
            
            // 清除IMAN中的IP位
            self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
                im.set_0_interrupt_pending(); // W1C
            });
        }
        
        // 打印事件统计信息
        if processed_events > 0 {
            debug!("{TAG} [process_event_ring] 处理了 {} 个事件", processed_events);
            
            // 每处理100个事件打印一次进度统计
            static mut EVENT_COUNTER: u64 = 0;
            unsafe {
                EVENT_COUNTER += processed_events as u64;
                if EVENT_COUNTER >= 100 {
                    self.print_endpoint_progress_summary();
                    EVENT_COUNTER = 0;
                }
            }
        }
        
        // 再次检查EHB
        let erdp_after = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        if erdp_after.event_handler_busy() {
            error!("{TAG} [process_event_ring] ⚠️ 处理后EHB仍为1！可能导致事件丢失");
        }
        
        processed_events
    }
    
    // 处理中断 - 在每次中断后或定期调用此方法
    /// 测试中断机制
    pub fn test_interrupt(&mut self) {
        warn!("{TAG} ===== 开始中断测试 =====");
        
        // 打印控制器基址以确认
        unsafe {
            warn!("{TAG} 当前控制器基址: 0x{:X}", CURRENT_XHCI_BASE);
        }
        
        // 1. 检查当前中断配置
        let iman = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
        let erdp = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        let usbsts = self.regs.operational.usbsts.read_volatile();
        
        warn!("{TAG} 测试前状态:");
        warn!("{TAG}   IMAN: IP={}, IE={}", iman.interrupt_pending(), iman.interrupt_enable());
        warn!("{TAG}   ERDP: 0x{:016X}, EHB={}", erdp.event_ring_dequeue_pointer(), erdp.event_handler_busy());
        warn!("{TAG}   USBSTS: EINT={}, PCD={}, HCH={}", 
              usbsts.event_interrupt(), usbsts.port_change_detect(), usbsts.hc_halted());
              
        // 如果有挂起的中断，先处理它们
        if iman.interrupt_pending() || usbsts.event_interrupt() {
            warn!("{TAG} 检测到挂起的中断，先处理它们...");
            
            // 处理事件环
            let events_processed = self.process_event_ring();
            warn!("{TAG} 处理了 {} 个事件", events_processed);
            
            // 更新ERDP
            self.update_erdp();
            
            // 清除中断
            self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
                im.clear_interrupt_pending();
            });
            self.regs.operational.usbsts.update_volatile(|s| {
                s.set_0_event_interrupt();
            });
            
            // 再次检查状态
            let iman_cleared = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
            let usbsts_cleared = self.regs.operational.usbsts.read_volatile();
            warn!("{TAG} 清除后: IMAN IP={}, USBSTS EINT={}", 
                  iman_cleared.interrupt_pending(), usbsts_cleared.event_interrupt());
        }
        
        // 2. 手动创建一个事件来触发中断
        warn!("{TAG} 发送Test命令来触发事件...");
        
        // 使用NOP命令作为测试
        let nop = xhci::ring::trb::command::Noop::new();
        let nop_allowed = xhci::ring::trb::command::Allowed::Noop(nop);
        
        // 直接操作命令环
        let trb_index = self.cmd.enque_command(nop_allowed);
        if trb_index > 0 {
            warn!("{TAG} NOP命令已发送");
            
            // 等待一段时间
            for _ in 0..1000000 {
                core::hint::spin_loop();
            }
            
            // 检查是否有中断产生
            let iman_after = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
            let usbsts_after = self.regs.operational.usbsts.read_volatile();
            
            warn!("{TAG} 发送命令后状态:");
            warn!("{TAG}   IMAN: IP={}, IE={}", iman_after.interrupt_pending(), iman_after.interrupt_enable());
            warn!("{TAG}   USBSTS: EINT={}", usbsts_after.event_interrupt());
            
            unsafe {
                warn!("{TAG}   全局中断计数: {}", INTERRUPT_COUNT.load(Ordering::Relaxed));
                warn!("{TAG}   INTERRUPT_ENABLED: {}", INTERRUPT_ENABLED);
            }
        } else {
            error!("{TAG} 发送NOP命令失败");
        }
        
        warn!("{TAG} ===== 中断测试结束 =====");
    }
    
    pub fn handle_pending_interrupts(&mut self) -> bool {
        // Linux方式: xhci_irq()
        
        // 读取状态
        let status = self.regs.operational.usbsts.read_volatile();
        
        // Linux: 检查是否有需要处理的中断
        if !status.event_interrupt() && !status.host_system_error() && !status.port_change_detect() {
            // 没有中断需要处理
            return false;
        }
        
        // Linux: 清除所有状态位（写1清除）
        // 移除status.get()调用，因为UsbStatusRegister没有get方法
        self.regs.operational.usbsts.update_volatile(|s| {
            // Linux只清除特定的位
            if status.event_interrupt() {
                s.clear_event_interrupt();
            }
            if status.host_system_error() {
                s.clear_host_system_error();
            }
            if status.host_controller_error() {
                // host_controller_error是只读位，不能被清除
            }
            if status.port_change_detect() {
                s.clear_port_change_detect();
            }
        });
        
        // Linux: 处理错误
        if status.host_system_error() || status.host_controller_error() {
            error!("{TAG} HC错误: HSE={}, HCE={}", 
                   status.host_system_error(), status.host_controller_error());
            // Linux会停止控制器，但我们暂时只记录
        }
        
        let mut work_done = false;
        
        // Linux: 处理事件
        if status.event_interrupt() {
            // Linux处理所有interrupter
            // 我们只有一个interrupter
            let events = self.process_event_ring();
            if events > 0 {
                work_done = true;
            }
        }
        
        // Linux: 端口状态变化会在事件环中处理
        // 不需要额外处理
        
        work_done
    }
    
    // 处理传输完成事件
    fn handle_transfer_event(&mut self, te: TransferEvent) {
        let slot_id = te.slot_id() as usize;
        let endpoint_id = te.endpoint_id();
        let trb_pointer = te.trb_pointer();
        let completion_code = te.completion_code().unwrap_or(CompletionCode::Invalid);
        
        let now = self.read_mfindex() as u64;
             
        // 查找并唤醒等待的异步传输
        let mut found_transfer = None;
        for (id, transfer) in self.async_transfers.iter_mut() {
            if transfer.slot_id == slot_id && 
               transfer.endpoint_dci == endpoint_id &&
               (transfer.trb_pointer == trb_pointer || 
                // 对于等时传输，可能不需要精确匹配TRB指针
                (endpoint_id == 3 || endpoint_id == 5 || endpoint_id == 7)) {
                // 更新传输状态
                transfer.state = TransferState::Completed(Ok(te.clone()));
                
                // 唤醒等待的任务
                if let Some(waker) = transfer.waker.take() {
                    waker.wake();
                }
                
                found_transfer = Some(*id);
                break;
            }
        }
        
        // 如果找到了传输，可以选择是否立即移除
        if let Some(id) = found_transfer {
            // 保留传输记录一段时间，让Future有机会读取结果
            // self.async_transfers.remove(&id);
        }
             
        // 更新端点进度跟踪 - 增加已完成事件计数
        let key = (slot_id, endpoint_id as usize);
        if let Some(progress) = self.endpoint_progress.get_mut(&key) {
            let old_completed = progress.completed_events;
            let old_last_event_time = progress.last_event_time;
            
            progress.completed_events += 1;
            progress.last_event_time = now;
            
            // 计算事件间隔
            let event_interval = now.saturating_sub(old_last_event_time);
            
            // 如果所有TRB都完成了，重置无进展计数器
            if progress.completed_events >= progress.submitted_trbs {
                progress.no_progress_count = 0;
            }
            
            let pending = progress.submitted_trbs.saturating_sub(progress.completed_events);
            trace!("{TAG} [进度更新] slot={}, DCI={}: 事件#{} -> #{}, 间隔={}ms, 待完成TRB={}",
                   slot_id, endpoint_id, old_completed, progress.completed_events, 
                   event_interval / 8, pending);
                  
            // 如果事件频率正常，清除无进展计数
            if event_interval < 8000 { // 小于1秒
                progress.no_progress_count = 0;
            }
        } else {
            // 没有进度跟踪，创建一个
            warn!("{TAG} [进度更新] 没有找到slot={}, DCI={}的进度跟踪，创建新的", slot_id, endpoint_id);
            self.endpoint_progress.insert(key, EndpointProgress {
                submitted_trbs: 0,
                completed_events: 1,
                last_event_time: now,
                last_submit_time: now,
                no_progress_count: 0,
            });
        }
             
        // 检查是否是错误状态，需要恢复端点
        match completion_code {
            CompletionCode::StallError => {
                error!("{TAG} [handle_transfer_event] 🔴 端点Stall！slot={}, DCI={}", slot_id, endpoint_id);
                // Stall错误需要重置端点
                if let Err(e) = self.recover_halted_endpoint(slot_id, endpoint_id as usize) {
                    error!("{TAG} [handle_transfer_event] ❌ 恢复Stalled端点失败: {:?}", e);
                }
            }
            CompletionCode::TrbError => {
                error!("{TAG} [handle_transfer_event] 🔴 TRB错误！slot={}, DCI={}, TRB=0x{:X}", 
                      slot_id, endpoint_id, trb_pointer);
                
                // 获取当前环状态用于调试
                if let Some(output_ctx) = self.dev_ctx.device_out_context_list.get(slot_id) {
                    let endpoint_handler = DeviceHandler::endpoint(&**output_ctx, endpoint_id as usize);
                    let hw_dequeue = endpoint_handler.tr_dequeue_pointer();
                    let hw_dcs = endpoint_handler.dequeue_cycle_state();
                    error!("{TAG} [handle_transfer_event] 硬件状态: TR Dequeue=0x{:X}, DCS={}", hw_dequeue, hw_dcs);
                }
                
                // TRB错误只需要移动Dequeue Pointer
                if let Err(e) = self.recover_stuck_endpoint(slot_id, endpoint_id as usize) {
                    error!("{TAG} [handle_transfer_event] ❌ 恢复TRB Error端点失败: {:?}", e);
                }
            }
            CompletionCode::BabbleDetectedError => {
                error!("{TAG} [handle_transfer_event] 🔴 Babble检测到！slot={}, DCI={}", slot_id, endpoint_id);
                // Babble错误需要重置端点
                if let Err(e) = self.recover_halted_endpoint(slot_id, endpoint_id as usize) {
                    error!("{TAG} [handle_transfer_event] ❌ 恢复Babble端点失败: {:?}", e);
                }
            }
            CompletionCode::UsbTransactionError => {
                warn!("{TAG} [handle_transfer_event] ⚠️ 事务错误，可能是USB总线错误: slot={}, DCI={}", 
                     slot_id, endpoint_id);
                // 事务错误可能是临时的，不一定需要重置
            }
            CompletionCode::RingOverrun => {
                // RingOverrun是正常的流控机制，不需要特殊处理
                if endpoint_id == 3 || endpoint_id == 5 || endpoint_id == 7 {
                    // 等时端点的RingOverrun是正常的流控
                    debug!("{TAG} [handle_transfer_event] Ring Overrun流控: slot={}, DCI={} - 硬件处理速度跟不上，这是正常现象", 
                          slot_id, endpoint_id);
                } else {
                    // 非等时端点的RingOverrun可能需要关注
                    warn!("{TAG} [handle_transfer_event] Ring Overrun: slot={}, DCI={} - 需要检查传输速率", 
                         slot_id, endpoint_id);
                }
            }
            CompletionCode::MissedServiceError => {
                warn!("{TAG} [handle_transfer_event] ⚠️ Missed Service错误，可能是系统响应太慢: slot={}, DCI={}", 
                     slot_id, endpoint_id);
                // MissedServiceError表示控制器无法及时处理TRB，需要重置端点状态
                // 这通常发生在系统负载高或中断处理延迟时
                if let Err(e) = self.recover_stuck_endpoint(slot_id, endpoint_id as usize) {
                    error!("{TAG} [handle_transfer_event] ❌ 恢复MissedService端点失败: {:?}", e);
                }
            }
            _ => {
                // Success, ShortPacket等正常情况
            }
        }
         
        if endpoint_id == 3 || endpoint_id == 5 || endpoint_id == 7 { // 特别关注等时传输端点
            debug!("{TAG} 等时传输完成事件: 槽ID={}, DCI={}, TRB指针=0x{:X}, 完成代码={:?}",
                 slot_id, endpoint_id, trb_pointer, completion_code);
                 
            // 对于等时传输，不需要特殊的出队管理
            // 硬件会自动处理环的循环使用

            // 检查是否有pending的异步URB
            // 直接使用TRB指针查找（现在我们使用TRB地址作为键）
            let found_urb = self.pending_isoch_urbs.remove(&trb_pointer);
            
            if found_urb.is_none() {
            }
            
            // 处理找到的URB
            if let Some(pending_urb) = found_urb {
                // 减少日志输出，只在调试级别输出详细信息
                
                // 只在调试模式下检查buffer内容
                #[cfg(debug_assertions)]
                unsafe {
                    let buffer_ptr = pending_urb.buffer_addr as *const u8;
                    let mut data_preview = [0u8; 16];
                    for i in 0..16 {
                        data_preview[i] = *buffer_ptr.add(i);
                    }
                    
                    // 检查是否是UVC帧头
                    if data_preview[0] & 0x0F == 0x0C { // HeaderLength = 12
                        let fid = (data_preview[1] >> 0) & 0x01;
                        let eof = (data_preview[1] >> 1) & 0x01;
                    }
                }
                
                // 创建UCB - 优化性能，减少条件判断
                let ucb = match completion_code {
                    CompletionCode::Success | CompletionCode::ShortPacket => {
                        UCB::new(CompleteCode::Event(TransferEventCompleteCode::Success(Some(pending_urb.buffer_addr))))
                    }
                    CompletionCode::RingOverrun => {
                        // 对于等时传输，Ring Overrun是正常的流控机制
                        // 继续传输，宁可丢帧也要保证实时性
                        trace!("{TAG} Ring Overrun - 正常流控，继续传输");
                        UCB::new(CompleteCode::Event(TransferEventCompleteCode::Success(Some(pending_urb.buffer_addr))))
                    }
                    _ => {
                        debug!("{TAG} 传输错误: 完成代码={:?}", completion_code);
                        UCB::new(CompleteCode::Event(TransferEventCompleteCode::Stall(Some(pending_urb.buffer_addr))))
                    }
                };
                
                // 发送完成事件给驱动
                pending_urb.sender.lock().receive_complete_event(ucb);
            } else {
                // 对于同步模式，这是正常的，因为同步模式不使用pending_isoch_urbs机制
                trace!("{TAG} 未找到pending URB（可能是同步模式）: slot={}, DCI={}, TRB=0x{:X}", slot_id, endpoint_id, trb_pointer);
            }
        }
        
        // 首先检查异步传输管理器
        let transfer_id = TransferId {
            slot_id,
            endpoint_id,
            trb_pointer,
        };
        
        // 查找匹配的异步传输（对于等时传输，忽略TRB指针）
        let matching_id = if endpoint_id == 3 || endpoint_id == 5 || endpoint_id == 7 {
            // 等时传输端点，只匹配槽ID和端点ID
            self.async_transfers.iter()
                .find(|(id, _)| id.slot_id == slot_id && id.endpoint_id == endpoint_id)
                .map(|(id, _)| *id)
        } else {
            // 其他端点，需要完全匹配
            if self.async_transfers.contains_key(&transfer_id) {
                Some(transfer_id)
            } else {
                None
            }
        };
        
        if let Some(id) = matching_id {
            let result = match completion_code {
                CompletionCode::Success => Ok(te),
                _ => Err(Error::CMD(completion_code)),
            };
            
            if let Some(transfer) = self.async_transfers.get_mut(&id) {
                transfer.state = TransferState::Completed(result);
                
                // 唤醒等待的Future
                if let Some(waker) = transfer.waker.take() {
                    waker.wake();
                }
            }
        }
        
    }
    
    // 处理命令完成事件
    fn handle_command_completion(&mut self, cc: CommandCompletion) {
        let slot_id = cc.slot_id() as usize;
        let trb_pointer = cc.command_trb_pointer();
        let completion_code = cc.completion_code().unwrap_or(CompletionCode::Invalid);
        
        trace!("{TAG} 处理命令完成事件: 槽ID={}, 命令TRB指针=0x{:X}, 完成代码={:?}",
             slot_id, trb_pointer, completion_code);
        
        // 查找匹配的异步命令
        let command_id = CommandId { trb_pointer };
        
        if let Some(command) = self.async_commands.get_mut(&command_id) {
            // 根据完成代码设置结果
            let result = match completion_code {
                CompletionCode::Success => Ok(cc),
                _ => Err(Error::CMD(completion_code)),
            };
            
            command.state = CommandState::Completed(result);
            
            // 唤醒等待的Future
            if let Some(waker) = command.waker.take() {
                waker.wake();
            }
            
            trace!("{TAG} 找到并唤醒异步命令: TRB=0x{:X}", trb_pointer);
        } else {
            // TRB=0x0 的事件可以安全忽略，这通常是某些特殊情况
            if trb_pointer == 0 {
                trace!("{TAG} 忽略 TRB=0x0 的命令完成事件");
            } else {
                warn!("{TAG} 收到未跟踪的命令完成事件: TRB=0x{:X}", trb_pointer);
            }
        }
    }
    
    // 处理端口状态变化事件
    fn handle_port_status_change(&mut self, psc: event::PortStatusChange) {
        let port_id = psc.port_id();
        
        info!("{TAG} 处理端口状态变化事件: 端口ID={}", port_id);
        
        // 读取并打印端口状态
        let portsc = self.regs.port_register_set.read_volatile_at(port_id as usize - 1).portsc;
        info!("{TAG} 端口{}状态: 连接={}, 使能={}, 速度={}", 
              port_id, portsc.current_connect_status(), portsc.port_enabled_disabled(), portsc.port_speed());
    }
    
    // 处理主控制器事件
    fn handle_host_controller_event(&mut self, hc: HostController) {
        info!("{TAG} 处理主控制器事件");
        
        match hc.completion_code() {
            Ok(CompletionCode::EventRingFullError) => {
                error!("{TAG} 事件环已满错误! 尝试恢复...");
                self.handle_event_ring_full_error();
            },
            _ => {
                info!("{TAG} 其他主控制器事件: {:?}", hc);
            }
        }
    }
    
    // 处理事件环已满错误
    fn handle_event_ring_full_error(&mut self) {
        // 不要立即执行强恢复，先尝试处理更多事件
        warn!("{TAG} 事件环可能已满，尝试处理更多事件");
        
        // 处理尽可能多的事件来清空事件环
        let mut events_processed = 0;
        let max_events = 100; // 最多处理100个事件
        
        while events_processed < max_events {
            if let Some((evt, _)) = self.event.next() {
                events_processed += 1;
                trace!("{TAG} 处理事件 #{} 以清空事件环", events_processed);
            } else {
                break;
            }
        }
        
        if events_processed > 0 {
            trace!("{TAG} 成功处理了 {} 个事件", events_processed);
            // 更新硬件ERDP
            let new_erdp = self.event.erdp();
            self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|f| {
                f.set_event_ring_dequeue_pointer(new_erdp);
            });
            return;
        }
        
        // 如果没有事件可处理，执行温和的恢复
        warn!("{TAG} 无法处理更多事件，执行温和恢复");
        
        // 步骤1: 清除中断pending位
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.clear_interrupt_pending();
        });
        
        // 步骤2: 强制更新ERDP到当前位置
        let current_erdp = self.event.erdp();
        self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|r| {
            r.set_event_ring_dequeue_pointer(current_erdp);
            r.set_0_event_handler_busy(); // 保持EHB=1
        });
        
        // 步骤3: 等待硬件清除EHB
        for _ in 0..10 {
            for _ in 0..1000 { core::hint::spin_loop(); }
            let ehb = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile().event_handler_busy();
            if !ehb {
                trace!("{TAG} EHB位已清除");
                return;
            }
        }
        
        warn!("{TAG} EHB位仍未清除，事件处理可能受影响");
        self.regs.operational.usbsts.update_volatile(|s| {
            s.set_0_event_interrupt();  // 写1清除
        });
        
        // 步骤9: 确保ERDP的EHB位被清除
        let current_erdp = self.event.erdp();
        self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|r| {
            r.set_event_ring_dequeue_pointer(current_erdp);
        });
        
        // 步骤10: 重新启用中断
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.set_interrupt_enable();
            im.set_0_interrupt_pending();
        });
        
        // 步骤11: 重新启用USBCMD的中断使能
        self.regs.operational.usbcmd.update_volatile(|r| {
            r.set_interrupter_enable();
        });
        
        // 检查恢复后的状态
        let ehb_after_recovery = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile().event_handler_busy();
        if ehb_after_recovery {
            error!("{TAG} 事件环恢复后EHB仍为1，这可能需要重置控制器");
        } else {
            trace!("{TAG} 事件环恢复完成，EHB已清除");
        }
    }
    
    // 创建异步传输并返回Future
    fn create_async_transfer(&mut self, dev_slot_id: usize, addr: u64, expected_endpoint_dci: u8) -> TransferFuture<O> {
        let transfer_id = TransferId::new(dev_slot_id, expected_endpoint_dci, addr);
        
        info!(
            "{TAG} 创建异步传输: ID={:?}, 槽{} @{:#X}, DCI {}",
            transfer_id, dev_slot_id, addr, expected_endpoint_dci
        );
        
        // 创建异步传输对象
        let async_transfer = AsyncTransfer {
            id: transfer_id,
            slot_id: dev_slot_id,
            endpoint_dci: expected_endpoint_dci,
            trb_pointer: addr,
            transfer_type: AsyncTransferType::Normal,
            state: TransferState::Pending,
            waker: None,
            _phantom: core::marker::PhantomData,
        };
        
        // 注册异步传输
        self.async_transfers.insert(transfer_id, async_transfer);
        
        // 返回Future
        TransferFuture {
            id: transfer_id,
            xhci: self.self_arc.as_ref().unwrap().clone(),
        }
    }
    
    // 这个函数已经被异步机制替代，保留仅供兼容
    fn event_busy_wait_transfer(&mut self, dev_slot_id: usize, addr: u64, expected_endpoint_dci: u8) -> crate::err::Result<event::TransferEvent> {
        warn!("{TAG} event_busy_wait_transfer被调用，应该使用异步传输！");
        
        // 创建异步传输并阻塞等待
        let transfer_id = TransferId {
            slot_id: dev_slot_id,
            endpoint_id: expected_endpoint_dci,
            trb_pointer: addr,
        };
        
        let transfer_type = if expected_endpoint_dci == 1 {
            AsyncTransferType::Normal
        } else if expected_endpoint_dci == 3 || expected_endpoint_dci == 5 || expected_endpoint_dci == 7 {
            AsyncTransferType::Normal
        } else {
            AsyncTransferType::Normal
        };
        
        let transfer = AsyncTransfer {
            id: transfer_id,
            slot_id: dev_slot_id,
            endpoint_dci: expected_endpoint_dci,
            trb_pointer: addr,
            transfer_type,
            state: TransferState::Pending,
            waker: None,
            _phantom: core::marker::PhantomData,
        };
        
        self.async_transfers.insert(transfer_id, transfer);
        
        // 使用简单的轮询等待（这不是最优方案，但保持兼容性）
        let start_time = (current_time() / 1000) as u64;
        
        loop {
            // 检查超时
            if ((current_time() / 1000) as u64).saturating_sub(start_time) > ISOC_TRANSFER_TIMEOUT_MS {
                self.async_transfers.remove(&transfer_id);
                return Err(Error::Timeout);
            }
            
            // 处理事件环
            self.process_event_ring();
            
            // 检查传输是否完成
            if let Some(transfer) = self.async_transfers.get(&transfer_id) {
                match &transfer.state {
                    TransferState::Completed(Ok(te)) => {
                        let result = *te;
                        self.async_transfers.remove(&transfer_id);
                        return Ok(result);
                    }
                    TransferState::Completed(Err(e)) => {
                        let err = e.clone();
                        self.async_transfers.remove(&transfer_id);
                        return Err(err);
                    }
                    TransferState::Pending => {
                        // 继续等待
                    }
                    TransferState::Submitted { .. } => {
                        // 继续等待
                    }
                    TransferState::Cancelled => {
                        self.async_transfers.remove(&transfer_id);
                        return Err(Error::Unknown("传输被取消".to_owned()));
                    }
                }
            } else {
                // 传输已被移除，可能是被中断处理了
                return Err(Error::Unknown("传输已被移除".to_owned()));
            }
            
            // 短暂等待
            for _ in 0..1000 { core::hint::spin_loop(); }
        }
    }

    pub fn supported_protocol(&mut self, port: usize) -> Option<SupportedProtocol> {
        info!("[XHCI] Find port {} protocol", port);

        if let Some(ext_list) = &mut self.ext_list {
            ext_list
                .into_iter()
                .filter_map(|one| {
                    if let Ok(ExtendedCapability::XhciSupportedProtocol(protcol)) = one {
                        return Some(protcol);
                    }
                    None
                })
                .find(|p| {
                    let head = p.header.read_volatile();
                    let port_range = head.compatible_port_offset() as usize
                        ..head.compatible_port_count() as usize;
                    port_range.contains(&port)
                })
        } else {
            None
        }
    }

    fn chip_hardware_reset(&mut self) -> &mut Self {
        info!("{TAG} Reset begin");
        info!("{TAG} Stop");

        self.regs.operational.usbcmd.update_volatile(|c| {
            c.clear_run_stop();
        });
        info!("{TAG} Until halt");
        while !self.regs.operational.usbsts.read_volatile().hc_halted() {}
        info!("{TAG} Halted");

        let mut o = &mut self.regs.operational;
        // info!("xhci stat: {:?}", o.usbsts.read_volatile());

        info!("{TAG} Wait for ready...");
        while o.usbsts.read_volatile().controller_not_ready() {}
        info!("{TAG} Ready");

        o.usbcmd.update_volatile(|f| {
            f.set_host_controller_reset();
        });

        while o.usbcmd.read_volatile().host_controller_reset() {}

        info!("{TAG} Reset HC");

        while self
            .regs
            .operational
            .usbcmd
            .read_volatile()
            .host_controller_reset()
            || self
                .regs
                .operational
                .usbsts
                .read_volatile()
                .controller_not_ready()
        {}

        info!("{TAG} XCHI reset ok");
        self
    }

    fn set_max_device_slots(&mut self) -> &mut Self {
        let max_slots = self.max_slots;
        info!("{TAG} Setting enabled slots to {}.", max_slots);
        self.regs.operational.config.update_volatile(|r| {
            r.set_max_device_slots_enabled(max_slots);
        });
        self
    }

    fn set_dcbaap(&mut self) -> &mut Self {
        let dcbaap = self.dev_ctx.dcbaap();
        info!("{TAG} Writing DCBAAP: {:X}", dcbaap);
        self.regs.operational.dcbaap.update_volatile(|r| {
            r.set(dcbaap as u64);
        });
        self
    }

    fn set_cmd_ring(&mut self) -> &mut Self {
        let crcr = self.cmd.register();
        let cycle = self.cmd.cycle;

        let regs = &mut self.regs;

        info!("{TAG} Writing CRCR: {:X}", crcr);
        regs.operational.crcr.update_volatile(|r| {
            r.set_command_ring_pointer(crcr);
            if cycle {
                r.set_ring_cycle_state();
            } else {
                r.clear_ring_cycle_state();
            }
        });
        
        // 验证写入 - CRCR是write-only寄存器，无法回读验证
        info!("{TAG} 命令环设置完成 - 地址=0x{:X}", crcr);

        self
    }

    fn start(&mut self) -> &mut Self {
        let regs = &mut self.regs;
        info!("{TAG} Start run");
        
        // 根据XHCI规范，必须先等待CNR=0
        let mut cnr_wait = 0;
        while regs.operational.usbsts.read_volatile().controller_not_ready() {
            cnr_wait += 1;
            if cnr_wait > 1000 {
                error!("{TAG} 等待CNR超时！");
                break;
            }
            for _ in 0..1000 { core::hint::spin_loop(); }
        }
        
        if cnr_wait > 0 {
            info!("{TAG} 等待了{}次循环直到CNR=0", cnr_wait);
        }
        
        // 设置Run/Stop位
        regs.operational.usbcmd.update_volatile(|r| {
            r.set_run_stop();
        });
        
        // 等待HCHalted清除
        let mut halt_wait = 0;
        while regs.operational.usbsts.read_volatile().hc_halted() {
            halt_wait += 1;
            if halt_wait > 1000 {
                error!("{TAG} 等待HCHalted清除超时！");
                break;
            }
            for _ in 0..1000 { core::hint::spin_loop(); }
        }

        let usbsts = regs.operational.usbsts.read_volatile();
        info!("{TAG} 控制器启动完成 - HCH={}, CNR={}", usbsts.hc_halted(), usbsts.controller_not_ready());
        
        if usbsts.hc_halted() || usbsts.controller_not_ready() {
            error!("{TAG} 控制器启动失败！");
        } else {
            info!("{TAG} Is running");
        }

        // 确保命令TRB已写入内存
        fence(Ordering::Release);
        debug!("{TAG} [启动] 敲响命令环Doorbell[0]");
        regs.doorbell.update_volatile_at(0, |r| {
            r.set_doorbell_stream_id(0);
            r.set_doorbell_target(0);
        });

        self
    }

    fn init_ir(&mut self) -> &mut Self {
        info!("{TAG} Disable interrupts");
        let regs = &mut self.regs;

        regs.operational.usbcmd.update_volatile(|r| {
            r.clear_interrupter_enable();
        });

        let mut ir0 = regs.interrupter_register_set.interrupter_mut(0);
        {
            info!("{TAG} Writing ERSTZ");
            ir0.erstsz.update_volatile(|r| r.set(1));

            let erdp = self.event.erdp();
            info!("{TAG} Writing ERDP: {:X}", erdp);

            ir0.erdp.update_volatile(|r| {
                r.set_event_ring_dequeue_pointer(erdp);
            });

            let erstba = self.event.erstba();
            info!("{TAG} Writing ERSTBA: {:X}", erstba);
            // Print the ERSTBA value for debugging
            info!("{TAG} ERSTBA (Event Ring Segment Table Base Address): {:#0X}", erstba);
            
            // 打印事件环段表信息
            let segment_base = self.event.segment_base_address();
            let segment_size = self.event.ring.len();
            info!("{TAG} Event Ring Segment[0]: Base=0x{:016X}, Size={} TRBs", segment_base, segment_size);

            ir0.erstba.update_volatile(|r| {
                r.set(erstba);
            });
            ir0.imod.update_volatile(|im| {
                im.set_interrupt_moderation_interval(0);
                im.set_interrupt_moderation_counter(0);
            });

            // 不在这里启用中断，延迟到所有初始化完成后
            info!("{TAG} Interrupter 0 配置完成（中断尚未启用）");
        }

        // };

        // self.setup_scratchpads(buf_count);

        self
    }

    fn get_speed(&self, port: usize) -> u8 {
        self.regs
            .port_register_set
            .read_volatile_at(port)
            .portsc
            .port_speed()
    }

    fn parse_default_max_packet_size_from_port(&self, port: usize) -> u16 {
        match self.get_speed(port) {
            1 | 3 => 64,
            2 => 8,
            4 => 512,
            v => unimplemented!("PSI: {}", v),
        }
    }

    fn reset_cic(&mut self) -> &mut Self {
        let regs = &mut self.regs;
        let cic = regs
            .capability
            .hccparams2
            .read_volatile()
            .configuration_information_capability();
        regs.operational.config.update_volatile(|r| {
            if cic {
                r.set_configuration_information_enable();
            } else {
                r.clear_configuration_information_enable();
            }
        });
        self
    }

    fn reset_ports(&mut self) -> &mut Self {
        let regs = &mut self.regs;
        let port_len = regs.port_register_set.len();

        for i in 0..port_len {
            info!("{TAG} Port {} start reset", i,);
            regs.port_register_set.update_volatile_at(i, |port| {
                port.portsc.set_0_port_enabled_disabled();
                port.portsc.set_port_reset();
            });

            while regs
                .port_register_set
                .read_volatile_at(i)
                .portsc
                .port_reset()
            {}

            info!("{TAG} Port {} reset ok", i);
        }
        self
    }

    fn setup_scratchpads(&mut self) -> &mut Self {
        let scratchpad_buf_arr = {
            let buf_count = {
                let count = self
                    .regs
                    .capability
                    .hcsparams2
                    .read_volatile()
                    .max_scratchpad_buffers();
                info!("{TAG} Scratch buf count: {}", count);
                count
            };
            if buf_count == 0 {
                // error!("buf count=0,is it a error?");
                // Ensure DCBAA[0] is 0 if it's not already (DMA::new should zero it, but for clarity).
                self.dev_ctx.dcbaa[0] = 0;
                info!("{TAG} No scratchpad buffers required/supported by hardware. DCBAA[0] is set to 0.");
                return self;
            }
            let scratchpad_buf_arr =
                ScratchpadBufferArray::new(buf_count, self.config.lock().os.clone());

            self.dev_ctx.dcbaa[0] = scratchpad_buf_arr.register() as u64;

            info!(
                "{TAG} Setting up {} scratchpads, at {:#0x}",
                buf_count,
                scratchpad_buf_arr.register()
            );
            scratchpad_buf_arr
        };

        self.scratchpad_buf_arr = Some(scratchpad_buf_arr);
        self
    }

    fn test_cmd(&mut self) -> &mut Self {
        //TODO:assert like this in runtime if build with info mode?
        info!("{TAG} Test command ring");
        for _ in 0..3 {
            let completion = self
                .post_cmd(command::Allowed::Noop(command::Noop::new()))
                .unwrap();
        }
        info!("{TAG} Command ring ok");
        self
    }

    /// 检查端点是否卡住，如果卡住则尝试恢复
    fn check_endpoint_progress(&mut self, slot_id: usize, dci: usize) -> bool {
        // 获取端点状态 - 不再使用TR Dequeue Pointer
        let ep_state = if let Some(output_ctx) = self.dev_ctx.device_out_context_list.get(slot_id) {
            let endpoint_handler = DeviceHandler::endpoint(&**output_ctx, dci);
            endpoint_handler.endpoint_state()
        } else {
            return false;
        };
        
        let key = (slot_id, dci);
        let now = self.read_mfindex() as u64;
        let mut needs_recovery = false;
        
        // 获取或创建进度跟踪
        let progress = self.endpoint_progress.entry(key).or_insert(EndpointProgress {
            submitted_trbs: 0,
            completed_events: 0,
            last_event_time: now,
            last_submit_time: now,
            no_progress_count: 0,
        });
        
        // 检查是否长时间没有收到事件
        let time_since_last_event = now.saturating_sub(progress.last_event_time);
        let time_since_last_submit = now.saturating_sub(progress.last_submit_time);
        
        // 计算未完成的TRB数量
        let pending_trbs = progress.submitted_trbs.saturating_sub(progress.completed_events);
        
        // 每次检查时打印详细状态（临时调试）
        if pending_trbs > 0 || log::log_enabled!(log::Level::Debug) {
            debug!("{TAG} [watchdog] 端点进度: slot={}, DCI={}, 状态={:?}", slot_id, dci, ep_state);
            debug!("{TAG} [watchdog]   - 统计: 提交TRB={}, 完成事件={}, 未完成={}", 
                  progress.submitted_trbs, progress.completed_events, pending_trbs);
            debug!("{TAG} [watchdog]   - 时间: 距上次事件={}ms, 距上次提交={}ms", 
                  time_since_last_event / 8, time_since_last_submit / 8);
            debug!("{TAG} [watchdog]   - MFINDEX: 当前={}, 上次事件={}, 上次提交={}", 
                  now, progress.last_event_time, progress.last_submit_time);
        }
        
        // 如果有未完成的TRB且超过3秒没有事件
        if pending_trbs > 0 && time_since_last_event > 24000 {
            progress.no_progress_count += 1;
            
            warn!("{TAG} [watchdog] ⚠️ 检测到可能的端点停滞: slot={}, DCI={}", slot_id, dci);
            warn!("{TAG} [watchdog]   - 提交TRB数: {}, 完成事件数: {}, 未完成: {}", 
                  progress.submitted_trbs, progress.completed_events, pending_trbs);
            warn!("{TAG} [watchdog]   - 距上次事件: {}ms (MFINDEX: {} -> {})", 
                  time_since_last_event / 8, progress.last_event_time, now);
            warn!("{TAG} [watchdog]   - 端点状态: {:?}, 无进展次数: {}", 
                  ep_state, progress.no_progress_count);
            
            // 只有在端点确实不在Running状态且多次检测到无进展时才恢复
            if ep_state != EndpointState::Running && progress.no_progress_count >= 2 {
                error!("{TAG} [watchdog] 🔴 端点确实卡死，需要恢复");
                needs_recovery = true;
            }
        } else if pending_trbs == 0 && progress.no_progress_count > 0 {
            // 所有TRB都完成了，重置计数器
            info!("{TAG} [watchdog] ✅ 端点恢复正常: slot={}, DCI={}, 所有TRB已完成", slot_id, dci);
            progress.no_progress_count = 0;
        } else if time_since_last_event < 1000 {
            // 最近125ms内收到过事件，说明端点活跃
            if progress.no_progress_count > 0 {
                debug!("{TAG} [watchdog] 端点活跃，清除无进展计数");
                progress.no_progress_count = 0;
            }
        }
        
        // 如果需要恢复
        if needs_recovery {
            error!("{TAG} [watchdog] 开始恢复卡住的端点...");
            match self.recover_stuck_endpoint(slot_id, dci) {
                Ok(_) => {
                    info!("{TAG} [watchdog] ✅ 端点恢复成功");
                    // 重置跟踪信息
                    if let Some(progress) = self.endpoint_progress.get_mut(&key) {
                        // 重置所有计数器，因为端点已经被重置
                        progress.submitted_trbs = 0;
                        progress.completed_events = 0;
                        progress.no_progress_count = 0;
                        progress.last_event_time = now;
                        progress.last_submit_time = now;
                    }
                    true
                }
                Err(e) => {
                    error!("{TAG} [watchdog] ❌ 端点恢复失败: {:?}", e);
                    false
                }
            }
        } else {
            false
        }
    }
    
    /// 打印所有端点的进度摘要
    fn print_endpoint_progress_summary(&self) {
        let now = self.read_mfindex() as u64;
        info!("{TAG} [=== 端点进度摘要 MFINDEX={} ===]", now);
        
        for ((slot_id, dci), progress) in &self.endpoint_progress {
            let pending = progress.submitted_trbs.saturating_sub(progress.completed_events);
            let time_since_event = now.saturating_sub(progress.last_event_time);
            
            if pending > 0 || progress.no_progress_count > 0 {
                info!("{TAG}   Slot {} DCI {}: 提交={}, 完成={}, 待完成={}, 距上次事件={}ms, 无进展次数={}",
                     slot_id, dci, 
                     progress.submitted_trbs, progress.completed_events, pending,
                     time_since_event / 8, progress.no_progress_count);
            }
        }
        info!("{TAG} [=== 端点进度摘要结束 ===]");
    }

    /// 恢复卡住的端点
    fn recover_stuck_endpoint(&mut self, slot_id: usize, dci: usize) -> crate::err::Result<()> {
        error!("{TAG} [recover_stuck_endpoint] 🔧 开始恢复卡住的端点: slot={}, dci={}", slot_id, dci);
        
        // 对于TRB Error，不需要重置端点，只需要设置新的Dequeue Pointer
        // 根据XHCI规范，ResetEndpoint只用于Halted状态的端点
        
        // 1. 停止端点
        let mut stop_ep = command::StopEndpoint::new();
        stop_ep.set_slot_id(slot_id as u8)
            .set_endpoint_id(dci as u8);
        
        match self.post_cmd(command::Allowed::StopEndpoint(stop_ep)) {
            Ok(completion) => {
                if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                    info!("{TAG} [recover_stuck_endpoint] ✅ 停止端点成功");
                } else {
                    warn!("{TAG} [recover_stuck_endpoint] ⚠️ 停止端点返回: {:?}", completion.completion_code());
                }
            }
            Err(e) => {
                error!("{TAG} [recover_stuck_endpoint] ❌ 停止端点失败: {:?}", e);
                return Err(e);
            }
        }
        
        // 2. 读取当前的Output Context来获取硬件状态
        let (current_dequeue_ptr, current_dcs) = if let Some(output_ctx) = self.dev_ctx.device_out_context_list.get(slot_id) {
            let endpoint_handler = DeviceHandler::endpoint(&**output_ctx, dci);
            let dequeue_ptr = endpoint_handler.tr_dequeue_pointer();
            let dcs = endpoint_handler.dequeue_cycle_state();
            info!("{TAG} [recover_stuck_endpoint] 当前硬件状态: TR Dequeue=0x{:X}, DCS={}", dequeue_ptr, dcs);
            (dequeue_ptr, dcs)
        } else {
            error!("{TAG} [recover_stuck_endpoint] 无法获取Output Context");
            return Err(crate::err::Error::Unknown("无法获取Output Context".to_string()));
        };
        
        // 3. 获取ring并同步状态
        let ring = self.ep_ring_mut(slot_id, dci);
        let ring_base = ring.register();
        
        // 计算当前硬件指向的TRB索引
        let current_trb_index = if current_dequeue_ptr >= ring_base {
            ((current_dequeue_ptr - ring_base) / 16) as usize
        } else {
            0
        };
        
        info!("{TAG} [recover_stuck_endpoint] 环状态: base=0x{:X}, 当前TRB索引={}, ring.i={}, ring.cycle={}", 
              ring_base, current_trb_index, ring.i, ring.cycle);
        
        // 计算新的dequeue位置：跳过当前错误的TRB
        let mut new_index = current_trb_index + 1;
        let mut new_cycle = current_dcs;
        
        // 处理环绕的情况
        if ring.has_link() && new_index >= ring.len() - 1 {
            // 有Link TRB，环绕到开头
            new_index = 0;
            new_cycle = !new_cycle; // 翻转cycle bit
            info!("{TAG} [recover_stuck_endpoint] 环绕: new_index=0, new_cycle={}", new_cycle);
        } else if !ring.has_link() && new_index >= ring.len() {
            // 无Link TRB，直接环绕
            new_index = 0;
            new_cycle = !new_cycle;
            info!("{TAG} [recover_stuck_endpoint] 环绕(无link): new_index=0, new_cycle={}", new_cycle);
        }
        
        // XHCI规范要求TR Dequeue Pointer必须是64字节对齐的
        // 找到下一个满足64字节对齐要求的TRB位置
        let mut aligned_index = new_index;
        loop {
            let addr = ring_base + (aligned_index * 16) as u64;
            if addr % 64 == 0 {
                break;
            }
            aligned_index += 1;
            if ring.has_link() && aligned_index >= ring.len() - 1 {
                aligned_index = 0;
                new_cycle = !new_cycle;
            } else if !ring.has_link() && aligned_index >= ring.len() {
                aligned_index = 0;
                new_cycle = !new_cycle;
            }
            // 防止无限循环
            if aligned_index == current_trb_index {
                error!("{TAG} [recover_stuck_endpoint] 无法找到64字节对齐的TRB位置！");
                break;
            }
        }
        new_index = aligned_index;
        info!("{TAG} [recover_stuck_endpoint] 调整到64字节对齐的索引: {}", new_index);
        
        // 清理从当前位置到新位置之间的所有TRB
        // 将它们设置为No-Op TRB，避免硬件处理旧数据
        if current_trb_index != new_index {
            info!("{TAG} [recover_stuck_endpoint] 清理TRB: 从索引{}到{}", current_trb_index, new_index);
            let mut i = current_trb_index;
            let mut clear_cycle = current_dcs;
            
            while i != new_index {
                // 创建No-Op TRB - 使用空TRB来跳过错误TRB
                let mut noop_data = [0u32; 4];
                // TRB Type = 8 (No-Op)
                // 重要：保持当前位置的cycle bit，这样硬件会处理这些No-Op
                noop_data[3] = (8 << 10) | if clear_cycle { 1 } else { 0 };
                ring.trbs[i].copy_from_slice(&noop_data);
                
                i += 1;
                if ring.has_link() && i >= ring.len() - 1 {
                    i = 0;
                    clear_cycle = !clear_cycle;
                } else if !ring.has_link() && i >= ring.len() {
                    i = 0;
                    clear_cycle = !clear_cycle;
                }
            }
        }
        
        // 更新ring的软件状态
        ring.i = new_index;
        ring.cycle = new_cycle;
        
        // 计算新的dequeue pointer地址
        let new_addr = ring_base + (new_index * 16) as u64;
        
        info!("{TAG} [recover_stuck_endpoint] 设置新的TR Dequeue Pointer: addr=0x{:08X}, DCS={}", 
              new_addr, new_cycle);
        
        // 确保内存屏障，让硬件看到清理后的TRB
        fence(Ordering::SeqCst);
        
        let mut set_tr_deq = command::SetTrDequeuePointer::new();
        set_tr_deq.set_slot_id(slot_id as u8)
            .set_endpoint_id(dci as u8)
            .set_new_tr_dequeue_pointer(new_addr);
            
        if new_cycle {
            set_tr_deq.set_dequeue_cycle_state();
        } else {
            set_tr_deq.clear_dequeue_cycle_state();
        }
        
        match self.post_cmd(command::Allowed::SetTrDequeuePointer(set_tr_deq)) {
            Ok(completion) => {
                if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                    info!("{TAG} [recover_stuck_endpoint] ✅ 设置TR Dequeue Pointer成功");
                } else {
                    warn!("{TAG} [recover_stuck_endpoint] ⚠️ 设置TR Dequeue Pointer返回: {:?}", completion.completion_code());
                }
            }
            Err(e) => {
                error!("{TAG} [recover_stuck_endpoint] ❌ 设置TR Dequeue Pointer失败: {:?}", e);
                return Err(e);
            }
        }
        
        // 4. 配置端点（重新激活）
        // 创建输入控制上下文
        let mut input_control_context = [0u32; 8];
        // 设置add context flag以添加指定的DCI
        input_control_context[1] = 1u32 << dci;
        // Already set above: input_control_context[1] = 1u32 << dci;
        
        // 在配置端点后，重新敲门铃让硬件开始处理
        info!("{TAG} [recover_stuck_endpoint] 配置端点后敲门铃，slot={}, dci={}", slot_id, dci);
        self.regs.doorbell.update_volatile_at(slot_id, |r| { 
            r.set_doorbell_stream_id(0);
            r.set_doorbell_target(dci as _); 
        });
        
        // 获取输入上下文
        if let Some(mut input_ctx) = self.dev_ctx.device_input_context_list.get_mut(slot_id) {
            // 更新输入上下文中的control context
            let input_ctx_ref = &mut **input_ctx;
            let control_ctx = InputHandler::control_mut(input_ctx_ref);
            control_ctx.set_add_context_flag(dci);
            
            // 更新端点上下文 - 设置新的dequeue pointer和cycle state
            let endpoint_ctx = InputHandler::device_mut(input_ctx_ref).endpoint_mut(dci);
            endpoint_ctx.set_tr_dequeue_pointer(new_addr);
            if new_cycle {
                endpoint_ctx.set_dequeue_cycle_state();
            } else {
                endpoint_ctx.clear_dequeue_cycle_state();
            }
            // 保持其他端点设置不变
            endpoint_ctx.set_endpoint_state(EndpointState::Running);
            
            info!("{TAG} [recover_stuck_endpoint] 更新输入上下文: TR Dequeue=0x{:X}, DCS={}", new_addr, new_cycle);
            
            let input_ctx_addr = input_ctx.addr() as u64;
            
            let mut config_ep = command::ConfigureEndpoint::new();
            config_ep.set_slot_id(slot_id as u8)
                .set_input_context_pointer(input_ctx_addr);
                
            match self.post_cmd(command::Allowed::ConfigureEndpoint(config_ep)) {
                Ok(completion) => {
                    if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                        info!("{TAG} [recover_stuck_endpoint] ✅ 配置端点成功，端点已恢复");
                        Ok(())
                    } else {
                        error!("{TAG} [recover_stuck_endpoint] ❌ 配置端点失败: {:?}", completion.completion_code());
                        Err(crate::err::Error::CMD(completion.completion_code().unwrap_or(CompletionCode::UndefinedError)))
                    }
                }
                Err(e) => {
                    error!("{TAG} [recover_stuck_endpoint] ❌ 配置端点命令失败: {:?}", e);
                    Err(e)
                }
            }
        } else {
            error!("{TAG} [recover_stuck_endpoint] ❌ 获取输入上下文失败: slot_id={}", slot_id);
            Err(crate::err::Error::InvalidSlot)
        }
    }

    /// 恢复Halted状态的端点（用于Stall和Babble错误）
    fn recover_halted_endpoint(&mut self, slot_id: usize, dci: usize) -> crate::err::Result<()> {
        error!("{TAG} [recover_halted_endpoint] 🔧 开始恢复Halted端点: slot={}, dci={}", slot_id, dci);
        
        // 对于Halted端点，需要Reset Endpoint命令
        let mut reset_ep = command::ResetEndpoint::new();
        reset_ep.set_slot_id(slot_id as u8)
            .set_endpoint_id(dci as u8);
            
        match self.post_cmd(command::Allowed::ResetEndpoint(reset_ep)) {
            Ok(completion) => {
                if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                    info!("{TAG} [recover_halted_endpoint] ✅ 重置端点成功");
                } else {
                    warn!("{TAG} [recover_halted_endpoint] ⚠️ 重置端点返回: {:?}", completion.completion_code());
                }
            }
            Err(e) => {
                error!("{TAG} [recover_halted_endpoint] ❌ 重置端点失败: {:?}", e);
                return Err(e);
            }
        }
        
        // 设置新的TR Dequeue Pointer
        let ring = self.ep_ring_mut(slot_id, dci);
        let new_addr = ring.register();
        let new_cycle = ring.cycle;
        ring.i = 0;
        
        info!("{TAG} [recover_halted_endpoint] 设置新的TR Dequeue Pointer: addr=0x{:08X}, DCS={}", 
              new_addr, new_cycle);
        
        let mut set_tr_deq = command::SetTrDequeuePointer::new();
        set_tr_deq.set_slot_id(slot_id as u8)
            .set_endpoint_id(dci as u8)
            .set_new_tr_dequeue_pointer(new_addr);
            
        if new_cycle {
            set_tr_deq.set_dequeue_cycle_state();
        } else {
            set_tr_deq.clear_dequeue_cycle_state();
        }
        
        match self.post_cmd(command::Allowed::SetTrDequeuePointer(set_tr_deq)) {
            Ok(completion) => {
                if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                    info!("{TAG} [recover_halted_endpoint] ✅ 设置TR Dequeue Pointer成功");
                } else {
                    warn!("{TAG} [recover_halted_endpoint] ⚠️ 设置TR Dequeue Pointer返回: {:?}", completion.completion_code());
                }
            }
            Err(e) => {
                error!("{TAG} [recover_halted_endpoint] ❌ 设置TR Dequeue Pointer失败: {:?}", e);
                return Err(e);
            }
        }
        
        // 重新敲门铃
        info!("{TAG} [recover_halted_endpoint] 恢复后敲门铃，slot={}, dci={}", slot_id, dci);
        self.regs.doorbell.update_volatile_at(slot_id, |r| { 
            r.set_doorbell_stream_id(0);
            r.set_doorbell_target(dci as _); 
        });
        
        Ok(())
    }

    // 异步版本的post_cmd
    fn post_cmd_async(&mut self, mut trb: command::Allowed) -> CommandFuture<O> {
        // 在发送命令前检查控制器状态
        let usbsts = self.regs.operational.usbsts.read_volatile();
        let usbcmd = self.regs.operational.usbcmd.read_volatile();
        trace!("{TAG} 发送命令前状态 - RS={}, HCH={}, CNR={}", 
              usbcmd.run_stop(), usbsts.hc_halted(), usbsts.controller_not_ready());
        
        if !usbcmd.run_stop() || usbsts.hc_halted() {
            error!("{TAG} 控制器未运行，无法发送命令！");
        }
        
        let addr = self.cmd.enque_command(trb);
        
        trace!("{TAG} 发送命令TRB到地址: 0x{:X}", addr);

        // 创建异步命令记录
        let command_id = CommandId { trb_pointer: addr as u64 };
        let async_command = AsyncCommand {
            id: command_id,
            trb_pointer: addr as u64,
            state: CommandState::Submitted {
                submitted_at: current_time(),
            },
            waker: None,
        };
        
        // 存储异步命令
        self.async_commands.insert(command_id, async_command);

        // 确保命令TRB已写入内存 - 必须在doorbell之前！
        fence(Ordering::Release);
        self.regs.doorbell.update_volatile_at(0, |r| {
            r.set_doorbell_stream_id(0);
            r.set_doorbell_target(0);
        });
        

        // 返回Future
        CommandFuture {
            id: command_id,
            xhci: self.self_arc.as_ref().unwrap().clone(),
        }
    }

    fn post_cmd(&mut self, trb: command::Allowed) -> crate::err::Result<CommandCompletion> {
        // 直接使用异步版本实现同步版本
        let future = self.post_cmd_async(trb);
        self.block_on_future(future)
    }

    #[deprecated(note = "使用post_cmd_async代替")]
    #[allow(dead_code)]
    fn event_busy_wait_cmd(&mut self, addr: u64) -> crate::err::Result<CommandCompletion> {
        error!("{TAG} event_busy_wait_cmd不应该被调用，使用post_cmd_async代替！");
        trace!("Wait result for CMD TRB @{:#X}", addr);
        
        // 首先给中断一个机会处理事件
        let usbsts = self.regs.operational.usbsts.read_volatile();
        if usbsts.event_interrupt() {
            trace!("{TAG} 检测到挂起的中断，等待中断处理...");
            // 等待一小段时间让中断触发
            for _ in 0..10000 { core::hint::spin_loop(); }
        }
        
        let start_time_ms = (current_time() / 1000) as u64;
        loop {
            if ((current_time() / 1000) as u64).saturating_sub(start_time_ms) > ISOC_TRANSFER_TIMEOUT_MS { // Reusing ISOC timeout for now
                error!("{TAG} Timeout waiting for CommandCompletion event for TRB @{:#X}. EventRing: index={}, cycle={}", 
                    addr, self.event.ring.i, self.event.ring.cycle);
                // 读取并打印HCSTS
                let hcsts = self.regs.operational.usbsts.read_volatile();
                error!("{TAG} Timeout HCSTS: HCHalted={}, HSE={}, EINT={}, PCD={}, CNR={}", 
                    hcsts.hc_halted(), hcsts.host_system_error(), hcsts.event_interrupt(), hcsts.port_change_detect(), hcsts.controller_not_ready());
                
                // 添加更多诊断信息
                let usbcmd = self.regs.operational.usbcmd.read_volatile();
                error!("{TAG} USBCMD: RS={}, INTE={}", usbcmd.run_stop(), usbcmd.interrupter_enable());
                
                // 检查命令环状态
                let crcr = self.regs.operational.crcr.read_volatile();
                error!("{TAG} CRCR: CRR={}", crcr.command_ring_running());
                
                // 检查中断状态
                let iman = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
                let erdp = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
                error!("{TAG} IMAN: IE={}, IP={}", iman.interrupt_enable(), iman.interrupt_pending());
                error!("{TAG} ERDP: 0x{:016X}, EHB={}", erdp.event_ring_dequeue_pointer(), erdp.event_handler_busy());
                
                return Err(Error::Timeout);
            }

            let usbsts = self.regs.operational.usbsts.read_volatile();
            if usbsts.event_interrupt() {
                for _ in 0..100 { core::hint::spin_loop(); }
            }
            
            if let Some((event_data, _cycle)) = self.event.next() { // next() logically dequeues, advances sw pointer
                self.update_erdp(); // Update HW ERDP immediately after software dequeue

                match event_data {
                    event::Allowed::CommandCompletion(c) => {
                        let cmd_trb_ptr = c.command_trb_pointer();
                        let completion_code_result = c.completion_code();
                        
                        info!(
                            "{TAG} [CMD Wait] Polled CommandCompletion: CmdTRBPtr=0x{:X}, ExpectedTRBPtr=0x{:X}, Code={:?}, SlotID={}, CycleBit={}", 
                            cmd_trb_ptr, addr, completion_code_result, c.slot_id(), c.cycle_bit()
                        );

                        if cmd_trb_ptr == addr {
                            match completion_code_result {
                                Ok(CompletionCode::Success) => {
                                    trace!("{TAG} [CMD Wait] Found MATCHING CommandCompletion for TRB @{:#X}: Success", addr);

                                    let completed_cmd_phys_ptr = c.command_trb_pointer();
                                    let cmd_trb_data_ptr = completed_cmd_phys_ptr as *const [u32; 4];
                                    let raw_cmd_trb_data: [u32; 4] = unsafe { cmd_trb_data_ptr.read_volatile() };
                                    let cmd_trb_type_field = (raw_cmd_trb_data[3] >> 10) & 0x3F;

                                    if cmd_trb_type_field == xhci::ring::trb::Type::ConfigureEndpoint as u32 {
                                        trace!("{TAG} [CMD Wait] ConfigureEndpoint command @0x{:X} completed. Now polling EHB state.", completed_cmd_phys_ptr);
                                        let mut ehb_poll_count_after_config_ep = 0;
                                        const CONFIGURE_EP_SUCCESS_EHB_POLL_LIMIT: usize = 70000; // Increased limit for observation
                                        let mut ehb_busy_after_config_ep = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile().event_handler_busy();
                                        
                                        while ehb_busy_after_config_ep && ehb_poll_count_after_config_ep < CONFIGURE_EP_SUCCESS_EHB_POLL_LIMIT {
                                            core::hint::spin_loop();
                                            ehb_poll_count_after_config_ep += 1;
                                            if ehb_poll_count_after_config_ep % 10000 == 0 { // Log progress
                                                trace!("{TAG} [CMD Wait] EHB still busy after {} polls for ConfigureEndpoint (TRB @0x{:X}) success...", ehb_poll_count_after_config_ep, completed_cmd_phys_ptr);
                                            }
                                            ehb_busy_after_config_ep = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile().event_handler_busy();
                                        }

                                        if ehb_busy_after_config_ep {
                                            error!("{TAG} [CMD Wait] EHB remained TRUE (after {} polls) after ConfigureEndpoint success (TRB @0x{:X}). This is a strong indicator of an issue.", 
                                                   ehb_poll_count_after_config_ep, completed_cmd_phys_ptr);
                                        } else {
                                            trace!("{TAG} [CMD Wait] EHB cleared for ConfigureEndpoint (TRB @0x{:X}) after {} polls.", 
                                                   completed_cmd_phys_ptr, ehb_poll_count_after_config_ep);
                                        }
                                    } else if cmd_trb_type_field == xhci::ring::trb::Type::SetTrDequeuePointer as u32 {
                                        let target_slot_id_from_event = c.slot_id(); 
                                        let target_dci_from_cmd = (raw_cmd_trb_data[0] >> 16) & 0x1F; 
                                        trace!(
                                            "{TAG} [CMD Wait Post-Check] Completed CMD TRB @0x{:X} was Set TR Dequeue Pointer. Target Slot: {}, Target DCI: {}. Raw CMD: [{:08X}, {:08X}, {:08X}, {:08X}]",
                                            completed_cmd_phys_ptr,
                                            target_slot_id_from_event,
                                            target_dci_from_cmd,
                                            raw_cmd_trb_data[0], raw_cmd_trb_data[1], raw_cmd_trb_data[2], raw_cmd_trb_data[3]
                                        );
                                        trace!("{TAG} [CMD Wait Post-Check] Set TR Dequeue Pointer for DCI {} (Slot {}) completed. Dumping context NOW.", target_dci_from_cmd, target_slot_id_from_event);
                                        self.trace_dump_context(target_slot_id_from_event as usize);
                                    }
                                    return Ok(c);
                                }
                                Ok(other_code) => {
                                    error!("{TAG} [CMD Wait] Found MATCHING CommandCompletion for TRB @{:#X} but got error code: {:?}", addr, other_code);
                                    return Err(Error::CMD(other_code));
                                }
                                Err(e) => {
                                    error!("{TAG} [CMD Wait] Found MATCHING CommandCompletion for TRB @{:#X} but failed to parse completion code: {}", addr, e);
                                    return Err(Error::CMD(CompletionCode::Invalid)); // Or a more specific error
                                }
                            }
                        } else {
                            // This CommandCompletion is not for the command we are currently waiting for.
                            // Log it and continue polling.
                            warn!(
                                "{TAG} [CMD Wait] Polled MISMATCHED CommandCompletion: CmdTRBPtr=0x{:X} (expected 0x{:X}). Code={:?}. Ignoring and continuing poll.", 
                                cmd_trb_ptr, addr, completion_code_result
                            );
                            // ERDP already updated, continue loop.
                            continue;
                        }
                    }
                    event::Allowed::PortStatusChange(psc) => {
                        trace!("{TAG} [CMD Wait] Polled PortStatusChange event: PortID={}. Continuing poll for CMD TRB @{:#X}", psc.port_id(), addr);
                        // ERDP already updated, continue loop.
                        continue;
                    }
                    event::Allowed::HostController(hc_event) => {
                        trace!("{TAG} [CMD Wait] Polled HostController event: {:?}. Continuing poll for CMD TRB @{:#X}", hc_event, addr);
                        if hc_event.completion_code().map_or(false, |c| c == CompletionCode::EventRingFullError) {
                            error!("{TAG} 事件环已满错误! (Polled during command wait)");
                            return Err(Error::CMD(CompletionCode::EventRingFullError));
                        }
                        // ERDP already updated, continue loop.
                        continue;
                    }
                    other_event => {
                        // Log other unexpected events but continue polling for the specific command completion.
                        warn!("{TAG} [CMD Wait] Polled UNEXPECTED event type ({:?}) while waiting for CommandCompletion for TRB @{:#X}. Ignoring and continuing poll.", other_event, addr);
                        // ERDP already updated, continue loop.
                        continue;
                    }
                }
            }
            // Short delay or yield to prevent busy-looping if event queue is empty
            // This might need platform-specific sleep/yield if current_time() is not advancing rapidly
            for _ in 0..1000 { core::hint::spin_loop(); } // Simple spin for now
        }
    }

    fn trace_dump_context(&self, slot_id: usize) {
        trace!("{TAG} trace_dump_context called for slot_id: {}", slot_id);

        let dev_ctx_maybe = self.dev_ctx.device_out_context_list.get(slot_id);
        if dev_ctx_maybe.is_none() {
            warn!("{TAG} trace_dump_context: Slot {} not found in device_out_context_list", slot_id);
            return;
        }
        let dev = dev_ctx_maybe.unwrap(); 

        let slot_handler = DeviceHandler::slot(&**dev);
        let num_valid_context_entries = slot_handler.context_entries();

        info!(
            "{TAG} slot {} OutputContext: SlotState={:?}, ContextEntries={}", 
            slot_id,
            slot_handler.slot_state(),
            num_valid_context_entries
        );

        // DCI 0 is the Slot Context
        if num_valid_context_entries > 0 {
            info!(
                "{TAG}   OutputContext DCI 0 (SlotContext): State={:?} (Details: Speed={}, RHPort={}, MaxExitLat={}, Route=0x{:X})",
                slot_handler.slot_state(),
                slot_handler.speed(),
                slot_handler.root_hub_port_number(),
                slot_handler.max_exit_latency(),
                slot_handler.route_string()
            );
        }

        // Endpoint Contexts are DCI 1 to (num_valid_context_entries - 1)
        for dci in 1..(num_valid_context_entries as usize) { 
            // Now dci is guaranteed to be >= 1
            let ep_ctx = dev.endpoint(dci); 
            let ep_state = ep_ctx.endpoint_state();

            if ep_state != EndpointState::Disabled { 
                info!(
                    "{TAG}   OutputContext DCI {}: State={:?}, Type={:?}, TR_DeqPtr=0x{:X}, MaxPktSize={}, AvgTRBLen={}, MaxBurstSize={}, ErrCnt={}",
                    dci,
                    ep_state,
                    ep_ctx.endpoint_type(),
                    ep_ctx.tr_dequeue_pointer(),
                    ep_ctx.max_packet_size(),
                    ep_ctx.average_trb_length(),
                    ep_ctx.max_burst_size(),
                    ep_ctx.error_count()
                );
            }
        }
    }

    fn append_port_to_route_string(route_string: u32, port_id: usize) -> u32 {
        let mut route_string = route_string;
        for tier in 0..5 {
            if route_string & (0x0f << (tier * 4)) == 0 {
                if tier < 5 {
                    route_string |= (port_id as u32) << (tier * 4);
                    return route_string;
                }
            }
        }

        route_string
    }

    // 添加计算端点DCI的辅助函数
    fn compute_endpoint_dci_number(&self, endpoint_id: u8) -> u8 {
        if endpoint_id == 0 {
            return 1;
        }
        // 正确计算DCI：
        // 对于非控制端点，DCI = 2*EP + Dir
        // 其中EP是端点号(0-15)，Dir是方向(0=OUT, 1=IN)
        let endpoint_number = endpoint_id & 0x0F;  // 低4位是端点号
        let direction = (endpoint_id & 0x80) >> 7; // 最高位是方向(1=IN, 0=OUT)
        
        if endpoint_number == 0 {
            // 对于端点0，DCI总是1
            1
        } else {
            // 计算公式：DCI = 2*EP + Dir
            (2 * endpoint_number + direction) as u8
        }
    }
    
    /// 从DCI计算端点地址（用于查找pending URB）
    fn compute_endpoint_number_from_dci(&self, dci: u8) -> u8 {
        if dci == 1 {
            // DCI 1 是控制端点
            0
        } else if dci % 2 == 0 {
            // 偶数DCI是OUT端点
            // DCI = 2*EP + 0，所以 EP = DCI/2
            (dci / 2) as u8
        } else {
            // 奇数DCI是IN端点
            // DCI = 2*EP + 1，所以 EP = (DCI-1)/2
            // IN端点地址的最高位是1
            0x80 | ((dci - 1) / 2) as u8
        }
    }

    pub fn ep_ring_mut(&mut self, device_slot_id: usize, dci: usize) -> &mut Ring<O> {
        // 直接使用传入的DCI
        info!("fetch transfer ring at slot{}-dci{}", device_slot_id, dci);

        // 使用新的get_or_create_ring函数获取或创建传输环
        self.dev_ctx.get_or_create_ring(device_slot_id, dci)
    }
    
    // 清除EHB位的辅助函数
    fn clear_ehb(&mut self) {
        let erdp_reg = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        if erdp_reg.event_handler_busy() {
            warn!("{TAG} 检测到EHB=1，尝试清除");
            
            // 方法1：写入当前ERDP值来清除EHB
            let current_erdp = erdp_reg.event_ring_dequeue_pointer();
            self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|r| {
                r.set_event_ring_dequeue_pointer(current_erdp);
            });
            
            // 等待EHB清除
            let mut retry = 0;
            while retry < 1000 {
                let check = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
                if !check.event_handler_busy() {
                    trace!("{TAG} EHB已成功清除");
                    return;
                }
                core::hint::spin_loop();
                retry += 1;
            }
            
            // 方法2：如果方法1失败，尝试更新到事件环的当前位置
            warn!("{TAG} 方法1失败，尝试方法2");
            let current_dequeue = self.event.erdp();
            self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|r| {
                r.set_event_ring_dequeue_pointer(current_dequeue);
            });
            
            // 再次检查
            let final_check = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
            if final_check.event_handler_busy() {
                error!("{TAG} 无法清除EHB位，事件处理可能受影响");
            } else {
                trace!("{TAG} 方法2成功清除EHB");
            }
        }
    }

    // Linux方式更新ERDP清除EHB位的函数
    fn update_erdp(&mut self) {
        // Linux: xhci_update_erst_dequeue()
        let ring_base_addr = self.event.ring.trbs.addr();
        let current_idx = self.event.ring.i;
        let item_size = mem::size_of::<ring::TrbData>();
        let deq = ring_base_addr + (current_idx * item_size);
        
        // Linux总是计算相对于当前段的ERDP
        let seg_base = self.event.ring.register() as usize;
        let temp_erdp = deq - seg_base;
        
        // 确保16字节对齐
        let new_erdp = (deq & !0xF) as u64;
        
        trace!("{TAG} [Linux方式] 更新ERDP: deq=0x{:X}, seg_base=0x{:X}, temp=0x{:X}, new_erdp=0x{:X}",
               deq, seg_base, temp_erdp, new_erdp);
        
        // Linux: 读取当前ERDP值
        let erdp_reg = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        let old_erdp = erdp_reg.event_ring_dequeue_pointer();
        
        // Linux不检查EHB位，直接写入新的ERDP值
        // 根据XHCI规范，写入ERDP会自动清除EHB
        
        // Linux: 保留DESI字段 (bits 2:0)
        let desi = old_erdp & 0x7;
        let erdp_val = new_erdp | desi;
        
        trace!("{TAG} [Linux方式] 写入ERDP: 0x{:016X} (DESI={})", erdp_val, desi);
        
        // Linux方式：直接写入，不做额外检查
        self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|f| {
            f.set_event_ring_dequeue_pointer(erdp_val);
        });
        
        // Linux使用的内存屏障
        fence(Ordering::SeqCst);
        
        // Linux不会再次检查EHB或做恢复操作
        // 如果硬件有问题，会在下次中断时处理
    }

    // 重置中断器0
    fn reset_interrupter_0(&mut self) {
        warn!("{TAG} 重置中断器0");
        
        // 1. 禁用中断
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.clear_interrupt_enable();
        });
        
        // 2. 清除所有挂起的中断
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.set_0_interrupt_pending(); // W1C
        });
        
        // 3. 重新设置ERDP到当前位置
        let current_erdp = self.event.erdp();
        self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|r| {
            r.set_event_ring_dequeue_pointer(current_erdp);
        });
        
        // 4. 等待一段时间
        for _ in 0..10000 { core::hint::spin_loop(); }
        
        // 5. 重新启用中断
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.set_interrupt_enable();
        });
        
        // 6. 检查EHB状态
        let ehb_after = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile().event_handler_busy();
        if !ehb_after {
            trace!("{TAG} 重置中断器后成功清除EHB");
        } else {
            error!("{TAG} 重置中断器后EHB仍为1");
        }
    }
    
    fn debug_controller_status(&self) {
        // 读取USB状态寄存器
        let usbsts = self.regs.operational.usbsts.read();
        let usbcmd = self.regs.operational.usbcmd.read();
        
        // 检查各种状态位 - 修复bool比较错误
        let hc_halted = usbsts.hc_halted();
        let host_system_error = usbsts.host_system_error();
        let event_interrupt_status = usbsts.event_interrupt(); // Renamed to avoid conflict with usbcmd bit
        let port_change_detect = usbsts.port_change_detect();
        let controller_not_ready = usbsts.controller_not_ready();
        let run_stop = usbcmd.run_stop();
        let interrupter_enable = usbcmd.interrupter_enable();

        trace!("{TAG} USB控制器状态: HCSTS[HCH={}, HSE={}, EINT={}, PCD={}, CNR={}] USBCMD[RS={}, INTE={}]",
             hc_halted, host_system_error, event_interrupt_status, port_change_detect, controller_not_ready,
             run_stop, interrupter_enable);
    }

    fn debug_interrupt_status(&self) {
        // 读取中断相关寄存器
        let usbsts = self.regs.operational.usbsts.read();
        let usbcmd = self.regs.operational.usbcmd.read();
        let iman = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
        
        trace!("{TAG} 中断状态调试:");
        trace!("{TAG}   USBSTS.EINT={}", usbsts.event_interrupt());
        trace!("{TAG}   USBCMD.INTE={}", usbcmd.interrupter_enable());
        trace!("{TAG}   IMAN.IP={}, IE={}", iman.interrupt_pending(), iman.interrupt_enable());
        trace!("{TAG}   当前中断使能状态: {}", self.interrupt_enabled);
    }
    
    fn debug_event_ring_status(&self) {
        // 获取事件环信息
        let erdp_reg = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        let erdp = erdp_reg.event_ring_dequeue_pointer();
        let ehb = erdp_reg.event_handler_busy();
        let erstba = self.event.erstba();
        let erstsz = self.regs.interrupter_register_set.interrupter(0).erstsz.read_volatile();
        let iman_reg = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
        let iman_ip = iman_reg.interrupt_pending();
        let iman_ie = iman_reg.interrupt_enable();
        
        trace!("{TAG} 事件环状态: ERDP=0x{:x} (EHB={}), ERSTBA=0x{:x}, ERSTSZ={}, IMAN[IP={}, IE={}]", 
            erdp, ehb, erstba, erstsz.get(), iman_ip, iman_ie);
        
        // 读取事件环中的第一个TRB (pointed to by software dequeue pointer)
        // It's more informative to see what the hardware *would* write to if EHB were clear,
        // or what is at the current software dequeue pointer.
        let sw_dequeue_ptr = self.event.ring.register() + (self.event.ring.i * mem::size_of::<ring::TrbData>()) as u64;
        
        // Safely read memory if address is valid (basic check, real MMU check is harder here)
        if sw_dequeue_ptr != 0 {
            let current_trb_data = &self.event.ring.trbs[self.event.ring.i];
            trace!("{TAG} 当前SW事件TRB (idx {} @ 0x{:X}): [{:08x}, {:08x}, {:08x}, {:08x}] (Cycle={})",
                  self.event.ring.i, sw_dequeue_ptr,
                  current_trb_data[0], current_trb_data[1], current_trb_data[2], current_trb_data[3],
                  self.event.ring.cycle);
        } else {
            warn!("{TAG} SW Dequeue Pointer for event ring is 0, cannot read TRB.");
        }
    }

    fn setup_device(
        &mut self,
        device_slot_id: usize,
        configure: &TopologicalUSBDescriptorConfiguration,
    ) -> crate::err::Result<UCB<O>> {
        for func in configure.child.iter() {
            match func {
                TopologicalUSBDescriptorFunction::InterfaceAssociation(assoc) => {
                    // todo!("enumrate complex device!")

                    // let (interface0, attributes, endpoints) =
                    assoc
                            .1
                            .iter()
                            .map(|f| match f {
                                TopologicalUSBDescriptorFunction::InterfaceAssociation(_) => {
                                    panic!("anyone could help this guy????")
                                }
                                TopologicalUSBDescriptorFunction::Interface(interface) => interface,
                            })
                            .for_each(|interface| {
                                interface
                                    .iter()
                                    .for_each(|(interface0, extras, endpoints)| {
                                        {
                                            let input = self.dev_ctx.device_input_context_list
                                                [device_slot_id]
                                                .deref_mut();

                                            let entries = endpoints
                                                .iter()
                                                .filter_map(|endpoint| {
                                                    if let TopologicalUSBDescriptorEndpoint::Standard(ep) =
                                                        endpoint
                                                    {
                                                        Some(ep)
                                                    } else {
                                                        None
                                                    }
                                                })
                                                .map(|endpoint| endpoint.doorbell_value_aka_dci())
                                                .max()
                                                .unwrap_or(1)
                                                .max(input.device().slot().context_entries() as u32);

                                            input
                                                .device_mut()
                                                .slot_mut()
                                                .set_context_entries(entries as u8);
                                        }

                                        // info!("endpoints:{:#?}", endpoints);

                                        for item in endpoints {
                                            if let TopologicalUSBDescriptorEndpoint::Standard(ep) = item {
                                                let dci = ep.doorbell_value_aka_dci() as usize;
                                                let max_packet_size = ep.max_packet_size;
                                                // 获取ring信息
                                                let (ring_addr, ring_cycle) = {
                                                    let ring = self.ep_ring_mut(device_slot_id, dci);
                                                    (ring.register(), ring.cycle)
                                                };

                                                let input = self.dev_ctx.device_input_context_list
                                                    [device_slot_id]
                                                    .deref_mut();
                                                let control_mut = input.control_mut();
                                                info!("init ep {} {:?}", dci, ep.endpoint_type());
                                                control_mut.set_add_context_flag(dci);
                                                let ep_mut = input.device_mut().endpoint_mut(dci);
                                                let xhci_interval = if ep.interval > 0 { ep.interval - 1 } else { 0 }; // HS/SS: bInterval-1
                                                info!(
                                                    "Setting interval for DCI {} to {} (from bInterval {})",
                                                    dci, xhci_interval, ep.interval
                                                );
                                                ep_mut.set_interval(xhci_interval);
                                                ep_mut.set_endpoint_type(ep.endpoint_type());
                                                ep_mut.set_tr_dequeue_pointer(ring_addr);
                                                ep_mut.set_max_packet_size(max_packet_size);
                                                ep_mut.set_error_count(3);
                                                info!("{TAG} [setup_device] 同步DCS与ring.cycle: DCI={}, ring.cycle={}", 
                                                      dci, ring_cycle);
                                                // 根据ring的cycle值设置DCS
                                                if ring_cycle {
                                                    ep_mut.set_dequeue_cycle_state();
                                                } else {
                                                    ep_mut.clear_dequeue_cycle_state();
                                                }
                                                let endpoint_type = ep.endpoint_type();
                                                match endpoint_type {
                                                    EndpointType::Control => {}
                                                    EndpointType::BulkOut | EndpointType::BulkIn => {
                                                        ep_mut.set_max_burst_size(0);
                                                        ep_mut.set_max_primary_streams(0);
                                                    }
                                                    EndpointType::IsochOut
                                                    | EndpointType::IsochIn
                                                    | EndpointType::InterruptOut
                                                    | EndpointType::InterruptIn => {
                                                        //init for isoch/interrupt
                                                        let actual_max_packet_size = max_packet_size & 0x7ff; //refer xhci page 162
                                                        ep_mut.set_max_packet_size(actual_max_packet_size); 
                                                        ep_mut.set_max_burst_size(
                                                            ((max_packet_size & 0x1800) >> 11)
                                                                .try_into()
                                                                .unwrap(),
                                                        );
                                                        ep_mut.set_mult(0); //always 0 for interrupt

                                                        if let EndpointType::IsochOut | EndpointType::IsochIn =
                                                            endpoint_type
                                                        {
                                                            ep_mut.set_error_count(0);
                                                            // 为等时端点设置合适的AvgTRBLen，防止RingOverrun
                                                            let avg_trb_len = 4096; // 默认使用4KB作为等时传输的平均TRB长度
                                                            ep_mut.set_average_trb_length(avg_trb_len);
                                                            info!("{TAG} [setup_device] Initial config for Isoch DCI {}: Set AvgTRBLen to {}", dci, avg_trb_len);
                                                        }

                                                        ep_mut.set_tr_dequeue_pointer(ring_addr);
                                                        ep_mut
                                                        .set_max_endpoint_service_time_interval_payload_low(
                                                            4,
                                                        );
                                                        //best guess?
                                                    }
                                                    EndpointType::NotValid => {
                                                        unreachable!("Not Valid Endpoint should not exist.")
                                                    }
                                                }
                                            }
                                        }
                                    });
                            });

                    let input_addr = {
                        let input =
                            self.dev_ctx.device_input_context_list[device_slot_id].deref_mut();
                        let control_mut = input.control_mut();
                        control_mut.set_add_context_flag(0);
                        control_mut.set_configuration_value(configure.data.config_val());

                        control_mut.set_interface_number(0); //
                        control_mut.set_alternate_setting(0); //always exist
                        (input as *const Input<16>).addr() as u64
                    };

                    let command_completion = self
                        .post_cmd(command::Allowed::ConfigureEndpoint(
                            *command::ConfigureEndpoint::default()
                                .set_slot_id(device_slot_id as _)
                                .set_input_context_pointer(input_addr),
                        ))
                        .unwrap();

                    self.trace_dump_context(device_slot_id);
                    match command_completion.completion_code() {
                        Ok(ok) => match ok {
                            CompletionCode::Success => UCB::<O>::new(CompleteCode::Event(
                                TransferEventCompleteCode::Success(None), // No specific TRB pointer for command completion
                            )),
                            other => panic!("err:{:?}", other),
                        },
                        Err(err) => {
                            UCB::new(CompleteCode::Event(TransferEventCompleteCode::Unknown(err as u8))) // Assuming err is u8 completion code
                        }
                    };
                }
                TopologicalUSBDescriptorFunction::Interface(interfaces) => {
                    let (interface0, attributes, endpoints) = interfaces.first().unwrap();
                    let input_addr = {
                        {
                            let input =
                                self.dev_ctx.device_input_context_list[device_slot_id].deref_mut();
                            {
                                let control_mut = input.control_mut();
                                control_mut.set_add_context_flag(0);
                                control_mut.set_configuration_value(configure.data.config_val());

                                control_mut.set_interface_number(interface0.interface_number);
                                control_mut.set_alternate_setting(interface0.alternate_setting);
                            }

                            let entries = endpoints
                                .iter()
                                .filter_map(|endpoint| {
                                    if let TopologicalUSBDescriptorEndpoint::Standard(ep) = endpoint
                                    {
                                        Some(ep)
                                    } else {
                                        None
                                    }
                                })
                                .map(|endpoint| endpoint.doorbell_value_aka_dci())
                                .max()
                                .unwrap_or(1);

                            input
                                .device_mut()
                                .slot_mut()
                                .set_context_entries(entries as u8);
                        }

                        // info!("endpoints:{:#?}", interface.endpoints);

                        for item in endpoints {
                            if let TopologicalUSBDescriptorEndpoint::Standard(ep) = item {
                                let dci = ep.doorbell_value_aka_dci() as usize;
                                let max_packet_size = ep.max_packet_size;
                                
                                // 获取ring信息
                                let (ring_addr, ring_cycle) = {
                                    let ring = self.ep_ring_mut(device_slot_id, dci);
                                    (ring.register(), ring.cycle)
                                };

                                let input = self.dev_ctx.device_input_context_list[device_slot_id]
                                    .deref_mut();
                                let control_mut = input.control_mut();
                                info!("init ep {} {:?}", dci, ep.endpoint_type());
                                control_mut.set_add_context_flag(dci);
                                let ep_mut = input.device_mut().endpoint_mut(dci);
                                let xhci_interval = if ep.interval > 0 { ep.interval - 1 } else { 0 }; // HS/SS: bInterval-1
                                info!(
                                    "Setting interval for DCI {} to {} (from bInterval {})",
                                    dci, xhci_interval, ep.interval
                                );
                                ep_mut.set_interval(xhci_interval);
                                ep_mut.set_endpoint_type(ep.endpoint_type());
                                ep_mut.set_tr_dequeue_pointer(ring_addr);
                                ep_mut.set_max_packet_size(max_packet_size);
                                ep_mut.set_error_count(3);
                                info!("{TAG} [setup_device] 同步DCS与ring.cycle: DCI={}, ring.cycle={}", 
                                      dci, ring_cycle);
                                // 根据ring的cycle值设置DCS
                                if ring_cycle {
                                    ep_mut.set_dequeue_cycle_state();
                                } else {
                                    ep_mut.clear_dequeue_cycle_state();
                                }
                                let endpoint_type = ep.endpoint_type();
                                match endpoint_type {
                                    EndpointType::Control => {}
                                    EndpointType::BulkOut | EndpointType::BulkIn => {
                                        ep_mut.set_max_burst_size(0);
                                        ep_mut.set_max_primary_streams(0);
                                    }
                                    EndpointType::IsochOut
                                    | EndpointType::IsochIn
                                    | EndpointType::InterruptOut
                                    | EndpointType::InterruptIn => {
                                        //init for isoch/interrupt
                                        let actual_max_packet_size = max_packet_size & 0x7ff; //refer xhci page 162
                                        ep_mut.set_max_packet_size(actual_max_packet_size);
                                        ep_mut.set_max_burst_size(
                                            ((max_packet_size & 0x1800) >> 11).try_into().unwrap(),
                                        );
                                        ep_mut.set_mult(0); //always 0 for interrupt

                                        if let EndpointType::IsochOut | EndpointType::IsochIn =
                                            endpoint_type
                                        {
                                            ep_mut.set_error_count(0);
                                            // 为等时端点设置合适的AvgTRBLen，防止RingOverrun
                                            let avg_trb_len = 4096; // 默认使用4KB作为等时传输的平均TRB长度
                                            ep_mut.set_average_trb_length(avg_trb_len);
                                            info!("{TAG} [setup_device] Initial config for Isoch DCI {}: Set AvgTRBLen to {}", dci, avg_trb_len);
                                        }

                                        ep_mut.set_tr_dequeue_pointer(ring_addr);
                                        ep_mut
                                            .set_max_endpoint_service_time_interval_payload_low(4);
                                        //best guess?
                                    }
                                    EndpointType::NotValid => {
                                        unreachable!("Not Valid Endpoint should not exist.")
                                    }
                                }
                            }
                        }

                        let input =
                            self.dev_ctx.device_input_context_list[device_slot_id].deref_mut();
                        (input as *const Input<16>).addr() as u64
                    };

                    let command_completion = self
                        .post_cmd(command::Allowed::ConfigureEndpoint(
                            *command::ConfigureEndpoint::default()
                                .set_slot_id(device_slot_id as _)
                                .set_input_context_pointer(input_addr),
                        ))
                        .unwrap();

                    self.trace_dump_context(device_slot_id);
                    
                    match command_completion.completion_code() {
                        Ok(ok) => match ok {
                            CompletionCode::Success => UCB::<O>::new(CompleteCode::Event(
                                TransferEventCompleteCode::Success(None), // No specific TRB pointer for command completion
                            )),
                            other => panic!("err:{:?}", other),
                        },
                        Err(err) => {
                            UCB::new(CompleteCode::Event(TransferEventCompleteCode::Unknown(err as u8))) // Assuming err is u8 completion code
                        }
                    };
                }
            }
        }
        //TODO: Improve
        Ok(UCB::new(CompleteCode::Event(
            TransferEventCompleteCode::Success(None), // General success for setup_device
        )))
    }

    fn prepare_transfer_normal(&mut self, device_slot_id: usize, dci: u8) {
        // Transfer Ring的初始PCS=1，所以填充的TRB也使用cycle=1
        let mut normal = transfer::Normal::default();
        normal.set_cycle_bit();
        let ring = self.ep_ring_mut(device_slot_id, dci as usize);
        ring.enque_trbs(vec![normal.into_raw(); 31]) //the 32 is link trb
    }

    // 修复read_mfindex函数返回类型错误
    fn read_mfindex(&self) -> u32 {
        // 读取微帧索引寄存器 (MFINDEX)
        let reg = self.regs.runtime.mfindex.read_volatile();
        // 返回实际的微帧索引值，屏蔽保留位，并转换为u32
        reg.microframe_index().into()
    }

    // 改进的端点初始化函数 - 支持任意端点类型
    fn init_endpoint(&mut self, 
        device_slot_id: usize, 
        endpoint_id: usize, 
        endpoint_type: EndpointType,
        max_packet_size: u16
    ) -> bool {
        info!("init ep {} IsochIn", endpoint_id);
        
        // 计算DCI
        // let endpoint_number = (endpoint_id & 0x0F) as usize; // endpoint_id is already DCI here
        // let direction = ((endpoint_id & 0x80) >> 7) as usize;
        let dci = endpoint_id; // endpoint_id parameter in this function is used as DCI
        
        // 获取传输环信息
        let (ring_addr, ring_cycle) = {
            let ring = self.ep_ring_mut(device_slot_id, endpoint_id); // endpoint_id is already usize DCI
            (ring.register(), ring.cycle)
        };
        
        // 配置输入上下文
        let input = self.dev_ctx.device_input_context_list[device_slot_id].deref_mut();
        
        // 设置控制上下文
        let control = input.control_mut();
        control.set_add_context_flag(0); // 槽位上下文
        control.set_add_context_flag(dci); // 端点上下文
        
        // 设置端点上下文
        let ep_mut = input.device_mut().endpoint_mut(dci);
        ep_mut.set_endpoint_type(endpoint_type);
        ep_mut.set_max_packet_size(max_packet_size);
        ep_mut.set_interval(8); // 设置默认间隔为8
        ep_mut.set_error_count(3);
        ep_mut.set_tr_dequeue_pointer(ring_addr);
        if ring_cycle {
            ep_mut.set_dequeue_cycle_state();
            info!("{TAG} [init_endpoint] 设置DCS=1 (ring.cycle=true) for DCI={}", dci);
        } else {
            ep_mut.clear_dequeue_cycle_state();
            info!("{TAG} [init_endpoint] 设置DCS=0 (ring.cycle=false) for DCI={}", dci);
        }
        
        // 根据端点类型设置特殊参数
        match endpoint_type {
            EndpointType::IsochOut | EndpointType::IsochIn => {
                let mult_val: u8 = ((max_packet_size & 0x1800) >> 11) as u8;
                ep_mut.set_max_packet_size(max_packet_size & 0x7ff);
                
                // 需要获取设备速度来正确设置mult和max_burst_size
                if let Some(dev_ctx_slot) = self.dev_ctx.device_out_context_list.get(device_slot_id) {
                    let slot_ctx = dev_ctx_slot.slot();
                    let device_speed = slot_ctx.speed();
                    
                    match device_speed {
                        1 | 3 => {  // Full-Speed (1) / High-Speed (3)
                            ep_mut.set_mult(mult_val); // 0-2 -> 1-3 transactions per microframe
                            ep_mut.set_max_burst_size(0);
                        }
                        4 => {      // Super-Speed (4)
                            ep_mut.set_mult(0);
                            ep_mut.set_max_burst_size(mult_val);
                        }
                        _ => {
                            // 默认按HS处理
                            ep_mut.set_mult(mult_val);
                            ep_mut.set_max_burst_size(0);
                        }
                    }
                    info!("{TAG} 设备速度={}, mult={}, max_burst_size={} for DCI {}", 
                          device_speed, 
                          if device_speed == 4 { 0 } else { mult_val },
                          if device_speed == 4 { mult_val } else { 0 },
                          dci);
                } else {
                    // 如果无法获取设备速度，默认按HS处理
                    ep_mut.set_mult(mult_val);
                    ep_mut.set_max_burst_size(0);
                }
                
                ep_mut.set_error_count(0); // 等时端点不需要错误计数
                ep_mut.set_max_endpoint_service_time_interval_payload_low(4);

                // NEW: Set Average TRB Length and corrected Interval for Isoch
                const ISOC_PACKETS_PER_URB_CONST: usize = 2; // Assuming this matches generic_uvc.rs
                let average_trb_length = (ISOC_PACKETS_PER_URB_CONST * max_packet_size as usize).min(u16::MAX as usize) as u16;
                ep_mut.set_average_trb_length(average_trb_length);
                info!("{TAG} Set Average TRB Length to {} for DCI {}", average_trb_length, dci);

                ep_mut.set_interval(0); // CORRECTED Interval for HS Isoch with bInterval=1 (bInterval-1)
                info!("{TAG} Set Interval to 0 for DCI {}", dci);
            },
            EndpointType::InterruptOut | EndpointType::InterruptIn => {
                let mult_val: u8 = ((max_packet_size & 0x1800) >> 11) as u8;
                ep_mut.set_max_packet_size(max_packet_size & 0x7ff);
                
                // 需要获取设备速度来正确设置mult和max_burst_size
                if let Some(dev_ctx_slot) = self.dev_ctx.device_out_context_list.get(device_slot_id) {
                    let slot_ctx = dev_ctx_slot.slot();
                    let device_speed = slot_ctx.speed();
                    
                    match device_speed {
                        1 | 3 => {  // Full-Speed (1) / High-Speed (3)
                            ep_mut.set_mult(mult_val);
                            ep_mut.set_max_burst_size(0);
                        }
                        4 => {      // Super-Speed (4)
                            ep_mut.set_mult(0);
                            ep_mut.set_max_burst_size(mult_val);
                        }
                        _ => {
                            // 默认按HS处理
                            ep_mut.set_mult(mult_val);
                            ep_mut.set_max_burst_size(0);
                        }
                    }
                } else {
                    // 如果无法获取设备速度，默认按HS处理
                    ep_mut.set_mult(mult_val);
                    ep_mut.set_max_burst_size(0);
                }
                
                ep_mut.set_max_endpoint_service_time_interval_payload_low(4);
            },
            EndpointType::BulkOut | EndpointType::BulkIn => {
                ep_mut.set_max_burst_size(0);
                ep_mut.set_max_primary_streams(0);
            },
            EndpointType::Control => {
                // 控制端点已在address_device中设置
            },
            EndpointType::NotValid => {
                warn!("Trying to initialize invalid endpoint type");
                return false;
            }
        }
        
        // 发送配置端点命令
        let input_addr = (input as *const Input<16>).addr() as u64;
        
        match self.post_cmd(command::Allowed::ConfigureEndpoint(
            *command::ConfigureEndpoint::default()
                .set_slot_id(device_slot_id as _)
                .set_input_context_pointer(input_addr),
        )) {
            Ok(_) => {
                info!("Successfully configured endpoint 0x{:02x} (DCI={})", endpoint_id, dci);
                true
            },
            Err(e) => {
                warn!("Failed to configure endpoint 0x{:02x}: {:?}", endpoint_id, e);
                false
            }
        }
    }

    // 添加一个新的辅助函数，用于检查端点上下文状态
    fn dump_endpoint_context(&mut self, dev_slot_id: usize, dci: u8) {
        trace!("{TAG} 检查端点上下文状态: 槽ID={}, DCI={}", dev_slot_id, dci);
        
        if let Some(dev_ctx_slot) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
            // 获取端点上下文
            let ep_ctx = dev_ctx_slot.endpoint(dci as usize);
            
            
            // 获取端点状态、类型和其他关键参数
            let ep_state = ep_ctx.endpoint_state();
            let ep_type = ep_ctx.endpoint_type();
            let tr_deque_ptr = ep_ctx.tr_dequeue_pointer();
            let max_packet_size = ep_ctx.max_packet_size();
            let max_burst_size = ep_ctx.max_burst_size();
            let avg_trb_len = ep_ctx.average_trb_length();
            let interval = ep_ctx.interval();
            let error_count = ep_ctx.error_count();
            let dequeue_cycle_state = ep_ctx.dequeue_cycle_state();
            
            // 打印详细信息
            info!(
                "{TAG} [端点状态] State={:?}, Type={:?}, TR_DeqPtr=0x{:X}, DCS={}, MaxPktSize={}, AvgTRBLen={}, MaxBurstSize={}, Interval={}, ErrorCount={}",
                ep_state, ep_type, tr_deque_ptr, dequeue_cycle_state, max_packet_size, avg_trb_len, max_burst_size, interval, error_count
            );
            
                    // 获取并比较传输环地址
        let ring = self.ep_ring_mut(dev_slot_id, dci as usize);
        let ring_addr = ring.register();
        // TR_DeqPtr的bit 0是DCS (Dequeue Cycle State)位，需要屏蔽掉再比较
        let tr_deque_ptr_masked = tr_deque_ptr & !1u64;
        if tr_deque_ptr_masked != ring_addr {
            // 这个警告实际上是正常的 - TR_DeqPtr包含了DCS位
            trace!("{TAG} 端点TR_DeqPtr=0x{:X}与环地址=0x{:X}不匹配 (DCS={})，这是正常的", 
                  tr_deque_ptr, ring_addr, dequeue_cycle_state);
        }
        } else {
            error!("{TAG} [端点状态] 设备槽{}不存在", dev_slot_id);
        }
    }

    // 添加修复环指针不匹配的函数
    fn fix_endpoint_ring_pointer(&mut self, dev_slot_id: usize, dci: u8) -> bool {
        trace!("{TAG} [修复环指针] 尝试修复槽ID={}, DCI={}的环指针不匹配问题", dev_slot_id, dci);
        
        if let Some(dev_ctx_slot) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
            // 获取端点上下文
            let ep_ctx = dev_ctx_slot.endpoint(dci as usize);
            
            // 获取当前值
            let tr_deque_ptr = ep_ctx.tr_dequeue_pointer();
            let dequeue_cycle_state = ep_ctx.dequeue_cycle_state();
            
                    // 获取环地址 - 正确处理
        let ring = self.ep_ring_mut(dev_slot_id, dci as usize);
        let ring_addr = ring.register();
        // TR_DeqPtr的bit 0是DCS (Dequeue Cycle State)位，需要屏蔽掉再比较
        let tr_deque_ptr_masked = tr_deque_ptr & !1u64;
        
        if tr_deque_ptr_masked != ring_addr {
            // 没有create_input_context函数，这里暂时不实现环指针修复
            warn!("{TAG} [修复环指针] 发现指针不匹配，但当前无法修复: 0x{:X}(masked=0x{:X}) vs 0x{:X}", 
                  tr_deque_ptr, tr_deque_ptr_masked, ring_addr);
            return false;
        } else {
            trace!("{TAG} [修复环指针] 环指针匹配，无需修复: 0x{:X}", ring_addr);
            return true;
        }
        } else {
            warn!("{TAG} [修复环指针] 设备上下文不存在: 槽={}", dev_slot_id);
            return false;
        }
    }

    // 修复另一处if let Ok(ring)错误
    fn fix_tr_deq_ptr(&mut self, dev_slot_id: usize, dci: u8) -> bool {
        info!("{TAG} 尝试修复端点传输环指针: 槽ID={}, DCI={}", dev_slot_id, dci);
        
        // 获取端点上下文
        if let Some(dev_ctx_slot) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
            let ep_ctx = dev_ctx_slot.endpoint(dci as usize);
            let tr_deque_ptr = ep_ctx.tr_dequeue_pointer();
            
                    // 获取实际的环地址
        let ring = self.ep_ring_mut(dev_slot_id, dci as usize);
        let ring_addr = ring.register();
        // TR_DeqPtr的bit 0是DCS (Dequeue Cycle State)位，需要屏蔽掉再比较
        let tr_deque_ptr_masked = tr_deque_ptr & !1u64;
        if tr_deque_ptr_masked != ring_addr {
            warn!("{TAG} 需要修复的环指针: 上下文=0x{:X}(masked=0x{:X})，实际环=0x{:X}", 
                  tr_deque_ptr, tr_deque_ptr_masked, ring_addr);
            
            // 暂时没有可执行的修复方法
            return false;
        } else {
            info!("{TAG} 环指针匹配，无需修复");
            return true;
        }
        } else {
            error!("{TAG} 设备上下文不存在，无法修复环指针");
            return false;
        }
    }

// 等时传输函数已移至Controller trait实现中

    // 新增：排空事件环的辅助函数
    fn drain_event_ring(&mut self) -> usize {
        let mut events_processed = 0;
        let max_events = 256; // 防止无限循环
        
        while events_processed < max_events {
            if let Some((_trb, _cycle)) = self.event.next() {
                events_processed += 1;
                // 简单记录，不做实际处理
                trace!("{TAG} 排空事件 #{}", events_processed);
                
                // 每处理16个事件后更新ERDP
                if events_processed % 16 == 0 {
                    self.update_erdp();
                }
            } else {
                break;
            }
        }
        
        // 处理完后必须更新ERDP
        if events_processed > 0 {
            self.update_erdp();
        }
        
        events_processed
    }
    
    // 新增：调试中断状态的辅助函数
    fn debug_interrupt_state(&self) {
        let iman = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
        let imod = self.regs.interrupter_register_set.interrupter(0).imod.read_volatile();
        let erstsz = self.regs.interrupter_register_set.interrupter(0).erstsz.read_volatile();
        let erdp = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        let erstba = self.regs.interrupter_register_set.interrupter(0).erstba.read_volatile();
        let usbsts = self.regs.operational.usbsts.read_volatile();
        
        error!("{TAG} 中断状态调试信息:");
        error!("{TAG}   IMAN: IE={}, IP={}", iman.interrupt_enable(), iman.interrupt_pending());
        error!("{TAG}   IMOD: Interval={}", imod.interrupt_moderation_interval());
        error!("{TAG}   ERSTSZ: {}", erstsz.get());
        error!("{TAG}   ERSTBA: 0x{:016X}", erstba.get());
        error!("{TAG}   ERDP: 0x{:016X}, EHB={}", erdp.event_ring_dequeue_pointer(), erdp.event_handler_busy());
        error!("{TAG}   USBSTS: EINT={}, HSE={}, HCE={}", 
               usbsts.event_interrupt(), usbsts.host_system_error(), usbsts.host_controller_error());
        error!("{TAG}   事件环: index={}, cycle={}, size={}", self.event.ring.i, self.event.ring.cycle, self.event.ring.len());
        
        // 打印事件环段表信息
        let segment_base = self.event.segment_base_address();
        let segment_size = self.event.ring.len();
        error!("{TAG}   事件环段表[0]: Base=0x{:016X}, Size={}", segment_base, segment_size);
        
        // 打印事件环前几个条目的内容
        self.dump_event_ring_entries();
    }
    
    // 新增：打印事件环内容用于调试
    fn dump_event_ring_entries(&self) {
        error!("{TAG} 事件环内容快照:");
        let ring_size = self.event.ring.len();
        let current_idx = self.event.ring.i;
        
        // 打印当前位置附近的10个条目
        let start = if current_idx >= 5 { current_idx - 5 } else { 0 };
        let end = if current_idx + 5 < ring_size { current_idx + 5 } else { ring_size - 1 };
        
        // 统计有效事件数量
        let mut valid_events = 0;
        let mut last_valid_idx = 0;
        
        for i in start..=end {
            let trb_data = &self.event.ring.trbs[i];
            let data = unsafe {
                let mut out = [0u32; 4];
                for j in 0..4 {
                    out[j] = (trb_data.as_ptr() as *const u32).offset(j as _).read_volatile();
                }
                out
            };
            
            let marker = if i == current_idx { " <-- 当前" } else { "" };
            let cycle_bit = (data[3] & 1) == 1;
            let trb_type = (data[3] >> 10) & 0x3F;
            
            // 检查是否是有效事件
            if data != [0, 0, 0, 0] {
                valid_events += 1;
                last_valid_idx = i;
            }
            
            error!("{TAG}   [{:3}]: [{:08X}, {:08X}, {:08X}, {:08X}] Type={:2}, Cycle={}{}", 
                   i, data[0], data[1], data[2], data[3], trb_type, cycle_bit, marker);
            
            // 如果是传输事件，解析更多信息
            if trb_type == 32 { // Transfer Event
                let trb_ptr = (data[0] as u64) | ((data[1] as u64) << 32);
                let completion_code = (data[2] >> 24) & 0xFF;
                let trb_length = data[2] & 0xFFFFFF;
                let endpoint_id = (data[3] >> 16) & 0x1F;
                let slot_id = (data[3] >> 24) & 0xFF;
                
                error!("{TAG}     TransferEvent: TRB=0x{:X}, Slot={}, EP={}, CC={}, Len={}", 
                       trb_ptr, slot_id, endpoint_id, completion_code, trb_length);
            }
        }
        
        error!("{TAG} 有效事件数: {}, 最后有效索引: {}", valid_events, last_valid_idx);
    }
    
    // 尝试恢复措施清除EHB
    fn try_clear_ehb_recovery(&mut self) {
        warn!("{TAG} 尝试EHB恢复措施...");
        
        // 首先调试事件环状态
        self.debug_interrupt_state();
        
        // 1. 禁用中断
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.clear_interrupt_enable();
        });
        
        // 2. 处理所有积压事件
        let drained = self.drain_event_ring();
        warn!("{TAG} 排空了{}个事件", drained);
        
        // 3. 检查是否有环绕回问题
        let current_idx = self.event.ring.i;
        let ring_size = self.event.ring.len();
        if current_idx >= ring_size {
            error!("{TAG} 事件环索引超出范围！idx={}, size={}", current_idx, ring_size);
            // 重置索引
            self.event.ring.i = 0;
            self.event.ring.cycle = true;
        }
        
        // 4. 重置事件环指针
        warn!("{TAG} 重置事件环指针到起始位置");
        self.event.ring.i = 0;
        self.event.ring.cycle = true;
        
        // 5. 清空事件环内容（避免旧数据干扰）
        warn!("{TAG} 清空事件环内容");
        for i in 0..self.event.ring.len() {
            let trb_data = &self.event.ring.trbs[i];
            unsafe {
                // 将所有TRB设置为0，除了cycle bit
                let ptr = trb_data.as_ptr() as *mut u32;
                ptr.offset(0).write_volatile(0);
                ptr.offset(1).write_volatile(0);
                ptr.offset(2).write_volatile(0);
                // 设置cycle bit为0（与初始cycle=1相反）
                ptr.offset(3).write_volatile(0);
            }
        }
        
        // 6. 重置中断器
        // 保存当前配置
        let erstba = self.event.erstba();
        let erstsz = 1;
        
        // 清除中断器配置
        self.regs.interrupter_register_set.interrupter_mut(0).erstsz.update_volatile(|r| {
            r.set(0);
        });
        
        // 等待一段时间
        for _ in 0..10000 { core::hint::spin_loop(); }
        
        // 重新配置
        self.regs.interrupter_register_set.interrupter_mut(0).erstsz.update_volatile(|r| {
            r.set(erstsz);
        });
        self.regs.interrupter_register_set.interrupter_mut(0).erstba.update_volatile(|r| {
            r.set(erstba);
        });
        
        // 7. 重新设置ERDP到起始位置
        let erdp = self.event.ring.register() as u64; // 环的起始地址
        self.regs.interrupter_register_set.interrupter_mut(0).erdp.update_volatile(|r| {
            r.set_event_ring_dequeue_pointer(erdp);
        });
        
        // 等待一段时间
        for _ in 0..10000 { core::hint::spin_loop(); }
        
        // 8. 重新启用中断
        self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
            im.set_interrupt_enable();
            im.clear_interrupt_pending(); // 清除任何挂起的中断
        });
        
        // 9. 检查EHB状态
        let ehb_after = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile().event_handler_busy();
        if !ehb_after {
            trace!("{TAG} 重置中断器后成功清除EHB");
        } else {
            error!("{TAG} 重置中断器后EHB仍为1");
            error!("{TAG} 可能需要重启控制器");
        }
    }
    
    // 新增：为UVC设置优化的中断调制参数
    fn setup_interrupt_moderation_for_uvc(&mut self) {
        // 根据XHCI 1.1规范 5.5.2.2节
        // IMOD寄存器控制中断频率：
        // - Interval: 以250ns为单位的中断间隔
        // - Counter: 触发中断前的最小事件数
        
        // 对于UVC视频流，我们需要平衡：
        // 1. 低延迟（快速响应）
        // 2. 避免中断风暴（CPU负载）
        
        // 使用4 * 250ns = 1μs的间隔，这对于视频流是合理的
        // 计数器设为1，确保每个事件都能及时处理
        let interval = 4;  // 1μs
        let counter = 1;   // 每个事件触发中断
        
        self.regs.interrupter_register_set.interrupter_mut(0).imod.update_volatile(|im| {
            im.set_interrupt_moderation_interval(interval);
            im.set_interrupt_moderation_counter(counter);
        });
        
        info!("{TAG} UVC优化IMOD设置: 间隔={} ({}μs), 计数器={}", 
              interval, interval * 250 / 1000, counter);
    }
    
    // 新增：动态调整中断调制参数
    pub fn adjust_interrupt_moderation(&mut self, event_rate: u32) {
        // 根据事件率动态调整中断参数
        let (interval, counter) = if event_rate > 1000 {
            // 高事件率：稍微增加间隔和计数器
            (8, 4)  // 2μs, 4个事件
        } else if event_rate > 500 {
            // 中等事件率
            (4, 2)  // 1μs, 2个事件
        } else {
            // 低事件率：最小延迟
            (2, 1)  // 500ns, 1个事件
        };
        
        self.regs.interrupter_register_set.interrupter_mut(0).imod.update_volatile(|im| {
            im.set_interrupt_moderation_interval(interval);
            im.set_interrupt_moderation_counter(counter);
        });
        
        trace!("{TAG} 动态调整IMOD: 事件率={}/s, 间隔={}, 计数器={}", 
               event_rate, interval, counter);
    }

    // Linux风格的端点prime函数
    fn prime_endpoint_for_isoch(&mut self, dev_slot_id: usize, dci: usize) {
        trace!("[XHCI] Prime端点 slot={}, DCI={} 准备等时传输", dev_slot_id, dci);
        
        // Linux在首次使用等时端点前会"prime"它
        // 这通过提交一个dummy TD来实现
        if let Some(dev_ctx) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
            let ep_ctx = dev_ctx.endpoint(dci);
            let ep_state = ep_ctx.endpoint_state();
            
            trace!("[XHCI] Prime检查: 端点状态={:?}", ep_state);
            
            if ep_state == EndpointState::Stopped {
                info!("[XHCI] 端点处于停止状态，需要重启");
                // 这里应该发送Set TR Dequeue Pointer命令
                // 但为了简化，我们暂时跳过
            } else if ep_state == EndpointState::Running {
                info!("[XHCI] 端点已在Running状态，Prime操作正常");
            } else {
                warn!("[XHCI] 端点处于异常状态: {:?}", ep_state);
            }
        } else {
            error!("[XHCI] Prime失败：无法获取设备上下文 slot={}", dev_slot_id);
        }
        
        // 清理事件环，确保没有旧事件
        let events_cleared = self.process_event_ring();
        if events_cleared > 0 {
            info!("[XHCI] Prime时清理了{}个旧事件", events_cleared);
        } else {
            trace!("[XHCI] Prime时事件环为空");
        }
        
        // 重要修复：Prime操作不应该提交dummy TRB
        // 根据日志分析，dummy TRB可能导致硬件停滞
        // 我们只需要确保端点处于正确状态即可
        trace!("[XHCI] Prime操作完成，端点DCI={}已准备好接收数据", dci);
        
        trace!("[XHCI] Prime操作结束");
    }

    // Linux风格的等时TD生成函数
    fn queue_isoch_td_linux_style(
        &mut self,
        dev_slot_id: usize,
        dci: usize,
        addr: usize,
        total_len: usize,
        num_packets: usize,
        packet_size: usize,
        use_sia: bool,
    ) -> Result<usize, Error> {
        // Linux方式：计算TD中需要多少个TRB
        let mut trbs_needed = if num_packets > 0 {
            // 每个包一个TRB（Linux典型方式）
            num_packets
        } else {
            // 至少一个TRB
            1
        };
        
        // Linux还会为大的传输添加额外的TRB
        if total_len > 48 * 1024 {
            // 对于大传输，每48KB一个TRB
            trbs_needed = (total_len + (48 * 1024 - 1)) / (48 * 1024);
        }
        
        trace!("[XHCI] Linux TD模式：需要{}个TRB来传输{}字节（{}个包，每包{}字节）", 
              trbs_needed, total_len, num_packets, packet_size);
        
        let ring = self.ep_ring_mut(dev_slot_id, dci);
        let ring_size = ring.len();
        let start_index = ring.i;
        
        // 检查环空间
        if start_index + trbs_needed >= ring_size - 2 {
            error!("[XHCI] 环空间不足：需要{}个TRB，但只有{}个可用", 
                   trbs_needed, ring_size - start_index - 2);
            return Err(Error::RingFull);
        }
        
        let mut current_addr = addr;
        let mut remaining_len = total_len;
        let first_trb_addr = ring.register() + (start_index as u64 * 16);
        let mut last_trb_addr = first_trb_addr;
        
        // Linux方式：构建多个TRB组成一个TD
        for i in 0..trbs_needed {
            let is_first = i == 0;
            let is_last = i == (trbs_needed - 1);
            
            // 计算这个TRB的传输长度
            let trb_len = if num_packets > 0 && i < num_packets {
                // 等时模式：每个TRB一个包
                core::cmp::min(packet_size, remaining_len)
            } else {
                // 批量模式：最多48KB per TRB
                core::cmp::min(48 * 1024, remaining_len)
            };
            
            // 计算TD Size（Linux算法）
            let packets_remaining = if num_packets > 0 {
                num_packets.saturating_sub(i + 1)
            } else {
                0
            };
            
            let td_size = if is_last {
                0  // 最后一个TRB的TD Size总是0
            } else {
                // Linux使用剩余包数作为TD Size
                core::cmp::min(packets_remaining, 31) as u32
            };
            
            let mut raw_trb = [0u32; 4];
            raw_trb[0] = current_addr as u32;
            raw_trb[1] = (current_addr >> 32) as u32;
            raw_trb[2] = (trb_len as u32 & 0x1FFFF) | (td_size << 17);
            // 重要：清除Interrupter Target字段 (bits 22:31)
            raw_trb[2] &= 0x3FFFFF; // 清除高位
            raw_trb[2] |= 0 << 22; // Interrupter Target = 0
            
            // DW3设置
            raw_trb[3] = 5 << 10;  // Type = Isoch
            
            // Chain位：除了最后一个TRB，都设置Chain
            if !is_last {
                raw_trb[3] |= 1 << 4;  // CH = 1
            }
            
            // IOC位：只在最后一个TRB设置
            if is_last {
                raw_trb[3] |= 1 << 5;  // IOC = 1
            }
            
            // 第一个TRB设置SIA（如果启用）
            if is_first && use_sia {
                raw_trb[3] |= 1 << 31;  // SIA = 1
            }
            
            // BEI位：Linux在中间的TRB上设置，减少中断
            if !is_last && trbs_needed > 2 {
                raw_trb[3] |= 1 << 9;  // BEI = 1
            }
            
            trace!("[XHCI] TD TRB[{}]: addr=0x{:X}, len={}, TD_Size={}, Chain={}, IOC={}, SIA={}", 
                  i, current_addr, trb_len, td_size, !is_last, is_last, is_first && use_sia);
            
            // 入队TRB
            let isoch_trb = unsafe { 
                core::mem::transmute::<[u32; 4], transfer::Isoch>(raw_trb) 
            };
            let trb_addr = ring.enque_transfer(transfer::Allowed::Isoch(isoch_trb));
            
            // 记录最后一个TRB的地址
            if is_last {
                last_trb_addr = trb_addr as u64;
            }
            
            current_addr += trb_len;
            remaining_len -= trb_len;
        }
        
        // Linux等待的是TD中最后一个带IOC的TRB
        Ok(last_trb_addr as usize)
    }
}

impl<O> Controller<O> for XHCI<O>
where
    O: PlatformAbstractions + 'static,
{
    fn new(config: Arc<SpinNoIrq<USBSystemConfig<O>>>) -> Self
    where
        Self: Sized,
    {
        let mmio_base = config.lock().base_addr.clone().into();
        unsafe {
            let regs = RegistersBase::new(mmio_base, MemMapper);
            let ext_list =
                RegistersExtList::new(mmio_base, regs.capability.hccparams1.read(), MemMapper);

            let hcsp1 = regs.capability.hcsparams1.read_volatile();
            let max_slots = hcsp1.number_of_device_slots();
            let max_ports = hcsp1.number_of_ports();
            let max_irqs = hcsp1.number_of_interrupts();
            let page_size = regs.operational.pagesize.read_volatile().get();
            info!(
                "{TAG} Max_slots: {}, max_ports: {}, max_irqs: {}, page size: {}",
                max_slots, max_ports, max_irqs, page_size
            );

            info!("new dev ctx!");
            let dev_ctx = DeviceContextList::new(max_slots, config.clone());

            // Create the command ring with 4096 / 16 (TRB size) entries, so that it uses all of the
            // DMA allocation (which is at least a 4k page).
            let entries_per_page = O::PAGE_SIZE / mem::size_of::<ring::TrbData>();
            info!("new cmd ring");
            let cmd = Ring::new_command_ring(config.lock().os.clone(), entries_per_page).unwrap();
            info!("new evt ring");
            let event = EventRing::new(config.lock().os.clone()).unwrap();

            // Print the Event Ring Segment's base address using the new public method
            let ring_segment_base = event.segment_base_address();
            info!("{TAG} Event Ring Segment 0 Base Address (from STE[0] via method): {:#0X}", ring_segment_base);

            info!("{TAG} ring size {}", cmd.len());

            Self {
                regs,
                ext_list,
                config: config.clone(),
                max_slots: max_slots,
                max_ports: max_ports,
                max_irqs: max_irqs,
                scratchpad_buf_arr: None,
                cmd: cmd,
                event: event,
                dev_ctx: dev_ctx,
                interrupt_enabled: false,
                isoc_first_transfer: true,
                async_transfers: BTreeMap::new(),
                async_commands: BTreeMap::new(),
                self_arc: None,
                pending_isoch_urbs: BTreeMap::new(),
                endpoint_progress: BTreeMap::new(),
            }
        }
    }

    fn init(&mut self) {
        self.chip_hardware_reset()
            .set_max_device_slots()
            .set_dcbaap()
            .set_cmd_ring()
            .init_ir()
            .setup_scratchpads()
            .start()
            .test_cmd()
            .reset_ports();

        // 增大事件环大小以减少RingOverrun风险
        let current_event_ring_size = self.event.ring.len();
        info!("{TAG} 当前事件环大小: {}", current_event_ring_size);
        
        // 优化中断调制参数 - 为UVC等时传输优化
        self.setup_interrupt_moderation_for_uvc();
        
        // 先初始化中断处理（设置全局指针，但不启用中断）
        self.init_interrupt_handling_without_enable();
        
        // 确保所有挂起的事件都被清除
        self.clear_pending_events();
        
        // 延迟中断启用直到self_arc被设置
        info!("{TAG} 初始化完成，中断将在self_arc设置后启用");
        
//         // 最后启用中断 - 这是Linux的做法
//         info!("{TAG} 所有初始化完成，现在启用中断");
//         
//         // 先启用Interrupter 0的中断（如果还没启用）
//         self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
//             im.set_interrupt_enable();
//         });
//         
//         // 然后启用全局XHCI中断
//         self.regs.operational.usbcmd.update_volatile(|r| {
//             r.set_interrupter_enable();
//         });
//         
//         // 最后在系统中启用中断路由
//         const XHCI_IRQ_NUM: usize = 48;
//         kycall::ky_debug_setirq(XHCI_IRQ_NUM, true);
//         info!("{TAG} 已在系统中启用XHCI中断(IRQ {})", XHCI_IRQ_NUM);
//         
//         let usbcmd_val = self.regs.operational.usbcmd.read_volatile();
//         info!("{TAG} USBCMD.INTE (中断使能) 最终状态: {}", usbcmd_val.interrupter_enable());
//         info!("{TAG} USBCMD.RunStop (RS) 最终状态: {}", usbcmd_val.run_stop());
//         
//         // 确保控制器正在运行
        // 确保控制器正在运行（但不启用中断）
        let usbcmd = self.regs.operational.usbcmd.read_volatile();
        let usbsts = self.regs.operational.usbsts.read_volatile();
        info!("{TAG} 初始化完成 - USBCMD: RS={}, INTE={}", usbcmd.run_stop(), usbcmd.interrupter_enable());
        info!("{TAG}            - USBSTS: HCH={}, IE={}", usbsts.hc_halted(), usbsts.event_interrupt());
        
//         // 打印关键寄存器地址和值 - CRCR大部分字段是write-only
        let crcr = self.regs.operational.crcr.read_volatile();
        info!("{TAG} CRCR: CRR={}", crcr.command_ring_running());
        
        // 打印事件环配置
        let erstsz = self.regs.interrupter_register_set.interrupter(0).erstsz.read_volatile();
        let erstba = self.regs.interrupter_register_set.interrupter(0).erstba.read_volatile();
        let erdp = self.regs.interrupter_register_set.interrupter(0).erdp.read_volatile();
        info!("{TAG} 事件环: ERSTSZ={}, ERSTBA=0x{:016X}", erstsz.get(), erstba.get());
        info!("{TAG} ERDP=0x{:016X}, EHB={}", erdp.event_ring_dequeue_pointer(), erdp.event_handler_busy());
        
        // 确保EHB位在初始化时被清除
        if erdp.event_handler_busy() {
            warn!("{TAG} 初始化时发现EHB=1，清除它");
            self.clear_ehb();
        }
        
        if !usbcmd.run_stop() || usbsts.hc_halted() {
            error!("{TAG} 控制器没有正确启动！");
        }
        
        // 初始化完成后，检查是否有挂起的中断
        if self.handle_pending_interrupts() {
            info!("{TAG} 初始化后处理了挂起的中断");
        }
        
        // 发送一个NoOp命令测试命令环
        info!("{TAG} 发送NoOp命令测试控制器...");
        match self.post_cmd(command::Allowed::Noop(command::Noop::new())) {
            Ok(_) => info!("{TAG} NoOp命令成功！控制器工作正常"),
            Err(e) => error!("{TAG} NoOp命令失败: {:?}", e),
        }
        
        // 检查控制器与中断状态
        self.debug_controller_status();
        self.debug_event_ring_status();
    }

    fn probe(&mut self) -> Vec<usize> {
        let mut founded = Vec::new();

        {
            let mut port_id_list = Vec::new();
            let port_len = self.regs.port_register_set.len();
            for i in 0..port_len {
                let portsc = &self.regs.port_register_set.read_volatile_at(i).portsc;
                info!(
                    "{TAG} Port {}: Enabled: {}, Connected: {}, Speed {}, Power {}",
                    i,
                    portsc.port_enabled_disabled(),
                    portsc.current_connect_status(),
                    portsc.port_speed(),
                    portsc.port_power()
                );

                if !portsc.port_enabled_disabled() {
                    continue;
                }

                port_id_list.push(i);
            }

            for port_idx in port_id_list {
                let port_id = port_idx + 1;
                //↓
                let slot_id = self.device_slot_assignment();
                self.dev_ctx.new_slot(slot_id as usize, 0, port_id, 32); 
                info!("assign complete!");
                //↓
                self.address_device(slot_id, port_id);
                // self.trace_dump_context(slot_id); // Moved down

                // +++ 新增测试点 +++
                info!("{TAG} Attempting to fetch control point packet size for slot {} immediately after addressing.", slot_id);
                let test_packet_size = self.control_fetch_control_point_packet_size(slot_id);
                info!("{TAG} Test fetch for slot {} resulted in packet size: {}", slot_id, test_packet_size);
                // +++++++++++++++++

                //↓
                let packet_size0 = self.control_fetch_control_point_packet_size(slot_id);
                info!("packet_size0: {}", packet_size0);
                //↓
                self.set_ep0_packet_size(slot_id, packet_size0 as _);
                self.trace_dump_context(slot_id); // MOVED HERE
                founded.push(slot_id)
            }
        }

        founded
    }

    fn control_transfer(
        &mut self,
        dev_slot_id: usize,
        urb_req: ControlTransfer,
    ) -> crate::err::Result<UCB<O>> {
        // 使用新的异步控制传输方法
        let future = self.control_transfer_async(dev_slot_id, urb_req);
        
        // 阻塞等待Future完成
        self.block_on_transfer_future(future)
    }

    fn configure_device(
        &mut self,
        dev_slot_id: usize,
        urb_req: Configuration,
    ) -> crate::err::Result<UCB<O>> {
        match urb_req {
            Configuration::SetupDevice(config) => self.setup_device(dev_slot_id, &config),
            Configuration::SwitchInterface(_, _) => todo!(),
        }
    }

    fn address_device(&mut self, slot_id: usize, port_id: usize) {
        let port_idx = port_id - 1;
        let port_speed = self.get_speed(port_idx);
        let max_packet_size = self.parse_default_max_packet_size_from_port(port_idx);
        let dci = 1;

        let transfer_ring_0_addr = self.ep_ring_mut(slot_id, dci as usize).register();
        let ring_cycle_bit = self.ep_ring_mut(slot_id, dci as usize).cycle;
        let context_addr = {
            let context_mut = self
                .dev_ctx
                .device_input_context_list
                .get_mut(slot_id)
                .unwrap()
                .deref_mut();

            let control_context = context_mut.control_mut();
            control_context.set_add_context_flag(0);
            control_context.set_add_context_flag(1);
            for i in 2..32 {
                control_context.clear_drop_context_flag(i);
            }

            let slot_context = context_mut.device_mut().slot_mut();
            slot_context.clear_multi_tt();
            slot_context.clear_hub();
            slot_context.set_route_string(Self::append_port_to_route_string(0, port_id)); // for now, not support more hub ,so hardcode as 0.//TODO: generate route string
            slot_context.set_context_entries(1);
            slot_context.set_max_exit_latency(0);
            slot_context.set_root_hub_port_number(port_id as _); //todo: to use port number
            slot_context.set_number_of_ports(0);
            slot_context.set_parent_hub_slot_id(0);
            slot_context.set_tt_think_time(0);
            slot_context.set_interrupter_target(0);
            slot_context.set_speed(port_speed);

            let endpoint_0 = context_mut.device_mut().endpoint_mut(dci as _);
            endpoint_0.set_endpoint_type(xhci::context::EndpointType::Control);
            endpoint_0.set_max_packet_size(max_packet_size);
            endpoint_0.set_max_burst_size(0);
            endpoint_0.set_error_count(3);
            endpoint_0.set_tr_dequeue_pointer(transfer_ring_0_addr);
            if ring_cycle_bit {
                endpoint_0.set_dequeue_cycle_state();
            } else {
                endpoint_0.clear_dequeue_cycle_state();
            }
            endpoint_0.set_interval(0);
            endpoint_0.set_max_primary_streams(0);
            endpoint_0.set_mult(0);
            endpoint_0.set_error_count(3);

            (context_mut as *const Input<16>).addr() as u64
        };

        fence(Ordering::Release);

        let result = self
            .post_cmd(command::Allowed::AddressDevice(
                *command::AddressDevice::new()
                    .set_slot_id(slot_id as _)
                    .set_input_context_pointer(context_addr),
            ))
            .unwrap_or_else(|e| {
                panic!("{TAG} address_device failed: {:?}", e);
            });

        info!("address slot [{}] ok", slot_id);
        
        // 在地址分配后，端点应该已经处于正确状态
        // 如果需要，可以在这里添加额外的端点配置
    }

    fn control_fetch_control_point_packet_size(&mut self, slot_id: usize) -> u8 {
        info!("control_fetch_control_point_packet_size");
        let mut buffer = DMA::new_vec(0u8, 8, 64, self.config.lock().os.dma_alloc());
        self.control_transfer(
            slot_id,
            ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::In,
                    DataTransferType::Standard,
                    trasnfer::control::Recipient::Device,
                ),
                request: bRequest::GetDescriptor,
                index: 0,
                value: crate::usb::descriptors::construct_control_transfer_type(
                    USBStandardDescriptorTypes::Device as u8,
                    0,
                )
                .bits(),
                data: Some((buffer.addr() as usize, buffer.length_for_bytes())),
                response: false,
            },
        )
        .unwrap();

        // Log the raw bytes of the descriptor just read
        let mut temp_data_log = [0u8; 8];
        temp_data_log.copy_from_slice(&buffer[0..8]);
        info!("{TAG} GET_DESCRIPTOR (Device) response raw data (first 8 bytes): {:02X?}", temp_data_log);

        let mut data = [0u8; 8];
        data[..8].copy_from_slice(&buffer);
        info!("got {:?}", data);
        data.last()
            .and_then(|len| Some(if *len == 0 { 8u8 } else { *len }))
            .unwrap()
    }

    fn set_ep0_packet_size(&mut self, dev_slot_id: usize, max_packet_size: u16) {
        let addr = {
            let input = self.dev_ctx.device_input_context_list[dev_slot_id as usize].deref_mut();
            input
                .device_mut()
                .endpoint_mut(1) //dci=1: endpoint 0
                .set_max_packet_size(max_packet_size);

            info!(
                "CMD: evaluating context for set endpoint0 packet size {}",
                max_packet_size
            );
            (input as *mut Input<16>).addr() as u64
        };
        self.post_cmd(command::Allowed::EvaluateContext(
            *command::EvaluateContext::default()
                .set_slot_id(dev_slot_id as _)
                .set_input_context_pointer(addr),
        ))
        .unwrap();
    }

    fn interrupt_transfer(
        &mut self,
        dev_slot_id: usize,
        urb_req: trasnfer::interrupt::InterruptTransfer,
    ) -> crate::err::Result<UCB<O>> {
        // 使用新的异步中断传输方法
        let future = self.interrupt_transfer_async(dev_slot_id, urb_req);
        
        // 阻塞等待Future完成
        self.block_on_transfer_future(future)
    }

    fn extra_step(&mut self, dev_slot_id: usize, urb_req: ExtraStep) -> crate::err::Result<UCB<O>> {
        match urb_req {
            ExtraStep::PrepareForTransfer(dci) => {
                if dci == 0xFF { // 特殊值，用于触发DCI 1(EP 0)的重置 (ResetEndpoint)
                    info!("{TAG} Received special ExtraStep to reset endpoint DCI 1 (Control EP) for slot_id {}", dev_slot_id);
                    let reset_ep_cmd = command::Allowed::ResetEndpoint(
                        *command::ResetEndpoint::new()
                            .set_slot_id(dev_slot_id as u8) 
                            .set_endpoint_id(1) // 控制端点0的DCI是1
                            .set_transfer_state_preserve() 
                    );
                    match self.post_cmd(reset_ep_cmd) {
                        Ok(_) => {
                            info!("{TAG} Reset Endpoint DCI 1 command for slot_id {} posted successfully.", dev_slot_id);
                            Ok(UCB::<O>::new(CompleteCode::Event(
                                TransferEventCompleteCode::Success(None), // Command success, no specific event TRB ptr
                            )))
                        }
                        Err(e) => {
                            error!("{TAG} Failed to post Reset Endpoint DCI 1 command for slot_id {}: {:?}", dev_slot_id, e);
                            Err(e)
                        }
                    }
                } else if dci == 0xFE { // UVC "Configure ISOC IN DCI" 信号
                    // 此处之前硬编码为 DCI 5。实际的 UVC ISOC IN 端点是 0x81 -> DCI 3。
                    // Corrected: Device descriptor shows ISOC IN EP 0x81 (DCI 3)
                    let target_dci_for_uvc_isoc: usize = 3; 
                    let target_ep_addr_for_uvc_isoc = 0x81; // USB Endpoint Address for EP 1 IN

                    info!("{TAG} Received special ExtraStep to CONFIGURE UVC Isochronous IN endpoint DCI {} (USB EP 0x{:02X}) (Slot {}) with new sequence (Configure first).",
                        target_dci_for_uvc_isoc, target_ep_addr_for_uvc_isoc, dev_slot_id);
                    self.trace_dump_context(dev_slot_id); // 在开始序列前转储上下文

                    // 新增：先清理挂起的事件，避免Stop Endpoint超时
                    info!("{TAG} [ExtraStep 0xFE] Step 0: 清理所有挂起的事件");
                    let cleared_events = self.drain_event_ring();
                    if cleared_events > 0 {
                        info!("{TAG} [ExtraStep 0xFE] 清理了{}个事件", cleared_events);
                    }
                    
                    // Skip deconfigure step - it causes the endpoint to be disabled
                    // We'll just stop it and reconfigure with new ring
                    info!("{TAG} [ExtraStep 0xFE] Skipping deconfigure to avoid disabling the endpoint");
                    
                    // 先停止端点，确保它不在运行状态
                    info!("{TAG} [ExtraStep 0xFE] Step 1: Stopping endpoint DCI {} before reconfiguration", target_dci_for_uvc_isoc);
                    let stop_ep_cmd = command::Allowed::StopEndpoint(
                        *command::StopEndpoint::new()
                            .set_slot_id(dev_slot_id as u8)
                            .set_endpoint_id(target_dci_for_uvc_isoc as u8)
                    );
                    
                    match self.post_cmd(stop_ep_cmd) {
                        Ok(completion) => {
                            if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                                info!("{TAG} [ExtraStep 0xFE] StopEndpoint for DCI {} completed successfully", target_dci_for_uvc_isoc);
                            } else if completion.completion_code().map_or(false, |cc| cc == CompletionCode::ContextStateError) {
                                // Context State Error表示端点已经停止了
                                info!("{TAG} [ExtraStep 0xFE] Endpoint DCI {} already stopped (Context State Error)", target_dci_for_uvc_isoc);
                            } else {
                                warn!("{TAG} [ExtraStep 0xFE] StopEndpoint for DCI {} completed with code: {:?}", 
                                      target_dci_for_uvc_isoc, completion.completion_code());
                            }
                        }
                        Err(e) => {
                            warn!("{TAG} [ExtraStep 0xFE] Error stopping endpoint DCI {}: {:?}. Continuing anyway...", target_dci_for_uvc_isoc, e);
                        }
                    }
                    
                    // 清除任何旧的挂起事件
                    info!("{TAG} [ExtraStep 0xFE] 清理旧的挂起事件");
                    self.handle_pending_interrupts();
                    
                    // 新增：在停止端点后，发送Set TR Dequeue Pointer命令来强制更新环地址
                    info!("{TAG} [ExtraStep 0xFE] Step 1.5: 重置端点的传输环指针");
                    
                    // 完全删除旧环，强制创建新环
                    info!("{TAG} [ExtraStep 0xFE] Removing old ring for DCI {} from transfer_rings", target_dci_for_uvc_isoc);
                    if dev_slot_id < self.dev_ctx.transfer_rings.len() {
                        if let Some(old_ring) = self.dev_ctx.transfer_rings[dev_slot_id].remove(&target_dci_for_uvc_isoc) {
                            info!("{TAG} [ExtraStep 0xFE] Removed old ring at address {:#X}", old_ring.register());
                        }
                    }
                    
                    // 先获取新环的地址
                    let new_ring_addr = {
                        let ring = self.ep_ring_mut(dev_slot_id, target_dci_for_uvc_isoc);
                        ring.register()
                    };
                    
                    // Step 1.5: Reset Endpoint command to clear any internal state
                    info!("{TAG} [ExtraStep 0xFE] Step 1.5: Reset Endpoint for DCI {} to clear any stale state", target_dci_for_uvc_isoc);
                    
                    let mut reset_ep = command::ResetEndpoint::new();
                    reset_ep.set_slot_id(dev_slot_id as u8)
                        .set_endpoint_id(target_dci_for_uvc_isoc as u8)
                        .clear_transfer_state_preserve(); // Don't preserve transfer state
                    
                    let reset_ep_cmd = command::Allowed::ResetEndpoint(reset_ep);
                    
                    match self.post_cmd(reset_ep_cmd) {
                        Ok(completion) => {
                            info!("{TAG} [ExtraStep 0xFE] Reset Endpoint completed with code: {:?}", completion.completion_code());
                            if completion.completion_code().map_or(false, |cc| cc != CompletionCode::Success) {
                                warn!("{TAG} [ExtraStep 0xFE] Reset Endpoint returned non-success code: {:?}", completion.completion_code());
                            }
                        }
                        Err(e) => {
                            warn!("{TAG} [ExtraStep 0xFE] Error resetting endpoint DCI {}: {:?}. Continuing anyway...", target_dci_for_uvc_isoc, e);
                        }
                    }
                    
                    // Step 1.6: 发送Set TR Dequeue Pointer命令
                    let mut set_tr_deq = command::SetTrDequeuePointer::new();
                    set_tr_deq.set_slot_id(dev_slot_id as u8)
                        .set_endpoint_id(target_dci_for_uvc_isoc as u8)
                        .set_new_tr_dequeue_pointer(new_ring_addr) // 只设置地址，不包含DCS位
                        .set_dequeue_cycle_state(); // 单独设置DCS=1
                    
                    let set_tr_deq_cmd = command::Allowed::SetTrDequeuePointer(set_tr_deq);
                    
                    match self.post_cmd(set_tr_deq_cmd) {
                        Ok(completion) => {
                            if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                                info!("{TAG} [ExtraStep 0xFE] SetTrDequeuePointer for DCI {} completed successfully, new addr=0x{:X}", 
                                      target_dci_for_uvc_isoc, new_ring_addr);
                            } else {
                                warn!("{TAG} [ExtraStep 0xFE] SetTrDequeuePointer for DCI {} completed with code: {:?}", 
                                      target_dci_for_uvc_isoc, completion.completion_code());
                            }
                        }
                        Err(e) => {
                            warn!("{TAG} [ExtraStep 0xFE] Error setting TR dequeue pointer for DCI {}: {:?}", target_dci_for_uvc_isoc, e);
                        }
                    }

                    // 2. Configure Endpoint Command (to ADD DCI target_dci_for_uvc_isoc, set TR Deq Ptr, DCS, EP State to Stopped)
                    info!("{TAG} [ExtraStep 0xFE] Step 2: ConfigureEndpoint to ADD/UPDATE DCI {} to Isoch IN, with new TR Deq Ptr.", target_dci_for_uvc_isoc);

                    let new_tr_deq_ptr;
                    let ring_cycle_for_dcs_check;
                    {
                        if dev_slot_id < self.dev_ctx.transfer_rings.len() {
                            if self.dev_ctx.transfer_rings[dev_slot_id].remove(&target_dci_for_uvc_isoc).is_some() {
                                info!("{TAG} [ExtraStep 0xFE - Configure First] Removed existing ring for DCI {} in slot {}", target_dci_for_uvc_isoc, dev_slot_id);
                            }
                        }
                        // 创建/获取环，这个环实例将被用于填充Input Context
                        let mut fresh_isoch_ring_for_input_ctx = self.ep_ring_mut(dev_slot_id, target_dci_for_uvc_isoc);
                        new_tr_deq_ptr = fresh_isoch_ring_for_input_ctx.register();
                        ring_cycle_for_dcs_check = fresh_isoch_ring_for_input_ctx.cycle;
                        info!("{TAG} [ExtraStep 0xFE] Fetched ring for InputCtx: Addr={:#X}, Cycle={}", new_tr_deq_ptr, ring_cycle_for_dcs_check);
                    }
                    info!("{TAG} [ExtraStep 0xFE - Configure First] DCI {} New Ring: TR_Deq_Ptr=0x{:X}, RingCycleBitForDCSCheck={}",
                        target_dci_for_uvc_isoc, new_tr_deq_ptr, ring_cycle_for_dcs_check);

                    if !ring_cycle_for_dcs_check {
                        warn!(
                            "{TAG} [ExtraStep 0xFE - Configure First] DCI {} new ring cycle bit is FALSE. This is unexpected for a new ring intended for DCS=1 via ConfigureEndpoint.",
                            target_dci_for_uvc_isoc
                        );
                    }

                    let input_ctx_ptr = {
                        let input_context_handle = &mut self.dev_ctx.device_input_context_list[dev_slot_id];
                        let input_ref_mut = input_context_handle.deref_mut(); 

                        if let Some(output_ctx_dma) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
                            let output_slot_handler = DeviceHandler::slot(&**output_ctx_dma);
                            let input_slot_ctx_mut = input_ref_mut.device_mut().slot_mut();

                            info!("{TAG} [ExtraStep 0xFE - Copy] Copying fields from OutputSlotContext to InputSlotContext for Slot {}", dev_slot_id);
                            input_slot_ctx_mut.set_route_string(output_slot_handler.route_string());
                            input_slot_ctx_mut.set_speed(output_slot_handler.speed());
                            if output_slot_handler.multi_tt() {
                                input_slot_ctx_mut.set_multi_tt();
                            } else {
                                input_slot_ctx_mut.clear_multi_tt();
                            }
                            if output_slot_handler.hub() {
                                input_slot_ctx_mut.set_hub();
                                input_slot_ctx_mut.set_parent_hub_slot_id(output_slot_handler.parent_hub_slot_id());
                                input_slot_ctx_mut.set_parent_port_number(output_slot_handler.parent_port_number());
                                if output_slot_handler.multi_tt() {
                                    input_slot_ctx_mut.set_tt_think_time(output_slot_handler.tt_think_time());
                                } else {
                                    input_slot_ctx_mut.set_tt_think_time(0); 
                                }
                            } else {
                                input_slot_ctx_mut.clear_hub();
                                input_slot_ctx_mut.set_parent_hub_slot_id(0);
                                input_slot_ctx_mut.set_parent_port_number(0);
                                input_slot_ctx_mut.set_tt_think_time(0);
                            }
                            input_slot_ctx_mut.set_max_exit_latency(output_slot_handler.max_exit_latency());
                            input_slot_ctx_mut.set_root_hub_port_number(output_slot_handler.root_hub_port_number());
                            input_slot_ctx_mut.set_number_of_ports(output_slot_handler.number_of_ports());
                            input_slot_ctx_mut.set_interrupter_target(output_slot_handler.interrupter_target());
                            input_slot_ctx_mut.set_usb_device_address(output_slot_handler.usb_device_address());
                            
                            info!("{TAG} [ExtraStep 0xFE - Copy] Output SlotContext (source for copy):");
                            info!("{TAG}   RouteString: 0x{:X}, Speed: {}, MTT: {}, Hub: {}", output_slot_handler.route_string(), output_slot_handler.speed(), output_slot_handler.multi_tt(), output_slot_handler.hub());
                            info!("{TAG}   MaxExitLatency: {}, RHPort: {}, NumPorts: {}", output_slot_handler.max_exit_latency(), output_slot_handler.root_hub_port_number(), output_slot_handler.number_of_ports());
                            if output_slot_handler.hub() {
                                info!("{TAG}   ParentHubSlotID: {}, ParentPortNum: {}, TTThinkTime: {}", output_slot_handler.parent_hub_slot_id(), output_slot_handler.parent_port_number(), output_slot_handler.tt_think_time());
                            }
                            info!("{TAG}   InterrupterTarget: {}, USBDevAddr: {}", output_slot_handler.interrupter_target(), output_slot_handler.usb_device_address());
                            info!("{TAG}   OutputContextEntries: {}, OutputSlotState: {:?}", output_slot_handler.context_entries(), output_slot_handler.slot_state());

                        } else {
                            warn!("{TAG} [ExtraStep 0xFE - Copy] Could not get OutputSlotContext for Slot {}. Skipping copy.", dev_slot_id);
                        }
                        
                        { 
                            let slot_ctx_mut = input_ref_mut.device_mut().slot_mut();
                            let current_output_slot_entries = match self.dev_ctx.device_out_context_list.get(dev_slot_id) {
                                Some(out_ctx) => DeviceHandler::slot(&**out_ctx).context_entries(),
                                None => 1, 
                            };
                            let new_context_entries = current_output_slot_entries.max(target_dci_for_uvc_isoc as u8 + 1); 
                            
                            info!("{TAG} [ExtraStep 0xFE - Configure First] Current Output SlotContext.ContextEntries = {}. Setting Input SlotContext.ContextEntries to {}",
                                current_output_slot_entries, new_context_entries
                            );
                            slot_ctx_mut.set_context_entries(new_context_entries);
                            
                            info!(
                                "{TAG} [ExtraStep 0xFE - Configure First] Input Slot Context (DCI 0) for Slot {}:\nContextEntries: {}\nRouteString   : 0x{:X}\nRootHubPortNum: {}\nSpeed         : {}", // Comma after the entire concatenated string literal
                                dev_slot_id,
                                slot_ctx_mut.context_entries(),
                                slot_ctx_mut.route_string(),
                                slot_ctx_mut.root_hub_port_number(),
                                slot_ctx_mut.speed()
                            );
                            info!("{TAG}   InputSlotCtx - MTT: {}, Hub: {}", slot_ctx_mut.multi_tt(), slot_ctx_mut.hub());
                            info!("{TAG}   InputSlotCtx - MaxExitLatency: {}, NumPorts: {}", slot_ctx_mut.max_exit_latency(), slot_ctx_mut.number_of_ports());
                            if slot_ctx_mut.hub() { 
                                info!("{TAG}   InputSlotCtx - ParentHubSlotID: {}, ParentPortNum: {}, TTThinkTime: {}", slot_ctx_mut.parent_hub_slot_id(), slot_ctx_mut.parent_port_number(), slot_ctx_mut.tt_think_time());
                            }
                            info!("{TAG}   InputSlotCtx - InterrupterTarget: {}, USBDevAddr: {}", slot_ctx_mut.interrupter_target(), slot_ctx_mut.usb_device_address());
                        } 
                        
                        { 
                            let control_mut = input_ref_mut.control_mut();
                            
                            control_mut.clear_add_context_flag(0); 
                            control_mut.clear_add_context_flag(1); 

                            control_mut.set_add_context_flag(target_dci_for_uvc_isoc as usize);
                            control_mut.clear_drop_context_flag(target_dci_for_uvc_isoc as usize);
    
                            for i in 2..=31 { 
                                if i != target_dci_for_uvc_isoc as usize { 
                                    control_mut.clear_add_context_flag(i);
                                    control_mut.clear_drop_context_flag(i);
                                }
                            }
                            let mut add_flags_val: u32 = 0;
                            for i in 0..32 { 
                                if control_mut.add_context_flag(i) {
                                    add_flags_val |= 1 << i;
                                }
                            }
                            let mut drop_flags_val: u32 = 0;
                            for i in 2..32 { 
                                if control_mut.drop_context_flag(i) {
                                    drop_flags_val |= 1 << i;
                                }
                            }
                            info!(
                                "{TAG} [ExtraStep 0xFE - Configure First] Input Control Context for Slot {}:\nAddContextFlags : 0x{:08X}\nDropContextFlags: 0x{:08X} (Note: Bits 0 and 1 are implicitly 0 for DropFlags)", // Comma after the entire concatenated string literal
                                dev_slot_id,
                                add_flags_val,
                                drop_flags_val
                            );
                            info!("{TAG}   InputCtrlCtx - ConfigValue: {}, InterfaceNum: {}, AltSetting: {}",
                                control_mut.configuration_value(), control_mut.interface_number(), control_mut.alternate_setting());
                        } 
                        
                        { 
                            let ep_ctx_mut = input_ref_mut.device_mut().endpoint_mut(target_dci_for_uvc_isoc as usize);
                            
                            let wMaxPacketSize_from_device_descriptor: u16 = 0x1400;

                            let ep_max_packet_size_val: u16 = wMaxPacketSize_from_device_descriptor & 0x7ff; // 1024
                            let mult_val: u8 = ((wMaxPacketSize_from_device_descriptor >> 11) & 0x3) as u8; // 2 (表示3个transactions per microframe)
                            // 对于High-Speed设备，mult表示每微帧的额外事务数，max_burst_size应为0
                            let ep_max_burst_size_val: u8 = 0; // HS/FS总是0，SS才使用
                            let ep_mult_val: u8 = mult_val;

                                    // dwMaxPayloadTransferSize 从UVC驱动的Probe/Commit协商中获取，此处为6144
        let ep_average_trb_length: u16 = 6144; 

                            let ep_interval: u8 = 1; // 尝试使用1而不是0


                            info!("{TAG} [ExtraStep 0xFE - Configure First] Setting EP Context for DCI {}: MPS={}, MaxBurst={}, Mult={}, AvgTRBLen={}, Interval={}",
                                target_dci_for_uvc_isoc, ep_max_packet_size_val, ep_max_burst_size_val, ep_mult_val, ep_average_trb_length, ep_interval);
                            
                            ep_ctx_mut.set_endpoint_type(xhci::context::EndpointType::IsochIn);
                            ep_ctx_mut.set_max_packet_size(ep_max_packet_size_val); 
                            ep_ctx_mut.set_average_trb_length(ep_average_trb_length); 
                            
                            ep_ctx_mut.set_mult(ep_mult_val);
                            ep_ctx_mut.set_max_burst_size(ep_max_burst_size_val);
                            
                            // Linux驱动通常设置error count为3，允许一些重试
                            ep_ctx_mut.set_error_count(3); 
                            
                            // Linux计算interval的方式：对于HS设备，bInterval-1
                            // 对于FS设备，使用bInterval直接值
                            // 这里假设是HS设备（USB 2.0高速）
                            let linux_style_interval = if ep_interval > 0 { ep_interval - 1 } else { 0 };
                            ep_ctx_mut.set_interval(linux_style_interval);
                            info!("{TAG} 使用Linux风格的interval计算: bInterval={} -> XHCI interval={}", 
                                  ep_interval, linux_style_interval);
                            
                            ep_ctx_mut.set_tr_dequeue_pointer(new_tr_deq_ptr);
                            if ring_cycle_for_dcs_check {
                                ep_ctx_mut.set_dequeue_cycle_state();
                            } else {
                                warn!("{TAG} [ExtraStep 0xFE - Configure First] DCI {} new ring cycle bit is false. Setting DCS=0 for Input Ctx.", target_dci_for_uvc_isoc);
                                ep_ctx_mut.clear_dequeue_cycle_state();
                            }
                            
                            // Linux不会在ConfigureEndpoint时设置Stopped状态
                            // 让控制器自己管理状态转换
                            // ep_ctx_mut.set_endpoint_state(xhci::context::EndpointState::Stopped); 
                            
                            info!(
                                "{TAG} [ExtraStep 0xFE - Configure First] FINAL Input EP Context for DCI {} (Slot {}):\nEP Type     : {:?}\nEP State    : {:?} (in InputContext, for ConfigureEp)\nMaxPktSize  : {}\nMaxBurstSize: {}\nMult        : {}\nAvgTRBLen   : {}\nInterval    : {}\nTR Deq Ptr  : 0x{:X}\nDeqCycleSt  : {}\nErrorCount  : {}", // Comma after the entire concatenated string literal
                                target_dci_for_uvc_isoc, dev_slot_id,
                                ep_ctx_mut.endpoint_type(),
                                ep_ctx_mut.endpoint_state(),
                                ep_ctx_mut.max_packet_size(),
                                ep_ctx_mut.max_burst_size(),
                                ep_ctx_mut.mult(),
                                ep_ctx_mut.average_trb_length(),
                                ep_ctx_mut.interval(),
                                ep_ctx_mut.tr_dequeue_pointer(),
                                ep_ctx_mut.dequeue_cycle_state(),
                                ep_ctx_mut.error_count()
                            );
                        } 
    
                        (input_ref_mut as *const Input<16>).addr() as u64
                    };

                    let config_ep_cmd = command::Allowed::ConfigureEndpoint(
                        *command::ConfigureEndpoint::default()
                            .set_slot_id(dev_slot_id as u8)
                            .set_input_context_pointer(input_ctx_ptr)
                    );

                    match self.post_cmd(config_ep_cmd) {
                        Ok(completion) => {
                            if completion.completion_code().map_or(false, |cc| cc == CompletionCode::Success) {
                                info!("{TAG} [ExtraStep 0xFE - Configure First] ConfigureEndpoint for DCI {} command completed successfully. Dumping context...", target_dci_for_uvc_isoc);
                                self.trace_dump_context(dev_slot_id);
                                
                                // Verify the endpoint context actually got updated
                                if let Some(output_ctx_dma) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
                                    let endpoint_handler = DeviceHandler::endpoint(&**output_ctx_dma, target_dci_for_uvc_isoc as usize);
                                    let actual_tr_deq_ptr = endpoint_handler.tr_dequeue_pointer();
                                    let actual_dcs = endpoint_handler.dequeue_cycle_state();
                                    
                                    // TR_DeqPtr包含了DCS位在最低位，需要清除它来比较地址
                                    let actual_tr_deq_ptr_without_dcs = actual_tr_deq_ptr & !1;
                                    let expected_tr_deq_ptr_with_dcs = if actual_dcs { new_tr_deq_ptr | 1 } else { new_tr_deq_ptr };
                                    
                                    info!("{TAG} [ExtraStep 0xFE - VERIFY] After ConfigureEndpoint, Output Endpoint Context DCI {}:", target_dci_for_uvc_isoc);
                                    info!("{TAG}   TR_DeqPtr: {:#X} (包含DCS位)", actual_tr_deq_ptr);
                                    info!("{TAG}   TR_DeqPtr (不含DCS): {:#X} (expected: {:#X})", actual_tr_deq_ptr_without_dcs, new_tr_deq_ptr);
                                    info!("{TAG}   DCS: {} (expected: 1)", actual_dcs);
                                    info!("{TAG}   EP State: {:?}", endpoint_handler.endpoint_state());
                                    
                                    if actual_tr_deq_ptr_without_dcs != new_tr_deq_ptr {
                                        error!("{TAG} [ExtraStep 0xFE - VERIFY] CRITICAL: Endpoint context was NOT updated with new ring address!");
                                        error!("{TAG}   Controller is still using old ring at {:#X}", actual_tr_deq_ptr);
                                        error!("{TAG}   Expected new ring at {:#X}", new_tr_deq_ptr);
                                        error!("{TAG}   This will cause transfers to fail!");
                                        
                                        // 尝试强制重置端点
                                        warn!("{TAG} 尝试强制重置端点以更新环地址...");
                                        let reset_ep_cmd = command::Allowed::ResetEndpoint(
                                            *command::ResetEndpoint::new()
                                                .set_slot_id(dev_slot_id as u8)
                                                .set_endpoint_id(target_dci_for_uvc_isoc as u8)
                                        );
                                        
                                        if let Err(e) = self.post_cmd(reset_ep_cmd) {
                                            error!("{TAG} 重置端点失败: {:?}", e);
                                        }
                                        
                                        // 尝试再次运行端点命令来启用它
                                        error!("{TAG} [ExtraStep 0xFE - RETRY] 尝试再次发送ConfigureEndpoint命令");
                                        
                                        // 清除添加标志，重新设置
                                        let retry_input_ctx_ptr = {
                                            let input_context_handle = &mut self.dev_ctx.device_input_context_list[dev_slot_id];
                                            let input_ref_mut = input_context_handle.deref_mut();
                                            
                                            // 重新设置标志
                                            for i in 0..32 {
                                                input_ref_mut.control_mut().clear_add_context_flag(i);
                                            }
                                            input_ref_mut.control_mut().set_add_context_flag(target_dci_for_uvc_isoc);
                                            
                                            (input_ref_mut as *const Input<16>).addr() as u64
                                        };
                                        
                                        let retry_config_cmd = command::Allowed::ConfigureEndpoint(
                                            *command::ConfigureEndpoint::default()
                                                .set_slot_id(dev_slot_id as u8)
                                                .set_input_context_pointer(retry_input_ctx_ptr)
                                        );
                                        
                                        match self.post_cmd(retry_config_cmd) {
                                            Ok(completion) => {
                                                info!("{TAG} [ExtraStep 0xFE - RETRY] 重试ConfigureEndpoint完成: {:?}", completion.completion_code());
                                            }
                                            Err(e) => {
                                                error!("{TAG} [ExtraStep 0xFE - RETRY] 重试ConfigureEndpoint失败: {:?}", e);
                                            }
                                        }
                                    }
                                }
                                
                                // 重置等时传输首次标志，以便第一次传输使用ASAP模式
                                self.isoc_first_transfer = true;
                             // 新增日志：检查此时 transfer_rings 中 DCI 3 的环地址
                                if dev_slot_id < self.dev_ctx.transfer_rings.len() {
                                    if let Some(ring_after_config) = self.dev_ctx.transfer_rings[dev_slot_id].get(&target_dci_for_uvc_isoc) {
                                        info!("{TAG} [ExtraStep 0xFE - DEBUG] Ring for DCI {} in transfer_rings after ConfigureEndpoint success. Addr: {:#X}, Cycle: {}",
                                             target_dci_for_uvc_isoc, ring_after_config.register(), ring_after_config.cycle);
                                   } else {
                                        warn!("{TAG} [ExtraStep 0xFE - DEBUG] Ring for DCI {} NOT FOUND in transfer_rings after ConfigureEndpoint success.", target_dci_for_uvc_isoc);
                                   }
                               }
                            } else {
                                error!("{TAG} [ExtraStep 0xFE - Configure First] ConfigureEndpoint for DCI {} command FAILED with code: {:?}. Dumping context.", target_dci_for_uvc_isoc, completion.completion_code());
                                self.trace_dump_context(dev_slot_id);
                                return Err(Error::CMD(completion.completion_code().unwrap_or(CompletionCode::CommandAborted)));
                            }
                        }
                        Err(e_config) => { 
                            error!("{TAG} [ExtraStep 0xFE - Configure First] Error WAITING for ConfigureEndpoint DCI {} completion: {:?}. Dumping context.", target_dci_for_uvc_isoc, e_config);
                            self.trace_dump_context(dev_slot_id);
                            return Err(e_config); 
                        }
                    }
                    
                    let tec = TransferEventCompleteCode::Success(None); // Explicitly create the enum variant with None
                    Ok(UCB::<O>::new(CompleteCode::Event(tec))) // Pass the created enum variant

                } else if dci > 1 { 
                    self.prepare_transfer_normal(dev_slot_id, dci as u8);
                    let tec = TransferEventCompleteCode::Success(None); // General success for prepare_transfer_normal
                    Ok(UCB::<O>::new(CompleteCode::Event(tec)))
                } else {
                    warn!("{TAG} PrepareForTransfer called on control endpoint (DCI {}), which is unusual.", dci);
                    Err(Error::DontDoThatOnControlPipe)
                }
            }
        }
    }

    fn isoch_transfer_no_wait(
        &mut self,
        dev_slot_id: usize,
        urb_req: IsochTransfer,
    ) -> crate::err::Result<()> {
        // 调用带sender的版本，传入None
        self.isoch_transfer_no_wait_with_sender(dev_slot_id, urb_req, None)
    }
    
    fn isoch_transfer_no_wait_with_sender(
        &mut self,
        dev_slot_id: usize,
        urb_req: IsochTransfer,
        sender: Option<Arc<SpinNoIrq<dyn USBSystemDriverModuleInstance<'static, O>>>>,
    ) -> crate::err::Result<()> {
        // 提取参数
        let (addr, len) = urb_req.buffer_addr_len;
        let endpoint_id = urb_req.endpoint_id;
        let num_packets = urb_req.num_packets;
        let packet_size = urb_req.packet_size;
        
        let dci = self.compute_endpoint_dci_number(endpoint_id as u8) as usize;
        
        info!("[XHCI] 提交等时传输（不等待）: slot={}, endpoint=0x{:02x}, DCI={}, buffer=0x{:x}, len={}", 
              dev_slot_id, endpoint_id, dci, addr, len);
        
        // 先构建和提交TRB，获取TRB地址
        // 保存当前的sender，稍后使用
        let saved_sender = sender;
        
        // 获取传输环信息
        let (ring_addr, ring_index, ring_cycle) = {
            let ring = self.ep_ring_mut(dev_slot_id, dci);
            (ring.register(), ring.i, ring.cycle)
        };
        
        // 检查端点是否已配置使用这个环，以及是否出现了硬件停滞的情况
        let needs_configure = if let Some(output_ctx) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
            let endpoint_handler = DeviceHandler::endpoint(&**output_ctx, dci);
            let tr_deq_ptr_raw = endpoint_handler.tr_dequeue_pointer();
            let current_deq_ptr = tr_deq_ptr_raw & !1u64; // 屏蔽DCS位
            let current_dcs = (tr_deq_ptr_raw & 1) != 0;
            
            // 对于等时端点，检查是否已经有成功的传输记录
            let key = (dev_slot_id, dci as usize);
            let has_successful_transfers = if let Some(progress) = self.endpoint_progress.get(&key) {
                progress.completed_events > 0
            } else {
                false
            };
            
            // 打印DCS和cycle比较信息
            trace!("[XHCI] 检查端点状态: TR_DeqPtr=0x{:X}, DCS={}, ring.cycle={}, 已完成事件={}", 
                  current_deq_ptr, current_dcs as u8, ring_cycle as u8, 
                  if has_successful_transfers { "是" } else { "否" });
            
            // 检查是否需要配置
            if current_deq_ptr == 0 {
                // 端点未初始化
                warn!("[XHCI] 端点的TR Dequeue指针为0，需要初始化");
                true
            } else if (current_deq_ptr & !0xF) != (ring_addr & !0xF) {
                // 端点可能使用了不同的环（忽略低4位对齐）
                info!("[XHCI] 端点的TR Dequeue指针(0x{:X})与当前环地址(0x{:X})不匹配", 
                      current_deq_ptr, ring_addr);
                true
            } else if current_dcs != ring_cycle && !has_successful_transfers {
                // 只有在没有成功传输记录时才检查DCS
                warn!("[XHCI] DCS不匹配: 硬件DCS={}, 软件cycle={} - 需要同步", 
                      current_dcs as u8, ring_cycle as u8);
                true
            } else {
                // DCS匹配或已有成功传输
                if current_dcs != ring_cycle && has_successful_transfers {
                    trace!("[XHCI] DCS不匹配但已有成功传输，可能是环循环: 硬件DCS={}, 软件cycle={}", 
                          current_dcs as u8, ring_cycle as u8);
                }
                false
            }
        } else {
            false
        };
        
        // 如果需要，配置端点
        if needs_configure {
            info!("[XHCI] 需要配置端点以使用传输环");
            
            // 停止端点
            let mut stop_ep = command::StopEndpoint::new();
            stop_ep.set_slot_id(dev_slot_id as u8);
            stop_ep.set_endpoint_id(dci as u8);
            
            if let Err(e) = self.post_cmd(command::Allowed::StopEndpoint(stop_ep)) {
                error!("[XHCI] 停止端点失败: {:?}", e);
                return Err(crate::err::Error::InvalidUsbState);
            }
            
            // 添加内存屏障和延迟，确保硬件完成停止操作
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            for _ in 0..10000 {  // 增加10倍延迟
                core::hint::spin_loop();
            }
            
            
            // 设置TR Dequeue指针
            let mut set_tr_deq = command::SetTrDequeuePointer::new();
            set_tr_deq.set_slot_id(dev_slot_id as u8)
                .set_endpoint_id(dci as u8)
                .set_new_tr_dequeue_pointer(ring_addr);
            
            // 设置DCS - 必须明确设置
            let ring_cycle = self.ep_ring_mut(dev_slot_id, dci).cycle;
            if ring_cycle {
                set_tr_deq.set_dequeue_cycle_state();
            } else {
                set_tr_deq.clear_dequeue_cycle_state();
            }
            
            // 打印更详细的DCS和cycle信息
            info!("[XHCI] 设置TR Dequeue指针: addr=0x{:X}, ring.cycle={}, DCS将被设置为{}", 
                  ring_addr, ring_cycle, if ring_cycle { 1 } else { 0 });
            
            if let Err(e) = self.post_cmd(command::Allowed::SetTrDequeuePointer(set_tr_deq)) {
                error!("[XHCI] 设置TR Dequeue指针失败: {:?}", e);
                return Err(crate::err::Error::InvalidUsbState);
            }
            
            info!("[XHCI] 成功配置端点，TR Dequeue指针=0x{:X}, DCS={}", ring_addr, ring_cycle);
            
            // 添加内存屏障，确保所有写操作完成
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            
            // 给硬件更多时间处理SetTRDequeuePointer命令
            // 这是必要的，因为某些硬件需要时间来更新内部状态
            // 在trace模式下，日志输出提供了这个延迟
            for _ in 0..10000 {  // 增加10倍延迟
                core::hint::spin_loop();
            }
            
            info!("[XHCI] 成功配置端点，TR Dequeue指针=0x{:X}, DCS={}", ring_addr, ring_cycle);
            
            // 添加内存屏障，确保所有写操作完成
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            
            for _ in 0..10000 {  // 增加10倍延迟
                core::hint::spin_loop();
            }
            
            // 重新敲门铃，让硬件开始处理新配置的端点
            info!("[XHCI] 配置端点后重新敲门铃，启动硬件处理");
            self.regs.doorbell.update_volatile_at(dev_slot_id, |r| { 
                r.set_doorbell_stream_id(0);
                r.set_doorbell_target(dci as _); 
            });
        }
        
        // 先读取MFINDEX，避免借用冲突
        let mfindex = self.regs.runtime.mfindex.read_volatile().microframe_index();
        let frame_id = ((mfindex + 64) & 0x7FF) as u32; // 加64微帧（8毫秒），给硬件更多准备时间
        
        // 重新获取ring的可变引用
        let ring = self.ep_ring_mut(dev_slot_id, dci);
        
        let mut raw_trb = [0u32; 4];
        raw_trb[0] = addr as u32;
        raw_trb[1] = (addr >> 32) as u32;
        // 修复：使用24位长度字段（0:23），而不是17位
        raw_trb[2] = (len as u32 & 0xFFFFFF); // 传输长度(bits 0:23)
        raw_trb[2] |= 0 << 17; // TD Size为0 (bits 17:21)
        raw_trb[2] |= 0 << 22; // Interrupter Target = 0 (bits 22:31) - 关键修复！
        raw_trb[3] = 5 << 10;  // Type = Isoch (TRB Type = 5)
        raw_trb[3] |= 1 << 5;  // IOC = 1 (Interrupt on Completion)
        
        raw_trb[3] |= 1 << 31; // SIA = 1 (Start Isoch ASAP)
        raw_trb[3] |= frame_id << 20; // Frame ID (bits 20:30)
        
        info!("[XHCI] 等时传输使用SIA模式（立即开始）: 当前MFINDEX={}", mfindex);
        
        let isoch_trb = unsafe { 
            core::mem::transmute::<[u32; 4], transfer::Isoch>(raw_trb) 
        };
        
        // 添加调试信息以验证TRB内容
        trace!("[XHCI] 等时TRB原始数据: [{:08X}, {:08X}, {:08X}, {:08X}]", 
              raw_trb[0], raw_trb[1], raw_trb[2], raw_trb[3]);
        trace!("[XHCI] 解析: addr=0x{:X}, len={} (0x{:X}), Frame ID={} (bits 20-30 = 0x{:03X})", 
              addr, len, len, frame_id, (raw_trb[3] >> 20) & 0x7FF);
        trace!("[XHCI] InterrupterTarget={} (bits 22-31 of DW2 = 0x{:03X})", 
              (raw_trb[2] >> 22) & 0x3FF, (raw_trb[2] >> 22) & 0x3FF);
        
        let trb_ptr = ring.enque_transfer(transfer::Allowed::Isoch(isoch_trb)) as usize;
        trace!("[XHCI] 等时TRB已提交，地址: 0x{:X}", trb_ptr);
        
        // 更新端点进度跟踪 - 增加已提交TRB计数
        let key = (dev_slot_id, dci);
        let now = self.read_mfindex() as u64;
        self.endpoint_progress.entry(key).and_modify(|p| {
            p.submitted_trbs += 1;
            p.last_submit_time = now;
        }).or_insert(EndpointProgress {
            submitted_trbs: 1,
            completed_events: 0,
            last_event_time: now,
            last_submit_time: now,
            no_progress_count: 0,
        });
        
        // 确保TRB写入到内存，使用更强的内存屏障
        fence(Ordering::SeqCst);
        
        // 在提交TRB后，使用TRB地址作为键保存URB信息
        if let Some(sender_ref) = saved_sender {
            let urb_info = PendingIsochUrb {
                sender: sender_ref,
                buffer_addr: addr as u64,
                submitted_at: self.read_mfindex() as u64,
            };
            // 使用TRB地址作为键，这样在事件处理时可以正确匹配
            self.pending_isoch_urbs.insert(trb_ptr as u64, urb_info);
            trace!("[XHCI] 保存了URB信息，TRB地址=0x{:x}，buffer=0x{:x}，等待异步完成", trb_ptr, addr);
        }
        
        // 检查端点状态
        if let Some(output_ctx) = self.dev_ctx.device_out_context_list.get(dev_slot_id) {
            let endpoint_handler = DeviceHandler::endpoint(&**output_ctx, dci);
            let ep_state = endpoint_handler.endpoint_state();
            info!("[XHCI] 端点状态检查: DCI={}, State={:?}", dci, ep_state);
            
            if ep_state != EndpointState::Running {
                warn!("[XHCI] ⚠️ 端点不在Running状态！当前状态: {:?}", ep_state);
                warn!("[XHCI] ⚠️ 这可能导致TRB不被处理");
            }
        }
        
        // 确保同步TRB已写入内存 - 使用最强内存屏障
        fence(Ordering::SeqCst);
        
        // 按门铃通知控制器
        debug!("[XHCI] 敲响同步端点Doorbell[{}], DCI={}", dev_slot_id, dci);
        
        // 重要修复：不要读取Doorbell寄存器！
        // xHCI规范 4.8.2：Doorbell是只写寄存器，读取总是返回0
        
        // 写入前检查DCI值
        if dci == 0 {
            error!("[XHCI] ⚠️ 错误：DCI为0！这将导致doorbell target错误");
            error!("[XHCI] ⚠️ 对于非控制端点，DCI不应该为0");
        }
        
        // 额外的安全检查
        if dev_slot_id == 1 && dci != 1 && dci != 3 {
            warn!("[XHCI] ⚠️ 警告：slot 1的DCI={}可能不正确", dci);
        }
        
        // 写入Doorbell，打印即将写入的值
        info!("[XHCI] 写入Doorbell[{}]: Target={}, StreamID=0", dev_slot_id, dci);
        self.regs.doorbell.update_volatile_at(dev_slot_id, |r| { 
            r.set_doorbell_stream_id(0);  // 明确设置 Stream ID 为 0
            r.set_doorbell_target(dci as _); 
        });
        
        info!("[XHCI] 已敲响doorbell，等待控制器处理端点DCI={}的TRB", dci);
        
        // 通过端点状态来验证控制器是否开始处理
        self.check_endpoint_progress(dev_slot_id, dci);
        
        // 不等待完成，直接返回
        Ok(())
    }
    
    fn isoch_transfer(
        &mut self,
        dev_slot_id: usize,
        urb_req: IsochTransfer,
    ) -> crate::err::Result<UCB<O>> {
        // 使用新的异步等时传输方法
        let future = self.isoch_transfer_async(dev_slot_id, urb_req);
        
        // 阻塞等待Future完成
        self.block_on_transfer_future(future)
    }
    
    fn poll_events(&mut self) {
        // 检查是否有待处理的事件
        let usbsts = self.regs.operational.usbsts.read_volatile();
        let iman = self.regs.interrupter_register_set.interrupter(0).iman.read_volatile();
        
        if usbsts.event_interrupt() || iman.interrupt_pending() {
            // 处理事件环
            let events = self.process_event_ring();
            if events > 0 {
                trace!("{TAG} poll_events: 处理了 {} 个事件", events);
                
                // 清除中断标志
                self.regs.interrupter_register_set.interrupter_mut(0).iman.update_volatile(|im| {
                    im.clear_interrupt_pending();
                });
                self.regs.operational.usbsts.update_volatile(|s| {
                    s.set_0_event_interrupt();
                });
            }
        }
        
        // 处理异步传输超时
        self.handle_async_timeouts((ISOC_TRANSFER_TIMEOUT_MS * 1_000_000) as u128); // 转换为纳秒
    }
    
    fn device_slot_assignment(&mut self) -> usize {
        // 发送Enable Slot命令来分配新的设备槽位
        info!("{TAG} 执行设备槽位分配");
        
        let enable_slot_cmd = command::Allowed::EnableSlot(command::EnableSlot::new());
        
        match self.post_cmd(enable_slot_cmd) {
            Ok(completion) => {
                let slot_id = completion.slot_id() as usize;
                info!("{TAG} 成功分配设备槽位: {}", slot_id);
                slot_id
            }
            Err(e) => {
                error!("{TAG} 设备槽位分配失败: {:?}", e);
                0 // 返回无效的槽位ID
            }
        }
    }
    
    fn get_xhci_arc(&self) -> Option<Arc<SpinNoIrq<crate::host::data_structures::host_controllers::xhci::XHCI<O>>>> {
        // 返回self_arc的克隆
        self.self_arc.clone()
    }
}

// ===== 异步传输支持方法实现 =====

impl<O> XHCI<O>
where
    O: PlatformAbstractions + 'static,
{
    // 设置自引用Arc（在初始化后调用） - 已在上面定义
    // pub fn set_self_arc(&mut self, arc: Arc<SpinNoIrq<XHCI<O>>>) {
    //     self.self_arc = Some(arc);
    // }
    
    /// 提交异步控制传输
    pub fn control_transfer_async(
        &mut self,
        dev_slot_id: usize,
        urb_req: ControlTransfer,
    ) -> TransferFuture<O> {
        // 准备传输TRB
        let direction = urb_req.request_type.direction.clone();
        let buffer = urb_req.data;
        let request_type_value: u8 = urb_req.request_type.clone().into();
        let value = urb_req.value;
        let index = urb_req.index;
        
        info!("{TAG} [control_transfer_async] 控制传输参数: request_type=0x{:02X}, request={:?}, value=0x{:04X}, index=0x{:04X}", 
              request_type_value, urb_req.request, value, index);
    
        let mut len = 0;
        let data_stage = if let Some((addr, length)) = buffer {
            len = length;
            let mut data_trb = transfer::DataStage::default();
            data_trb.set_data_buffer_pointer(addr as u64)
                .set_trb_transfer_length(len as _)
                .set_direction(direction);
            Some(data_trb)
        } else {
            None
        };
    
        let setup_stage = *transfer::SetupStage::default()
            .set_request_type(request_type_value)
            .set_request(urb_req.request.clone() as u8)
            .set_value(value)
            .set_index(index)
            .set_transfer_type({
                if buffer.is_some() {
                    match direction {
                        Direction::In => TransferType::In,
                        Direction::Out => TransferType::Out,
                    }
                } else {
                    TransferType::No
                }
            })
            .set_length(len as u16);
    
        let mut status_stage = *transfer::StatusStage::default().set_interrupt_on_completion();
        if direction == Direction::In && buffer.is_some() {
            status_stage.clear_direction();
        } else {
            status_stage.set_direction();
        }
    
        // 提交TRB到传输环
        let last_trb_ptr;
        {
            // =================================================================
            // 关键修复：从 dev_ctx 获取环，而不是直接从 XHCI 自身。
            // 这确保了我们使用的是最新的、由 context.rs 管理的环实例。
            let ring = self.dev_ctx.get_or_create_ring(dev_slot_id, 1); // DCI 1 for Control EP
            // =================================================================
    
            ring.enque_transfer(setup_stage.into());
            if let Some(data_trb) = data_stage {
                ring.enque_transfer(data_trb.into());
            }
            last_trb_ptr = ring.enque_transfer(status_stage.into()) as u64;
        }
    
        // 创建传输ID
        let transfer_id = TransferId {
            slot_id: dev_slot_id,
            endpoint_id: 1, // Control EP DCI is always 1
            trb_pointer: last_trb_ptr,
        };
        
        // 创建异步传输对象
        let async_transfer = AsyncTransfer {
            id: transfer_id,
            slot_id: dev_slot_id,
            endpoint_dci: 1, // Control EP DCI is always 1
            trb_pointer: last_trb_ptr,
            transfer_type: AsyncTransferType::Control(urb_req),
            state: TransferState::Submitted { submitted_at: current_time() },
            waker: None,
            _phantom: core::marker::PhantomData,
        };
    
        // 添加到异步传输管理器
        self.async_transfers.insert(transfer_id, async_transfer);
    
        // 记录ring状态用于调试
        {
            let ring = self.dev_ctx.get_or_create_ring(dev_slot_id, 1);
            info!("{TAG} [control_transfer_async] 控制传输提交后: ring.i={}, ring.cycle={}, last_trb=0x{:X}", 
                  ring.i, ring.cycle, last_trb_ptr);
        }
        
        // 按门铃
        fence(Ordering::Release);
        self.regs.doorbell.update_volatile_at(dev_slot_id, |r| {
            r.set_doorbell_target(1);
        });
        
        info!("{TAG} [control_transfer_async] 已按门铃，等待传输完成...");
    
        // 创建Future
        TransferFuture {
            id: transfer_id,
            xhci: self.self_arc.as_ref().unwrap().clone(),
        }
    }
    
    /// 提交异步中断传输
    pub fn interrupt_transfer_async(
        &mut self,
        dev_slot_id: usize,
        urb_req: trasnfer::interrupt::InterruptTransfer,
    ) -> TransferFuture<O> {
        let endpoint_id = urb_req.endpoint_id;
        let dci = self.compute_endpoint_dci_number(endpoint_id as u8);
        let (buffer_addr, buffer_len) = urb_req.buffer_addr_len;
        
        // 准备传输TRB
        let mut normal = Normal::default();
        normal.set_data_buffer_pointer(buffer_addr as u64)
            .set_trb_transfer_length(buffer_len as u32)
            .set_td_size(0)
            .set_interrupt_on_completion()
            .set_interrupt_on_short_packet();

        // 提交到传输环
        let trb_pointer = {
            let ring = self.ep_ring_mut(dev_slot_id, dci as usize);
            ring.enque_transfer(normal.into())
        };

        // 创建传输ID
        let transfer_id = TransferId {
            slot_id: dev_slot_id,
            endpoint_id: dci,
            trb_pointer: trb_pointer as u64,
        };

        // 创建异步传输
        let async_transfer = AsyncTransfer {
            id: transfer_id,
            slot_id: dev_slot_id,
            endpoint_dci: dci,
            trb_pointer: trb_pointer as u64,
            transfer_type: AsyncTransferType::Interrupt(urb_req),
            state: TransferState::Submitted { submitted_at: current_time() },
            waker: None,
            _phantom: core::marker::PhantomData,
        };

        // 添加到异步传输管理器
        self.async_transfers.insert(transfer_id, async_transfer);

        // 按门铃
        fence(Ordering::Release);
        self.regs.doorbell.update_volatile_at(dev_slot_id, |r| {
            r.set_doorbell_target(dci);
        });

        // 创建Future
        TransferFuture {
            id: transfer_id,
            xhci: self.self_arc.as_ref().unwrap().clone(),
        }
    }
    
    /// 提交异步等时传输
    pub fn isoch_transfer_async(
        &mut self,
        dev_slot_id: usize,
        urb_req: IsochTransfer,
    ) -> TransferFuture<O> {
        let endpoint_id = urb_req.endpoint_id;
        let dci = self.compute_endpoint_dci_number(endpoint_id as u8);
        let (buffer_addr, buffer_len) = urb_req.buffer_addr_len;
        
        // 准备等时传输TRB
        let mut isoch = transfer::Isoch::default();
        
        let td_size = 0; // 单TRB传输
        
        isoch.set_data_buffer_pointer(buffer_addr as u64)
            .set_trb_transfer_length(buffer_len as u32)
            .set_td_size_or_tbc(td_size)
            .set_interrupt_on_completion()
            .set_interrupt_on_short_packet()
            .set_start_isoch_asap();

        // 提交到传输环
        let trb_pointer = {
            let ring = self.ep_ring_mut(dev_slot_id, dci as usize);
            ring.enque_transfer(isoch.into())
        };

        // 创建传输ID
        let transfer_id = TransferId {
            slot_id: dev_slot_id,
            endpoint_id: dci,
            trb_pointer: trb_pointer as u64,
        };

        // 创建异步传输
        let async_transfer = AsyncTransfer {
            id: transfer_id,
            slot_id: dev_slot_id,
            endpoint_dci: dci,
            trb_pointer: trb_pointer as u64,
            transfer_type: AsyncTransferType::Isoch(urb_req),
            state: TransferState::Submitted { submitted_at: current_time() },
            waker: None,
            _phantom: core::marker::PhantomData,
        };

        // 添加到异步传输管理器
        self.async_transfers.insert(transfer_id, async_transfer);

        // 按门铃
        fence(Ordering::Release);
        self.regs.doorbell.update_volatile_at(dev_slot_id, |r| {
            r.set_doorbell_target(dci as u8);
        });

        // 创建Future
        TransferFuture {
            id: transfer_id,
            xhci: self.self_arc.as_ref().unwrap().clone(),
        }
    }
    
    /// 清理已完成的异步传输
    pub fn cleanup_completed_transfers(&mut self) {
        self.async_transfers.retain(|_, transfer| {
            !matches!(transfer.state, TransferState::Completed(_) | TransferState::Cancelled)
        });
    }
    
    /// 处理超时的异步传输
    pub fn handle_async_timeouts(&mut self, timeout_ns: u128) {
        let current = current_time();
        let mut timed_out_transfers = Vec::new();
        let mut timed_out_commands = Vec::new();
        
        // 检查传输超时
        for (id, transfer) in self.async_transfers.iter() {
            if let TransferState::Submitted { submitted_at } = transfer.state {
                if current.saturating_sub(submitted_at) > timeout_ns {
                    timed_out_transfers.push(*id);
                }
            }
        }
        
        // 检查命令超时
        for (id, command) in self.async_commands.iter() {
            if let CommandState::Submitted { submitted_at } = command.state {
                if current.saturating_sub(submitted_at) > timeout_ns {
                    timed_out_commands.push(*id);
                }
            }
        }
        
        // 处理超时的传输
        for id in timed_out_transfers {
            if let Some(transfer) = self.async_transfers.get_mut(&id) {
                transfer.state = TransferState::Completed(Err(Error::Timeout));
                
                // 唤醒等待的Future
                if let Some(waker) = transfer.waker.take() {
                    waker.wake();
                }
            }
        }
        
        // 处理超时的命令
        for id in timed_out_commands {
            if let Some(command) = self.async_commands.get_mut(&id) {
                command.state = CommandState::Completed(Err(Error::Timeout));
                
                // 唤醒等待的Future
                if let Some(waker) = command.waker.take() {
                    waker.wake();
                }
                
                warn!("{TAG} 命令超时: TRB=0x{:X}", command.trb_pointer);
            }
        }
    }
    
    /// 阻塞等待命令Future完成
    fn block_on_future(&mut self, mut future: CommandFuture<O>) -> crate::err::Result<CommandCompletion> {
        use core::task::{Context, Poll, Waker};
        use core::pin::Pin;
        
        // 创建一个noop waker
        struct NoopWaker;
        
        unsafe fn noop_clone(_: *const ()) -> core::task::RawWaker {
            core::task::RawWaker::new(core::ptr::null(), &NOOP_WAKER_VTABLE)
        }
        
        unsafe fn noop(_: *const ()) {}
        
        const NOOP_WAKER_VTABLE: core::task::RawWakerVTable = core::task::RawWakerVTable::new(
            noop_clone,
            noop,
            noop,
            noop,
        );
        
        let raw_waker = core::task::RawWaker::new(core::ptr::null(), &NOOP_WAKER_VTABLE);
        let waker = unsafe { Waker::from_raw(raw_waker) };
        let mut context = Context::from_waker(&waker);
        
        let start_time = current_time();
        
        loop {
            // 尝试轮询Future
            match Pin::new(&mut future).poll(&mut context) {
                Poll::Ready(result) => {
                    return result;
                },
                Poll::Pending => {
                    // 处理事件环，这可能会推进Future的状态
                    self.process_event_ring();
                    
                    // 处理异步传输超时
                    self.handle_async_timeouts((ISOC_TRANSFER_TIMEOUT_MS * 1_000_000) as u128); // 转换为纳秒
                    
                    // 检查超时
                    if (current_time() - start_time) > 2_000_000_000 { // 2秒超时
                        error!("{TAG} block_on_future: 超时！等待了 {} 秒", 
                               (current_time() - start_time) / 2_000_000_000);
                        return Err(Error::Timeout);
                    }
                    
                    // 更短的等待，提高响应速度
                    for _ in 0..10 { core::hint::spin_loop(); }
                }
            }
        }
    }
    
    /// 阻塞等待传输Future完成
    fn block_on_transfer_future(&mut self, mut future: TransferFuture<O>) -> crate::err::Result<UCB<O>> {
        use core::task::{Context, Poll, Waker};
        use core::pin::Pin;
        
        // 创建一个noop waker
        fn noop_raw_waker() -> core::task::RawWaker {
            core::task::RawWaker::new(
                core::ptr::null(),
                &core::task::RawWakerVTable::new(
                    |_| noop_raw_waker(),
                    |_| {},
                    |_| {},
                    |_| {},
                ),
            )
        }
        
        let waker = unsafe { Waker::from_raw(noop_raw_waker()) };
        let mut cx = Context::from_waker(&waker);
        
        let start_time = current_time();
        let timeout_ns = ISOC_TRANSFER_TIMEOUT_MS as u128 * 1_000_000; // 转换为纳秒
        
        // 我们需要获取Arc引用以便在循环中处理事件
        let xhci_arc = self.self_arc.as_ref().unwrap().clone();
        
        loop {
            // 处理事件环以推进中断处理
            {
                let mut xhci_lock = xhci_arc.lock();
                xhci_lock.process_event_ring();
            }
            
            // 尝试poll Future
            match Pin::new(&mut future).poll(&mut cx) {
                Poll::Ready(result) => return result,
                Poll::Pending => {
                    // 检查超时
                    if current_time().saturating_sub(start_time) > timeout_ns {
                        error!("{TAG} 异步传输超时");
                        return Err(Error::Timeout);
                    }
                    
                    // 短暂让出CPU，等待中断
                    for _ in 0..1000 {
                        core::hint::spin_loop();
                    }
                }
            }
        }
    }
}
