pub mod xhci;
use core::sync::atomic::{fence, Ordering};

use ::xhci::{
    context::{EndpointType, Input},
    ring::trb::{command, event},
};
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use log::{info, warn};
use spinlock::SpinNoIrq;

use crate::{
    abstractions::{dma::DMA, OSAbstractions, PlatformAbstractions},
    err::Result,
    glue::ucb::UCB,
    usb::{
        operation::{Configuration, ExtraStep},
        trasnfer::{control::ControlTransfer, interrupt::InterruptTransfer, isoch::IsochTransfer},
    },
    USBSystemConfig,
};

pub trait Controller<O>: Send
where
    O: PlatformAbstractions,
{
    fn new(config: Arc<SpinNoIrq<USBSystemConfig<O>>>) -> Self
    where
        Self: Sized;

    fn init(&mut self);
    fn probe(&mut self) -> Vec<usize>;
    fn control_transfer(
        &mut self,
        dev_slot_id: usize,
        urb_req: ControlTransfer,
    ) -> crate::err::Result<UCB<O>>;

    fn interrupt_transfer(
        &mut self,
        dev_slot_id: usize,
        urb_req: InterruptTransfer,
    ) -> crate::err::Result<UCB<O>>;
    
    fn isoch_transfer(
        &mut self,
        dev_slot_id: usize,
        urb_req: IsochTransfer,
    ) -> crate::err::Result<UCB<O>>;
    
    /// 提交等时传输但不等待完成
    fn isoch_transfer_no_wait(
        &mut self,
        dev_slot_id: usize,
        urb_req: IsochTransfer,
    ) -> crate::err::Result<()>;
    
    /// 提交等时传输但不等待完成（带sender信息）
    fn isoch_transfer_no_wait_with_sender(
        &mut self,
        dev_slot_id: usize,
        urb_req: IsochTransfer,
        sender: Option<Arc<SpinNoIrq<dyn crate::usb::drivers::driverapi::USBSystemDriverModuleInstance<'static, O>>>>,
    ) -> crate::err::Result<()>;

    fn configure_device(
        &mut self,
        dev_slot_id: usize,
        urb_req: Configuration,
    ) -> crate::err::Result<UCB<O>>;

    fn extra_step(&mut self, dev_slot_id: usize, urb_req: ExtraStep) -> crate::err::Result<UCB<O>>;

    fn device_slot_assignment(&mut self) -> usize;
    fn address_device(&mut self, slot_id: usize, port_id: usize);
    fn control_fetch_control_point_packet_size(&mut self, slot_id: usize) -> u8;
    fn set_ep0_packet_size(&mut self, dev_slot_id: usize, max_packet_size: u16);
    
    /// 测试中断机制（可选实现）
    fn test_interrupt(&mut self) {
        // 默认实现：什么都不做
        warn!("Controller: test_interrupt not implemented for this controller type");
    }
    
    /// 轮询事件（用于中断不工作时的备用方案）
    fn poll_events(&mut self) {
        // 默认实现：什么都不做
    }
    
    /// 获取xHCI控制器的Arc引用（如果是xHCI控制器的话）
    fn get_xhci_arc(&self) -> Option<Arc<SpinNoIrq<crate::host::data_structures::host_controllers::xhci::XHCI<O>>>> {
        None
    }
}

pub(crate) type ControllerArc<O> = Arc<SpinNoIrq<Box<dyn Controller<O>>>>;
