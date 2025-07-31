use alloc::{boxed::Box, collections::binary_heap::Iter, sync::Arc, vec::Vec};
use core::any::Any;
use core::mem::transmute;
use data_structures::host_controllers::{xhci::XHCI, Controller, ControllerArc};
use log::{debug, error, info, trace, warn};
use spinlock::SpinNoIrq;
use xhci::ring::trb::event;

use crate::{
    abstractions::PlatformAbstractions,
    err,
    glue::{
        driver_independent_device_instance::DriverIndependentDeviceInstance,
        ucb::{CompleteCode, TransferEventCompleteCode, UCB},
    },
    usb::{
        self,
        operation::{Configuration, ExtraStep},
        trasnfer::{control::ControlTransfer, interrupt::InterruptTransfer, isoch::IsochTransfer},
        urb::URB,
    },
    USBSystemConfig,
};

pub mod data_structures;

impl<O> USBSystemConfig<O>
where
    O: PlatformAbstractions,
{
    pub fn new(mmio_base_addr: usize, irq_num: u32, irq_priority: u32, os_dep: O) -> Self {
        let base_addr = O::VirtAddr::from(mmio_base_addr);
        Self {
            base_addr,
            irq_num,
            irq_priority,
            os: os_dep,
        }
    }
}

#[derive(Clone)]
pub struct USBHostSystem<O>
where
    O: PlatformAbstractions,
{
    config: Arc<SpinNoIrq<USBSystemConfig<O>>>,
    controller: ControllerArc<O>,
    #[cfg(feature = "xhci")]
    xhci_arc: Option<Arc<SpinNoIrq<XHCI<O>>>>,
}

impl<O> USBHostSystem<O>
where
    O: PlatformAbstractions + 'static,
{
    pub fn new(config: Arc<SpinNoIrq<USBSystemConfig<O>>>) -> crate::err::Result<Self> {
        // 创建XHCI实例并设置self_arc
        #[cfg(feature = "xhci")]
        {
            // 创建唯一的XHCI实例
            let mut xhci = XHCI::new(config.clone());

            // 创建Arc包装的XHCI
            let xhci_arc = Arc::new(SpinNoIrq::new(xhci));

            // 设置self_arc
            xhci_arc.lock().set_self_arc(xhci_arc.clone());

            let controller: Arc<SpinNoIrq<Box<dyn Controller<O> + 'static>>> = {
                // 克隆Arc引用，而不是创建新实例
                let xhci_clone = xhci_arc.clone();

                // 创建一个包装器来实现Controller trait
                struct XHCIControllerWrapper<O: PlatformAbstractions + 'static> {
                    xhci: Arc<SpinNoIrq<XHCI<O>>>,
                }

                impl<O: PlatformAbstractions + 'static> Controller<O> for XHCIControllerWrapper<O> {
                    fn new(_config: Arc<SpinNoIrq<USBSystemConfig<O>>>) -> Self
                    where
                        Self: Sized,
                    {
                        panic!("Should not create wrapper directly")
                    }

                    fn init(&mut self) {
                        self.xhci.lock().init()
                    }

                    fn probe(&mut self) -> Vec<usize> {
                        self.xhci.lock().probe()
                    }

                    fn control_transfer(
                        &mut self,
                        dev_slot_id: usize,
                        urb_req: ControlTransfer,
                    ) -> crate::err::Result<UCB<O>> {
                        self.xhci.lock().control_transfer(dev_slot_id, urb_req)
                    }

                    fn interrupt_transfer(
                        &mut self,
                        dev_slot_id: usize,
                        urb_req: InterruptTransfer,
                    ) -> crate::err::Result<UCB<O>> {
                        self.xhci.lock().interrupt_transfer(dev_slot_id, urb_req)
                    }

                    fn isoch_transfer(
                        &mut self,
                        dev_slot_id: usize,
                        urb_req: IsochTransfer,
                    ) -> crate::err::Result<UCB<O>> {
                        self.xhci.lock().isoch_transfer(dev_slot_id, urb_req)
                    }

                    fn isoch_transfer_no_wait(
                        &mut self,
                        dev_slot_id: usize,
                        urb_req: IsochTransfer,
                    ) -> crate::err::Result<()> {
                        self.xhci
                            .lock()
                            .isoch_transfer_no_wait(dev_slot_id, urb_req)
                    }

                    fn isoch_transfer_no_wait_with_sender(
                        &mut self,
                        dev_slot_id: usize,
                        urb_req: IsochTransfer,
                        sender: Option<Arc<SpinNoIrq<dyn crate::usb::drivers::driverapi::USBSystemDriverModuleInstance<'static, O>>>>,
                    ) -> crate::err::Result<()> {
                        self.xhci.lock().isoch_transfer_no_wait_with_sender(
                            dev_slot_id,
                            urb_req,
                            sender,
                        )
                    }

                    fn configure_device(
                        &mut self,
                        dev_slot_id: usize,
                        urb_req: Configuration,
                    ) -> crate::err::Result<UCB<O>> {
                        self.xhci.lock().configure_device(dev_slot_id, urb_req)
                    }

                    fn extra_step(
                        &mut self,
                        dev_slot_id: usize,
                        urb_req: ExtraStep,
                    ) -> crate::err::Result<UCB<O>> {
                        self.xhci.lock().extra_step(dev_slot_id, urb_req)
                    }

                    fn device_slot_assignment(&mut self) -> usize {
                        self.xhci.lock().device_slot_assignment()
                    }

                    fn address_device(&mut self, slot_id: usize, port_id: usize) {
                        self.xhci.lock().address_device(slot_id, port_id)
                    }

                    fn control_fetch_control_point_packet_size(&mut self, slot_id: usize) -> u8 {
                        self.xhci
                            .lock()
                            .control_fetch_control_point_packet_size(slot_id)
                    }

                    fn set_ep0_packet_size(&mut self, dev_slot_id: usize, max_packet_size: u16) {
                        self.xhci
                            .lock()
                            .set_ep0_packet_size(dev_slot_id, max_packet_size)
                    }

                    fn test_interrupt(&mut self) {
                        self.xhci.lock().test_interrupt()
                    }

                    fn poll_events(&mut self) {
                        self.xhci.lock().poll_events()
                    }
                }

                let wrapper = XHCIControllerWrapper { xhci: xhci_clone };
                let boxed: Box<dyn Controller<O> + 'static> = Box::new(wrapper);
                Arc::new(SpinNoIrq::new(boxed))
            };

            return Ok(Self {
                config,
                controller,
                xhci_arc: Some(xhci_arc),
            });
        }

        #[cfg(not(feature = "xhci"))]
        {
            let controller = Arc::new(SpinNoIrq::new({ panic!("no host controller defined") }));
            Ok(Self { config, controller })
        }
    }

    pub fn init(&self) {
        self.controller.lock().init();
        trace!("controller init complete");
    }

    #[cfg(feature = "xhci")]
    pub fn get_xhci_arc(&self) -> Option<Arc<SpinNoIrq<XHCI<O>>>> {
        self.xhci_arc.clone()
    }

    pub fn probe<F>(&self, consumer: F)
    where
        F: FnMut(DriverIndependentDeviceInstance<O>),
    {
        let mut probe = self.controller.lock().probe();
        probe
            .iter()
            .map(|slot_id| {
                let mut device_instance =
                    DriverIndependentDeviceInstance::new(slot_id.clone(), self.controller.clone());
                #[cfg(feature = "xhci")]
                {
                    device_instance = device_instance.with_xhci_arc(self.xhci_arc.clone());
                }
                device_instance
            })
            .for_each(consumer);
    }

    pub fn control_transfer(
        &mut self,
        dev_slot_id: usize,
        urb_req: ControlTransfer,
    ) -> crate::err::Result<UCB<O>> {
        self.controller
            .lock()
            .control_transfer(dev_slot_id, urb_req)
    }

    pub fn configure_device(
        &mut self,
        dev_slot_id: usize,
        urb_req: Configuration,
    ) -> crate::err::Result<UCB<O>> {
        self.controller
            .lock()
            .configure_device(dev_slot_id, urb_req)
    }

    pub fn urb_request<'a>(&mut self, urb: URB<'a, O>) -> err::Result<UCB<O>> {
        debug!("URB request: slot_id={}", urb.device_slot_id);
        let slot_id = urb.device_slot_id;
        match urb.operation {
            usb::urb::RequestedOperation::Control(control) => {
                trace!("request transfer!");
                self.control_transfer(slot_id, control)
            }
            usb::urb::RequestedOperation::Bulk => todo!(),
            usb::urb::RequestedOperation::Interrupt(interrupt_transfer) => self
                .controller
                .lock()
                .interrupt_transfer(slot_id, interrupt_transfer),
            usb::urb::RequestedOperation::Isoch(isoch) => {
                trace!(
                    "处理等时URB: slot_id={}, endpoint=0x{:02x}, buffer=0x{:x}, len={}",
                    slot_id,
                    isoch.endpoint_id,
                    isoch.buffer_addr_len.0,
                    isoch.buffer_addr_len.1
                );
                trace!(
                    "  - 包数量: {}, 每包大小: {}",
                    isoch.num_packets,
                    isoch.packet_size
                );
                trace!(
                    "  - 总传输大小: {} 字节",
                    isoch.num_packets * isoch.packet_size
                );

                // 取得 sender，并转换为 'static 生命周期以满足接口要求
                let sender_static_opt = urb.sender.as_ref().map(|arc_sender| {
                    // SAFETY: USB 驱动模块实例在整个系统存活期间都有效，
                    //         因此将其生命周期提升为 'static 是安全的。
                    unsafe {
                        transmute::<
                            alloc::sync::Arc<spinlock::SpinNoIrq<dyn crate::usb::drivers::driverapi::USBSystemDriverModuleInstance<'_, O>>>,
                            alloc::sync::Arc<spinlock::SpinNoIrq<dyn crate::usb::drivers::driverapi::USBSystemDriverModuleInstance<'static, O>>>,
                        >(arc_sender.clone())
                    }
                });

                trace!("使用异步模式提交等时传输并传递 sender");
                match self.controller.lock().isoch_transfer_no_wait_with_sender(
                    slot_id,
                    isoch.clone(),
                    sender_static_opt,
                ) {
                    Ok(_) => {
                        // 异步模式立即返回成功，实际完成事件会通过中断处理
                        Ok(crate::glue::ucb::UCB::new(
                            crate::glue::ucb::CompleteCode::Event(
                                crate::glue::ucb::TransferEventCompleteCode::Success(Some(
                                    isoch.buffer_addr_len.0 as u64,
                                )),
                            ),
                        ))
                    }
                    Err(e) => {
                        debug!("异步提交失败: {:?}", e);
                        Err(e)
                    }
                }
            }
            usb::urb::RequestedOperation::ConfigureDevice(configure) => {
                self.controller.lock().configure_device(slot_id, configure)
            }
            usb::urb::RequestedOperation::ExtraStep(step) => {
                self.controller.lock().extra_step(slot_id, step)
            }
        }
    }

    /// 测试中断机制
    pub fn test_interrupt(&mut self) {
        // 调用控制器的测试方法
        self.controller.lock().test_interrupt();
    }

    /// 轮询事件（用于中断不工作时的备用方案）
    pub fn poll_events(&mut self) {
        self.controller.lock().poll_events();
    }

    /// 获取控制器的Arc引用（用于异步模式）
    pub fn get_controller_arc(&self) -> ControllerArc<O> {
        self.controller.clone()
    }

    pub fn tock<'a>(&mut self, todo_list_list: Vec<Vec<URB<'a, O>>>) {
        trace!("tock! 收到 {} 个URB列表", todo_list_list.len());

        // 移除轮询机制 - 事件处理由中断驱动
        // self.poll_events();

        let mut total_urbs = 0;
        todo_list_list.iter().for_each(|list| {
            total_urbs += list.len();
            trace!("  处理URB列表，包含 {} 个URB", list.len());
            list.iter().for_each(|todo| {
                //debug!("tock! req: {:#?}", todo.operation);

                // 暂时保持所有传输都是同步的，避免生命周期问题
                // TODO: 后续优化为真正的异步模式
                if let Ok(ok) = self.urb_request(todo.clone()) {
                    if let Some(sender) = &todo.sender {
                        trace!("URB处理完成，发送回驱动");
                        sender.lock().receive_complete_event(ok);
                    } else {
                        debug!("URB处理失败或没有sender");
                    }
                } else {
                    warn!("URB处理失败或没有sender");
                };
            })
        });

        trace!("tock完成，共处理 {} 个URB", total_urbs);

        // 移除轮询机制
        // self.poll_events();
    }
}
