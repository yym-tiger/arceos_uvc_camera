use alloc::{boxed::Box, format, string::String, sync::Arc, vec, vec::Vec};
use log::{debug, error, info, trace, warn};
use spinlock::SpinNoIrq;

// 全局视频数据缓冲区
pub static VIDEO_STREAM_BUFFER: SpinNoIrq<Option<Vec<u8>>> = SpinNoIrq::new(None);

// 数据回调函数类型
pub type DataCallback = fn(&[u8]);

// 导入新模块
use super::{
    constants::*,
    frame_processor::{MjpegFrameProcessor, FrameState},
    statistics::FrameStatistics,
    camera_state::{UVCInitStage, UVCCameraState, BufferState, IsocURBState, IsocURBInfo},
    control_types::VideoProbeCommitControl,
    transfer_types::{AsyncTransferId, AsyncTransferState, AsyncTransferInfo, AsyncTransferFuture, TransferMode},
};

// 添加时间相关导入
use core::time::Duration;

// 添加Error类型导入
use crate::err::Error;

// 添加异步支持
use alloc::collections::{BTreeMap, BTreeSet};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use crate::{
    abstractions::{dma::DMA, PlatformAbstractions},
    glue::{
        driver_independent_device_instance::DriverIndependentDeviceInstance,
        ucb::{CompleteCode, TransferEventCompleteCode, UCB},
    },
    host::data_structures::MightBeInited,
    usb::{
        descriptors::parser::ParserMetaData,
        drivers::driverapi::{USBSystemDriverModule, USBSystemDriverModuleInstance},
        operation::{self, ExtraStep},
        trasnfer::{
            control::{
                bRequest, bmRequestType, ControlTransfer, DataTransferType, Recipient, UvcRequest,
            },
            interrupt::InterruptTransfer,
            isoch::IsochTransfer,
        },
        urb::{RequestedOperation, URB},
    },
    USBSystemConfig,
};
use xhci::ring::trb::transfer::Direction;


pub struct GenericUVCDriverModule; //TODO: Create annotations to register
pub struct GenericUVCDriver<O>
where
    O: PlatformAbstractions,
{
    config: Arc<SpinNoIrq<USBSystemConfig<O>>>,
    device_slotid: usize,
    config_value: u8,
    interface_value: u8,
    is_initialized: bool,
    vendor_id: u16,
    device_id: u16,

    // 图像缓冲区 - 用于存储一帧数据
    image_buffer: Option<SpinNoIrq<DMA<[u8], O::DMA>>>,

    // 当前图像缓冲区状态
    buffer_state: BufferState,

    // 摄像头状态机
    camera_state: UVCCameraState,

    // 视频流配置
    video_format: u8,     // 当前选择的视频格式 (MJPEG)
    resolution_index: u8, // 当前选择的分辨率索引 (320x240)
    frame_interval: u32,  // 当前选择的帧率 (30fps)

    // 帧数据统计
    frame_counter: usize,      // 已接收帧数
    current_frame_size: usize, // 当前帧大小

    // 等时传输相关
    isoc_endpoint_address: u8,      // 等时传输端点地址
    isoc_alt_setting: u8,           // 等时传输接口设置
    isoc_urbs: Vec<IsocURBInfo<O>>, // 等时传输URB列表
    isoc_active: bool,              // 等时传输是否激活
    current_urb_index: usize,       // 当前正在处理的URB索引

    // MJPEG帧处理相关
    frame_processor: MjpegFrameProcessor, // MJPEG帧处理器
    temp_buffer: Vec<u8>,                 // 临时缓冲区，用于存储跨URB的帧数据
    next_urb_id: usize,                   // 下一个URB ID
    urb_id_map: Vec<(usize, usize)>,      // (URB ID, URB索引)对应表

    // 控制传输缓冲区
    probe_buffer: Option<SpinNoIrq<DMA<[u8], O::DMA>>>, // 探测请求缓冲区
    commit_buffer: Option<SpinNoIrq<DMA<[u8], O::DMA>>>, // 提交请求缓冲区
    probe_response_buffer: Option<SpinNoIrq<DMA<[u8], O::DMA>>>, // 用于GET_CUR(Probe)响应的缓冲区

    // 帧统计信息
    frame_statistics: FrameStatistics,

    // 错误恢复计数器
    consecutive_errors: usize,
    max_consecutive_errors: usize,

    // 添加MissedServiceError计数器
    missed_service_count: usize,

    // USB事务错误计数器
    transaction_error_count: usize,

    // TRB错误计数器
    trb_error_count: usize,

    // 延迟计时器
    last_command_time: u64,

    // 等时传输配置
    current_packet_size: usize,

    // 初始化阶段跟踪
    init_stage: UVCInitStage,

    // 零数据计数器
    consecutive_zero_data_count: usize,

    // URB监控相关字段
    last_stall_check_time: u64, // 上次检查卡住URB的时间

    // 设备电源状态
    device_suspended: bool, // 设备是否处于休眠状态

    // 当前UVC探测设置 (用于 Probe/Commit 序列)
    current_probe_settings: Option<VideoProbeCommitControl>,
    // 新增: 存储上一次成功用于 SET_CUR(Probe) 的参数
    last_successful_probe_params: Option<VideoProbeCommitControl>,

    // 帧边界检测相关
    last_frame_id: Option<u8>,    // 上一个帧ID (FID)
    current_frame_has_data: bool, // 当前帧是否已有数据

    // 异步传输支持
    pub async_transfers: BTreeMap<AsyncTransferId, AsyncTransferInfo>,
    pending_futures: Vec<AsyncTransferId>,
    use_async_mode: bool, // 是否使用异步模式

    // xHCI控制器引用（用于异步传输）
    xhci_controller:
        Option<Arc<SpinNoIrq<crate::host::data_structures::host_controllers::xhci::XHCI<O>>>>,

    // 模式切换相关
    transfer_mode: TransferMode, // 当前传输模式
    mode_switch_pending: bool,   // 是否有模式切换待处理
    init_use_sync: bool,         // 初始化阶段是否强制使用同步模式

    // 立即重新提交URB队列（类似Linux实现）
    pending_resubmit_urbs: Vec<usize>, // 需要立即重新提交的URB索引

    // 新增: TRB 指针 -> URB 索引 映射，用于准确匹配完成事件
    trb_to_urb_map: BTreeMap<u64, usize>,

    // VideoControl中断端点支持
    vc_interrupt_endpoint: Option<u8>, // VideoControl中断端点地址（通常是0x83）
    vc_interrupt_urb: Option<SpinNoIrq<DMA<[u8], O::DMA>>>, // 中断传输缓冲区
    vc_interrupt_active: bool,         // 中断端点是否激活
    vc_last_interrupt_time: u64,       // 上次中断时间
}

impl<'a, O> USBSystemDriverModule<'a, O> for GenericUVCDriverModule
where
    O: PlatformAbstractions + 'static,
{
    fn should_active(
        &self,
        independent_dev: &DriverIndependentDeviceInstance<O>,
        config: Arc<SpinNoIrq<crate::USBSystemConfig<O>>>,
    ) -> Option<Vec<Arc<SpinNoIrq<dyn USBSystemDriverModuleInstance<'a, O>>>>> {
        // 检查是否为UVC设备
        if let MightBeInited::Inited(desc) = &*independent_dev.descriptors {
            // 获取设备ID信息
            let vendor_id = if desc.device.is_empty() {
                0
            } else {
                desc.device[0].data.vendor
            };

            let device_id = if desc.device.is_empty() {
                0
            } else {
                desc.device[0].data.product_id
            };

            // 检查是否为UVC设备 - 只需通过ParserMetaData确认
            let is_uvc = matches!(desc.metadata, ParserMetaData::UVC(_));

            if is_uvc {
                info!(
                    "发现UVC摄像头：VID={:04x}, PID={:04x}",
                    vendor_id, device_id
                );

                let slotid = independent_dev.slotid;
                let config_value = 1; // 默认配置值
                let interface_value = 0; // 默认接口值

                // 创建驱动实例
                let driver = GenericUVCDriver {
                    config: config.clone(),
                    device_slotid: slotid,
                    config_value,
                    interface_value,
                    is_initialized: false,
                    vendor_id,
                    device_id,

                    // 初始化字段
                    image_buffer: None,
                    buffer_state: BufferState::Idle,
                    camera_state: UVCCameraState::Idle,
                    video_format: TARGET_FORMAT_INDEX, // 使用新的目标格式
                    resolution_index: TARGET_FRAME_INDEX, // 使用新的目标帧索引
                    frame_interval: TARGET_FRAME_INTERVAL, // 使用新的目标帧间隔

                    frame_counter: 0,
                    current_frame_size: 0,

                    isoc_endpoint_address: 0x81, // 修正：根据lsusb，视频流端点是0x81
                    isoc_alt_setting: 3, // 初始尝试最高带宽的Alternate Setting 7 (对应3072 payload)
                    isoc_urbs: Vec::new(), // 初始化为空列表
                    isoc_active: false,  // 初始未激活
                    current_urb_index: 0, // 初始URB索引

                    frame_processor: MjpegFrameProcessor::new(),
                    temp_buffer: Vec::new(),
                    next_urb_id: 1,
                    urb_id_map: Vec::new(),

                    probe_buffer: None,
                    commit_buffer: None,
                    probe_response_buffer: None,

                    frame_statistics: FrameStatistics::new(),

                    consecutive_errors: 0,
                    max_consecutive_errors: ERROR_THRESHOLD_RESET,
                    missed_service_count: 0,
                    transaction_error_count: 0,
                    trb_error_count: 0,

                    last_command_time: 0,
                    current_packet_size: ISOC_PACKET_SIZE_ALT1,
                    init_stage: UVCInitStage::Idle,

                    consecutive_zero_data_count: 0,

                    last_stall_check_time: 0,

                    device_suspended: false,
                    current_probe_settings: None,
                    last_successful_probe_params: None, // 初始化新字段

                    last_frame_id: None,
                    current_frame_has_data: false,

                    async_transfers: BTreeMap::new(),
                    pending_futures: Vec::new(),
                    use_async_mode: false, // 默认禁用异步模式，需要手动设置xHCI控制器后启用
                    xhci_controller: None,

                    // 模式切换相关字段
                    transfer_mode: TransferMode::default(), // 默认使用混合模式
                    mode_switch_pending: false,
                    init_use_sync: true, // 初始化阶段强制使用同步模式

                    // 立即重新提交URB队列
                    pending_resubmit_urbs: Vec::new(),

                    // 新增: TRB 指针 -> URB 索引 映射，用于准确匹配完成事件
                    trb_to_urb_map: BTreeMap::new(),

                    // VideoControl中断端点支持
                    vc_interrupt_endpoint: None,
                    vc_interrupt_urb: None,
                    vc_interrupt_active: false,
                    vc_last_interrupt_time: 0,
                };

                // 创建驱动实例的Arc
                let driver_arc = Arc::new(SpinNoIrq::new(driver));

                // 如果有xHCI控制器引用，设置异步模式
                #[cfg(feature = "xhci")]
                if let Some(ref xhci_arc) = independent_dev.xhci_arc {
                    info!("UVC驱动: 检测到xHCI控制器，启用异步模式");
                    driver_arc.lock().set_xhci_controller(xhci_arc.clone());
                }

                Some(vec![driver_arc])
            } else {
                None
            }
        } else {
            None
        }
    }

    // 预加载模块函数
    fn preload_module(&self) {
        info!("加载通用UVC驱动模块");
    }
}

impl<'a, O> GenericUVCDriver<O>
where
    O: PlatformAbstractions + 'static,
{
    const ALT_SETTINGS: [(u8, usize); 7] = [
        (7, 3072), // AltSetting 7: EP 0x81, 3 * 1024 bytes - 最优先尝试
        (6, 2688), // AltSetting 6: EP 0x81, 3 * 896 bytes
        (5, 2048), // AltSetting 5: EP 0x81, 2 * 1024 bytes
        (4, 2304), // AltSetting 4: EP 0x81, 2 * 768 bytes
        (3, 1024), // AltSetting 3: EP 0x81, 1 * 1024 bytes
        (2, 512),  // AltSetting 2: EP 0x81, 1 * 512 bytes
        (1, 128),  // AltSetting 1: EP 0x81, 1 * 128 bytes - 最后选择
    ];

    pub fn new(
        config: Arc<SpinNoIrq<USBSystemConfig<O>>>,
    ) -> Arc<SpinNoIrq<dyn USBSystemDriverModuleInstance<'a, O>>> {
        Arc::new(SpinNoIrq::new(Self {
            config: config.clone(),
            device_slotid: 1,   // 默认值，会在should_active中更新
            config_value: 1,    // 默认值
            interface_value: 0, // 默认值
            is_initialized: false,
            vendor_id: 0,
            device_id: 0,

            // 初始化字段
            image_buffer: None,
            buffer_state: BufferState::Idle,
            camera_state: UVCCameraState::Idle,
            video_format: TARGET_FORMAT_INDEX, // 使用新的目标格式
            resolution_index: TARGET_FRAME_INDEX, // 使用新的目标帧索引
            frame_interval: TARGET_FRAME_INTERVAL, // 使用新的目标帧间隔

            frame_counter: 0,
            current_frame_size: 0,

            isoc_endpoint_address: 0x81, // 修正：根据lsusb，视频流端点是0x81
            isoc_alt_setting: 3,         // 初始尝试最高带宽的Alternate Setting 7 (对应3072 payload)
            isoc_urbs: Vec::new(),       // 初始化为空列表
            isoc_active: false,          // 初始未激活
            current_urb_index: 0,        // 初始URB索引

            frame_processor: MjpegFrameProcessor::new(),
            temp_buffer: Vec::new(),
            next_urb_id: 1,
            urb_id_map: Vec::new(),

            probe_buffer: None,
            commit_buffer: None,
            probe_response_buffer: None,

            frame_statistics: FrameStatistics::new(),

            consecutive_errors: 0,
            max_consecutive_errors: ERROR_THRESHOLD_RESET,
            missed_service_count: 0,
            transaction_error_count: 0,
            trb_error_count: 0,

            last_command_time: 0,
            current_packet_size: ISOC_PACKET_SIZE_ALT1,
            init_stage: UVCInitStage::Idle,

            consecutive_zero_data_count: 0,

            last_stall_check_time: 0,

            device_suspended: false,
            current_probe_settings: None,
            last_successful_probe_params: None, // 初始化新字段

            last_frame_id: None,
            current_frame_has_data: false,

            async_transfers: BTreeMap::new(),
            pending_futures: Vec::new(),
            use_async_mode: false, // 默认禁用异步模式，需要手动设置xHCI控制器后启用
            xhci_controller: None,

            // 模式切换相关字段
            transfer_mode: TransferMode::default(), // 默认使用混合模式
            mode_switch_pending: false,
            init_use_sync: true, // 初始化阶段强制使用同步模式

            // 立即重新提交URB队列
            pending_resubmit_urbs: Vec::new(),

            // 新增: TRB 指针 -> URB 索引 映射，用于准确匹配完成事件
            trb_to_urb_map: BTreeMap::new(),

            // VideoControl中断端点支持
            vc_interrupt_endpoint: None,
            vc_interrupt_urb: None,
            vc_interrupt_active: false,
            vc_last_interrupt_time: 0,
        }))
    }

    // 初始化图像缓冲区
    fn init_image_buffer(&mut self) {
        if self.image_buffer.is_none() {
            info!("初始化图像缓冲区 ({}字节)", IMAGE_BUFFER_SIZE);
            self.image_buffer = Some(SpinNoIrq::new(DMA::zeroed(
                IMAGE_BUFFER_SIZE,
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
            self.buffer_state = BufferState::Idle;
        }
    }

    // 改进的等时传输URB初始化函数 - 增加错误处理和重试机制
    fn robust_init_isoc_urbs(&mut self) -> bool {
        // 清空现有的URB列表和映射表
        self.isoc_urbs.clear();
        self.urb_id_map.clear();

        // 初始化成功标志和失败计数
        let mut success = true;
        let mut fail_count = 0;
        let mut allocated_addresses = Vec::new();

        // 为每个URB分配资源
        for i in 0..ISOC_NUM_URBS {
            // 尝试为URB分配缓冲区
            let buffer_result = match self.allocate_urb_buffer() {
                Some(buffer) => {
                    let addr = buffer.lock().addr();

                    // 检查地址是否重复
                    if allocated_addresses.contains(&addr) {
                        error!(
                            "⚠️ 警告：URB #{}的缓冲区地址0x{:X}与之前分配的地址重复!",
                            i, addr
                        );
                    } else {
                        allocated_addresses.push(addr);
                    }

                    buffer
                }
                None => {
                    error!("❌ 分配URB #{}缓冲区失败", i);
                    fail_count += 1;
                    if fail_count >= ISOC_NUM_URBS / 2 {
                        error!("过多URB缓冲区分配失败，中止初始化");
                        success = false;
                        break;
                    }
                    continue;
                }
            };

            // 分配URB ID
            let urb_id = self.next_urb_id;
            self.next_urb_id += 1;

            // 创建URB信息
            let urb_info = IsocURBInfo {
                state: IsocURBState::Idle,
                buffer: Some(buffer_result),
                index: i,
                urb_id,
                submit_time: 0,
            };

            // 记录URB ID和索引的对应关系
            self.urb_id_map.push((urb_id, i));

            // 添加到URB列表
            self.isoc_urbs.push(urb_info);
        }

        if self.isoc_urbs.is_empty() {
            error!("未能成功分配任何URB缓冲区");
            success = false;
        } else {
            info!("成功初始化 {} 个等时传输URB", self.isoc_urbs.len());
        }

        success
    }

    fn allocate_urb_buffer(&self) -> Option<SpinNoIrq<DMA<[u8], O::DMA>>> {
        const URB_BUFFER_SIZE: usize = 3072; // 3个1024字节的包

        let aligned_size = (URB_BUFFER_SIZE + 63) & !63;

        // 创建多次分配尝试
        for attempt in 1..=3 {
            match SpinNoIrq::new(DMA::zeroed(
                aligned_size,
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )) {
                buffer if !buffer.lock().is_empty() => {
                    // 验证缓冲区地址是否16字节对齐
                    let addr = buffer.lock().addr();
                    let len = buffer.lock().len();

                    // xHCI严格要求64字节对齐以避免TrbError
                    if (addr & 0x3F) != 0 {
                        warn!("URB缓冲区地址未64字节对齐: 0x{:X}，可能导致TrbError", addr);
                        // 继续尝试分配，希望下次能获得对齐的地址
                        continue;
                    }
                    return Some(buffer);
                }
                _ => {
                    // 分配失败，记录并重试
                    warn!(
                        "URB缓冲区分配失败 (尝试 {}/3), 大小={}",
                        attempt, aligned_size
                    );
                }
            }
        }

        error!("❌ URB缓冲区分配彻底失败，所有尝试都失败了");
        None
    }

    // 添加一个检测卡住URB的函数 - 优化版本
    fn check_stalled_urbs(&mut self) {
        let current_time = Self::get_clock_count();
        let mut stalled_count = 0;

        // 检查每个处于Pending状态的URB
        for i in 0..self.isoc_urbs.len() {
            // 安全访问isoc_urbs - 直接使用索引而不是get()
            if i < self.isoc_urbs.len() {
                let urb_state = self.isoc_urbs[i].state.clone();
                let urb_submit_time = self.isoc_urbs[i].submit_time;

                if urb_state == IsocURBState::Pending {
                    // 计算URB已经等待的时间
                    let wait_time = current_time.saturating_sub(urb_submit_time as u64);

                    // 如果URB等待时间超过1秒（原为3秒），认为它已经卡住
                    if wait_time > URB_STALL_TIMEOUT_MS {
                        trace!(
                            "检测到卡住的URB: 索引={}, 提交时间={}, 等待时间={}ms",
                            i,
                            urb_submit_time,
                            wait_time
                        );
                        stalled_count += 1;

                        // 将URB状态重置为Idle，以便重新提交
                        self.isoc_urbs[i].state = IsocURBState::Idle;

                        // 记录错误，以便后续可能需要的错误恢复
                        self.consecutive_errors += 1;

                        // 创建并提交新的URB
                        if let Some(new_urb) = self.create_isoc_urb_for_idx(i) {
                            if let Err(e) = self.submit_urb(new_urb, i) {
                                error!("重新提交卡住的URB #{} 失败: {:?}", i, e);
                            } else {
                                info!("成功重新提交卡住的URB #{}", i);
                            }
                        }
                    }
                }
            }
        }

        if stalled_count > 0 {
            if stalled_count >= ISOC_NUM_URBS / 2 {
                warn!(
                    "大量URB ({}/{}) 卡住，可能需要重置设备",
                    stalled_count, ISOC_NUM_URBS
                );

                // 将错误计数设置为切换阈值，在下一次gather_urb时切换alternate setting
                if self.consecutive_errors < ERROR_THRESHOLD_SWITCH_ALT {
                    self.consecutive_errors = ERROR_THRESHOLD_SWITCH_ALT;
                }
            }
        }
    }

    // 创建视频流格式探测控制URB
    fn create_video_probe_urb(
        &self,
        probe_data: &VideoProbeCommitControl,
        response: bool,
    ) -> URB<'a, O> {
        // 确保缓冲区大小符合UVC 1.0规范
        let buffer_size = 26;

        // 如果probe_buffer未初始化，创建它
        if self.probe_buffer.is_none() {
            warn!("probe_buffer未初始化，无法创建probe URB");
            panic!("probe_buffer必须在调用create_video_probe_urb前初始化");
        }

        // 为GET_CUR准备缓冲区，或为SET_CUR填充缓冲区
        let data_tuple = if let Some(ref buffer) = self.probe_buffer {
            let mut buffer_data = buffer.lock();

            // 填充或清零缓冲区
            if response {
                // 对于GET_CUR请求，确保缓冲区清零
                for i in 0..buffer_size {
                    if i < buffer_data.len() {
                        buffer_data[i] = 0;
                    }
                }
            } else {
                // 对于SET_CUR请求，填充数据
                let bytes = probe_data.as_bytes();
                for i in 0..bytes.len() {
                    if i < buffer_data.len() {
                        buffer_data[i] = bytes[i];
                    }
                }
            }

            Some(buffer.lock().addr_len_tuple())
        } else {
            None
        };

        let data_length = if response {
            buffer_size as u16
        } else {
            buffer_size as u16
        };

        URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    if response {
                        Direction::In
                    } else {
                        Direction::Out
                    },
                    DataTransferType::Class,
                    Recipient::Interface,
                ),
                request: if response {
                    bRequest::from(UvcRequest::GetCur)
                } else {
                    bRequest::from(UvcRequest::SetCur)
                },
                value: (u16::from(UVC_VS_PROBE_CONTROL) << 8), // VS_PROBE_CONTROL
                index: (self.interface_value + 1) as u16,      // 视频流接口
                data: data_tuple,
                response,
            }),
        )
    }

    // 发送VideoPower控制
    fn send_video_power_control(&self, power_mode: u8) -> URB<'a, O> {
        // 创建一个小的缓冲区保存电源模式
        let mut buffer = SpinNoIrq::new(DMA::new_vec(
            0u8,
            1, // 只需要1字节
            O::PAGE_SIZE,
            self.config.lock().os.dma_alloc(),
        ));

        // 设置电源模式值
        {
            let mut data = buffer.lock();
            data[0] = power_mode; // 1 = 活跃模式，2 = 低功耗模式
        }

        let addr_len_tuple = buffer.lock().addr_len_tuple();

        // 创建电源控制URB
        URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::Out,
                    DataTransferType::Class,
                    Recipient::Interface,
                ),
                request: bRequest::from(UvcRequest::SetCur),
                value: (u16::from(UVC_VC_VIDEO_POWER_MODE_CONTROL) << 8),
                index: 0, // 控制接口
                data: Some(addr_len_tuple),
                response: false, // 通常不需要响应
            }),
        )
    }

    // 创建接口设置URB
    fn create_set_interface_urb(&self) -> URB<'a, O> {
        // 创建SET_INTERFACE请求
        URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::Out,
                    DataTransferType::Standard,
                    Recipient::Interface,
                ),
                request: bRequest::SetInterfaceSpec,
                index: 1,                            // 接口1
                value: self.isoc_alt_setting as u16, // alternate_setting
                data: None,
                response: true,
            }),
        )
    }

    /// 创建指定索引的等时传输URB，解决借用冲突问题
    fn create_isoc_urb_for_idx(&self, urb_index: usize) -> Option<URB<'a, O>> {
        if urb_index >= self.isoc_urbs.len() {
            warn!(
                "create_isoc_urb_for_idx: urb_index {} out of bounds (len {})",
                urb_index,
                self.isoc_urbs.len()
            );
            return None;
        }

        let urb_info_state = match self.isoc_urbs.get(urb_index) {
            Some(info) => info.state.clone(),
            None => {
                error!("create_isoc_urb_for_idx: URB at index {} not found (should not happen here due to bounds check)", urb_index);
                return None;
            }
        };

        if urb_info_state != IsocURBState::Idle {
            trace!(
                "create_isoc_urb_for_idx: URB #{} is not Idle (state: {:?}), cannot create URB.",
                urb_index,
                urb_info_state
            );
            return None;
        }

        // 获取缓冲区地址和长度
        let buffer_tuple = match self
            .isoc_urbs
            .get(urb_index)
            .and_then(|info| info.buffer.as_ref())
        {
            Some(buffer_lock) => {
                // 对于等时传输，不需要清零缓冲区
                // USB控制器会直接覆盖整个缓冲区内容
                let tuple = buffer_lock.lock().addr_len_tuple();
                trace!(
                    "URB #{} 缓冲区: 地址=0x{:X}, 长度={}",
                    urb_index,
                    tuple.0,
                    tuple.1
                );

                tuple
            }
            None => {
                error!("create_isoc_urb_for_idx: URB #{} has no buffer!", urb_index);
                return None;
            }
        };

        let buffer_length = buffer_tuple.1;

        // 确保我们使用的是标准的3072字节（3个1024字节的包）
        let single_payload_size = if buffer_length >= 3072 {
            3072 // 使用标准的3072字节
        } else {
            warn!(
                "URB #{} 缓冲区长度 {} 小于3072，使用实际长度",
                urb_index, buffer_length
            );
            buffer_length
        };

        if single_payload_size == 0 {
            error!(
                "create_isoc_urb_for_idx: payload size is 0, cannot create Isoch URB for URB #{}",
                urb_index
            );
            return None;
        }

        let actual_usb_packet_size = 1024; // ** MODIFICATION HERE ** 原为 512

        const SIMPLIFIED_TEST: bool = false;

        let (num_packets_for_xhci, total_length_for_xhci) = if SIMPLIFIED_TEST {
            warn!(
                "【简化测试模式】启用：每个URB只传输1个包({}字节)",
                actual_usb_packet_size
            );
            (1, actual_usb_packet_size) // 只传1个包
        } else if single_payload_size == 3072 {
            (3, single_payload_size) // 固定为3个包
        } else {
            let packets =
                (single_payload_size + actual_usb_packet_size - 1) / actual_usb_packet_size;
            (packets, single_payload_size)
        };

        trace!(
            "创建等时URB #{}: 端点=0x{:02X}, 缓冲区=0x{:X}, 缓冲区大小={}, 使用长度={}, 包数={}, 每包大小={}",
            urb_index,
            self.isoc_endpoint_address,
            buffer_tuple.0, // addr
            buffer_length,  // 实际缓冲区大小
            total_length_for_xhci, // 使用的长度
            num_packets_for_xhci,
            actual_usb_packet_size
        );

        let isoc_transfer = IsochTransfer::new(
            self.isoc_endpoint_address as usize,
            (buffer_tuple.0, total_length_for_xhci), // 使用调整后的长度
            num_packets_for_xhci,
            actual_usb_packet_size, // Pass the actual USB packet size for this EP
        )
        .with_timeout(5000); // 增加到5秒超时，给予更多时间处理

        // 创建新的等时传输URB
        let urb = URB::new(self.device_slotid, RequestedOperation::Isoch(isoc_transfer));

        Some(urb)
    }


    // 重置相机状态
    fn reset_camera_state(&mut self) {
        self.camera_state = UVCCameraState::Idle;
        self.buffer_state = BufferState::Idle;
        self.frame_counter = 0;
        self.current_frame_size = 0;
        // self.frame_data_offset = 0; // This line was causing the error
    }

    /// 获取帧统计信息
    pub fn get_frame_statistics(&self) -> FrameStatistics {
        self.frame_statistics.clone()
    }

    /// 重置帧统计信息
    pub fn reset_frame_statistics(&mut self) {
        self.frame_statistics.reset();
    }

    // 检查是否应该使用异步模式 - 保留向后兼容性
    pub fn should_use_async(&self) -> bool {
        self.use_async_mode && self.xhci_controller.is_some()
    }

    /// 判断当前阶段是否应该使用同步模式
    pub fn should_use_sync_now(&self) -> bool {
        // 如果强制初始化使用同步模式且还未完成初始化
        if self.init_use_sync && self.init_stage != UVCInitStage::Done {
            return true;
        }

        // 使用传输模式的逻辑判断
        let sync_for_stage = self
            .transfer_mode
            .should_use_sync_for_stage(self.init_stage);
        let sync_for_camera = self
            .transfer_mode
            .should_use_sync_for_camera_state(self.camera_state);

        // 两个条件任一为真即使用同步模式
        sync_for_stage || sync_for_camera
    }

    /// 获取当前有效的传输模式
    pub fn get_effective_transfer_mode(&self) -> TransferMode {
        // 如果强制初始化使用同步且未完成初始化，返回同步模式
        if self.init_use_sync && self.init_stage != UVCInitStage::Done {
            return TransferMode::Sync;
        }

        self.transfer_mode
    }

    /// 设置传输模式
    pub fn set_transfer_mode(&mut self, mode: TransferMode) {
        info!("传输模式切换: {:?} -> {:?}", self.transfer_mode, mode);
        self.transfer_mode = mode;
        self.mode_switch_pending = true;

        // 根据新模式更新use_async_mode标志
        match mode {
            TransferMode::Sync => {
                self.use_async_mode = false;
                info!("已切换到同步模式：所有传输都将使用阻塞方式");
            }
            TransferMode::Async => {
                if self.xhci_controller.is_some() {
                    self.use_async_mode = true;
                    info!("已切换到异步模式：所有传输都将使用非阻塞方式");
                } else {
                    warn!("无法切换到异步模式：缺少xHCI控制器支持");
                    self.transfer_mode = TransferMode::Sync;
                    self.use_async_mode = false;
                }
            }
            TransferMode::Hybrid => {
                if self.xhci_controller.is_some() {
                    self.use_async_mode = true; // 设为true，具体使用时根据阶段判断
                    info!("已切换到混合模式：初始化使用同步，数据传输使用异步");
                } else {
                    warn!("无法使用混合模式：缺少xHCI控制器支持，将使用纯同步模式");
                    self.transfer_mode = TransferMode::Sync;
                    self.use_async_mode = false;
                }
            }
        }
    }

    /// 检查是否有模式切换待处理
    pub fn has_pending_mode_switch(&self) -> bool {
        self.mode_switch_pending
    }

    /// 清除模式切换待处理状态
    pub fn clear_pending_mode_switch(&mut self) {
        self.mode_switch_pending = false;
    }

    /// 设置是否在初始化阶段强制使用同步模式
    pub fn set_init_force_sync(&mut self, force_sync: bool) {
        info!("设置初始化强制同步模式: {}", force_sync);
        self.init_use_sync = force_sync;
    }

    /// 执行模式切换（如果有待处理的切换）
    pub fn apply_pending_mode_switch(&mut self) {
        if self.mode_switch_pending {
            trace!("应用待处理的模式切换");
            self.clear_pending_mode_switch();

            // 重新应用传输模式设置
            let current_mode = self.transfer_mode;
            self.set_transfer_mode(current_mode);

            info!(
                "模式切换已完成，当前有效模式: {:?}",
                self.get_effective_transfer_mode()
            );
        }
    }

    /// 测试模式切换功能
    pub fn test_mode_switching(&mut self) {
        info!("====== 测试传输模式切换功能 ======");
        trace!("初始状态:");
        trace!("  - 传输模式: {:?}", self.transfer_mode);
        trace!("  - init_stage: {:?}", self.init_stage);
        trace!("  - camera_state: {:?}", self.camera_state);
        trace!("  - 应使用同步: {}", self.should_use_sync_now());

        // 测试各种模式
        let modes_to_test = [
            TransferMode::Sync,
            TransferMode::Async,
            TransferMode::Hybrid,
        ];

        for mode in modes_to_test.iter() {
            info!("测试模式: {:?}", mode);
            self.set_transfer_mode(*mode);

            trace!("  -> 有效模式: {:?}", self.get_effective_transfer_mode());
            trace!("  -> 应使用同步: {}", self.should_use_sync_now());
            trace!("  -> use_async_mode: {}", self.use_async_mode);

            // 测试不同阶段的行为
            let test_stages = [
                UVCInitStage::Idle,
                UVCInitStage::ProbeGetDefSent,
                UVCInitStage::Done,
            ];
            for stage in test_stages.iter() {
                let sync_for_stage = mode.should_use_sync_for_stage(*stage);
                trace!("    阶段 {:?}: 使用同步={}", stage, sync_for_stage);
            }

            // 测试不同摄像头状态的行为
            let test_states = [
                UVCCameraState::Configuring,
                UVCCameraState::WaitingForFrame,
                UVCCameraState::CollectingFrame,
            ];
            for state in test_states.iter() {
                let sync_for_state = mode.should_use_sync_for_camera_state(*state);
                trace!("    状态 {:?}: 使用同步={}", state, sync_for_state);
            }

            trace!("  ----");
        }

        info!("==============================");
    }

    // 打印调试信息 - 添加帧统计信息
    pub fn print_debug_info(&self) {
        info!("====== UVC摄像头调试信息 ======");
        info!("📷 设备信息:");
        trace!("  - 厂商ID: 0x{:04x}", self.vendor_id);
        trace!("  - 产品ID: 0x{:04x}", self.device_id);
        trace!("  - 设备槽位: {}", self.device_slotid);
        trace!("");

        trace!("📊 状态信息:");
        trace!("  - 摄像头状态: {:?}", self.camera_state);
        trace!("  - 缓冲区状态: {:?}", self.buffer_state);
        trace!("  - 初始化阶段: {:?}", self.init_stage);
        trace!("  - 设备是否休眠: {}", self.device_suspended);
        trace!("  - 传输模式: {:?}", self.transfer_mode);
        trace!("  - 当前有效模式: {:?}", self.get_effective_transfer_mode());
        trace!("  - 应使用同步: {}", self.should_use_sync_now());
        trace!("  - 模式切换待处理: {}", self.mode_switch_pending);
        trace!("  - 初始化强制同步: {}", self.init_use_sync);
        trace!(
            "  - 异步模式标志: {} (xHCI控制器: {})",
            if self.use_async_mode {
                "启用"
            } else {
                "禁用"
            },
            if self.xhci_controller.is_some() {
                "已设置"
            } else {
                "未设置"
            }
        );
        trace!("  - 活跃的异步传输: {}", self.async_transfers.len());
        trace!("");

        info!("🖼️ 视频配置:");
        trace!("  - 分辨率: 320x240 (帧索引: {})", self.resolution_index);
        trace!("  - 格式: MJPEG (格式索引: {})", self.video_format);
        trace!(
            "  - 帧间隔: {} (约 {} FPS)",
            self.frame_interval,
            if self.frame_interval > 0 {
                10000000 / self.frame_interval
            } else {
                0
            }
        );
        trace!("");

        trace!("💾 缓冲区信息:");
        if let Some(buffer) = &self.image_buffer {
            trace!(
                "  - 图像缓冲区: 已分配 ({} 字节)",
                buffer.lock().length_for_bytes()
            );
        } else {
            trace!("  - 图像缓冲区: 未分配");
        }
        trace!("  - 临时缓冲区: {} 字节", self.temp_buffer.len());
        trace!("  - 当前帧大小: {} 字节", self.current_frame_size);
        trace!("");

        trace!("🔄 等时传输信息:");
        trace!(
            "  - 传输状态: {}",
            if self.isoc_active {
                "✅ 激活"
            } else {
                "❌ 未激活"
            }
        );
        trace!("  - 端点地址: 0x{:02x}", self.isoc_endpoint_address);
        trace!("  - 接口设置: AltSetting={}", self.isoc_alt_setting);
        trace!("  - 包大小: {} 字节", self.current_packet_size);
        trace!("  - URB总数: {}", self.isoc_urbs.len());

        // 打印URB状态分布
        let mut idle_count = 0;
        let mut pending_count = 0;
        let mut completed_count = 0;
        for urb in &self.isoc_urbs {
            match urb.state {
                IsocURBState::Idle => idle_count += 1,
                IsocURBState::Pending => pending_count += 1,
                IsocURBState::Completed => completed_count += 1,
            }
        }
        trace!(
            "  - URB状态: 空闲={}, 等待={}, 完成={}",
            idle_count,
            pending_count,
            completed_count
        );
        trace!("");

        trace!("📈 帧处理信息:");
        trace!("  - MJPEG处理状态: {:?}", self.frame_processor.get_state());
        trace!(
            "  - 帧处理器: 开始={}, 结束={}",
            self.frame_processor.start_found,
            self.frame_processor.end_found
        );
        trace!("  - 已接收帧数: {}", self.frame_counter);
        trace!("");

        info!("📊 帧统计信息:");
        trace!("  - 总帧数: {}", self.frame_statistics.total_frames);
        trace!(
            "  - 有效帧数: {} ({:.1}%)",
            self.frame_statistics.valid_frames,
            if self.frame_statistics.total_frames > 0 {
                self.frame_statistics.valid_frames as f32 * 100.0
                    / self.frame_statistics.total_frames as f32
            } else {
                0.0
            }
        );
        trace!("  - 无效帧数: {}", self.frame_statistics.invalid_frames);
        trace!("  - 超大帧数: {}", self.frame_statistics.oversized_frames);
        trace!(
            "  - 不完整帧数: {}",
            self.frame_statistics.incomplete_frames
        );
        trace!(
            "  - 平均帧大小: {} 字节",
            self.frame_statistics.average_frame_size
        );
        trace!("");

        info!("⚠️ 错误信息:");
        trace!(
            "  - 连续错误: {} / {} (切换阈值) / {} (重置阈值)",
            self.consecutive_errors,
            ERROR_THRESHOLD_SWITCH_ALT,
            ERROR_THRESHOLD_RESET
        );
        trace!("  - MissedService计数: {}", self.missed_service_count);
        trace!("  - 事务错误计数: {}", self.transaction_error_count);
        trace!(
            "  - 连续零数据包: {} / {} (最大允许)",
            self.consecutive_zero_data_count,
            MAX_ZERO_DATA_COUNT
        );
        info!("===============================");
    }

    /// 拍照功能 - 公共API方法，用于触发摄像头拍照
    /// 返回值：是否成功触发拍照
    pub fn take_photo(&mut self) -> bool {
        info!("====== 拍照请求 ======");
        info!("当前摄像头状态: {:?}", self.camera_state);
        info!("当前帧计数: {}", self.frame_counter);

        match self.camera_state {
            UVCCameraState::CollectingFrame => {
                info!("✅ 触发拍照请求成功");
                info!("将在下一个完整帧时捕获照片");
                self.camera_state = UVCCameraState::CapturingPhoto;
                true
            }
            _ => {
                warn!(
                    "❌ 拍照失败: 摄像头当前状态({:?})不允许拍照",
                    self.camera_state
                );
                warn!("需要先启动视频流并等待摄像头进入CollectingFrame状态");
                false
            }
        }
    }

    /// 获取照片数据 - 返回照片数据和大小
    /// 返回值：是否成功获取照片，以及照片数据和大小
    pub fn get_photo(&mut self) -> Option<(Vec<u8>, usize)> {
        info!("====== 获取照片数据 ======");
        info!("当前摄像头状态: {:?}", self.camera_state);
        trace!("缓冲区状态: {:?}", self.buffer_state);

        if self.camera_state == UVCCameraState::PhotoCaptured
            && self.buffer_state == BufferState::Locked
        {
            if let Some(buffer) = &self.image_buffer {
                let buf_data = buffer.lock();

                // 复制照片数据
                let mut photo_data = Vec::with_capacity(self.current_frame_size);
                for i in 0..self.current_frame_size {
                    if i < buf_data.len() {
                        photo_data.push(buf_data[i]);
                    }
                }

                info!("✅ 成功获取照片数据");
                trace!("照片大小: {} 字节", self.current_frame_size);

                // 验证照片数据
                if self.current_frame_size >= 4 {
                    trace!("照片数据验证:");
                    trace!(
                        "  - 头部: 0x{:02X}{:02X} (应为0xFFD8)",
                        photo_data[0],
                        photo_data[1]
                    );
                    trace!(
                        "  - 尾部: 0x{:02X}{:02X} (应为0xFFD9)",
                        photo_data[self.current_frame_size - 2],
                        photo_data[self.current_frame_size - 1]
                    );
                }

                // 重置为收集帧状态
                self.camera_state = UVCCameraState::CollectingFrame;
                self.buffer_state = BufferState::Idle;

                info!("照片已成功获取，摄像头恢复到视频流模式");
                info!("========================");

                return Some((photo_data, self.current_frame_size));
            }
        } else {
            warn!("❌ 获取照片失败");
            warn!(
                "摄像头状态: {:?}, 缓冲区状态: {:?}",
                self.camera_state, self.buffer_state
            );
            warn!("需要先调用take_photo()并等待PhotoCaptured状态");
        }

        None
    }

    /// 获取相机状态
    pub fn get_camera_state(&self) -> UVCCameraState {
        self.camera_state.clone()
    }

    /// 获取当前视频流数据（如果有的话）
    /// 返回: Option<(数据, 大小)>
    pub fn get_stream_data(&mut self) -> Option<(Vec<u8>, usize)> {
        // 检查是否在收集帧状态
        if self.camera_state != UVCCameraState::CollectingFrame {
            return None;
        }

        // 如果临时缓冲区有数据，返回它
        if !self.temp_buffer.is_empty() {
            let data = self.temp_buffer.clone();
            let size = data.len();
            self.temp_buffer.clear(); // 清空缓冲区
            return Some((data, size));
        }

        None
    }

    /// 启动视频流
    pub fn start_video_stream(&mut self) -> Result<(), Error> {
        // 检查是否已经在运行
        if self.camera_state == UVCCameraState::WaitingForFrame
            || self.camera_state == UVCCameraState::CollectingFrame
        {
            info!("视频流已经在运行中");
            return Ok(());
        }

        info!("====== 启动视频流 ======");
        info!("开始视频流处理...");
        info!("当前配置:");
        trace!("  - 格式: MJPEG (索引 {})", self.video_format);
        trace!("  - 分辨率: 320x240 (帧索引 {})", self.resolution_index);
        trace!(
            "  - 帧间隔: {} (约 {} FPS)",
            self.frame_interval,
            if self.frame_interval > 0 {
                10000000 / self.frame_interval
            } else {
                0
            }
        );
        trace!("  - 最大有效负载: {} 字节", self.current_packet_size);
        trace!("  - 等时端点: 0x{:02X}", self.isoc_endpoint_address);
        trace!("  - 接口设置: {}", self.isoc_alt_setting);

        // 设置状态为等待帧
        self.camera_state = UVCCameraState::WaitingForFrame;

        // 重置帧边界检测状态
        self.last_frame_id = None;
        self.current_frame_has_data = false;
        self.temp_buffer.clear();

        // 使用可靠的URB初始化函数
        if !self.robust_init_isoc_urbs() {
            error!("初始化等时传输URB失败");
            return Err(Error::Dma);
        }

        // 设置计数器
        let mut urb_submit_count = 0;
        let mut check_counter = 0;

        trace!("提交等时传输URB...");

        // 提交所有URB
        for i in 0..self.isoc_urbs.len() {
            if let Some(urb) = self.create_isoc_urb_for_idx(i) {
                if let Err(e) = self.submit_urb(urb, i) {
                    error!("提交URB #{} 失败: {:?}", i, e);
                    continue;
                }

                // 设置URB状态为Pending
                if let Some(urb_info) = self.isoc_urbs.get_mut(i) {
                    // 先获取当前时间，避免借用冲突
                    let current_time = Self::get_clock_count();
                    urb_info.state = IsocURBState::Pending;
                    urb_info.submit_time = current_time;
                }

                urb_submit_count += 1;
                debug!("成功提交URB #{}", i);
            }

            // 每提交2个URB后检查一次事件处理
            check_counter += 1;
            if check_counter >= 2 {
                self.process_events_non_blocking();
                check_counter = 0;
            }
        }

        info!("✅ 成功提交 {} 个等时传输URB", urb_submit_count);
        info!("摄像头状态: {:?}", self.camera_state);
        trace!("等时传输已激活: {}", self.isoc_active);

        if urb_submit_count == 0 {
            error!("未能提交任何URB，视频流启动失败");
            return Err(Error::Dma);
        }

        // 启动周期性URB检测
        self.start_stall_detection();

        info!("====== 视频流启动完成 ======");
        info!("摄像头现在应该开始发送视频数据...");
        info!("===========================");

        Ok(())
    }

    /// 停止视频流
    pub fn stop_video_stream(&mut self) -> bool {
        if self.camera_state == UVCCameraState::CollectingFrame
            || self.camera_state == UVCCameraState::WaitingForFrame
        {
            info!("停止视频流");
            self.camera_state = UVCCameraState::Idle;
            self.isoc_active = false;
            true
        } else {
            false
        }
    }

    /// 是否有照片可用
    pub fn is_photo_available(&self) -> bool {
        self.camera_state == UVCCameraState::PhotoCaptured
            && self.buffer_state == BufferState::Locked
    }

    // 更新URB状态 - 辅助方法
    fn update_urb_status(&mut self) {
        // 更新最近使用的URB状态为空闲
        let prev_index = if self.current_urb_index == 0 {
            ISOC_NUM_URBS - 1
        } else {
            self.current_urb_index - 1
        };

        if let Some(urb_info) = self.isoc_urbs.get_mut(prev_index) {
            urb_info.state = IsocURBState::Idle; // 将URB状态设为空闲，可以重新使用
        }
    }

    // 输出URB状态信息
    fn debug_urb_status(&self) {
        let mut idle_count = 0;
        let mut pending_count = 0;
        let mut completed_count = 0;

        for urb_info in &self.isoc_urbs {
            match urb_info.state {
                IsocURBState::Idle => idle_count += 1,
                IsocURBState::Pending => pending_count += 1,
                IsocURBState::Completed => completed_count += 1,
            }
        }

        if self.frame_counter < 10 || self.frame_counter % 100 == 0 {
            info!(
                "URB状态: 空闲={}, 等待={}, 完成={}, 总计={}",
                idle_count,
                pending_count,
                completed_count,
                self.isoc_urbs.len()
            );
        }
    }

    // 预提交等时URB，确保数据流连续性
    fn pre_submit_isoc_urbs(&mut self) {
        trace!("预提交等时传输URB，确保数据流连续性");

        // 预先提交多个URB以建立稳定的数据流
        let mut urbs_submitted = 0;
        let max_pre_submit = 4; // 预提交的URB数量

        // 查找空闲的URB并提交
        for i in 0..self.isoc_urbs.len() {
            let idx = (self.current_urb_index + i) % self.isoc_urbs.len();

            // 先创建URB，避免借用冲突
            if let Some(urb) = self.create_isoc_urb_for_idx(idx) {
                // 创建成功后再更新状态
                if let Some(urb_info) = self.isoc_urbs.get_mut(idx) {
                    if urb_info.state == IsocURBState::Idle {
                        urb_info.state = IsocURBState::Pending;
                        urbs_submitted += 1;

                        trace!(
                            "预提交等时URB: 索引={}, 端点=0x{:02x}",
                            idx,
                            self.isoc_endpoint_address
                        );

                        // 在实际应用中这里会提交URB
                        // system.submit_urb(urb);

                        if urbs_submitted >= max_pre_submit {
                            self.current_urb_index = (idx + 1) % self.isoc_urbs.len();
                            break;
                        }
                    }
                }
            }
        }

        trace!("预提交了{}个等时传输URB", urbs_submitted);
    }

    // 尝试下一个alternate设置
    fn try_next_alternate_setting(&mut self) {
        trace!("try_next_alternate_setting: Called. Before: isoc_alt_setting={}, current_packet_size={}", self.isoc_alt_setting, self.current_packet_size);
        info!("尝试切换到下一个alternate setting");

        // 记录当前配置
        let current_alt = self.isoc_alt_setting;
        let current_size = self.current_packet_size;

        // 找到当前配置在列表中的位置
        let current_idx = Self::ALT_SETTINGS
            .iter()
            .position(|&(alt, _)| alt == current_alt)
            .unwrap_or_else(|| {
                warn!(
                    "当前alternate setting {} 在ALT_SETTINGS中未找到，使用默认索引0",
                    current_alt
                );
                0
            });

        // 尝试下一个配置，如果已是最后一个则循环回第一个
        if Self::ALT_SETTINGS.is_empty() {
            error!("ALT_SETTINGS为空，无法切换alternate setting");
            return;
        }
        let next_idx = (current_idx + 1) % Self::ALT_SETTINGS.len();
        let (next_alt, next_size) = Self::ALT_SETTINGS[next_idx];

        // 更新配置
        self.isoc_alt_setting = next_alt;
        self.current_packet_size = next_size;

        info!(
            "从alternate setting {} (packet_size={}) 切换到 {} (packet_size={})",
            current_alt, current_size, next_alt, next_size
        );
        trace!(
            "try_next_alternate_setting: After: isoc_alt_setting={}, current_packet_size={}",
            self.isoc_alt_setting,
            self.current_packet_size
        );

        // 重置所有URB状态以应用新配置
        for urb_info in self.isoc_urbs.iter_mut() {
            urb_info.state = IsocURBState::Idle;
        }

        // 将设备初始化状态设置为需要重新初始化
        self.init_stage = UVCInitStage::InterfaceSetSent;

        // 更新命令时间戳以便后续操作可以正确延迟
        self.update_command_timestamp();
    }

    // 设置xHCI控制器引用
    pub fn set_xhci_controller(
        &mut self,
        controller: Arc<SpinNoIrq<crate::host::data_structures::host_controllers::xhci::XHCI<O>>>,
    ) {
        self.xhci_controller = Some(controller.clone());

        // 根据当前传输模式决定是否启用异步
        match self.transfer_mode {
            TransferMode::Sync => {
                self.use_async_mode = false;
                info!("✅ 已设置xHCI控制器引用，但保持同步模式");
            }
            TransferMode::Async | TransferMode::Hybrid => {
                self.use_async_mode = true;
                info!("✅ 已设置xHCI控制器引用，启用异步传输模式");
                info!("当前传输模式: {:?}", self.transfer_mode);
            }
        }

        // 设置xHCI控制器的self_arc - 已在USBHostSystem中设置
        // controller.lock().set_self_arc(controller.clone());

        info!("xHCI控制器配置完成，传输行为将根据当前阶段和摄像头状态动态决定");
    }

    // 提交异步等时传输
    async fn submit_isoc_async(
        &mut self,
        endpoint: u8,
        buffer: (usize, usize),
        num_packets: usize,
        packet_size: usize,
    ) -> crate::err::Result<CompleteCode> {
        if let Some(ref controller) = self.xhci_controller {
            // 创建等时传输
            let isoc_transfer =
                IsochTransfer::new(endpoint as usize, buffer, num_packets, packet_size)
                    .with_timeout(5000);

            // 调用xHCI的异步等时传输方法
            let future = controller
                .lock()
                .isoch_transfer_async(self.device_slotid, isoc_transfer);

            // 等待传输完成
            match future.await {
                Ok(ucb) => Ok(ucb.code),
                Err(e) => Err(e),
            }
        } else {
            Err(Error::InvalidSlot)
        }
    }

    // 提交异步控制传输
    async fn submit_control_async(
        &mut self,
        control_transfer: ControlTransfer,
    ) -> crate::err::Result<CompleteCode> {
        if let Some(ref controller) = self.xhci_controller {
            // 调用xHCI的异步控制传输方法
            let future = controller
                .lock()
                .control_transfer_async(self.device_slotid, control_transfer);

            // 等待传输完成
            match future.await {
                Ok(ucb) => Ok(ucb.code),
                Err(e) => Err(e),
            }
        } else {
            Err(Error::InvalidSlot)
        }
    }

    // 获取当前时间（毫秒）- 不再使用self引用
    fn get_clock_count() -> u64 {
        // 使用 ArceOS 的时间函数
        // 转换为毫秒以保持与现有代码的兼容性
        (axhal::time::current_time_nanos() / 1_000_000) as u64
    }

    // 修改Alcor Micro处理函数中使用get_current_time方法而不是直接用字段
    fn update_command_timestamp(&mut self) {
        self.last_command_time = Self::get_clock_count();
    }

    // 检查是否经过了指定的延迟时间
    fn has_delay_elapsed(&self) -> bool {
        let current_time = Self::get_clock_count();
        let elapsed_time = current_time.saturating_sub(self.last_command_time);
        elapsed_time >= STANDARD_DELAY_MS as u64
    }

    // 轮询异步传输完成状态
    fn poll_async_transfers(&mut self) {
        if self.should_use_sync_now() || self.xhci_controller.is_none() {
            return;
        }

        // 检查xHCI控制器的异步传输状态
        if let Some(ref controller) = self.xhci_controller {
            let mut xhci = controller.lock();

            // 处理超时的传输（5秒超时，转换为纳秒）
            xhci.handle_async_timeouts(5_000_000_000u128);

            // 清理已完成的传输
            xhci.cleanup_completed_transfers();
        }

        // 检查并更新本地的异步传输状态
        let mut completed_transfers = Vec::new();

        for (id, info) in self.async_transfers.iter() {
            if let AsyncTransferState::Completed(_) = info.state {
                completed_transfers.push(*id);
            }
        }

        // 处理已完成的传输
        for id in completed_transfers {
            if let Some(mut info) = self.async_transfers.remove(&id) {
                // 如果有waker，唤醒等待的任务
                if let Some(waker) = info.waker.take() {
                    waker.wake();
                }

                info!("异步传输 {:?} 已完成", id);
            }
        }
    }

    // 处理零数据包
    fn handle_zero_data(&mut self, urb_idx: usize) -> bool {
        // 处理零数据包的逻辑
        if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
            // 增加零数据计数
            self.consecutive_zero_data_count += 1;

            // 标记为错误
            self.consecutive_errors += 1;
            warn!(
                "接收到全零数据，错误计数: {}，零数据计数: {}",
                self.consecutive_errors, self.consecutive_zero_data_count
            );

            // 更新URB状态为空闲，以便重新使用
            urb_info.state = IsocURBState::Idle;

            // 对于全零数据，增加一个小延迟再重新提交
            // 这给USB控制器更多时间完成DMA写入
            #[cfg(target_arch = "aarch64")]
            unsafe {
                // 延迟约10微秒
                for _ in 0..1000 {
                    core::arch::asm!("nop", options(nostack, preserves_flags));
                }
                // 再次内存屏障，确保后续读取能看到最新数据
                core::arch::asm!("dmb ish", options(nostack, preserves_flags));
            }

            // 如果连续收到过多零数据包，尝试重置
            if self.consecutive_zero_data_count >= MAX_ZERO_DATA_COUNT {
                info!(
                    "连续收到{}个零数据包，尝试重新启动视频流",
                    self.consecutive_zero_data_count
                );
                self.consecutive_zero_data_count = 0;

                // 设置错误计数达到阈值，下次gather_urb将尝试重置
                self.consecutive_errors = ERROR_THRESHOLD_SWITCH_ALT;
            }

            return true; // 已处理错误
        }

        false // 未处理
    }

    // 初始化VideoControl中断端点
    fn init_vc_interrupt_endpoint(&mut self) {
        info!("初始化VideoControl中断端点");

        // 通常VideoControl中断端点是0x83（IN端点，端点号3）
        self.vc_interrupt_endpoint = Some(0x83);

        // 分配中断传输缓冲区（UVC规范定义StatusPacket大小最小为16字节）
        if self.vc_interrupt_urb.is_none() {
            self.vc_interrupt_urb = Some(SpinNoIrq::new(DMA::zeroed(
                64, // 为Status Interrupt Endpoint分配64字节缓冲区
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
            info!("分配VideoControl中断缓冲区成功");
        }

        self.vc_interrupt_active = false;
        self.vc_last_interrupt_time = 0;
    }

    // 启动VideoControl中断端点
    fn start_vc_interrupt_endpoint(&mut self) -> Option<URB<'a, O>> {
        if let (Some(endpoint), Some(ref buffer)) =
            (self.vc_interrupt_endpoint, &self.vc_interrupt_urb)
        {
            if !self.vc_interrupt_active {
                info!("启动VideoControl中断端点 0x{:02x}", endpoint);

                let buffer_locked = buffer.lock();
                let (addr, len) = buffer_locked.addr_len_tuple();

                // 创建中断传输URB
                let interrupt_transfer = InterruptTransfer {
                    endpoint_id: endpoint as usize,
                    buffer_addr_len: (addr, len),
                };

                self.vc_interrupt_active = true;
                self.vc_last_interrupt_time = Self::get_clock_count();

                return Some(URB::new(
                    self.device_slotid,
                    RequestedOperation::Interrupt(interrupt_transfer),
                ));
            }
        }
        None
    }

    // 处理VideoControl中断数据
    fn handle_vc_interrupt_data(&mut self, data: &[u8]) {
        if data.len() >= 2 {
            let bStatusType = data[0];
            let bOriginator = data[1];

            trace!(
                "收到VideoControl中断: StatusType=0x{:02x}, Originator=0x{:02x}",
                bStatusType,
                bOriginator
            );

            // 根据UVC规范处理不同类型的状态中断
            match bStatusType & 0x0F {
                0x00 => trace!("VideoControl状态中断"),
                0x01 => warn!("VideoStreaming错误"),
                _ => warn!("未知的状态中断类型"),
            }

            // 更新最后中断时间
            self.vc_last_interrupt_time = Self::get_clock_count();
        }
    }

    // 添加init_control_buffers函数，确保在使用前初始化UVC控制缓冲区
    fn init_control_buffers(&mut self) {
        // 初始化probe缓冲区
        if self.probe_buffer.is_none() {
            trace!("初始化UVC控制缓冲区 (probe_buffer)");
            self.probe_buffer = Some(SpinNoIrq::new(DMA::zeroed(
                26, // UVC 1.0规范定义的probe/commit结构体大小
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
        }

        // 初始化commit缓冲区
        if self.commit_buffer.is_none() {
            trace!("初始化UVC控制缓冲区 (commit_buffer)");
            self.commit_buffer = Some(SpinNoIrq::new(DMA::zeroed(
                26, // UVC 1.0规范定义的probe/commit结构体大小
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
        }

        // 初始化probe响应缓冲区（新增）
        if self.probe_response_buffer.is_none() {
            trace!("初始化UVC控制缓冲区 (probe_response_buffer)");
            self.probe_response_buffer = Some(SpinNoIrq::new(DMA::zeroed(
                26, // UVC 1.0规范定义的probe/commit结构体大小
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
        }
    }

    // 添加启动视频流函数
    fn start_stream(&self) -> Option<URB<'a, O>> {
        // 创建启动流控制传输
        trace!("发送START_STREAM控制传输");

        let value = VS_STREAM_ENABLE;
        let index = (self.interface_value + 1) as u16; // 视频流接口

        info!(
            "创建开始流控制请求: SetFeature=0x03, 值=0x{:04x}, 索引=0x{:04x}",
            value, index
        );

        Some(URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::Out,
                    DataTransferType::Standard,
                    Recipient::Interface,
                ),
                request: bRequest::SetFeature,
                value,
                index,
                data: None,      // 无数据
                response: false, // 无需响应
            }),
        ))
    }

    // 添加周期性检测卡住URB的方法
    fn start_stall_detection(&mut self) {
        // 记录当前时间
        self.last_stall_check_time = Self::get_clock_count();
        info!("启动URB卡住检测");
    }

    // 修改process_events_non_blocking方法，添加周期性检测卡住URB的逻辑
    fn process_events_non_blocking(&mut self) -> bool {
        // 检查是否需要进行卡住URB检测
        let current_time = Self::get_clock_count();
        if current_time.saturating_sub(self.last_stall_check_time) > 1000 {
            // 每1秒检查一次
            trace!(
                "执行周期性卡住URB检测 (上次检查: {}ms, 当前: {}ms)",
                self.last_stall_check_time,
                current_time
            );
            self.check_stalled_urbs();
            self.last_stall_check_time = current_time;
        }

        // 处理USB事件
        let mut processed = false;

        // 检查是否有卡住的URB需要重置
        for i in 0..self.isoc_urbs.len() {
            // 安全访问isoc_urbs
            if i < self.isoc_urbs.len() {
                // 直接使用索引访问，避免使用get()方法
                let urb_state = self.isoc_urbs[i].state.clone();
                let urb_submit_time = self.isoc_urbs[i].submit_time;

                if urb_state == IsocURBState::Pending {
                    let current_time = Self::get_clock_count();
                    let elapsed = current_time.saturating_sub(urb_submit_time);

                    if elapsed > 2000 {
                        trace!("检测到可能完成的URB #{}", i);

                        // 获取可变引用并更新状态
                        self.isoc_urbs[i].state = IsocURBState::Completed;

                        // 标记为已处理
                        processed = true;

                        // 将URB状态重置为Idle以便重新使用
                        self.isoc_urbs[i].state = IsocURBState::Idle;

                        // 创建并提交新的URB以维持数据流
                        if let Some(new_urb) = self.create_isoc_urb_for_idx(i) {
                            let current_time = Self::get_clock_count();
                            self.isoc_urbs[i].state = IsocURBState::Pending;
                            self.isoc_urbs[i].submit_time = current_time;
                        }
                    }
                }
            }
        }

        processed
    }

    // 添加一个通用的错误处理方法
    fn handle_transfer_error(&mut self, urb_id: u64) {
        info!("====== 传输错误处理 ======");
        trace!(
            "handle_transfer_error: Called for urb_id {}. consecutive_errors before: {}",
            urb_id,
            self.consecutive_errors
        );
        info!("当前摄像头状态: {:?}", self.camera_state);
        info!("当前初始化阶段: {:?}", self.init_stage);

        // 增加错误计数
        self.consecutive_errors += 1;
        info!("错误计数增加到: {}", self.consecutive_errors);

        // 如果出错URB ID已知，重置其状态
        if let Some(urb_idx) = self.urb_id_to_idx(urb_id) {
            trace!("找到出错的URB索引: {}", urb_idx);

            if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
                trace!("重置URB #{} 状态从 {:?} 到 Idle", urb_idx, urb_info.state);
                urb_info.state = IsocURBState::Idle;
            }

            // 创建并提交新URB
            trace!("尝试重新提交URB #{}...", urb_idx);
            if let Some(new_urb) = self.create_isoc_urb_for_idx(urb_idx) {
                if let Err(e) = self.submit_urb(new_urb, urb_idx) {
                    error!("重新提交错误URB #{} 失败: {:?}", urb_idx, e);
                } else {
                    info!("成功重新提交URB #{}", urb_idx);
                }
            }
        } else {
            warn!("无法找到URB ID {} 对应的索引", urb_id);
        }

        // 错误严重程度处理
        if self.consecutive_errors >= ERROR_THRESHOLD_RESET {
            // 严重错误，需要完全重置
            error!("======❌ 严重错误 ======");
            error!(
                "连续错误计数 ({}) 达到重置阈值({})",
                self.consecutive_errors, ERROR_THRESHOLD_RESET
            );
            warn!("执行完全重置...");

            // 重置所有URB状态
            info!("重置所有 {} 个URB的状态", self.isoc_urbs.len());
            for (i, urb_info) in self.isoc_urbs.iter_mut().enumerate() {
                trace!("  URB #{}: {:?} -> Idle", i, urb_info.state);
                urb_info.state = IsocURBState::Idle;
            }

            // 设置摄像头为错误状态，触发下一次gather_urb中的重置处理
            self.camera_state = UVCCameraState::Error;
            info!("已将摄像头状态设置为Error，将在下一次gather_urb中执行完全重置");
            info!("======================");
        } else if self.consecutive_errors >= ERROR_THRESHOLD_SWITCH_ALT {
            // 较严重错误，尝试切换alternate设置
            warn!("====== ⚠️ 中等错误 ======");
            warn!(
                "连续错误计数 ({}) 达到切换阈值({})",
                self.consecutive_errors, ERROR_THRESHOLD_SWITCH_ALT
            );
            warn!("计划在下次切换alternate设置");
            trace!(
                "当前alternate设置: {}, 包大小: {}",
                self.isoc_alt_setting,
                self.current_packet_size
            );
            info!("======================");
            // 实际切换会在gather_urb中进行
        } else {
            info!(
                "错误计数: {}/{} (切换阈值), {}/{} (重置阈值)",
                self.consecutive_errors,
                ERROR_THRESHOLD_SWITCH_ALT,
                self.consecutive_errors,
                ERROR_THRESHOLD_RESET
            );
        }

        info!("错误处理完成");
        info!("=======================");
    }

    /// 从URB ID查找对应的索引
    /// 对于同步(Isochronous)传输事件, urb_id 参数应为事件TRB中报告的数据缓冲区指针 (u64)
    fn urb_id_to_idx(&mut self, trb_or_buf_ptr: u64) -> Option<usize> {
        // 1. 先尝试从 TRB->URB 映射中查找
        if let Some(idx) = self.trb_to_urb_map.get(&trb_or_buf_ptr) {
            return Some(*idx);
        }

        for (idx, urb_info) in self.isoc_urbs.iter().enumerate() {
            if let Some(buffer_lock) = &urb_info.buffer {
                if buffer_lock.lock().addr() as u64 == trb_or_buf_ptr {
                    // 记录映射，后续可直接命中
                    self.trb_to_urb_map.insert(trb_or_buf_ptr, idx);
                    return Some(idx);
                }
            }
        }

        if let Some((idx, _)) = self
            .isoc_urbs
            .iter()
            .enumerate()
            .find(|(_, info)| info.state == IsocURBState::Pending)
        {
            // 建立临时映射，优化下一次查找
            self.trb_to_urb_map.insert(trb_or_buf_ptr, idx);
            warn!(
                "urb_id_to_idx: Fallback to first Pending URB index {} for ptr {:#X}",
                idx, trb_or_buf_ptr
            );
            return Some(idx);
        }

        warn!(
            "urb_id_to_idx: Failed to resolve URB index for ptr {:#X}. No Pending URB found.",
            trb_or_buf_ptr
        );
        None
    }

    // 添加导入
    fn submit_urb(&mut self, urb: URB<'a, O>, idx: usize) -> Result<(), Error> {
        // 记录提交信息
        debug!(
            "提交URB #{} 到端点0x{:02x}",
            idx, self.isoc_endpoint_address
        );

        // 如果当前应该使用异步模式且有xHCI控制器引用，使用异步传输
        if !self.should_use_sync_now() && self.xhci_controller.is_some() {
            info!(
                "使用异步模式提交URB #{} (传输模式: {:?})",
                idx,
                self.get_effective_transfer_mode()
            );

            // 创建异步传输ID
            let transfer_id = AsyncTransferId::new();

            // 创建异步传输信息
            let transfer_info = AsyncTransferInfo {
                id: transfer_id,
                state: AsyncTransferState::Pending,
                waker: None,
            };

            // 保存传输信息
            self.async_transfers.insert(transfer_id, transfer_info);
            self.pending_futures.push(transfer_id);

            // 更新URB状态
            if let Some(urb_info) = self.isoc_urbs.get_mut(idx) {
                let current_time = Self::get_clock_count();
                urb_info.state = IsocURBState::Pending;
                urb_info.submit_time = current_time;
            }

            // 这里暂时还是返回Ok，实际的异步提交会在gather_urb中处理
            return Ok(());
        }

        if let Some(urb_info) = self.isoc_urbs.get_mut(idx) {
            // 先获取当前时间，避免借用冲突
            let current_time = Self::get_clock_count();
            urb_info.state = IsocURBState::Pending;
            urb_info.submit_time = current_time;

            // 记录URB ID到映射表
            let urb_id = urb_info.urb_id;
            if !self.urb_id_map.iter().any(|(id, _)| *id == urb_id) {
                self.urb_id_map.push((urb_id, idx));
            }
        }

        // 这里不直接提交URB，而是依赖USBDriverSystem通过tick方法收集URB

        Ok(())
    }

    // 初始化等时传输URB - 保留兼容性，调用robust_init_isoc_urbs
    fn init_isoc_urbs(&mut self) {
        if !self.robust_init_isoc_urbs() {
            warn!("使用init_isoc_urbs初始化等时传输URB失败，这可能导致驱动工作异常");
        }
    }

    fn get_current_urb_idx(&self) -> Option<usize> {
        // 查找第一个处于Pending状态的URB
        for (i, urb_info) in self.isoc_urbs.iter().enumerate() {
            if urb_info.state == IsocURBState::Pending {
                return Some(i);
            }
        }

        // 如果没有找到Pending状态的URB，使用当前URB索引
        if self.isoc_urbs.len() > 0 {
            return Some(self.current_urb_index);
        }

        None
    }

    fn handle_isoc_transfer_success(&mut self, urb_idx: usize) {
        trace!(
            "handle_isoc_transfer_success: URB Index={}, Camera State={:?}",
            urb_idx,
            self.camera_state
        );

        if self.camera_state != UVCCameraState::CollectingFrame
            && self.camera_state != UVCCameraState::CapturingPhoto
        {
            warn!(
                "收到等时数据但相机状态错误: {:?}，尝试自动修正状态",
                self.camera_state
            );
            // 如果在WaitingForFrame状态，自动转换到CollectingFrame
            if self.camera_state == UVCCameraState::WaitingForFrame {
                trace!("自动从WaitingForFrame转换到CollectingFrame状态");
                self.camera_state = UVCCameraState::CollectingFrame;
            } else {
                if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
                    urb_info.state = IsocURBState::Idle;
                }
                return;
            }
        }

        // 增强的DMA缓冲区同步机制
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

        #[cfg(target_arch = "aarch64")]
        unsafe {
            core::arch::asm!("dsb sy");
        }

        let buffer_data_opt = self.isoc_urbs.get(urb_idx).and_then(|info| {
            info.buffer.as_ref().map(|b| {
                let mut buffer_lock = b.lock();

                #[cfg(target_arch = "aarch64")]
                unsafe {
                    // 数据内存屏障 - 确保之前的所有内存访问完成
                    core::arch::asm!("dmb ish", options(nostack, preserves_flags));
                    let addr = buffer_lock.addr();
                    let len = buffer_lock.len();
                    if len > 0 {
                        // 触摸第一个缓存行
                        let _ = core::ptr::read_volatile(addr as *const u8);
                        // 触摸最后一个缓存行
                        if len > 1 {
                            let _ = core::ptr::read_volatile((addr + len - 1) as *const u8);
                        }
                        core::arch::asm!("dmb ish", options(nostack, preserves_flags));
                    }
                }

                // 读取数据时使用volatile读取，确保从内存读取
                let mut result = Vec::with_capacity(buffer_lock.len());
                unsafe {
                    let src = buffer_lock.addr() as *const u8;
                    for i in 0..buffer_lock.len() {
                        result.push(core::ptr::read_volatile(src.add(i)));
                    }
                }
                result
            })
        });

        if let Some(buffer_data) = buffer_data_opt {
            // 检查是否是全零数据
            let is_all_zero = buffer_data.iter().all(|&b| b == 0);

            if is_all_zero && !buffer_data.is_empty() {
                // 数据全为零，说明USB控制器还没有写入数据
                warn!(
                    "⚠️ URB #{} 收到全零数据（{}字节），可能是时序问题",
                    urb_idx,
                    buffer_data.len()
                );

                // 不处理零数据，但仍然将URB设置为Idle以便重新使用
                if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
                    urb_info.state = IsocURBState::Idle;
                }

                // 增加零数据计数
                self.handle_zero_data(urb_idx);

                // 对于全零数据，尝试延迟重试
                if self.consecutive_zero_data_count < 3 {
                    // 短暂延迟后立即重新提交
                    info!(
                        "全零数据第{}次，准备快速重试",
                        self.consecutive_zero_data_count
                    );
                    self.pending_resubmit_urbs.push(urb_idx);
                }
                return;
            }

            if !buffer_data.is_empty() {
                // 解析UVC头部长度
                let uvc_header_len =
                    if buffer_data.len() >= 2 && buffer_data[0] >= 2 && buffer_data[0] <= 12 {
                        buffer_data[0] as usize
                    } else {
                        0
                    };

                // 跳过UVC头部，只存储实际的JPEG数据
                if buffer_data.len() > uvc_header_len {
                    let jpeg_data = &buffer_data[uvc_header_len..];

                    // 立即将JPEG数据放入全局缓冲区供UDP发送
                    {
                        let mut global_buffer = VIDEO_STREAM_BUFFER.lock();
                        // 如果缓冲区为空，创建新的；否则追加数据
                        match &mut *global_buffer {
                            Some(existing) => {
                                existing.extend_from_slice(jpeg_data);
                                trace!(
                                    "UVC: 追加JPEG数据到VIDEO_STREAM_BUFFER，当前大小: {} 字节",
                                    existing.len()
                                );
                            }
                            None => {
                                *global_buffer = Some(jpeg_data.to_vec());
                                info!(
                                    "UVC: 初始化VIDEO_STREAM_BUFFER，大小: {} 字节",
                                    jpeg_data.len()
                                );
                            }
                        }
                    }
                }

                // 打印接收到的原始数据的前32字节
                // 使用静态变量跟踪每个URB的数据变化
                static mut URB_DATA_TRACKER: [[u8; 64]; 8] = [[0; 64]; 8]; // 增加到64字节
                static mut URB_UPDATE_COUNT: [u32; 8] = [0; 8];

                unsafe {
                    let urb_idx_mod = urb_idx % 8;
                    let mut data_changed = false;

                    // 检查前64字节是否变化（跳过UVC header后的数据）
                    let uvc_header_len =
                        if buffer_data.len() >= 2 && buffer_data[0] >= 2 && buffer_data[0] <= 12 {
                            buffer_data[0] as usize
                        } else {
                            0
                        };

                    // 比较UVC header后的实际视频数据
                    let compare_start = uvc_header_len;
                    let compare_end = (compare_start + 64).min(buffer_data.len());

                    for i in compare_start..compare_end {
                        let tracker_idx = i - compare_start;
                        if tracker_idx < 64
                            && URB_DATA_TRACKER[urb_idx_mod][tracker_idx] != buffer_data[i]
                        {
                            data_changed = true;
                            URB_DATA_TRACKER[urb_idx_mod][tracker_idx] = buffer_data[i];
                        }
                    }

                    if data_changed {
                        URB_UPDATE_COUNT[urb_idx_mod] += 1;
                        trace!(
                            "📦 URB #{} 接收到 {} 字节数据 (✅ 数据已更新，第{}次更新)",
                            urb_idx,
                            buffer_data.len(),
                            URB_UPDATE_COUNT[urb_idx_mod]
                        );
                    } else {
                        trace!(
                            "📦 URB #{} 接收到 {} 字节数据 (⚠️  数据未变化！)",
                            urb_idx,
                            buffer_data.len()
                        );
                    }
                }

                if buffer_data.len() >= 32 {
                    trace!("  前32字节: {:02X?}", &buffer_data[0..32]);
                } else {
                    trace!("  全部数据: {:02X?}", &buffer_data);
                }

                // 解析UVC payload header
                let uvc_header_len =
                    if buffer_data.len() >= 2 && buffer_data[0] >= 2 && buffer_data[0] <= 12 {
                        buffer_data[0] as usize
                    } else {
                        0
                    };

                // UVC payload header位定义 (参考Linux kernel uvcvideo.h)
                const UVC_STREAM_FID: u8 = 0x01; // bit 0: Frame ID - 帧之间切换
                const UVC_STREAM_EOF: u8 = 0x02; // bit 1: End of Frame - 帧结束标志
                const UVC_STREAM_PTS: u8 = 0x04; // bit 2: PTS present
                const UVC_STREAM_SCR: u8 = 0x08; // bit 3: SCR present
                const UVC_STREAM_RES: u8 = 0x10; // bit 4: Reserved
                const UVC_STREAM_STI: u8 = 0x20; // bit 5: Still Image
                const UVC_STREAM_ERR: u8 = 0x40; // bit 6: Error
                const UVC_STREAM_EOH: u8 = 0x80; // bit 7: End of header

                let mut frame_complete = false;
                let mut new_frame_started = false;

                if uvc_header_len >= 2 {
                    let header_flags = buffer_data[1];
                    let fid = (header_flags & UVC_STREAM_FID) != 0;
                    let eof = (header_flags & UVC_STREAM_EOF) != 0;
                    let err = (header_flags & UVC_STREAM_ERR) != 0;

                    // 只在关键事件时打印日志，减少日志噪音
                    if eof
                        || err
                        || (self.last_frame_id.is_some()
                            && self.last_frame_id.unwrap() != (fid as u8))
                    {
                        trace!(
                            "UVC Header: FID={}, EOF={}, ERR={}, PayloadSize={}",
                            fid as u8,
                            eof as u8,
                            err as u8,
                            buffer_data.len() - uvc_header_len
                        );
                    }

                    // 错误位处理
                    // if err {
                    //     // 某些摄像头会错误地总是设置ERR位，即使数据是有效的
                    //     // 根据"首先解决问题"的原则，暂时忽略ERR位
                    //     warn!("⚠️ UVC错误位被设置，但继续处理数据（某些摄像头会错误设置此位）");
                    //     // TODO: 后续可以添加设备特定的quirk机制来处理这种情况
                    //     // self.temp_buffer.clear();
                    //     // self.current_frame_has_data = false;
                    //     // return;
                    // }

                    // FID切换检测 - 新帧开始
                    if let Some(last_fid) = self.last_frame_id {
                        let last_fid_bit = last_fid != 0;
                        if fid != last_fid_bit && self.current_frame_has_data {
                            info!(
                                "FID切换检测到 ({} -> {})，上一帧完成",
                                last_fid_bit as u8, fid as u8
                            );
                            frame_complete = true;
                        }
                    }
                    self.last_frame_id = Some(fid as u8);

                    // EOF标志检测 - 当前帧结束
                    if eof && self.current_frame_has_data {
                        info!("EOF标志检测到，当前帧完成");
                        frame_complete = true;

                        // 对于不正确切换FID的设备，在EOF时强制切换last_fid
                        // 这确保下一个数据包能正确开始新帧
                        if let Some(last_fid) = self.last_frame_id {
                            self.last_frame_id = Some(1 - last_fid);
                        }
                    }

                    // 如果检测到帧完成，处理已收集的数据
                    if frame_complete && !self.temp_buffer.is_empty() {
                        info!("帧完成! 大小: {} 字节", self.temp_buffer.len());

                        // 验证JPEG格式
                        let is_valid_jpeg = self.temp_buffer.len() >= 4
                            && self.temp_buffer[0] == 0xFF
                            && self.temp_buffer[1] == 0xD8
                            && self.temp_buffer[self.temp_buffer.len() - 2] == 0xFF
                            && self.temp_buffer[self.temp_buffer.len() - 1] == 0xD9;

                        if is_valid_jpeg {
                            info!("有效的JPEG帧，保存到图像缓冲区");
                            // 将完整帧复制到图像缓冲区
                            if let Some(buffer) = &self.image_buffer {
                                let mut buffer_guard = buffer.lock();
                                let copy_size = self.temp_buffer.len().min(IMAGE_BUFFER_SIZE);
                                buffer_guard[..copy_size]
                                    .copy_from_slice(&self.temp_buffer[..copy_size]);
                                self.current_frame_size = copy_size;
                                self.frame_counter += 1;
                                self.frame_statistics
                                    .update_with_frame(copy_size, true, false, false);
                            }
                        } else {
                            warn!(
                                "无效的JPEG帧 (SOI={:02X}{:02X}, EOI={:02X}{:02X})",
                                self.temp_buffer.get(0).unwrap_or(&0),
                                self.temp_buffer.get(1).unwrap_or(&0),
                                self.temp_buffer
                                    .get(self.temp_buffer.len().saturating_sub(2))
                                    .unwrap_or(&0),
                                self.temp_buffer
                                    .get(self.temp_buffer.len().saturating_sub(1))
                                    .unwrap_or(&0)
                            );
                            self.frame_statistics.update_with_frame(
                                self.temp_buffer.len(),
                                false,
                                false,
                                false,
                            );
                        }

                        // 清空临时缓冲区，准备下一帧
                        self.temp_buffer.clear();
                        self.current_frame_has_data = false;
                    }
                }

                let payload = if buffer_data.len() > uvc_header_len {
                    &buffer_data[uvc_header_len..]
                } else {
                    &[]
                };

                // 只有当payload不为空时才添加到缓冲区
                if !payload.is_empty() {
                    // 防止缓冲区溢出
                    if self.temp_buffer.len() + payload.len() > MAX_FRAME_SIZE {
                        warn!("帧数据超过最大限制 {} 字节，丢弃当前帧", MAX_FRAME_SIZE);
                        self.temp_buffer.clear();
                        self.current_frame_has_data = false;
                        self.frame_statistics
                            .update_with_frame(0, false, true, false);
                        return;
                    }

                    self.temp_buffer.extend_from_slice(payload);
                    self.current_frame_has_data = true;

                    // 检查是否是新帧开始（用于调试）
                    if self.temp_buffer.len() == payload.len()
                        && payload.len() >= 2
                        && payload[0] == 0xFF
                        && payload[1] == 0xD8
                    {
                        info!("新帧开始，检测到JPEG SOI标记");
                    }

                    // 后备机制：如果UVC协议没有检测到帧完成，使用JPEG EOI标记检测
                    if !frame_complete && self.temp_buffer.len() >= 2 {
                        // 检查缓冲区末尾是否有JPEG EOI标记 (0xFFD9)
                        let len = self.temp_buffer.len();
                        if self.temp_buffer[len - 2] == 0xFF && self.temp_buffer[len - 1] == 0xD9 {
                            info!(
                                "⚠️ UVC协议未检测到帧结束，但发现JPEG EOI标记，使用后备机制完成帧"
                            );
                            frame_complete = true;
                        }
                    }
                }
            } else {
                trace!("⚠️ URB #{} 接收到空数据", urb_idx);
            }
        } else {
            warn!(
                "handle_isoc_transfer_success: Could not get buffer for URB #{}",
                urb_idx
            );
        }

        loop {
            if !self.frame_processor.start_found {
                if let Some(start_index) = MjpegFrameProcessor::find_soi(&self.temp_buffer) {
                    self.temp_buffer.drain(..start_index);
                    self.frame_processor.start_found = true;
                    trace!(
                        "Found frame start, buffer size now {}",
                        self.temp_buffer.len()
                    );
                } else {
                    if self.temp_buffer.len() > MAX_FRAME_SIZE {
                        warn!("No SOI found and buffer is too large, clearing.");
                        self.temp_buffer.clear();
                    }
                    break;
                }
            }

            if self.frame_processor.start_found {
                if let Some(end_index) = MjpegFrameProcessor::find_eoi(&self.temp_buffer) {
                    let frame_end_pos = end_index + 2;
                    trace!(
                        "Found frame end at position {}. Processing frame.",
                        frame_end_pos
                    );

                    // 从主缓冲区中提取完整的帧数据，并从主缓冲区移除
                    let frame_to_process: Vec<u8> =
                        self.temp_buffer.drain(..frame_end_pos).collect();

                    // 使用提取出的帧数据调用新的处理函数
                    self.process_a_complete_frame(&frame_to_process);

                    // 重置帧处理器，准备在缓冲区的剩余部分中寻找下一帧
                    self.frame_processor.reset();

                    // 恢复为 continue，以在一个事件回调中处理所有可用的完整帧
                    continue;
                } else {
                    // 添加调试信息：检查缓冲区大小和内容
                    if self.temp_buffer.len() > 5000 {
                        trace!("⚠️ 大缓冲区未找到EOI: {} 字节", self.temp_buffer.len());
                        // 检查缓冲区末尾是否可能有不完整的EOI
                        if self.temp_buffer.len() >= 2 {
                            let len = self.temp_buffer.len();
                            trace!(
                                "缓冲区末尾2字节: {:02X} {:02X}",
                                self.temp_buffer[len - 2],
                                self.temp_buffer[len - 1]
                            );
                        }

                        if self.temp_buffer.len() > 20480 {
                            // 在末尾添加EOI标记
                            self.temp_buffer.push(0xFF);
                            self.temp_buffer.push(0xD9);

                            // 处理这个"完整"的帧
                            let frame_to_process: Vec<u8> = self.temp_buffer.drain(..).collect();
                            self.process_a_complete_frame(&frame_to_process);
                            self.frame_processor.reset();
                            continue;
                        }
                    }
                    if self.temp_buffer.len() > MAX_FRAME_SIZE {
                        warn!(
                            "No EOI found, frame buffer exceeded max size ({} bytes). Discarding.",
                            self.temp_buffer.len()
                        );
                        // 打印缓冲区的前后数据用于调试
                        if self.temp_buffer.len() >= 100 {
                            trace!("缓冲区前50字节: {:02X?}", &self.temp_buffer[0..50]);
                            let end = self.temp_buffer.len();
                            trace!("缓冲区后50字节: {:02X?}", &self.temp_buffer[end - 50..end]);
                        }
                        self.temp_buffer.clear();
                        self.frame_processor.reset();
                    }
                    break;
                }
            }
        }

        // 立即重新提交URB以保持数据流连续性（类似Linux实现）
        if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
            urb_info.state = IsocURBState::Idle;

            // 立即创建并准备新的URB以便下次gather_urb时提交
            // 这模拟了Linux中的立即重新提交行为
            // 修正：只要等时传输处于激活状态，就应该持续重新提交URB
            if self.isoc_active
                && (self.camera_state == UVCCameraState::CollectingFrame
                    || self.camera_state == UVCCameraState::WaitingForFrame
                    || self.camera_state == UVCCameraState::CapturingPhoto)
            {
                trace!(
                    "准备立即重新提交URB #{} 以保持数据流 (状态: {:?})",
                    urb_idx,
                    self.camera_state
                );
                self.pending_resubmit_urbs.push(urb_idx);
            } else {
                warn!(
                    "⚠️ URB #{} 未重新提交: isoc_active={}, camera_state={:?}",
                    urb_idx, self.isoc_active, self.camera_state
                );
            }
        }
    }

    // 处理完整帧，包括拍照逻辑
    fn process_a_complete_frame(&mut self, frame_data: &[u8]) {
        info!("====== 完整帧处理 ======");
        info!("处理第 {} 个完整帧", self.frame_counter + 1);

        // 确保图像缓冲区已初始化
        if self.image_buffer.is_none() {
            self.init_image_buffer();
        }

        // 获取图像缓冲区
        if let Some(buffer) = &self.image_buffer {
            let mut img_buf = buffer.lock();

            // 拷贝帧数据到图像缓冲区
            let copy_size = core::cmp::min(frame_data.len(), img_buf.len());
            for i in 0..copy_size {
                img_buf[i] = frame_data[i];
            }

            // 设置当前帧大小
            self.current_frame_size = copy_size;
            self.frame_counter += 1;

            // 打印帧详细信息
            info!("📸 完整MJPEG帧信息:");
            trace!("  - 帧序号: {}", self.frame_counter);
            trace!("  - 帧大小: {} 字节", copy_size);
            trace!("  - 分辨率: 320x240 (推测)");
            trace!("  - 格式: MJPEG");

            // 打印帧数据预览（前几个字节和后几个字节）
            if copy_size >= 4 {
                trace!(
                    "  - 帧头标记: 0x{:02X}{:02X} (应为0xFFD8)",
                    frame_data[0],
                    frame_data[1]
                );
                trace!(
                    "  - 帧尾标记: 0x{:02X}{:02X} (应为0xFFD9)",
                    frame_data[copy_size - 2],
                    frame_data[copy_size - 1]
                );

                // 打印帧数据摘要
                info!("====== MJPEG帧数据 (帧#{}) ======", self.frame_counter);
                info!("帧大小: {} 字节", copy_size);

                if self.frame_counter == 1 {
                    // 第一帧打印完整内容

                    // 分块打印，每行16字节
                    let mut hex_output = String::new();
                    for (i, chunk) in frame_data[..copy_size].chunks(16).enumerate() {
                        hex_output.clear();
                        hex_output.push_str(&format!("{:08X}: ", i * 16));

                        // 十六进制部分
                        for byte in chunk {
                            hex_output.push_str(&format!("{:02X} ", byte));
                        }

                        // 填充空格
                        for _ in chunk.len()..16 {
                            hex_output.push_str("   ");
                        }

                        hex_output.push_str(" |");

                        // ASCII部分
                        for byte in chunk {
                            if *byte >= 0x20 && *byte <= 0x7E {
                                hex_output.push(*byte as char);
                            } else {
                                hex_output.push('.');
                            }
                        }
                        hex_output.push('|');

                        info!("{}", hex_output);

                        // 不再省略，继续打印所有数据
                    }
                    trace!("总大小: {} 字节", copy_size);

                    // 验证JPEG结构
                    let mut marker_count = 0;
                    for i in 0..copy_size.saturating_sub(1) {
                        if frame_data[i] == 0xFF {
                            match frame_data.get(i + 1) {
                                Some(0xD8) => trace!("  - 找到SOI标记 @ 0x{:04X}", i),
                                Some(0xD9) => trace!("  - 找到EOI标记 @ 0x{:04X}", i),
                                Some(0xE0) => trace!("  - 找到APP0标记 @ 0x{:04X}", i),
                                Some(0xE1) => trace!("  - 找到APP1标记 @ 0x{:04X}", i),
                                Some(0xDB) => {
                                    trace!("  - 找到DQT标记 @ 0x{:04X}", i);
                                    marker_count += 1;
                                }
                                Some(0xC0) => {
                                    trace!("  - 找到SOF0标记 @ 0x{:04X}", i);
                                    marker_count += 1;
                                }
                                Some(0xC4) => {
                                    trace!("  - 找到DHT标记 @ 0x{:04X}", i);
                                    marker_count += 1;
                                }
                                Some(0xDA) => {
                                    trace!("  - 找到SOS标记 @ 0x{:04X}", i);
                                    marker_count += 1;
                                }
                                _ => {}
                            }
                        }
                    }
                    trace!("JPEG结构标记数量: {}", marker_count);

                    info!("====== 帧数据结束 ======");
                } else {
                    // 后续帧只打印头32字节
                    if copy_size >= 32 {
                        info!("帧头32字节: {:02X?}", &frame_data[..32]);
                    } else {
                        info!("帧数据: {:02X?}", &frame_data[..copy_size]);
                    }
                }
            }

            // 计算并打印帧率信息
            if self.frame_counter > 1 {
                // 简单的帧率估算（这里假设每帧处理时间相近）
                let estimated_fps = if self.frame_interval > 0 {
                    10000000 / self.frame_interval
                } else {
                    0
                };
                trace!("  - 预期帧率: {} FPS", estimated_fps);
            }

            // 处理拍照请求
            if self.camera_state == UVCCameraState::CapturingPhoto {
                info!("🎯 拍照成功: 捕获了{}字节的MJPEG帧", copy_size);
                trace!("  - 照片已锁定，等待应用获取");
                self.camera_state = UVCCameraState::PhotoCaptured;
                self.buffer_state = BufferState::Locked;
            } else {
                // 继续收集帧
                self.camera_state = UVCCameraState::CollectingFrame;
                self.buffer_state = BufferState::Filled;
                trace!("  - 状态: 正常收集视频帧");
            }

            // 打印帧统计信息（每10帧打印一次）
            if self.frame_counter % 10 == 0 {
                info!("====== 📊 帧统计汇总 (每10帧) ======");
                info!("🎬 总处理帧数: {}", self.frame_statistics.total_frames);
                info!(
                    "✅ 有效帧数: {} ({:.1}%)",
                    self.frame_statistics.valid_frames,
                    if self.frame_statistics.total_frames > 0 {
                        self.frame_statistics.valid_frames as f32 * 100.0
                            / self.frame_statistics.total_frames as f32
                    } else {
                        0.0
                    }
                );
                info!("❌ 无效帧数: {}", self.frame_statistics.invalid_frames);
                info!("⚠️ 超大帧数: {}", self.frame_statistics.oversized_frames);
                info!("🚫 不完整帧数: {}", self.frame_statistics.incomplete_frames);
                info!(
                    "📏 平均帧大小: {} 字节",
                    self.frame_statistics.average_frame_size
                );

                // 计算帧率估算
                if self.frame_counter > 0 && self.frame_interval > 0 {
                    let expected_fps = 10000000 / self.frame_interval;
                    info!("🎯 预期帧率: {} FPS", expected_fps);
                }

                // 工作状态判断
                if self.frame_statistics.valid_frames > 0 {
                    info!("💚 摄像头工作状态: ✅ 正常 - 成功接收到有效的MJPEG帧");
                } else {
                    info!("❤️ 摄像头工作状态: ⚠️ 异常 - 未接收到有效帧");
                }
                info!("====================================");
            }

            // 第一帧成功提示
            if self.frame_counter == 1 {
                info!("🎉🎉🎉 第一个完整MJPEG帧成功接收! 🎉🎉🎉");
                info!("摄像头已成功开始工作!");
            }
        }

        // 此函数不再管理缓冲区或帧处理器状态
        info!("帧处理完成，准备接收下一帧");
        info!("==========================");
    }

    /// 初始化所有控制传输缓冲区
    fn init_all_control_buffers(&mut self) {
        // 初始化probe缓冲区
        if self.probe_buffer.is_none() {
            trace!("初始化UVC控制缓冲区 (probe_buffer)");
            self.probe_buffer = Some(SpinNoIrq::new(DMA::zeroed(
                26, // UVC 1.0规范定义的probe/commit结构体大小
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
        }

        // 初始化commit缓冲区
        if self.commit_buffer.is_none() {
            trace!("初始化UVC控制缓冲区 (commit_buffer)");
            self.commit_buffer = Some(SpinNoIrq::new(DMA::zeroed(
                26, // UVC 1.0规范定义的probe/commit结构体大小
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
        }

        // 初始化probe响应缓冲区（新增）
        if self.probe_response_buffer.is_none() {
            trace!("初始化UVC控制缓冲区 (probe_response_buffer)");
            self.probe_response_buffer = Some(SpinNoIrq::new(DMA::zeroed(
                26, // UVC 1.0规范定义的probe/commit结构体大小
                O::PAGE_SIZE,
                self.config.lock().os.dma_alloc(),
            )));
        }
    }

    // 创建 GET_DEF(Probe) URB
    fn create_video_probe_get_def_urb(&self) -> URB<'a, O> {
        if self.probe_response_buffer.is_none() {
            warn!("probe_response_buffer未初始化，无法创建GET_DEF(Probe) URB");
            panic!("probe_response_buffer必须在调用前初始化");
        }

        let data_tuple = self.probe_response_buffer.as_ref().map(|b| {
            let mut guard = b.lock();
            for elem in guard.iter_mut() {
                *elem = 0;
            } // 清零
            guard.addr_len_tuple()
        });

        let data_length = 26;

        trace!("创建GET_DEF(Probe) UVC探测请求: ReqType=GET_DEF(0x87), wValue(Selector)=0x{:02X}00, wIndex(Interface)={}, Len={}", 
             UVC_VS_PROBE_CONTROL,
             (self.interface_value + 1), // VideoStreaming 接口号
             data_length);

        URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::In,
                    DataTransferType::Class,
                    Recipient::Interface,
                ),
                request: bRequest::from(UvcRequest::GetDef),
                value: (u16::from(UVC_VS_PROBE_CONTROL) << 8),
                index: (self.interface_value + 1) as u16, // VideoStreaming 接口号
                data: data_tuple,
                response: true,
            }),
        )
    }

    // 创建 GET_CUR(Probe) URB
    fn create_video_probe_get_cur_urb(&self) -> URB<'a, O> {
        if self.probe_response_buffer.is_none() {
            warn!("probe_response_buffer未初始化，无法创建GET_CUR(Probe) URB");
            panic!("probe_response_buffer必须在调用前初始化");
        }

        let data_tuple = self.probe_response_buffer.as_ref().map(|b| {
            let mut guard = b.lock();
            for elem in guard.iter_mut() {
                *elem = 0;
            } // 清零
            guard.addr_len_tuple()
        });

        let data_length = 26;

        trace!("创建GET_CUR(Probe) UVC探测请求: ReqType=GET_CUR(0x81), wValue(Selector)=0x{:02X}00, wIndex(Interface)={}, Len={}", 
             UVC_VS_PROBE_CONTROL,
             (self.interface_value + 1), // VideoStreaming 接口号
             data_length);

        URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::In,
                    DataTransferType::Class,
                    Recipient::Interface,
                ),
                request: bRequest::from(UvcRequest::GetCur),
                value: (u16::from(UVC_VS_PROBE_CONTROL) << 8),
                index: (self.interface_value + 1) as u16, // VideoStreaming 接口号
                data: data_tuple,
                response: true,
            }),
        )
    }

    // 创建 SET_CUR(Probe) URB
    fn create_video_probe_set_cur_urb(
        &self,
        probe_data_to_set: &VideoProbeCommitControl,
    ) -> URB<'a, O> {
        if self.probe_buffer.is_none() {
            // 使用 self.probe_buffer 来发送 SET 请求
            warn!("probe_buffer未初始化，无法创建SET_CUR(Probe) URB");
            panic!("probe_buffer必须在调用前初始化");
        }

        let data_bytes_to_write = probe_data_to_set.as_bytes();
        let required_len = data_bytes_to_write.len();

        let data_tuple = self.probe_buffer.as_ref().map(|b_lock| {
            let mut guard = b_lock.lock();
            if guard.len() >= required_len {
                guard[..required_len].copy_from_slice(&data_bytes_to_write);
                (guard.addr(), required_len)
            } else {
                error!(
                    "probe_buffer太小 ({} vs {} required)",
                    guard.len(),
                    required_len
                );
                panic!("probe_buffer太小");
            }
        });

        trace!("创建SET_CUR(Probe) UVC探测请求: ReqType=SET_CUR(0x01), wValue(Selector)=0x{:02X}00, wIndex(Interface)={}, Len={}",
             UVC_VS_PROBE_CONTROL,
             (self.interface_value + 1), // VideoStreaming 接口号
             required_len);

        URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::Out,
                    DataTransferType::Class,
                    Recipient::Interface,
                ),
                request: bRequest::from(UvcRequest::SetCur),
                value: (UVC_VS_PROBE_CONTROL as u16) << 8,
                index: (self.interface_value + 1) as u16, // VideoStreaming 接口号
                data: data_tuple,
                response: false, // SET请求通常不需要数据返回阶段，但可能有ACK
            }),
        )
    }

    // 创建视频提交控制URB (SET_CUR(Commit)) - 这个函数保持不变，它是正确的
    fn create_video_commit_urb(&self, commit_data: &VideoProbeCommitControl) -> URB<'a, O> {
        // 确保缓冲区大小符合UVC 1.0规范
        let buffer_size = 26;

        // 如果commit_buffer未初始化，创建它
        if self.commit_buffer.is_none() {
            warn!("commit_buffer未初始化，无法创建commit URB");
            panic!("commit_buffer必须在调用create_video_commit_urb前初始化");
        }

        // 为SET_CUR填充缓冲区
        let data_tuple = if let Some(ref buffer) = self.commit_buffer {
            let mut buffer_data = buffer.lock();

            // 填充缓冲区
            let bytes = commit_data.as_bytes();
            for i in 0..bytes.len() {
                if i < buffer_data.len() {
                    buffer_data[i] = bytes[i];
                }
            }

            Some(buffer.lock().addr_len_tuple())
        } else {
            None
        };

        trace!(
            "创建SET_CUR UVC提交请求: SetCur=0x01, 值=0x{:04x}, 索引={}, 长度={}",
            (u16::from(UVC_VS_COMMIT_CONTROL) << 8),
            (self.interface_value + 1),
            buffer_size
        );

        URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::Out,
                    DataTransferType::Class,
                    Recipient::Interface,
                ),
                request: bRequest::from(UvcRequest::SetCur),
                value: (u16::from(UVC_VS_COMMIT_CONTROL) << 8), // VS_COMMIT_CONTROL
                index: (self.interface_value + 1) as u16,       // 视频流接口
                data: data_tuple,
                response: true, // 设置为true以接收完成事件
            }),
        )
    }

    // 打印Probe/Commit详细信息
    fn print_probe_commit_details(&self, data: &VideoProbeCommitControl) {
        trace!("Probe/Commit详细数据:");
        trace!("  bmHint: 0x{:04X}", data.bmHint);
        trace!(
            "  format_index: {} ({})",
            data.format_index,
            if data.format_index == 1 {
                "MJPEG"
            } else {
                "其他"
            }
        );
        trace!("  frame_index: {} (可能是320x240)", data.frame_index);
        trace!(
            "  frame_interval: {} (约{}fps)",
            data.frame_interval,
            if data.frame_interval > 0 {
                10000000 / data.frame_interval
            } else {
                0
            }
        );
        trace!("  dwMaxVideoFrameSize: {}", data.dwMaxVideoFrameSize);
        trace!(
            "  dwMaxPayloadTransferSize: {}",
            data.dwMaxPayloadTransferSize
        );
    }

    /// 从probe_response_buffer解析VideoProbeCommitControl
    fn parse_probe_response(&self) -> Option<VideoProbeCommitControl> {
        if let Some(ref buffer) = self.probe_response_buffer {
            let data = buffer.lock();
            if data.len() >= 26 {
                // 基本检查确保数据有效
                if data[2] > 0 {
                    // format_index应该大于0
                    let mut ctrl = VideoProbeCommitControl::default();

                    // 从缓冲区解析各字段
                    ctrl.bmHint = u16::from_le_bytes([data[0], data[1]]);
                    ctrl.format_index = data[2];
                    ctrl.frame_index = data[3];
                    ctrl.frame_interval = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
                    ctrl.wKeyFrameRate = u16::from_le_bytes([data[8], data[9]]);
                    ctrl.wPFrameRate = u16::from_le_bytes([data[10], data[11]]);
                    ctrl.wCompQuality = u16::from_le_bytes([data[12], data[13]]);
                    ctrl.wCompWindowSize = u16::from_le_bytes([data[14], data[15]]);
                    ctrl.wDelay = u16::from_le_bytes([data[16], data[17]]);
                    ctrl.dwMaxVideoFrameSize =
                        u32::from_le_bytes([data[18], data[19], data[20], data[21]]);
                    ctrl.dwMaxPayloadTransferSize =
                        u32::from_le_bytes([data[22], data[23], data[24], data[25]]);

                    return Some(ctrl);
                }
            }
        }
        None
    }

    // 初始化UVC摄像头
    fn initialize_uvc_camera(&mut self) -> Option<Vec<URB<'a, O>>> {
        let mut urbs = Vec::new();

        info!(
            "UVC初始化[进入initialize_uvc_camera]: 当前init_stage = {:?}, consecutive_errors = {}",
            self.init_stage, self.consecutive_errors
        );

        // Only check for delay if we are not in a state that immediately follows an event
        let should_wait_for_delay = match self.init_stage {
            UVCInitStage::Idle => false, // First step, no prior event to wait for delay from
            UVCInitStage::ProbeSetSent |    // GET_DEF completed, ready to send SET_CUR(Probe)
            UVCInitStage::ProbeGetSent |    // SET_CUR(Probe) completed, ready to send GET_CUR(Probe)
            UVCInitStage::CommitSetSent |   // GET_CUR(Probe) completed, ready to send SET_CUR(Commit)
            UVCInitStage::InterfaceSetSent |// SET_CUR(Commit) completed, ready to send SET_INTERFACE
            UVCInitStage::XHCIConfigureDCI5 => false, // SET_INTERFACE completed, ready for XHCI config
            UVCInitStage::ProbeGetDefSent | // GET_DEF sent, waiting for completion
            // UVCInitStage::ProbeSetSent_WaitingEvent | // Hypothetical more granular state
            // UVCInitStage::ProbeGetSent_WaitingEvent |
            // UVCInitStage::CommitSetSent_WaitingEvent |
            // UVCInitStage::InterfaceSetSent_WaitingEvent |
            // UVCInitStage::XHCIConfigureDCI5_WaitingEvent |
            UVCInitStage::Done | UVCInitStage::Failed |
            UVCInitStage::StreamStartSent // Currently unused properly
            => true, // If in a waiting state, or final, check delay (or do nothing if final)
        };

        if should_wait_for_delay {
            if self.init_stage == UVCInitStage::Done || self.init_stage == UVCInitStage::Failed {
                return None;
            }
            if !self.has_delay_elapsed() {
                info!(
                    "UVC 初始化延迟 (等待事件或 {}ms) for stage {:?}",
                    STANDARD_DELAY_MS, self.init_stage
                );
                return None;
            }
            // 添加对等待状态的超时检测
            // 如果在ProbeGetDefSent或其他等待状态中停留太久，考虑进行恢复操作
            if self.init_stage == UVCInitStage::ProbeGetDefSent {
                let current_time = Self::get_clock_count();
                let elapsed_time = current_time.saturating_sub(self.last_command_time);

                // 如果等待超过5秒，执行恢复操作
                if elapsed_time > 5000 {
                    warn!(
                        "在ProbeGetDefSent阶段等待超时({}ms)，尝试恢复",
                        elapsed_time
                    );

                    // 如果没有解析到探测响应，使用默认值
                    if self.current_probe_settings.is_none() {
                        self.current_probe_settings =
                            Some(VideoProbeCommitControl::default_for_probe_set());
                    }

                    // 重置控制端点 - 暂时跳过
                    // let _ = self.reset_halted_endpoint(0);

                    // 推进到下一个阶段
                    self.init_stage = UVCInitStage::ProbeSetSent;
                    self.update_command_timestamp();

                    // 递归调用自身继续初始化过程
                    return self.initialize_uvc_camera();
                }
            }
            // 添加对ProbeGetSent阶段的超时检测
            else if self.init_stage == UVCInitStage::ProbeGetSent {
                let current_time = Self::get_clock_count();
                let elapsed_time = current_time.saturating_sub(self.last_command_time);

                // 如果等待超过5秒，认为GET_CUR请求卡住了
                if elapsed_time > 5000 {
                    warn!(
                        "在ProbeGetSent阶段等待超时({}ms)，跳过GET_CUR阶段",
                        elapsed_time
                    );

                    // 重置控制端点 - 暂时跳过
                    // self.reset_halted_endpoint(0);

                    // 直接推进到Commit阶段
                    self.init_stage = UVCInitStage::CommitSetSent;
                    self.update_command_timestamp();

                    // 递归调用自身继续初始化过程
                    return self.initialize_uvc_camera();
                }
            }
        }

        self.init_all_control_buffers();

        // 初始化VideoControl中断端点
        if self.vc_interrupt_endpoint.is_none() {
            self.init_vc_interrupt_endpoint();
        }

        if self.camera_state == UVCCameraState::Idle && self.init_stage == UVCInitStage::Idle {
            info!("执行标准UVC摄像头初始化序列...摄像头状态从Idle变为Configuring");
            self.camera_state = UVCCameraState::Configuring;
        } else {
            info!(
                "执行标准UVC摄像头初始化序列...当前摄像头状态: {:?}",
                self.camera_state
            );
        }

        match self.init_stage {
            UVCInitStage::Idle => {
                // Action: Send GET_DEF(Probe)
                info!("UVC初始化阶段1: 发送 GET_DEF(Probe)");
                // 检查是否应该使用异步模式（基于新的传输模式逻辑）
                if !self.should_use_sync_now() && self.xhci_controller.is_some() {
                    info!(
                        "使用异步模式发送GET_DEF(Probe) (传输模式: {:?})",
                        self.get_effective_transfer_mode()
                    );

                    let get_def_urb = self.create_video_probe_get_def_urb();
                    if let RequestedOperation::Control(control) = get_def_urb.operation {
                        if let Some(ref controller) = self.xhci_controller {
                            let future = controller
                                .lock()
                                .control_transfer_async(self.device_slotid, control);

                            // 创建传输ID
                            let transfer_id = AsyncTransferId::new();

                            // 保存传输信息
                            let transfer_info = AsyncTransferInfo {
                                id: transfer_id,
                                state: AsyncTransferState::Submitted,
                                waker: None,
                            };
                            self.async_transfers.insert(transfer_id, transfer_info);

                            self.init_stage = UVCInitStage::ProbeGetDefSent;
                            self.update_command_timestamp();

                            trace!("异步控制传输已提交");
                        }
                    }
                    return None; // 异步模式不返回URB
                }

                // 同步模式
                let get_def_urb = self.create_video_probe_get_def_urb();
                urbs.push(get_def_urb);
                self.init_stage = UVCInitStage::ProbeGetDefSent; // Next: Waiting for GET_DEF(Probe) completion
                self.update_command_timestamp();
                return Some(urbs);
            }
            UVCInitStage::ProbeGetDefSent => {
                // State: Waiting for GET_DEF(Probe) completion.
                // receive_complete_event will advance to ProbeSetSent.
                info!("UVC初始化阶段: 等待 GET_DEF(Probe) 完成 (当前阶段: ProbeGetDefSent)");
                return None;
            }
            UVCInitStage::ProbeSetSent => {
                // State: GET_DEF(Probe) completed. Now, send SET_CUR(Probe).
                info!("UVC初始化阶段2: 发送 SET_CUR(Probe)");
                let mut probe_data_to_set = self.current_probe_settings.clone().unwrap_or_else(|| {
                    warn!("SET_CUR(Probe): current_probe_settings (从GET_DEF)无效或设备不支持, 使用来自default_for_probe_set()的默认值.");
                    VideoProbeCommitControl::default_for_probe_set()
                });

                probe_data_to_set.dwMaxVideoFrameSize = 0;
                probe_data_to_set.dwMaxPayloadTransferSize = 0;

                probe_data_to_set.bmHint = 1 << 0; // 只协商 dwFrameInterval

                probe_data_to_set.format_index = TARGET_FORMAT_INDEX;
                probe_data_to_set.frame_index = TARGET_FRAME_INDEX;
                probe_data_to_set.frame_interval = TARGET_FRAME_INTERVAL;

                info!("UVC初始化[ProbeSetSent]: 最终用于SET_CUR(Probe)的数据 (dwMaxVideoFrameSize/dwMaxPayloadTransferSize已强制为0, bmHint主要协商dwFrameInterval):");
                self.print_probe_commit_details(&probe_data_to_set);

                let data_bytes_to_write = probe_data_to_set.as_bytes();
                let required_len = data_bytes_to_write.len();
                let mut urb_created = false;
                if let Some(buffer_lock_ref) = self.probe_buffer.as_ref() {
                    let mut guard = buffer_lock_ref.lock();
                    if guard.len() >= required_len {
                        guard[..required_len].copy_from_slice(&data_bytes_to_write);
                        let urb = URB::new(
                            self.device_slotid,
                            RequestedOperation::Control(ControlTransfer {
                                request_type: bmRequestType::new(
                                    Direction::Out,
                                    DataTransferType::Class,
                                    Recipient::Interface,
                                ),
                                request: bRequest::from(UvcRequest::SetCur),
                                value: (UVC_VS_PROBE_CONTROL as u16) << 8,
                                index: (self.interface_value + 1) as u16,
                                data: Some((guard.addr(), required_len)),
                                response: false,
                            }),
                        );
                        urbs.push(urb);
                        urb_created = true;
                    } else {
                        self.init_stage = UVCInitStage::Failed;
                        error!("probe_buffer太小");
                    }
                } else {
                    self.init_stage = UVCInitStage::Failed;
                    error!("probe_buffer未初始化");
                }

                if urb_created {
                    // 保持在ProbeSetSent状态，等待SET_CUR(Probe)完成
                    // 不要改变状态！事件处理器会在收到成功响应后将状态改为ProbeGetSent
                    self.update_command_timestamp();
                    return Some(urbs);
                } else {
                    return None;
                }
            }
            UVCInitStage::ProbeGetSent => {
                // 检查是否刚刚进入这个状态（需要发送命令）
                let current_time = Self::get_clock_count();
                let elapsed_time = current_time.saturating_sub(self.last_command_time);

                // 如果刚进入这个状态（时间差很小），发送GET_CUR命令
                if elapsed_time < 100 {
                    // 100ms内认为是刚进入状态
                    info!("UVC初始化阶段3: 发送 GET_CUR(Probe)");
                    // 使用修正后的函数名
                    let get_cur_probe_urb = self.create_video_probe_get_cur_urb();
                    urbs.push(get_cur_probe_urb);
                    // 保持当前状态，等待GET_CUR(Probe)完成
                    // 完成后事件处理器会设置为CommitSetSent
                    return Some(urbs);
                } else {
                    // 已经发送了命令，正在等待响应
                    trace!("等待GET_CUR(Probe)响应 (已等待{}ms)", elapsed_time);
                    return None;
                }
            }
            UVCInitStage::CommitSetSent => {
                // State: GET_CUR(Probe) completed and response parsed. Now, send SET_CUR(Commit).
                // 检查是否刚刚进入这个状态（需要发送命令）
                let current_time = Self::get_clock_count();
                let elapsed_time = current_time.saturating_sub(self.last_command_time);

                // 如果刚进入这个状态（时间差很小），发送SET_CUR(Commit)命令
                if elapsed_time < 100 {
                    // 100ms内认为是刚进入状态
                    info!("UVC初始化阶段4: 发送 SET_CUR(Commit)");
                    let commit_data: VideoProbeCommitControl;

                    if let Some(negotiated_params) = self.current_probe_settings.as_ref() {
                        trace!("SET_CUR(Commit): 使用从GET_CUR(Probe)获得的协商参数 (dwMaxPayloadTransferSize可能为0)。");
                        self.print_probe_commit_details(negotiated_params);
                        commit_data = *negotiated_params; // 关键：直接使用协商结果，包括可能为0的dwMaxPayloadTransferSize
                        self.last_successful_probe_params = Some(*negotiated_params);
                    // 更新最后成功的参数
                    } else {
                        warn!("SET_CUR(Commit): current_probe_settings (GET_CUR(Probe)的结果) 为None. 这是一个错误状态，回退到默认COMMIT参数。");
                        commit_data = VideoProbeCommitControl::default_for_commit();
                        self.print_probe_commit_details(&commit_data);
                        self.last_successful_probe_params = Some(commit_data);
                    }

                    // self.current_packet_size 用于创建URB，它必须非零。它应该已经在GET_CUR(Probe)响应处理中被正确设置了后备值。
                    if self.current_packet_size == 0 {
                        self.current_packet_size = ISOC_PACKET_SIZE_ALT2; // 最后的保障，例如3072
                        warn!(
                            "SET_CUR(Commit): current_packet_size 在此之前仍为0，强制设为 {}",
                            self.current_packet_size
                        );
                    }

                    let commit_urb = self.create_video_commit_urb(&commit_data);
                    urbs.push(commit_urb);
                    // 保持在CommitSetSent状态，等待SET_CUR(Commit)完成
                    // 完成后事件处理器会设置为InterfaceSetSent
                    self.update_command_timestamp();
                    return Some(urbs);
                } else {
                    // 已经发送了命令，正在等待响应
                    trace!("等待SET_CUR(Commit)响应 (已等待{}ms)", elapsed_time);

                    // 添加超时检查 - 如果等待超过5秒，尝试恢复
                    if elapsed_time > 5000 {
                        warn!("SET_CUR(Commit)响应超时 ({}ms)，尝试恢复...", elapsed_time);

                        // 增加错误计数
                        self.consecutive_errors += 1;

                        if self.consecutive_errors >= 3 {
                            error!(
                                "SET_CUR(Commit)连续失败{}次，初始化失败",
                                self.consecutive_errors
                            );
                            self.init_stage = UVCInitStage::Failed;
                            self.camera_state = UVCCameraState::Error;
                            return None;
                        }

                        // 重置控制端点 - 暂时跳过
                        // self.reset_halted_endpoint(0);

                        // 直接跳过Commit阶段，尝试SET_INTERFACE
                        warn!("跳过SET_CUR(Commit)，直接尝试SET_INTERFACE");
                        self.init_stage = UVCInitStage::InterfaceSetSent;
                        self.update_command_timestamp();

                        // 递归调用继续初始化
                        return self.initialize_uvc_camera();
                    }

                    return None;
                }
            }
            UVCInitStage::InterfaceSetSent => {
                // State: SET_CUR(Commit) completed. Now, send SET_INTERFACE.
                // 检查是否刚刚进入这个状态（需要发送命令）
                let current_time = Self::get_clock_count();
                let elapsed_time = current_time.saturating_sub(self.last_command_time);

                // 如果刚进入这个状态（时间差很小），发送SET_INTERFACE命令
                if elapsed_time < 100 {
                    // 100ms内认为是刚进入状态
                    info!("UVC初始化阶段5: 发送 SET_INTERFACE");

                    let target_pktsz = self.current_packet_size;
                    let mut chosen_alt_setting: Option<u8> = None;

                    for &(alt_val, size_val) in Self::ALT_SETTINGS.iter() {
                        if size_val == target_pktsz {
                            chosen_alt_setting = Some(alt_val);
                            break;
                        }
                    }

                    if let Some(alt) = chosen_alt_setting {
                        self.isoc_alt_setting = alt;
                        trace!(
                            "Selected isoc_alt_setting: {} for packet_size: {} (Exact Match)",
                            self.isoc_alt_setting,
                            self.current_packet_size
                        );
                    } else {
                        warn!("No exact match in ALT_SETTINGS for negotiated packet size {}. Falling back to the first entry or a default.", target_pktsz);
                        if let Some(&(fallback_alt, fallback_size)) = Self::ALT_SETTINGS.get(0) {
                            self.isoc_alt_setting = fallback_alt;
                            self.current_packet_size = fallback_size;
                            warn!(
                                "Using fallback alternate setting: {} with packet_size: {}",
                                self.isoc_alt_setting, self.current_packet_size
                            );
                        } else {
                            error!("ALT_SETTINGS is empty or invalid. Cannot select an alternate setting. Halting.");
                            self.init_stage = UVCInitStage::Failed;
                            return None;
                        }
                    }
                    // ADDED LOGGING HERE
                    trace!("[InterfaceSetSent Post-Selection] self.isoc_alt_setting = {}, self.current_packet_size = {}", self.isoc_alt_setting, self.current_packet_size);

                    let interface_urb = self.create_set_interface_urb();
                    urbs.push(interface_urb);
                    self.update_command_timestamp();
                    return Some(urbs);
                } else {
                    // 已经发送了命令，正在等待响应
                    trace!("等待SET_INTERFACE响应 (已等待{}ms)", elapsed_time);
                    return None;
                }
            }
            UVCInitStage::XHCIConfigureDCI5 => {
                // 这个状态不再使用
                // 跳过，直接标记为完成
                info!("UVC初始化: 跳过XHCI DCI配置（Linux方式不需要）");
                self.init_stage = UVCInitStage::Done;
                self.camera_state = UVCCameraState::WaitingForFrame;

                // 启动VideoControl中断端点
                if let Some(urb) = self.start_vc_interrupt_endpoint() {
                    info!("UVC初始化完成，启动VideoControl中断端点");
                    urbs.push(urb);
                }

                if !self.isoc_active {
                    trace!("激活等时传输");
                    self.isoc_active = true;
                    if self.isoc_urbs.is_empty() {
                        self.init_isoc_urbs();
                    }
                }

                if urbs.is_empty() {
                    return None;
                } else {
                    return Some(urbs);
                }
            }
            UVCInitStage::StreamStartSent | UVCInitStage::Done | UVCInitStage::Failed => {
                if self.init_stage == UVCInitStage::StreamStartSent {
                    warn!("UVC初始化: StreamStartSent 阶段被意外进入或完成，直接标记为Done。");
                    self.init_stage = UVCInitStage::Done; // Correctly go to Done
                    self.camera_state = UVCCameraState::WaitingForFrame;
                    if !self.isoc_active { /* ... activate isoc ... */ }
                }
                info!(
                    "UVC摄像头初始化处于 {:?} 状态, 不发送新URB",
                    self.init_stage
                );
                return None;
            }
        }
    }

    // 设备休眠和唤醒相关函数

}

// 创建GET_DEF(Probe)请求URB

// 为GenericUVCDriver<O>实现USBSystemDriverModuleInstance trait
impl<'a, O> USBSystemDriverModuleInstance<'a, O> for GenericUVCDriver<O>
where
    O: PlatformAbstractions + 'static,
{
    fn prepare_for_drive(&mut self) -> Option<Vec<URB<'a, O>>> {
        info!("====== UVC摄像头驱动初始化 ======");
        info!("准备初始化UVC摄像头驱动");
        info!("设备信息:");
        info!("  - 厂商ID (VID): 0x{:04x}", self.vendor_id);
        info!("  - 产品ID (PID): 0x{:04x}", self.device_id);
        info!("  - 设备槽位: {}", self.device_slotid);
        info!("  - 配置值: {}", self.config_value);
        info!("  - 接口值: {}", self.interface_value);

        // 初始化各种缓冲区
        info!("初始化各种缓冲区...");
        self.init_image_buffer();
        self.init_all_control_buffers(); // 确保所有控制缓冲区都已初始化

        // 设置为已初始化
        self.is_initialized = true;
        self.init_stage = UVCInitStage::Idle; // 初始阶段设为Idle, gather_urb 将从这里开始
        self.camera_state = UVCCameraState::Configuring;

        info!("创建设置配置请求...");

        // 1. 创建配置URB
        let set_config_urb = URB::new(
            self.device_slotid,
            RequestedOperation::Control(ControlTransfer {
                request_type: bmRequestType::new(
                    Direction::Out,
                    DataTransferType::Standard,
                    Recipient::Device,
                ),
                request: bRequest::SetConfiguration,
                index: 0,
                value: self.config_value as u16,
                data: None,
                response: true,
            }),
        );

        // 2. 创建设置视频电源模式为 D0 (Full Power) 的URB (暂时注释掉以进行调试)
        // VideoControl Interface number - 假设是 self.interface_value (通常为0 for VC)
        // Unit ID for Interface Controls is 0 according to UVC Spec Table A-3.
        // let vc_interface_number = self.interface_value; // 通常是0
        // let power_mode_data_dma = DMA::new_vec_with_data([0x00u8; 1], O::PAGE_SIZE, self.config.lock().os.dma_alloc());
        // 使用 DMA::new 来创建一个包含单个u8元素的DMA区域，并初始化为0x00
        // let power_mode_data_tuple = power_mode_data_dma.addr_len_tuple();

        // trace!("创建SET_CUR(VC_VIDEO_POWER_MODE_CONTROL)请求: Interface={}, Value=0 (Full Power)", vc_interface_number);
        // let set_power_mode_urb = URB::new(
        //     self.device_slotid,
        //     RequestedOperation::Control(ControlTransfer {
        //         request_type: bmRequestType::new(
        //             Direction::Out,
        //             DataTransferType::Class, // Class-specific request
        //             Recipient::Interface,    // Recipient is Interface
        //         ),
        //         request: bRequest::SetCur,   // SET_CUR
        //         value: (u16::from(UVC_VC_VIDEO_POWER_MODE_CONTROL) << 8), // Selector in high byte, 0 in low byte
        //         index: vc_interface_number as u16, // Interface number in low byte, Unit ID (0 for VC IF) in high byte
        //         data: Some(power_mode_data_tuple),
        //         response: true // 通常这些设置会有一个ACK，但可能没有数据阶段返回
        //     })
        // );

        // 返回配置URB，暂时不发送电源模式URB
        Some(vec![set_config_urb]) // , set_power_mode_urb])
    }

    fn gather_urb(&mut self) -> Option<Vec<URB<'a, O>>> {
        // 设备尚未初始化或处于错误状态时的处理
        trace!(
            "gather_urb: ENTER, init_stage={:?}, camera_state={:?}, isoc_active={}",
            self.init_stage,
            self.camera_state,
            self.isoc_active
        ); // 添加日志

        if !self.is_initialized {
            // is_initialized 在 prepare_for_drive 中设为 true, 如果到这里还是 false，说明 prepare_for_drive 没被调用
            warn!("gather_urb: 设备未初始化 (is_initialized is false)");
            return self.prepare_for_drive();
        }

        // 处理错误状态恢复
        if self.camera_state == UVCCameraState::Error {
            warn!("摄像头处于错误状态，尝试恢复...");
            self.consecutive_errors = 0;
            self.init_stage = UVCInitStage::Idle;
            self.camera_state = UVCCameraState::Configuring;
            // 在错误恢复后，initialize_uvc_camera将从Idle开始，它会重新发送电源模式（如果需要）
            return self.initialize_uvc_camera();
        }

        // 如果处于设备休眠状态，跳过处理
        if self.device_suspended {
            info!("设备处于休眠状态");
            return None;
        }

        // 摄像头初始化流程
        if self.init_stage != UVCInitStage::Done {
            return self.initialize_uvc_camera();
        }

        // 如果初始化已完成但camera_state还是Configuring，修正状态
        if self.init_stage == UVCInitStage::Done && self.camera_state == UVCCameraState::Configuring
        {
            warn!("检测到不一致状态：init_stage=Done但camera_state=Configuring，修正为WaitingForFrame");
            self.camera_state = UVCCameraState::WaitingForFrame;
            if !self.isoc_active {
                trace!("激活等时传输");
                self.isoc_active = true;
                if self.isoc_urbs.is_empty() {
                    self.init_isoc_urbs();
                }
            }
        }

        // 只有在初始化未完成或camera状态需要初始化时才调用
        if self.camera_state == UVCCameraState::Idle
            || (self.camera_state == UVCCameraState::Configuring
                && self.init_stage != UVCInitStage::Done)
        {
            return self.initialize_uvc_camera();
        }

        // 错误恢复：如果连续错误次数达到阈值，尝试切换alternate setting
        if self.consecutive_errors >= ERROR_THRESHOLD_SWITCH_ALT
            && self.consecutive_errors < ERROR_THRESHOLD_RESET
        {
            info!(
                "连续错误次数达到阈值({}), 尝试切换alternate setting",
                self.consecutive_errors
            );
            self.consecutive_errors = 0;
            self.try_next_alternate_setting();

            // 创建接口设置URB
            let interface_urb = self.create_set_interface_urb();
            return Some(vec![interface_urb]);
        }

        // 等时传输管理
        if (self.camera_state == UVCCameraState::WaitingForFrame
            || self.camera_state == UVCCameraState::CollectingFrame
            || self.camera_state == UVCCameraState::CapturingPhoto)
            && self.init_stage == UVCInitStage::Done
        {
            if !self.isoc_active {
                trace!("激活等时传输，尝试前置操作 (ResetEndpoint已注释)");
                self.isoc_active = true;
                if self.isoc_urbs.is_empty() {
                    // 确保URB列表已初始化
                    self.init_isoc_urbs();
                }
            }

            // 异步模式下，收集需要提交的URB并返回
            info!(
                "检查异步模式条件: should_use_sync_now()={}, xhci_controller.is_some()={}",
                self.should_use_sync_now(),
                self.xhci_controller.is_some()
            );

            if !self.should_use_sync_now() && self.xhci_controller.is_some() {
                info!(
                    "异步模式：收集待提交的等时传输URB (传输模式: {:?})",
                    self.get_effective_transfer_mode()
                );

                // 先统计URB状态
                let mut idle_count = 0;
                let mut pending_count = 0;

                for urb_info in &self.isoc_urbs {
                    match urb_info.state {
                        IsocURBState::Idle => idle_count += 1,
                        IsocURBState::Pending => pending_count += 1,
                        _ => {}
                    }
                }

                trace!(
                    "📊 URB状态: 空闲={}, 待处理={}, 总数={}",
                    idle_count,
                    pending_count,
                    self.isoc_urbs.len()
                );

                let mut urbs_to_return = Vec::new();

                // 检查是否需要初始提交
                if idle_count == self.isoc_urbs.len() && idle_count > 0 && pending_count == 0 {
                    trace!("检测到所有URB都是空闲状态，这是初始提交");
                }

                // 优先处理待重新提交的URB（类似Linux的立即重新提交）
                trace!(
                    "检查pending_resubmit_urbs，当前有 {} 个待重新提交",
                    self.pending_resubmit_urbs.len()
                );
                if !self.pending_resubmit_urbs.is_empty() {
                    trace!(
                        "🚀 优先处理 {} 个待重新提交的URB",
                        self.pending_resubmit_urbs.len()
                    );
                    let pending_urbs = self.pending_resubmit_urbs.drain(..).collect::<Vec<_>>();

                    for urb_idx in pending_urbs {
                        if let Some(urb_info) = self.isoc_urbs.get(urb_idx) {
                            if urb_info.state == IsocURBState::Idle {
                                if let Some(urb) = self.create_isoc_urb_for_idx(urb_idx) {
                                    // 更新URB状态
                                    if let Some(urb_info_mut) = self.isoc_urbs.get_mut(urb_idx) {
                                        urb_info_mut.state = IsocURBState::Pending;
                                        urb_info_mut.submit_time = Self::get_clock_count();
                                    }
                                    trace!("✅ 重新提交URB #{} (优先队列)", urb_idx);
                                    urbs_to_return.push(urb);
                                }
                            }
                        }
                    }
                }

                // 收集所有空闲的URB
                info!("开始收集空闲URB，总共有 {} 个URB", self.isoc_urbs.len());
                for i in 0..self.isoc_urbs.len() {
                    let should_submit = if let Some(urb_info) = self.isoc_urbs.get(i) {
                        let is_idle = urb_info.state == IsocURBState::Idle;
                        trace!(
                            "URB #{}: 状态={:?}, should_submit={}",
                            i,
                            urb_info.state,
                            is_idle
                        );
                        is_idle
                    } else {
                        trace!("URB #{}: 不存在", i);
                        false
                    };

                    if should_submit {
                        trace!("尝试为URB #{} 创建等时传输URB...", i);
                        if let Some(urb) = self.create_isoc_urb_for_idx(i) {
                            // 获取buffer地址用于日志
                            let buffer_addr = if let Some(urb_info) = self.isoc_urbs.get(i) {
                                urb_info.buffer.as_ref().unwrap().lock().addr()
                            } else {
                                0
                            };

                            // 更新URB状态为Pending
                            if let Some(urb_info_mut) = self.isoc_urbs.get_mut(i) {
                                urb_info_mut.state = IsocURBState::Pending;
                                urb_info_mut.submit_time = Self::get_clock_count();
                            }

                            trace!(
                                "✅ 添加等时传输URB #{} 到提交队列, buffer=0x{:x}",
                                i,
                                buffer_addr
                            );

                            urbs_to_return.push(urb);
                        } else {
                            error!("❌ 无法创建URB #{}", i);
                        }
                    } else {
                        debug!(
                            "URB #{} 状态为 {:?}，跳过",
                            i,
                            self.isoc_urbs
                                .get(i)
                                .map(|u| &u.state)
                                .unwrap_or(&IsocURBState::Idle)
                        );
                    }
                }

                // 如果没有空闲的URB，检查是否所有URB都已经处于Pending状态（但尚未提交）
                if urbs_to_return.is_empty() {
                    // 如果所有URB都处于Pending状态，说明可能从同步模式切换到了异步模式
                    // 或者URB已经卡住了，需要重置它们
                    if pending_count == self.isoc_urbs.len() && pending_count > 0 {
                        // 检查是否是第一次进入异步模式（通过检查是否有任何完成的传输）
                        let current_time = Self::get_clock_count();
                        let mut oldest_submit_time = u64::MAX;

                        for urb_info in &self.isoc_urbs {
                            if urb_info.submit_time > 0 && urb_info.submit_time < oldest_submit_time
                            {
                                oldest_submit_time = urb_info.submit_time as u64;
                            }
                        }

                        // 如果最早的提交时间距今超过1秒，或者这是第一次异步收集，重置URB
                        let should_reset = oldest_submit_time == u64::MAX
                            || (current_time.saturating_sub(oldest_submit_time) > 1000);

                        if should_reset {
                            info!(
                                "检测到所有{}个URB都处于Pending状态，重置为Idle以进行异步提交",
                                pending_count
                            );

                            // 重置所有URB为Idle状态
                            for urb_info in &mut self.isoc_urbs {
                                if urb_info.state == IsocURBState::Pending {
                                    urb_info.state = IsocURBState::Idle;
                                    urb_info.submit_time = 0;
                                }
                            }

                            // 递归调用以收集现在处于Idle状态的URB
                            return self.gather_urb();
                        }
                    }
                }

                // 定期检查卡住的URB - 缩短检查间隔
                let current_time = Self::get_clock_count();
                if current_time.saturating_sub(self.last_stall_check_time) > STALL_CHECK_INTERVAL_MS
                {
                    self.check_stalled_urbs();
                    self.last_stall_check_time = current_time;
                }

                // 确保维持最小URB队列深度（类似Linux实现）
                if urbs_to_return.len() < URB_QUEUE_DEPTH && pending_count < URB_QUEUE_DEPTH {
                    let needed = URB_QUEUE_DEPTH - pending_count - urbs_to_return.len();
                    if needed > 0 {
                        trace!("维持URB队列深度：需要额外提交 {} 个URB", needed);
                        for i in 0..self.isoc_urbs.len() {
                            if urbs_to_return.len() >= URB_QUEUE_DEPTH - pending_count {
                                break;
                            }

                            let should_submit = if let Some(urb_info) = self.isoc_urbs.get(i) {
                                urb_info.state == IsocURBState::Idle
                            } else {
                                false
                            };

                            if should_submit {
                                if let Some(urb) = self.create_isoc_urb_for_idx(i) {
                                    if let Some(urb_info_mut) = self.isoc_urbs.get_mut(i) {
                                        urb_info_mut.state = IsocURBState::Pending;
                                        urb_info_mut.submit_time = Self::get_clock_count();
                                    }
                                    trace!("✅ 添加URB #{} 以维持队列深度", i);
                                    urbs_to_return.push(urb);
                                }
                            }
                        }
                    }
                }

                if !urbs_to_return.is_empty() {
                    info!("提交 {} 个等时传输URB（异步模式）", urbs_to_return.len());
                    return Some(urbs_to_return);
                } else {
                    info!("⚠️ 异步模式下没有收集到任何URB，返回None");
                }

                return None;
            } else {
                // 同步模式下的处理
                let mut urbs_to_submit = Vec::new();
                let mut idle_count = 0;
                let mut pending_count = 0;

                // 先统计URB状态
                for urb_info in &self.isoc_urbs {
                    match urb_info.state {
                        IsocURBState::Idle => idle_count += 1,
                        IsocURBState::Pending => pending_count += 1,
                        _ => {}
                    }
                }

                trace!(
                    "📊 URB状态: 空闲={}, 待处理={}, 总数={}",
                    idle_count,
                    pending_count,
                    self.isoc_urbs.len()
                );

                // 优先处理待重新提交的URB（类似Linux的立即重新提交）
                if !self.pending_resubmit_urbs.is_empty() {
                    info!(
                        "🚀 优先处理 {} 个待重新提交的URB（同步模式）",
                        self.pending_resubmit_urbs.len()
                    );
                    let pending_urbs = self.pending_resubmit_urbs.drain(..).collect::<Vec<_>>();

                    for urb_idx in pending_urbs {
                        if let Some(urb_info) = self.isoc_urbs.get(urb_idx) {
                            if urb_info.state == IsocURBState::Idle {
                                if let Some(urb) = self.create_isoc_urb_for_idx(urb_idx) {
                                    // 更新URB状态
                                    if let Some(urb_info_mut) = self.isoc_urbs.get_mut(urb_idx) {
                                        urb_info_mut.state = IsocURBState::Pending;
                                        urb_info_mut.submit_time = Self::get_clock_count();
                                    }
                                    info!("✅ 重新提交URB #{} (优先队列，同步模式)", urb_idx);
                                    urbs_to_submit.push(urb);
                                }
                            }
                        }
                    }
                }

                for i in 0..self.isoc_urbs.len() {
                    if let Some(urb_info_ref) = self.isoc_urbs.get(i) {
                        if urb_info_ref.state == IsocURBState::Idle {
                            if let Some(urb) = self.create_isoc_urb_for_idx(i) {
                                if let Some(urb_info_mut_ref) = self.isoc_urbs.get_mut(i) {
                                    let current_time = Self::get_clock_count();
                                    urb_info_mut_ref.state = IsocURBState::Pending;
                                    urb_info_mut_ref.submit_time = current_time;
                                }
                                trace!("✅ 添加等时传输URB #{} 到提交队列", i);
                                urbs_to_submit.push(urb);
                            }
                        } else {
                            trace!("URB #{} 状态为 {:?}，跳过", i, urb_info_ref.state);
                        }
                    }
                }

                let current_time = Self::get_clock_count();
                if current_time.saturating_sub(self.last_stall_check_time) > STALL_CHECK_INTERVAL_MS
                {
                    self.check_stalled_urbs();
                    self.last_stall_check_time = current_time;
                }

                // 如果有URB需要提交，返回它们
                if !urbs_to_submit.is_empty() {
                    info!("提交 {} 个等时传输URB（同步模式）", urbs_to_submit.len());
                    return Some(urbs_to_submit);
                }
            }
        }

        // 若没有需要提交的URB，返回None
        None
    }

    fn receive_complete_event(&mut self, ucb: UCB<O>) {
        trace!(
            "UVC驱动收到事件: {:?}, 当前init_stage: {:?}, camera_state: {:?}",
            ucb.code,
            self.init_stage,
            self.camera_state
        );

        let mut urb_idx_opt: Option<usize> = None;
        // 使用新的辅助方法从UCB提取TRB指针
        if let Some(trb_ptr) = ucb.get_trb_pointer() {
            if trb_ptr != 0 {
                // 确保指针有效
                urb_idx_opt = self.urb_id_to_idx(trb_ptr);
                if urb_idx_opt.is_none() {
                    trace!("UCB事件中的TRB指针 0x{:X} 未映射到已知URB ID (可能是控制传输或未跟踪的ISOC URB)", trb_ptr);
                }
            } else {
                trace!("UCB事件中的TRB指针为0，忽略。");
            }
        } else {
            trace!(
                "UCB事件中未包含TRB指针 (ucb.get_trb_pointer() 返回 None)，依赖其他方式推断URB。"
            );
        }

        match ucb.code {
            CompleteCode::Event(event_code) => {
                match event_code {
                    TransferEventCompleteCode::Success(maybe_trb_ptr_from_code) => {
                        // 如果顶层 ucb.get_trb_pointer() 未解析出 urb_idx_opt，
                        // 且 Success 枚举本身也携带了指针，可以再次尝试
                        if urb_idx_opt.is_none() {
                            if let Some(trb_ptr) = maybe_trb_ptr_from_code {
                                if trb_ptr != 0 {
                                    urb_idx_opt = self.urb_id_to_idx(trb_ptr);
                                    if urb_idx_opt.is_none() {
                                        trace!("TransferEventCompleteCode::Success 中的TRB指针 0x{:X} 也未映射到已知URB ID", trb_ptr);
                                    }
                                }
                            }
                        }

                        self.consecutive_errors = 0;
                        let previous_init_stage = self.init_stage;

                        match self.init_stage {
                            UVCInitStage::Idle => {
                                info!("UVC初始化: SetConfiguration 和 (可能) SetPowerMode 已完成。开始UVC特定初始化。 init_stage 将在initialize_uvc_camera中从Idle推进。 ");
                            }
                            UVCInitStage::ProbeGetDefSent => {
                                // GET_DEF(Probe) URB 完成
                                info!("UVC初始化: GET_DEF(Probe)成功完成.");
                                if let Some(ref buffer) = self.probe_response_buffer {
                                    let data = buffer.lock();
                                    if data.len() >= 26 {
                                        trace!(
                                            "GET_DEF(Probe)响应数据 (前26字节): {:02X?}",
                                            &data[0..26]
                                        );
                                        if let Some(parsed_params) =
                                            VideoProbeCommitControl::from_bytes(&data[0..26])
                                        {
                                            info!("成功解析GET_DEF(Probe)响应:");
                                            self.print_probe_commit_details(&parsed_params);
                                            self.current_probe_settings = Some(parsed_params);
                                        } else {
                                            warn!("无法解析GET_DEF(Probe)响应数据，后续SET_CUR(Probe)将使用默认值。");
                                            self.current_probe_settings = None;
                                        }
                                    } else {
                                        warn!(
                                            "GET_DEF(Probe)响应数据太短 ({}字节)，无法解析",
                                            data.len()
                                        );
                                        self.current_probe_settings = None;
                                    }
                                } else {
                                    warn!(
                                        "probe_response_buffer 为 None，无法处理GET_DEF(Probe)响应"
                                    );
                                    self.current_probe_settings = None;
                                }
                                self.init_stage = UVCInitStage::ProbeSetSent; // 下一步: initialize_uvc_camera 将在 ProbeSetSent 状态发送 SET_CUR(Probe)
                            }
                            UVCInitStage::ProbeSetSent => {
                                // SET_CUR(Probe) URB 完成
                                info!("UVC初始化: SET_CUR(Probe)成功完成.");
                                // SET_CUR(Probe) 通常没有有意义的响应体需要解析。
                                // 下一步: initialize_uvc_camera 将在 ProbeGetSent 状态发送 GET_CUR(Probe)
                                self.init_stage = UVCInitStage::ProbeGetSent;
                                // 更新时间戳，以便ProbeGetSent状态能检测到刚进入
                                self.update_command_timestamp();
                            }
                            UVCInitStage::ProbeGetSent => {
                                // GET_CUR(Probe) URB 完成
                                info!("UVC初始化: GET_CUR(Probe)成功完成.");
                                if let Some(ref buffer) = self.probe_response_buffer {
                                    let data = buffer.lock();
                                    if data.len() >= 26 {
                                        trace!(
                                            "GET_CUR(Probe)响应数据 (前26字节): {:02X?}",
                                            &data[0..26]
                                        );
                                        if let Some(parsed_params) =
                                            VideoProbeCommitControl::from_bytes(&data[0..26])
                                        {
                                            info!("成功解析GET_CUR(Probe)响应:");
                                            self.print_probe_commit_details(&parsed_params);
                                            self.current_probe_settings = Some(parsed_params);
                                            if parsed_params.dwMaxPayloadTransferSize > 0 {
                                                self.current_packet_size =
                                                    parsed_params.dwMaxPayloadTransferSize as usize;
                                                trace!("从GET_CUR(Probe)响应更新 current_packet_size (dwMaxPayloadTransferSize) 为: {}", self.current_packet_size);
                                            } else {
                                                warn!("GET_CUR(Probe)响应中 dwMaxPayloadTransferSize 为0或无效。尝试使用GET_DEF的值或默认值。");
                                                let mut fallback_size_found = false;
                                                // 尝试使用之前GET_DEF(Probe)成功时存储的参数 (如果存在且有效)
                                                if let Some(def_params) =
                                                    self.last_successful_probe_params.as_ref()
                                                {
                                                    if def_params.dwMaxPayloadTransferSize > 0 {
                                                        self.current_packet_size = def_params
                                                            .dwMaxPayloadTransferSize
                                                            as usize;
                                                        trace!("dwMaxPayloadTransferSize为0，回退到 (last_successful_probe_params/GET_DEF) 的值: {}", self.current_packet_size);
                                                        fallback_size_found = true;
                                                    }
                                                }

                                                // 如果GET_DEF的值也无效，则尝试使用 Commit 的默认值
                                                if !fallback_size_found {
                                                    let default_commit_params =
                                                        VideoProbeCommitControl::default_for_commit(
                                                        );
                                                    if default_commit_params
                                                        .dwMaxPayloadTransferSize
                                                        > 0
                                                    {
                                                        self.current_packet_size =
                                                            default_commit_params
                                                                .dwMaxPayloadTransferSize
                                                                as usize;
                                                        trace!("dwMaxPayloadTransferSize为0，回退到 (default_for_commit) 的值: {}", self.current_packet_size);
                                                        fallback_size_found = true;
                                                    }
                                                }

                                                // 如果以上都无效，则使用硬编码的后备值
                                                if !fallback_size_found {
                                                    self.current_packet_size =
                                                        ISOC_PACKET_SIZE_ALT1; // 最后的硬编码后备 (1024)
                                                    warn!("dwMaxPayloadTransferSize仍为0且无有效后备，硬编码为: {}", self.current_packet_size);
                                                }
                                            }
                                            // 更新当前的帧间隔，以便SET_CUR(Commit)使用设备协商后的值
                                            if parsed_params.frame_interval > 0 {
                                                self.frame_interval = parsed_params.frame_interval;
                                                trace!("从GET_CUR(Probe)响应更新 frame_interval 为: {}", self.frame_interval);
                                            }
                                        } else {
                                            warn!("无法解析GET_CUR(Probe)响应数据。后续SET_CUR(Commit)可能使用旧的或默认参数。");
                                            // 如果解析失败，保留 self.current_probe_settings 和 self.current_packet_size 不变 (或使用更健壮的后备)
                                        }
                                    } else {
                                        warn!(
                                            "GET_CUR(Probe)响应数据太短 ({}字节)，无法解析。",
                                            data.len()
                                        );
                                    }
                                } else {
                                    warn!("probe_response_buffer 为 None，无法处理GET_CUR(Probe)响应。");
                                }
                                // GET_CUR(Probe)完成后，转换到CommitSetSent状态
                                // 这样下次initialize_uvc_camera()会发送SET_CUR(Commit)
                                self.init_stage = UVCInitStage::CommitSetSent;
                                // 更新时间戳，以便CommitSetSent状态能检测到刚进入
                                self.update_command_timestamp();
                                info!("GET_CUR(Probe)完成，状态转换到CommitSetSent，准备发送SET_CUR(Commit)");
                            }
                            UVCInitStage::CommitSetSent => {
                                info!("UVC初始化: SET_CUR(Commit)成功完成.");
                                self.init_stage = UVCInitStage::InterfaceSetSent;
                                // 更新时间戳，以便InterfaceSetSent状态能检测到刚进入
                                self.update_command_timestamp();
                            }
                            UVCInitStage::InterfaceSetSent => {
                                info!("UVC初始化: SET_INTERFACE成功完成.");
                                trace!("[InterfaceSetSent Event Success] self.isoc_alt_setting = {}, self.current_packet_size = {}", self.isoc_alt_setting, self.current_packet_size);

                                // Linux方式：SetInterface已经配置了端点，直接进入完成状态
                                info!("✅✅✅ UVC初始化成功完成! ✅✅✅");
                                info!("摄像头已准备就绪，等待接收视频数据...");
                                info!("摄像头状态: Configuring -> WaitingForFrame");
                                self.init_stage = UVCInitStage::Done;
                                self.camera_state = UVCCameraState::WaitingForFrame;
                                self.consecutive_errors = 0;
                                self.consecutive_zero_data_count = 0;

                                // 激活等时传输
                                if !self.isoc_active {
                                    info!("在SET_INTERFACE成功后激活等时传输");
                                    self.isoc_active = true;
                                    if self.isoc_urbs.is_empty() {
                                        self.init_isoc_urbs();
                                    }
                                }
                            }
                            UVCInitStage::XHCIConfigureDCI5 => {
                                info!("UVC初始化: XHCI DCI 配置命令 (ExtraStep 0xFE) 成功完成.");
                                info!("UVC初始化: XHCI DCI 配置完成。当前 UVC 驱动状态: isoc_endpoint_address=0x{:02X}, isoc_alt_setting={}, current_packet_size={}", 
                                      self.isoc_endpoint_address, self.isoc_alt_setting, self.current_packet_size);

                                info!("UVC初始化完成! 摄像头状态设置为 WaitingForFrame。");
                                self.init_stage = UVCInitStage::Done;
                                self.camera_state = UVCCameraState::WaitingForFrame;
                                self.consecutive_errors = 0;
                                self.consecutive_zero_data_count = 0;
                                if !self.isoc_active {
                                    info!("在XHCI DCI配置成功后激活等时传输");
                                    self.isoc_active = true;
                                    if self.isoc_urbs.is_empty() {
                                        self.init_isoc_urbs();
                                    }
                                }
                            }
                            UVCInitStage::StreamStartSent => {
                                warn!("UVC初始化: StreamStartSent 阶段被意外进入或完成，直接标记为Done。");
                                self.init_stage = UVCInitStage::Done;
                                self.camera_state = UVCCameraState::WaitingForFrame;
                                self.consecutive_errors = 0;
                                self.consecutive_zero_data_count = 0;
                                if !self.isoc_active {
                                    info!("激活等时传输 (从StreamStartSent完成)");
                                    self.isoc_active = true;
                                    if self.isoc_urbs.is_empty() {
                                        self.init_isoc_urbs();
                                    }
                                }
                                // No return here, function returns () implicitly
                            }
                            UVCInitStage::Done => {
                                // **MODIFICATION START**
                                // Ensure camera state is CollectingFrame BEFORE handling isoch data
                                if self.camera_state == UVCCameraState::WaitingForFrame {
                                    info!("UVC摄像头从 WaitingForFrame 进入帧收集状态 CollectingFrame (在处理isoc数据前)");
                                    self.camera_state = UVCCameraState::CollectingFrame;
                                }
                                // **MODIFICATION END**

                                if self.isoc_active {
                                    // 优先使用从事件中获取的精确 urb_idx
                                    if let Some(urb_idx_val) =
                                        urb_idx_opt.or_else(|| self.get_current_urb_idx())
                                    {
                                        debug!("等时传输成功 (URB #{})", urb_idx_val);
                                        self.handle_isoc_transfer_success(urb_idx_val);
                                    } else {
                                        warn!("收到等时传输成功事件，但无法确定对应的URB索引");
                                    }
                                }
                                /*
                                if self.camera_state == UVCCameraState::WaitingForFrame {
                                    info!("UVC摄像头从 WaitingForFrame 进入帧收集状态 CollectingFrame");
                                    self.camera_state = UVCCameraState::CollectingFrame;
                                }
                                */
                            }
                            UVCInitStage::Failed => {
                                info!("在Failed状态下收到成功事件，可能是之前的恢复操作成功了。尝试重新初始化。");
                                self.init_stage = UVCInitStage::Idle;
                                self.camera_state = UVCCameraState::Configuring;
                            }
                        }
                        trace!(
                            "UVC事件处理后: init_stage 从 {:?} 变为 {:?}",
                            previous_init_stage,
                            self.init_stage
                        );
                    }
                    TransferEventCompleteCode::Stall(maybe_trb_ptr_from_code) => {
                        if urb_idx_opt.is_none() {
                            if let Some(trb_ptr) = maybe_trb_ptr_from_code {
                                if trb_ptr != 0 {
                                    urb_idx_opt = self.urb_id_to_idx(trb_ptr);
                                }
                            }
                        }
                        error!("UVC驱动收到 STALL 事件. URB Index (推断): {:?}. 当前 init_stage: {:?}, camera_state: {:?}", urb_idx_opt, self.init_stage, self.camera_state);
                        self.consecutive_errors += 1;

                        match self.init_stage {
                            UVCInitStage::ProbeGetDefSent => {
                                warn!("GET_DEF(Probe)请求Stall，设备可能不支持。将使用预设的默认参数进行SET_CUR(Probe)。");
                                self.current_probe_settings = None;
                            }
                            UVCInitStage::ProbeSetSent => {
                                error!("SET_CUR(Probe)请求Stall。初始化失败。");
                                self.init_stage = UVCInitStage::Failed;
                            }
                            UVCInitStage::ProbeGetSent => {
                                error!("GET_CUR(Probe)请求Stall。初始化失败。");
                                self.init_stage = UVCInitStage::Failed;
                            }
                            UVCInitStage::CommitSetSent => {
                                error!("SET_CUR(Commit)请求Stall。初始化失败。");
                                self.init_stage = UVCInitStage::Failed;
                            }
                            UVCInitStage::InterfaceSetSent => {
                                error!("SET_INTERFACE请求Stall。");
                                let current_alt = self.isoc_alt_setting;
                                self.try_next_alternate_setting();
                                if self.isoc_alt_setting == current_alt
                                    && self.consecutive_errors > ERROR_THRESHOLD_SWITCH_ALT * 2
                                {
                                    error!("SET_INTERFACE 持续Stall，且无法切换alternate setting。标记初始化失败。");
                                    self.init_stage = UVCInitStage::Failed;
                                } else if self.isoc_alt_setting != current_alt {
                                    info!("因SET_INTERFACE Stall，切换到新的alternate setting，将重试SET_INTERFACE。");
                                } else {
                                    error!("SET_INTERFACE Stall，但无其他alternate setting可尝试或错误次数过多。标记初始化失败。");
                                    self.init_stage = UVCInitStage::Failed;
                                }
                            }
                            UVCInitStage::XHCIConfigureDCI5 => {
                                error!("XHCI DCI 配置命令 (ExtraStep) Stall。初始化失败。");
                                self.init_stage = UVCInitStage::Failed;
                            }
                            UVCInitStage::Done => {
                                warn!("等时传输Stall。 URB Index: {:?}", urb_idx_opt);
                                if let Some(urb_idx_val) =
                                    urb_idx_opt.or_else(|| self.get_current_urb_idx())
                                {
                                    if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx_val) {
                                        urb_info.state = IsocURBState::Idle;
                                    }

                                    // 重新提交URB以继续视频流
                                    if let Some(new_urb) = self.create_isoc_urb_for_idx(urb_idx_val)
                                    {
                                        if let Err(e) = self.submit_urb(new_urb, urb_idx_val) {
                                            error!(
                                                "重新提交Stall的URB #{} 失败: {:?}",
                                                urb_idx_val, e
                                            );
                                        } else {
                                            info!("成功重新提交Stall的URB #{}", urb_idx_val);
                                        }
                                    }
                                }
                            }
                            _ => {
                                error!(
                                    "在未知或不恰当的初始化阶段 {:?} 收到Stall。",
                                    self.init_stage
                                );
                                self.init_stage = UVCInitStage::Failed;
                            }
                        }
                    }
                    TransferEventCompleteCode::Babble(maybe_trb_ptr_from_code) => {
                        if urb_idx_opt.is_none() {
                            if let Some(trb_ptr) = maybe_trb_ptr_from_code {
                                if trb_ptr != 0 {
                                    urb_idx_opt = self.urb_id_to_idx(trb_ptr);
                                }
                            }
                        }
                        error!("UVC驱动收到 BABBLE 事件. URB Index (推断): {:?}. 当前 init_stage: {:?}, camera_state: {:?}", urb_idx_opt, self.init_stage, self.camera_state);
                        self.consecutive_errors += 1;
                        // Similar error handling as Stall or other errors
                        if self.init_stage != UVCInitStage::Done
                            && self.init_stage != UVCInitStage::Failed
                        {
                            self.init_stage = UVCInitStage::Failed;
                        } else if self.init_stage == UVCInitStage::Done {
                            if let Some(urb_idx_val) =
                                urb_idx_opt.or_else(|| self.get_current_urb_idx())
                            {
                                if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx_val) {
                                    urb_info.state = IsocURBState::Idle;
                                }
                                // Optionally resubmit
                            }
                        }
                    }
                    TransferEventCompleteCode::Timeout | TransferEventCompleteCode::Unknown(_) => {
                        // Handle Timeout and Unknown generically for now
                        error!("UVC驱动收到错误/超时事件: {:?}. URB Index (推断): {:?}. 当前 init_stage: {:?}, camera_state: {:?}", event_code, urb_idx_opt, self.init_stage, self.camera_state);
                        self.consecutive_errors += 1;

                        if self.init_stage != UVCInitStage::Done
                            && self.init_stage != UVCInitStage::Failed
                        {
                            error!(
                                "在初始化阶段 {:?} 发生错误 {:?}，标记为失败。",
                                self.init_stage, event_code
                            );
                            self.init_stage = UVCInitStage::Failed;
                        } else if self.init_stage == UVCInitStage::Done {
                            warn!("等时传输错误/超时。URB Index: {:?}", urb_idx_opt);
                            if let Some(urb_idx_val) =
                                urb_idx_opt.or_else(|| self.get_current_urb_idx())
                            {
                                if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx_val) {
                                    urb_info.state = IsocURBState::Idle;
                                }
                                if let Some(new_urb) = self.create_isoc_urb_for_idx(urb_idx_val) {
                                    if self.submit_urb(new_urb, urb_idx_val).is_err() {
                                        error!("重新提交错误/超时URB #{} 失败", urb_idx_val);
                                    }
                                }
                            }
                        }
                    }
                    // Consider if Halt needs specific handling or falls into Unknown/generic error
                    TransferEventCompleteCode::Halt(maybe_trb_ptr_from_code) => {
                        // <--- 修复 E0532
                        if urb_idx_opt.is_none() {
                            if let Some(trb_ptr) = maybe_trb_ptr_from_code {
                                if trb_ptr != 0 {
                                    urb_idx_opt = self.urb_id_to_idx(trb_ptr);
                                }
                            }
                        }
                        error!("UVC驱动收到 HALT 事件. URB Index (推断): {:?}. 当前 init_stage: {:?}, camera_state: {:?}", urb_idx_opt, self.init_stage, self.camera_state);
                        self.consecutive_errors += 1;
                        if self.init_stage != UVCInitStage::Done
                            && self.init_stage != UVCInitStage::Failed
                        {
                            self.init_stage = UVCInitStage::Failed;
                        } else if self.init_stage == UVCInitStage::Done {
                            // Similar to Stall on an active transfer
                            if let Some(urb_idx_val) =
                                urb_idx_opt.or_else(|| self.get_current_urb_idx())
                            {
                                if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx_val) {
                                    urb_info.state = IsocURBState::Idle;
                                }
                            }
                        }
                    }
                }
            }
            CompleteCode::XHCICMDError(cmd_completion_code) => {
                // <--- 修复 E0308：这个分支现在在顶层match ucb.code中
                use xhci::ring::trb::event::CompletionCode;

                error!("UVC驱动收到 XHCI 命令错误: {:?}", cmd_completion_code);

                // 特殊处理TrbError
                if let CompletionCode::TrbError = cmd_completion_code {
                    error!("检测到TrbError - TRB参数错误");

                    if self.init_stage == UVCInitStage::Done && self.isoc_active {
                        // 对于等时传输中的TRB错误
                        self.trb_error_count += 1;

                        if let Some(urb_idx) = urb_idx_opt.or_else(|| self.get_current_urb_idx()) {
                            warn!("TRB错误发生在URB #{}", urb_idx);

                            // 检查缓冲区对齐
                            if let Some(urb_info) = self.isoc_urbs.get(urb_idx) {
                                if let Some(buffer) = &urb_info.buffer {
                                    let addr = buffer.lock().addr();
                                    let len = buffer.lock().len();

                                    // 检查地址对齐（xHCI要求16字节对齐）
                                    if (addr & 0xF) != 0 {
                                        error!(
                                            "URB #{} 缓冲区地址未对齐: 0x{:X} (需要16字节对齐)",
                                            urb_idx, addr
                                        );
                                    }

                                    // 检查长度
                                    if len == 0 || len > 65536 {
                                        error!("URB #{} 缓冲区长度异常: {} 字节", urb_idx, len);
                                    }
                                }
                            }

                            // 重置URB状态
                            if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
                                urb_info.state = IsocURBState::Idle;
                            }
                        }

                        // 如果TRB错误过多，尝试重新初始化
                        if self.trb_error_count >= 5 {
                            error!("TRB错误过多({}次)，尝试重新配置端点", self.trb_error_count);
                            self.consecutive_errors = ERROR_THRESHOLD_SWITCH_ALT;
                            self.trb_error_count = 0;
                        }

                        return; // 不设置Failed状态，继续尝试
                    }
                }

                // 特殊处理MissedServiceError
                if let CompletionCode::MissedServiceError = cmd_completion_code {
                    warn!("检测到MissedServiceError - 帧调度失败");

                    if self.init_stage == UVCInitStage::Done && self.isoc_active {
                        // 增加MissedService计数
                        self.missed_service_count += 1;
                        self.consecutive_errors += 1;
                        warn!(
                            "等时传输帧调度错误 (第{}次)，当前帧延迟可能不适合",
                            self.missed_service_count
                        );

                        // 重置有问题的URB
                        if let Some(urb_idx) = urb_idx_opt.or_else(|| self.get_current_urb_idx()) {
                            if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
                                urb_info.state = IsocURBState::Idle;
                            }
                        }

                        // 如果连续多次MissedServiceError，尝试减少URB数量
                        if self.missed_service_count >= 3 && self.missed_service_count < 6 {
                            warn!(
                                "连续{}次MissedServiceError，减少并发URB数量",
                                self.missed_service_count
                            );
                            // 标记只使用一半的URB
                            for i in ISOC_NUM_URBS / 2..self.isoc_urbs.len() {
                                if let Some(urb_info) = self.isoc_urbs.get_mut(i) {
                                    urb_info.state = IsocURBState::Completed; // 临时禁用
                                }
                            }
                        } else if self.missed_service_count >= 6 {
                            warn!(
                                "过多MissedServiceError({}次)，尝试切换alternate setting",
                                self.missed_service_count
                            );
                            self.missed_service_count = 0; // 重置计数
                            self.consecutive_errors = ERROR_THRESHOLD_SWITCH_ALT;
                            // 触发切换
                        }

                        return; // 不设置Failed状态，继续尝试
                    }
                }

                // 特殊处理UsbTransactionError
                if let CompletionCode::UsbTransactionError = cmd_completion_code {
                    warn!("检测到UsbTransactionError - 设备可能还没准备好");

                    if self.init_stage == UVCInitStage::Done && self.isoc_active {
                        // 对于等时传输中的事务错误，继续尝试
                        self.transaction_error_count += 1;

                        if self.transaction_error_count < 20 {
                            info!(
                                "USB事务错误 #{}, 继续尝试（摄像头可能需要时间准备）",
                                self.transaction_error_count
                            );

                            // 重置URB状态以便重试
                            if let Some(urb_idx) =
                                urb_idx_opt.or_else(|| self.get_current_urb_idx())
                            {
                                if let Some(urb_info) = self.isoc_urbs.get_mut(urb_idx) {
                                    urb_info.state = IsocURBState::Idle;
                                }
                            }

                            return; // 不设置Failed状态，继续尝试
                        } else {
                            warn!(
                                "USB事务错误过多（{}次），将触发错误处理",
                                self.transaction_error_count
                            );
                        }
                    }
                }

                // 特殊处理RingOverrun
                if let CompletionCode::RingOverrun = cmd_completion_code {
                    warn!("检测到RingOverrun - 等时环溢出");
                    self.consecutive_errors += 1;

                    // 尝试减少同时活跃的URB数量
                    if self.init_stage == UVCInitStage::Done {
                        warn!("等时传输环溢出，尝试减少URB数量");
                        return; // 不设置Failed状态
                    }
                }

                // 其他错误照常处理
                self.init_stage = UVCInitStage::Failed;
                self.camera_state = UVCCameraState::Error;
            }
            _ => {
                warn!("收到未知完成代码: {:?}", ucb.code);
                self.consecutive_errors += 1;
                if self.init_stage != UVCInitStage::Done && self.init_stage != UVCInitStage::Failed
                {
                    self.init_stage = UVCInitStage::Failed;
                }
            }
        }

        if self.consecutive_errors >= ERROR_THRESHOLD_RESET {
            warn!(
                "连续错误次数 ({}) 达到重置阈值，将摄像头状态设置为Error，尝试完全重置。",
                self.consecutive_errors
            );
            self.camera_state = UVCCameraState::Error;
            self.init_stage = UVCInitStage::Idle;
        } else if self.consecutive_errors >= ERROR_THRESHOLD_SWITCH_ALT
            && (self.init_stage == UVCInitStage::Done
                || self.camera_state == UVCCameraState::Error
                || self.init_stage == UVCInitStage::InterfaceSetSent)
        {
            warn!(
                "连续错误次数 ({}) 达到切换阈值({}), 计划在下次gather_urb时切换alternate setting。",
                self.consecutive_errors, ERROR_THRESHOLD_SWITCH_ALT
            );
        }
        if self.init_stage == UVCInitStage::Failed {
            error!("UVC 初始化失败，摄像头状态设置为 Error。");
            self.camera_state = UVCCameraState::Error;
        }
    }
}
