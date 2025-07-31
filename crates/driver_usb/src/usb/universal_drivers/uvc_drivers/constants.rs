//! UVC驱动常量定义

// UVC格式和分辨率常量
pub const UVC_FORMAT_MJPEG: u8 = 0x01; // MJPEG格式索引
pub const TARGET_FORMAT_INDEX: u8 = 1; // MJPEG (bFormatIndex 1)
pub const TARGET_FRAME_INDEX: u8 = 2; // 320x240 (bFrameIndex 2 for MJPEG - 较低分辨率用于验证)
pub const TARGET_FRAME_INTERVAL: u32 = 333333; // 30 FPS (摄像头返回的实际帧率)
pub const TARGET_MAX_PAYLOAD_SIZE: u32 = 3072; // 使用摄像头协商的payload size

// 设备配置常量
pub const STANDARD_DELAY_MS: usize = 100; // 标准操作间隔延迟
pub const ISOC_PACKET_SIZE_ALT1: usize = 1024; // Alt1设置的包大小
pub const ISOC_PACKET_SIZE_ALT2: usize = 3072; // Alt2设置的包大小
pub const ISOC_PACKET_SIZE_ALT3: usize = 5120; // Alt3设置的包大小
pub const ISOC_PACKET_SIZE_ALT4: usize = 512; // Alt4设置的包大小

// 错误处理常量
pub const ERROR_THRESHOLD_SWITCH_ALT: usize = 5; // 切换alternate setting的错误阈值
pub const ERROR_THRESHOLD_RESET: usize = 10; // 完全重置设备的错误阈值

// UVC特定请求类型常量
pub const UVC_SET_CUR: u8 = 0x01; // SET_CUR请求
pub const UVC_GET_CUR: u8 = 0x81; // GET_CUR请求
pub const UVC_GET_INFO: u8 = 0x86; // GET_INFO请求
pub const UVC_GET_DEF: u8 = 0x87; // GET_DEF请求

// UVC控制请求常量
pub const UVC_VS_PROBE_CONTROL: u8 = 0x01; // 探测控制
pub const UVC_VS_COMMIT_CONTROL: u8 = 0x02; // 提交控制
pub const UVC_VC_REQUEST_ERROR_CODE_CONTROL: u8 = 0x02; // 错误代码控制
pub const UVC_VC_VIDEO_POWER_MODE_CONTROL: u8 = 0x01; // 电源模式控制

// 图像相关常量
pub const IMAGE_BUFFER_SIZE: usize = 320 * 240 * 2; // 图像缓冲区大小（足够320x240 MJPEG）
pub const MAX_FRAME_SIZE: usize = 100 * 1024; // 最大帧大小100KB（320x240 MJPEG压缩后的合理大小）

// 等时传输相关常量
pub const ISOC_NUM_URBS: usize = 8; // 等时传输URB数量
pub const ISOC_PACKETS_PER_URB: usize = 3; // 每个URB中的等时包数量 - 修改为3 (3072/1024)
pub const ISOC_PACKET_SIZE: usize = 1024; // 等时传输包大小（默认）

// 添加新常量
pub const MAX_ZERO_DATA_COUNT: usize = 10; // 允许的最大连续零数据包数
pub const VS_STREAM_ENABLE: u16 = 0x0100; // 视频流启用控制代码

// 优化后的错误检测和URB管理常量
pub const STALL_CHECK_INTERVAL_MS: u64 = 500; // 缩短到500ms进行更频繁的检查
pub const URB_STALL_TIMEOUT_MS: u64 = 1000; // 1秒后认为URB卡住（原为3秒）
pub const URB_QUEUE_DEPTH: usize = 4; // 保持至少4个URB在队列中

// 添加temp buffer最大大小限制 - 设置为60KB，适合大多数MJPEG帧
pub const TEMP_BUFFER_MAX_SIZE: usize = 40 * 1024; // 40KB，适合320x240 MJPEG帧大小范围