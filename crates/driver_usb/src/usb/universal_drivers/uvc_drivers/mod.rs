// 模块声明
pub mod constants;
pub mod frame_processor;
pub mod statistics;
pub mod camera_state;
pub mod control_types;
pub mod transfer_types;
pub mod generic_uvc;

// 重新导出常用类型
pub use generic_uvc::{GenericUVCDriver, GenericUVCDriverModule};
pub use camera_state::{UVCInitStage, UVCCameraState, BufferState, IsocURBState, IsocURBInfo};
pub use control_types::VideoProbeCommitControl;
pub use frame_processor::{MjpegFrameProcessor, FrameState};
pub use statistics::FrameStatistics;
pub use transfer_types::{TransferMode, AsyncTransferId, AsyncTransferState, AsyncTransferInfo};

// 添加异步示例模块
#[cfg(feature = "async")]
pub mod async_example;
