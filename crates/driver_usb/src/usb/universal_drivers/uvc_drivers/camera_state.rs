//! UVC摄像头状态管理模块

use spinlock::SpinNoIrq;
use crate::abstractions::{dma::DMA, PlatformAbstractions};

/// UVC初始化阶段
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UVCInitStage {
    Idle,              // 初始状态
    ProbeGetDefSent,   // 已发送 GET_DEF(Probe)，获取默认值
    ProbeSetSent,      // 已发送 SET_CUR(Probe) - 新增状态
    ProbeGetSent,      // 已发送 GET_CUR(Probe)
    CommitSetSent,     // 已发送 SET_CUR(Commit)
    InterfaceSetSent,  // 已发送 SET_INTERFACE
    XHCIConfigureDCI5, // 新增：准备发送XHCI配置DCI 5的 ExtraStep
    StreamStartSent,   // 已发送流启动命令 (新增)
    Done,              // 初始化完成
    Failed,            // 初始化失败
}

/// UVC摄像头状态机
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UVCCameraState {
    /// 初始状态
    Idle,
    /// 等待帧
    WaitingForFrame,
    /// 正在收集视频帧
    CollectingFrame,
    /// 已请求拍照
    CapturingPhoto,
    /// 拍照完成
    PhotoCaptured,
    /// 错误状态
    Error,
    /// 配置状态
    Configuring,
}

/// 缓冲区状态
#[derive(Debug, Clone, PartialEq)]
pub enum BufferState {
    /// 空闲状态
    Idle,
    /// 正在填充
    Filling,
    /// 已填满
    Filled,
    /// 已锁定（用于拍照）
    Locked,
}

/// 等时URB状态
#[derive(Debug, Clone, PartialEq)]
pub enum IsocURBState {
    /// 空闲，未提交
    Idle,
    /// 已提交，等待完成
    Pending,
    /// 已完成
    Completed,
}

/// 等时URB信息
pub struct IsocURBInfo<O>
where
    O: PlatformAbstractions,
{
    /// URB状态
    pub state: IsocURBState,
    /// 缓冲区
    pub buffer: Option<SpinNoIrq<DMA<[u8], O::DMA>>>,
    /// URB序号
    pub index: usize,
    /// URB ID - 用于跟踪URB和完成事件之间的对应关系
    pub urb_id: usize,
    /// URB提交时间 - 用于检测卡住的URB
    pub submit_time: u64,
}