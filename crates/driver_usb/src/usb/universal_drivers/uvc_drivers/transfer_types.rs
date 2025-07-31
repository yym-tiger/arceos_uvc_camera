//! 异步传输类型定义

use alloc::sync::Arc;
use alloc::collections::BTreeMap;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use spinlock::SpinNoIrq;
use crate::abstractions::PlatformAbstractions;
use crate::glue::ucb::CompleteCode;

use super::camera_state::{UVCInitStage, UVCCameraState};
use super::GenericUVCDriver;

/// 异步传输ID
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct AsyncTransferId(u64);

impl AsyncTransferId {
    pub fn new() -> Self {
        static COUNTER: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(1);
        Self(COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed))
    }
}

/// 异步传输状态
#[derive(Clone)]
pub enum AsyncTransferState {
    /// 等待提交
    Pending,
    /// 已提交，等待完成
    Submitted,
    /// 已完成，存储完成代码
    Completed(CompleteCode),
}

/// 异步传输信息
pub struct AsyncTransferInfo {
    pub id: AsyncTransferId,
    pub state: AsyncTransferState,
    pub waker: Option<Waker>,
}

/// 传输模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransferMode {
    /// 同步模式：阻塞式传输，等待完成后才继续
    Sync,
    /// 异步模式：非阻塞式传输，使用Future机制
    Async,
    /// 混合模式：初始化阶段使用同步，数据传输阶段使用异步
    Hybrid,
}

impl Default for TransferMode {
    fn default() -> Self {
        TransferMode::Hybrid // 默认使用混合模式
    }
}

impl TransferMode {
    /// 判断在给定阶段是否应该使用同步模式
    pub fn should_use_sync_for_stage(&self, stage: UVCInitStage) -> bool {
        match self {
            TransferMode::Sync => true,
            TransferMode::Async => false,
            TransferMode::Hybrid => {
                // 混合模式：初始化阶段使用同步，数据传输阶段使用异步
                !matches!(stage, UVCInitStage::Done)
            }
        }
    }

    /// 判断在当前摄像头状态下是否应该使用同步模式
    pub fn should_use_sync_for_camera_state(&self, camera_state: UVCCameraState) -> bool {
        match self {
            TransferMode::Sync => true,
            TransferMode::Async => false,
            TransferMode::Hybrid => {
                // 混合模式：数据传输阶段使用异步模式
                !matches!(
                    camera_state,
                    UVCCameraState::WaitingForFrame
                        | UVCCameraState::CollectingFrame
                        | UVCCameraState::CapturingPhoto
                )
            }
        }
    }
}

/// 异步传输Future
pub struct AsyncTransferFuture<O: PlatformAbstractions> {
    pub driver: Arc<SpinNoIrq<GenericUVCDriver<O>>>,
    pub transfer_id: AsyncTransferId,
}

impl<O: PlatformAbstractions> Future for AsyncTransferFuture<O> {
    type Output = crate::err::Result<CompleteCode>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut driver = self.driver.lock();

        if let Some(transfer_info) = driver.async_transfers.get_mut(&self.transfer_id) {
            match &transfer_info.state {
                AsyncTransferState::Completed(code) => {
                    let result = code.clone();
                    // 清理传输信息
                    driver.async_transfers.remove(&self.transfer_id);
                    Poll::Ready(Ok(result))
                }
                AsyncTransferState::Pending | AsyncTransferState::Submitted => {
                    // 保存waker以便完成时唤醒
                    transfer_info.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            }
        } else {
            // 传输不存在，可能已经被清理
            Poll::Ready(Err(crate::err::Error::InvalidSlot))
        }
    }
}