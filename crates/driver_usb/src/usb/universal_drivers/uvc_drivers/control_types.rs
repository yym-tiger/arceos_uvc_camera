//! UVC控制类型定义

use log::warn;
use super::constants::{TARGET_FORMAT_INDEX, TARGET_FRAME_INDEX, TARGET_FRAME_INTERVAL};

/// VideoProbeCommitControl结构体定义
#[derive(Debug, Clone, Copy)]
pub struct VideoProbeCommitControl {
    pub bmHint: u16,                   // 提示字段
    pub format_index: u8,              // 视频格式索引
    pub frame_index: u8,               // 视频帧索引
    pub frame_interval: u32,           // 帧间隔
    pub wKeyFrameRate: u16,            // 关键帧率
    pub wPFrameRate: u16,              // 预测帧率
    pub wCompQuality: u16,             // 压缩质量
    pub wCompWindowSize: u16,          // 压缩窗口大小
    pub wDelay: u16,                   // 延迟时间
    pub dwMaxVideoFrameSize: u32,      // 最大视频帧大小
    pub dwMaxPayloadTransferSize: u32, // 最大有效负载大小
}

impl VideoProbeCommitControl {
    pub fn default() -> Self {
        // 更新默认值以匹配我们的新目标，尽管主要依赖于 GET_DEF 的结果或特定阶段的默认值
        Self {
            bmHint: 1,
            format_index: TARGET_FORMAT_INDEX,
            frame_index: TARGET_FRAME_INDEX,
            frame_interval: TARGET_FRAME_INTERVAL,
            wKeyFrameRate: 0,
            wPFrameRate: 0,
            wCompQuality: 0, // Match Alcor's GET_DEF
            wCompWindowSize: 0,
            wDelay: 0,
            dwMaxVideoFrameSize: 153600,    // 320x240 max frame size
            dwMaxPayloadTransferSize: 3072, // Match Alcor's GET_DEF
        }
    }

    // 为SET_CUR(Probe)提供一个可能更合适的默认值
    // This is used as fallback if GET_DEF fails or self.current_probe_settings is None
    pub fn default_for_probe_set() -> Self {
        Self {
            bmHint: 1, // D0: dwFrameInterval is supported/requested
            format_index: TARGET_FORMAT_INDEX,
            frame_index: TARGET_FRAME_INDEX,
            frame_interval: TARGET_FRAME_INTERVAL,
            wKeyFrameRate: 0,
            wPFrameRate: 0,
            wCompQuality: 0,
            wCompWindowSize: 0,
            wDelay: 0,
            dwMaxVideoFrameSize: 0,      // Must be 0 for SET_CUR(Probe)
            dwMaxPayloadTransferSize: 0, // Must be 0 for SET_CUR(Probe)
        }
    }

    // 为SET_CUR(Commit)提供一个默认值（基于协商结果，但这里作为后备）
    // This is used as fallback if GET_CUR(Probe) fails (parse_probe_response returns None)
    pub fn default_for_commit() -> Self {
        Self {
            bmHint: 1, // D0: dwFrameInterval is supported/requested
            format_index: TARGET_FORMAT_INDEX,
            frame_index: TARGET_FRAME_INDEX,
            frame_interval: TARGET_FRAME_INTERVAL,
            wKeyFrameRate: 0,
            wPFrameRate: 0,
            wCompQuality: 0, // Match Alcor's GET_DEF (previously was 1)
            wCompWindowSize: 0,
            wDelay: 0,
            dwMaxVideoFrameSize: 153600,    // 320x240 max frame size
            dwMaxPayloadTransferSize: 3072, // Match Alcor's GET_DEF
        }
    }

    // 将结构体转换为字节数组
    pub fn as_bytes(&self) -> [u8; 26] {
        let mut bytes = [0u8; 26];

        // 填充字节数组
        bytes[0..2].copy_from_slice(&self.bmHint.to_le_bytes());
        bytes[2] = self.format_index;
        bytes[3] = self.frame_index;
        bytes[4..8].copy_from_slice(&self.frame_interval.to_le_bytes());
        bytes[8..10].copy_from_slice(&self.wKeyFrameRate.to_le_bytes());
        bytes[10..12].copy_from_slice(&self.wPFrameRate.to_le_bytes());
        bytes[12..14].copy_from_slice(&self.wCompQuality.to_le_bytes());
        bytes[14..16].copy_from_slice(&self.wCompWindowSize.to_le_bytes());
        bytes[16..18].copy_from_slice(&self.wDelay.to_le_bytes());
        bytes[18..22].copy_from_slice(&self.dwMaxVideoFrameSize.to_le_bytes());
        bytes[22..26].copy_from_slice(&self.dwMaxPayloadTransferSize.to_le_bytes());

        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 26 {
            warn!(
                "VideoProbeCommitControl::from_bytes: 数据太短 ({}字节)，需要至少26字节",
                bytes.len()
            );
            return None;
        }
        Some(Self {
            bmHint: u16::from_le_bytes([bytes[0], bytes[1]]),
            format_index: bytes[2],
            frame_index: bytes[3],
            frame_interval: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            wKeyFrameRate: u16::from_le_bytes([bytes[8], bytes[9]]),
            wPFrameRate: u16::from_le_bytes([bytes[10], bytes[11]]),
            wCompQuality: u16::from_le_bytes([bytes[12], bytes[13]]),
            wCompWindowSize: u16::from_le_bytes([bytes[14], bytes[15]]),
            wDelay: u16::from_le_bytes([bytes[16], bytes[17]]),
            dwMaxVideoFrameSize: u32::from_le_bytes([bytes[18], bytes[19], bytes[20], bytes[21]]),
            dwMaxPayloadTransferSize: u32::from_le_bytes([
                bytes[22], bytes[23], bytes[24], bytes[25],
            ]),
        })
    }
}