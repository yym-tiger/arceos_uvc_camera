//! MJPEG帧处理模块

use alloc::{format, string::String, vec::Vec};
use log::{info, warn};

use super::constants::TEMP_BUFFER_MAX_SIZE;

/// 帧处理状态
#[derive(Debug, Clone, PartialEq)]
pub enum FrameState {
    /// 等待帧起始
    WaitingForStart,
    /// 正在接收帧数据
    Receiving,
    /// 找到完整帧
    Complete,
}

/// MJPEG帧处理器
pub struct MjpegFrameProcessor {
    /// 帧处理状态
    pub state: FrameState,
    /// 当前帧大小
    pub current_size: usize,
    /// 已找到起始标记
    pub start_found: bool,
    /// 已找到结束标记
    pub end_found: bool,
    /// 帧起始位置
    pub start_offset: usize,
    /// 帧结束位置
    pub end_offset: usize,
}

impl MjpegFrameProcessor {
    pub fn new() -> Self {
        Self {
            state: FrameState::WaitingForStart,
            current_size: 0,
            start_found: false,
            end_found: false,
            start_offset: 0,
            end_offset: 0,
        }
    }

    pub fn find_soi(data: &[u8]) -> Option<usize> {
        data.windows(2).position(|w| w == [0xFF, 0xD8])
    }

    pub fn find_eoi(data: &[u8]) -> Option<usize> {
        data.windows(2).position(|w| w == [0xFF, 0xD9])
    }

    /// 重置帧处理器状态
    pub fn reset(&mut self) {
        self.state = FrameState::WaitingForStart;
        self.current_size = 0;
        self.start_found = false;
        self.end_found = false;
        self.start_offset = 0;
        self.end_offset = 0;
    }

    /// 处理数据块，查找MJPEG帧
    /// 返回值: (是否找到起始标记, 是否找到结束标记, 帧起始位置, 帧结束位置)
    pub fn process_data(&mut self, data: &[u8]) -> (bool, bool, usize, usize) {
        let mut start_found = false;
        let mut end_found = false;
        let mut start_pos = 0;
        let mut end_pos = 0;

        // 提前检查数据有效性
        if data.len() < 4 {
            return (false, false, 0, 0);
        }

        // 输出接收到的数据预览
        self.dump_data_preview(data);

        // 增强的起始标记检测 - 更严格的模式匹配
        if !self.start_found {
            for i in 0..data.len().saturating_sub(10) {
                // 标准JPEG帧头: SOI (0xFF,0xD8) 后通常跟着APP0 (0xFF,0xE0) 或 APP1 (0xFF,0xE1)
                // 或者其他JPEG段标记如DQT, DHT, SOF等
                let is_standard_jpeg = data[i] == 0xFF
                    && data[i + 1] == 0xD8
                    && ((i + 2 < data.len() && data[i + 2] == 0xFF)
                        && (i + 3 < data.len()
                            && (data[i + 3] == 0xE0
                                || data[i + 3] == 0xE1
                                || data[i + 3] == 0xDB
                                || data[i + 3] == 0xC0
                                || data[i + 3] == 0xC4)));

                // 简单SOI检测 - 作为备选
                let is_simple_soi = data[i] == 0xFF && data[i + 1] == 0xD8;

                if (is_standard_jpeg || (is_simple_soi && (i == 0 || data[i - 1] != 0xFF))) {
                    self.start_found = true;
                    self.start_offset = i;
                    start_found = true;
                    start_pos = i;
                    self.state = FrameState::Receiving;

                    // 如果找到标准格式，优先使用
                    if is_standard_jpeg {
                        break;
                    }
                }
            }
        }

        // 增强的帧结束标记检测
        if self.start_found && !self.end_found {
            // 双向搜索策略：从前向后和从后向前同时搜索

            // 从后向前搜索
            let mut i = data.len().saturating_sub(2);
            while i > 0 {
                if data[i] == 0xFF && data[i + 1] == 0xD9 {
                    // 验证这是真正的EOI而不是图像数据中的巧合
                    let is_valid_eoi = i == 0 || data[i - 1] != 0xFF;

                    if is_valid_eoi {
                        self.end_found = true;
                        self.end_offset = i + 2; // 包含结束标记
                        end_found = true;
                        end_pos = i + 2;
                        self.state = FrameState::Complete;

                        info!("MJPEG帧结束标记检测到: 位置={}", i);

                        // 计算帧大小
                        self.current_size = if self.start_offset <= self.end_offset {
                            self.end_offset - self.start_offset
                        } else {
                            warn!(
                                "帧边界异常: 结束位置({})小于起始位置({})",
                                self.end_offset, self.start_offset
                            );
                            0
                        };

                        break;
                    }
                }

                if i == 0 {
                    break;
                }
                i -= 1;
            }

            // 如果从后向前搜索未找到，再从前向后搜索
            if !end_found {
                for i in 0..data.len().saturating_sub(1) {
                    if data[i] == 0xFF && data[i + 1] == 0xD9 {
                        self.end_found = true;
                        self.end_offset = i + 2; // 包含结束标记
                        end_found = true;
                        end_pos = i + 2;
                        self.state = FrameState::Complete;

                        info!("MJPEG帧结束标记检测到: 位置={} (正向搜索)", i);

                        // 计算帧大小
                        self.current_size = if self.start_offset <= self.end_offset {
                            self.end_offset - self.start_offset
                        } else {
                            warn!(
                                "帧边界异常: 结束位置({})小于起始位置({})",
                                self.end_offset, self.start_offset
                            );
                            0
                        };

                        break;
                    }
                }
            }

            if !end_found && self.current_size > TEMP_BUFFER_MAX_SIZE {
                warn!(
                    "未找到MJPEG帧结束标记，但帧大小已达到{}KB，强制结束",
                    self.current_size / 1024
                );
                self.end_found = true;
                self.end_offset = data.len();
                end_found = true;
                end_pos = data.len();
                self.state = FrameState::Complete;
            }
        }

        // 更新帧大小（当找到起始标记但未找到结束标记时）
        if self.start_found && !self.end_found {
            // 在当前段中未找到结束标记，累加帧大小
            self.current_size += data.len();
        }

        (start_found, end_found, start_pos, end_pos)
    }

    /// 检查是否找到完整帧
    pub fn is_frame_complete(&self) -> bool {
        self.state == FrameState::Complete
    }

    /// 获取当前帧状态
    pub fn get_state(&self) -> FrameState {
        self.state.clone()
    }

    /// 获取当前帧大小
    pub fn get_frame_size(&self) -> usize {
        self.current_size
    }

    /// 获取帧起始偏移量
    pub fn get_start_offset(&self) -> usize {
        self.start_offset
    }

    /// 获取帧结束偏移量
    pub fn get_end_offset(&self) -> usize {
        self.end_offset
    }

    /// 验证帧的有效性 - 检查是否包含必要的JPEG标记
    pub fn validate_frame(&self, data: &[u8]) -> bool {
        // 检查数据长度是否合理
        if data.len() < 4 {
            return false;
        }

        // 检查JPEG头 (SOI)
        let has_soi = data[0] == 0xFF && data[1] == 0xD8;

        // 检查JPEG尾 (EOI)
        let has_eoi =
            data.len() >= 2 && data[data.len() - 2] == 0xFF && data[data.len() - 1] == 0xD9;

        // 检查是否包含至少一个基本的JPEG段标记
        let mut has_markers = false;
        for i in 2..data.len().saturating_sub(1) {
            if data[i] == 0xFF
                && (data[i+1] == 0xC0 || // SOF0
               data[i+1] == 0xC4 || // DHT
               data[i+1] == 0xDB || // DQT
               data[i+1] == 0xDA)
            {
                // SOS
                has_markers = true;
                break;
            }
        }

        has_soi && has_eoi && has_markers
    }

    // 添加用于数据预览的辅助方法
    pub fn dump_data_preview(&self, data: &[u8]) {
        // 输出调试信息 - 前16字节的十六进制表示
        let preview_len = core::cmp::min(16, data.len());
        let mut preview = String::with_capacity(preview_len * 3);
        for i in 0..preview_len {
            if i > 0 {
                preview.push(' ');
            }
            preview.push_str(&format!("{:02x}", data[i]));
        }

        // 查找任何与JPEG相关的标记
        let mut found_markers = Vec::new();
        for i in 0..data.len().saturating_sub(1) {
            if data[i] == 0xFF {
                match data.get(i + 1) {
                    Some(0xD8) => found_markers.push((i, "SOI")),
                    Some(0xD9) => found_markers.push((i, "EOI")),
                    Some(0xE0) => found_markers.push((i, "APP0")),
                    Some(0xE1) => found_markers.push((i, "APP1")),
                    Some(0xDB) => found_markers.push((i, "DQT")),
                    Some(0xC0) => found_markers.push((i, "SOF0")),
                    Some(0xC4) => found_markers.push((i, "DHT")),
                    Some(0xDA) => found_markers.push((i, "SOS")),
                    _ => {}
                }
            }
        }

        if !found_markers.is_empty() {
            let markers_str = found_markers
                .iter()
                .map(|(pos, name)| format!("{}@{}", name, pos))
                .collect::<Vec<_>>()
                .join(", ");
            info!("发现JPEG标记: {}", markers_str);
        }
    }
}