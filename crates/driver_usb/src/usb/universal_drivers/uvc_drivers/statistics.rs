//! UVC驱动帧统计模块

/// 帧统计信息
#[derive(Clone)]
pub struct FrameStatistics {
    /// 总共收到的帧数
    pub total_frames: usize,
    /// 有效帧数
    pub valid_frames: usize,
    /// 无效帧数
    pub invalid_frames: usize,
    /// 超大帧数
    pub oversized_frames: usize,
    /// 不完整帧数
    pub incomplete_frames: usize,
    /// 平均帧大小
    pub average_frame_size: usize,
}

impl FrameStatistics {
    pub fn new() -> Self {
        Self {
            total_frames: 0,
            valid_frames: 0,
            invalid_frames: 0,
            oversized_frames: 0,
            incomplete_frames: 0,
            average_frame_size: 0,
        }
    }

    pub fn update_with_frame(
        &mut self,
        size: usize,
        is_valid: bool,
        is_oversized: bool,
        is_incomplete: bool,
    ) {
        self.total_frames += 1;

        if is_valid {
            self.valid_frames += 1;

            // 更新平均帧大小 - 使用滑动平均算法
            if self.average_frame_size == 0 {
                self.average_frame_size = size;
            } else {
                self.average_frame_size = (self.average_frame_size * 3 + size) / 4;
                // 赋予新帧0.25的权重
            }
        } else {
            self.invalid_frames += 1;
        }

        if is_oversized {
            self.oversized_frames += 1;
        }

        if is_incomplete {
            self.incomplete_frames += 1;
        }
    }

    pub fn reset(&mut self) {
        self.total_frames = 0;
        self.valid_frames = 0;
        self.invalid_frames = 0;
        self.oversized_frames = 0;
        self.incomplete_frames = 0;
        self.average_frame_size = 0;
    }
}