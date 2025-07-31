/// 默认等时传输超时（毫秒）
pub const DEFAULT_ISOC_TIMEOUT_MS: u64 = 1000;

/// 等时传输请求
#[derive(Debug, Clone)]
pub struct IsochTransfer {
    /// 端点地址
    pub endpoint_id: usize,
    /// 缓冲区地址和长度
    pub buffer_addr_len: (usize, usize),
    /// 包数量
    pub num_packets: usize,
    /// 包大小
    pub packet_size: usize,
    /// 超时时间（毫秒）- 0表示使用默认值
    pub timeout_ms: u64,
}

impl IsochTransfer {
    /// 创建新的等时传输请求
    pub fn new(
        endpoint_id: usize,
        buffer_addr_len: (usize, usize),
        num_packets: usize,
        packet_size: usize,
    ) -> Self {
        Self {
            endpoint_id,
            buffer_addr_len,
            num_packets,
            packet_size,
            timeout_ms: DEFAULT_ISOC_TIMEOUT_MS,
        }
    }
    
    /// 设置自定义超时时间
    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }
    
    /// 获取此传输的超时时间
    pub fn get_timeout(&self) -> u64 {
        if self.timeout_ms == 0 {
            DEFAULT_ISOC_TIMEOUT_MS
        } else {
            self.timeout_ms
        }
    }
} 