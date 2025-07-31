use const_enum::ConstEnum;

#[derive(ConstEnum, Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub(crate) enum UVCVideoClassEndpointSubtypes {
    UNDEFINED = 0x00,
    GENERAL = 0x01,
    ENDPOINT = 0x02,
    INTERRUPT = 0x03,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
#[repr(C, packed)]
pub struct UVCVideoControlInterruptEndpoint {
    pub len: u8,                  // 字节0: 描述符长度
    pub descriptor_type: u8,      // 字节1: 描述符类型
    pub descriptor_sub_type: u8,  // 字节2: 描述符子类型
    // 字节3-4: 最大传输大小，处理为单个字节以匹配实际设备描述符
    pub max_transfer_size_low: u8, // 字节3: 最大传输大小低字节
    pub max_transfer_size_high: u8, // 字节4: 最大传输大小高字节 (可能不存在于某些设备)
}

impl UVCVideoControlInterruptEndpoint {
    // 添加方法用于获取完整的max_transfer_size
    pub fn max_transfer_size(&self) -> u16 {
        u16::from(self.max_transfer_size_low) | (u16::from(self.max_transfer_size_high) << 8)
    }
}

impl Default for UVCVideoControlInterruptEndpoint {
    fn default() -> Self {
        Self {
            len: 5,  // 最小长度，包含基本字段
            descriptor_type: 0x37, // 视频类特定CS_ENDPOINT
            descriptor_sub_type: UVCVideoClassEndpointSubtypes::INTERRUPT as u8,
            max_transfer_size_low: 16, // 默认值，通常足够小的中断传输
            max_transfer_size_high: 0, // 高字节为0
        }
    }
}
