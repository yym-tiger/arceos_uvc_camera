use core::ptr;

use alloc::vec::Vec;
use const_enum::ConstEnum;
use log::{info, error, warn};

use super::UVCDescriptorTypes;

#[derive(ConstEnum, Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub(crate) enum UVCStandardVideoInterfaceClass {
    CC_Video = 0x0e,
}

#[derive(ConstEnum, Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub(crate) enum UVCInterfaceSubclass {
    UNDEFINED = 0x00,
    VIDEOCONTROL = 0x01,
    VIDEOSTREAMING = 0x02,
    VIDEO_INTERFACE_COLLECTION = 0x03,
}

#[derive(ConstEnum, Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub(crate) enum UVCControlInterfaceSubclass {
    DESCRIPTOR_UNDEFINED = 0x00,
    HEADER = 0x01,
    INPUT_TERMINAL = 0x02,
    OUTPUT_TERMINAL = 0x03,
    SELECTOR_UNIT = 0x04,
    PROCESSING_UNIT = 0x05,
    EXTENSION_UNIT = 0x06,
    ENCODING_UNIT = 0x07,
}

#[derive(ConstEnum, Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub(crate) enum UVCVSInterfaceSubclass {
    UNDEFINED = 0x00,
    INPUT_HEADER = 0x01,
    OUTPUT_HEADER = 0x02,
    STILL_IMAGE_FRAME = 0x03,
    FORMAT_UNCOMPRESSED = 0x04,
    FRAME_UNCOMPRESSED = 0x05,
    FORMAT_MJPEG = 0x06,
    FRAME_MJPEG = 0x07,
    FORMAT_MPEG2TS = 0x0A,
    FORMAT_DV = 0x0C,
    COLORFORMAT = 0x0D,
    FORMAT_FRAME_BASED = 0x10,
    FRAME_FRAME_BASED = 0x11,
    FORMAT_STREAM_BASED = 0x12,
    FORMAT_H264 = 0x13,
    FRAME_H264 = 0x14,
    FORMAT_H264_SIMULCAST = 0x15,
    FORMAT_VP8 = 0x16,
    FRAME_VP8 = 0x17,
    FORMAT_VP8_SIMULCAST = 0x18,
}

#[derive(ConstEnum, Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub(crate) enum UVCStandardVideoInterfaceProtocols {
    PC_PROTOCOL_UNDEFINED = 0x00,
    PC_PROTOCOL_15 = 0x01,
}

#[derive(Debug, Clone)]
pub enum UVCInterface {
    Control(UVCControlInterface),
    Streaming(UVCStreamingInterface),
}

#[derive(Debug, Clone)]
pub enum UVCControlInterface {
    Header(UVCControlInterfaceHeader),
    OutputTerminal(UVCControlInterfaceOutputTerminal),
    InputTerminal(UVCControlInterfaceInputTerminal),
    ExtensionUnit(UVCControlInterfaceExtensionUnit),
    ProcessingUnit(UVCControlInterfaceProcessingUnit),
}

#[derive(Debug, Clone)]
pub enum UVCStreamingInterface {
    InputHeader(UVCVSInterfaceInputHeader),
    OutputHeader(UVCVSInterfaceOutputHeader),
    StillImageFrame(UVCVSInterfaceStillImageFrame),
    FormatUncompressed(UVCVSInterfaceFormatUncompressed),
    FrameUncompressed(UVCVSInterfaceFrameUncompressed),
    FormatMjpeg(UVCVSInterfaceFormatMJPEG),
    FrameMjpeg(UVCVSInterfaceFrameMJPEG),
    FormatMpeg2ts,
    FormatDv,
    COLORFORMAT(UVCVSInterfaceColorFormat),
    FormatFrameBased,
    FrameFrameBased,
    FormatStreamBased,
    FormatH264,
    FrameH264,
    FormatH264Simulcast,
    FormatVp8,
    FrameVp8,
    FormatVp8Simulcast,
    Error(u8),
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCControlInterfaceHeader {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    bcd_uvc: u16,
    total_length: u16,
    clock_frequency: u32,
    in_collection: u8,
    interface_nr: Vec<u8>,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCControlInterfaceInputTerminal {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    terminal_id: u8,
    terminal_type: u16,
    associated_terminal: u8,
    string_index_terminal: u8,
    reserved: Vec<u8>,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCControlInterfaceOutputTerminal {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    terminal_id: u8,
    terminal_type: u16,
    associated_terminal: u8,
    source_id: u8,
    string_index_terminal: u8,
    reserved: Vec<u8>,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCControlInterfaceExtensionUnit {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    unit_id: u8,
    guid_extension_code: [u8; 16],
    num_controls: u8,
    nr_in_pins: u8,
    source_ids: Vec<u8>,
    control_size: u8,
    controls: Vec<u8>,
    extension: u8,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCControlInterfaceProcessingUnit {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    unit_id: u8,
    source_id: u8,
    max_multiplier: u16,
    control_size: u8,
    controls: [u8; 3],
    processing: u8,
    video_standards: u8,
}

#[derive(ConstEnum, Copy, Clone, Debug, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u16)]
pub(crate) enum UVCCONTROLOutputTerminalType {
    OTT_VendorSpec = 0x300,
    OTT_Display = 0x301,
    OTT_MEDIA_TRANSPORT_OUTPUT = 0x302,
    TT_VendorSpec = 0x0100,
    TT_Streaming = 0x0101,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceInputHeader {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    num_formats: u8,
    total_length: u16,
    endpoint_address: u8,
    info: u8,
    terminal_link: u8,
    still_capture_method: u8,
    trigger_support: u8,
    trigger_useage: u8,
    control_size: u8,
    interface_nr: Vec<u8>,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceFormatMJPEG {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    format_index: u8,
    num_frame_descriptors: u8,
    flags: u8,
    default_frame_index: u8,
    aspect_ratio_x: u8,
    aspect_ratio_y: u8,
    interlace_flags: u8,
    is_copy_protect: u8,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceFrameMJPEG {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    frame_index: u8,
    capabilities: u8,
    width: u16,
    height: u16,
    min_bit_rate: u32,
    max_bit_rate: u32,
    max_video_frame_buffer_size: u32,
    default_frame_interval: u32,
    frame_interval_type: u8,
    frame_interval: FrameInterval,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceStillImageFrame {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    endpoint_address: u8,
    num_image_size_paterns: u8,
    width_heights: Vec<(u16, u16)>,
    num_compression_pattern: u8,
    compressions: Vec<u8>,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceFormatUncompressed {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    format_index: u8,
    number_frame_descriptor: u8,
    guid_format: [u8; 16],
    bits_per_pixel: u8,
    default_frame_index: u8,
    aspect_ratio_x: u8,
    aspect_ratio_y: u8,
    m_interlace_flags: u8,
    is_copy_protect: u8,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceFrameUncompressed {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    frame_index: u8,
    capabilities: u8,
    width: u16,
    height: u16,
    min_bit_rate: u32,
    max_bit_rate: u32,
    max_video_frame_buffer_size: u32,
    default_frame_interval: u32,
    frame_interval_type: u8,
    frame_interval: FrameInterval,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub enum FrameInterval {
    Continuous((u32, u32, u32)),
    Discrete(Vec<u32>),
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceColorFormat {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    color_primaries: u8,
    transfer_characteristics: u8,
    matrix_coefficients: u8,
}

#[derive(Clone, Debug)]
#[allow(non_camel_case_types)]
pub struct UVCVSInterfaceOutputHeader {
    length: u8,
    descriptor_type: u8,
    descriptor_sub_type: u8,
    num_formats: u8,
    total_length: u16,
    endpoint_address: u8,
    terminal_link: u8,
    control_size: u8,
    format_info: Vec<u8>,
}

impl UVCControlInterface {
    pub fn from_u8_array(raw: &[u8]) -> Self {
        info!("buffer:{:?}", raw);
        let len = raw[0];
        let descriptor_type = raw[1];
        let descriptor_sub_type = raw[2];
        info!(
            "subtype{:?}",
            UVCControlInterfaceSubclass::from(descriptor_sub_type)
        );

        match UVCControlInterfaceSubclass::from(descriptor_sub_type) {
            UVCControlInterfaceSubclass::DESCRIPTOR_UNDEFINED => panic!("impossible"),
            UVCControlInterfaceSubclass::HEADER => Self::Header({
                info!("header!");
                let len_array_nr = len - 12;
                UVCControlInterfaceHeader {
                    length: len.clone(),
                    descriptor_type,
                    descriptor_sub_type,
                    bcd_uvc: if raw.len() > 4 { 
                        u16::from_ne_bytes(raw[3..=4].try_into().unwrap()) 
                    } else { 0 },
                    total_length: if raw.len() > 6 { 
                        u16::from_ne_bytes(raw[5..=6].try_into().unwrap()) 
                    } else { 0 },
                    clock_frequency: if raw.len() > 10 { 
                        u32::from_ne_bytes(raw[7..=10].try_into().unwrap()) 
                    } else { 0 },
                    in_collection: if raw.len() > 11 { raw[11] } else { 0 },
                    interface_nr: {
                        let mut vec = Vec::new();
                        if raw.len() > 12 && len > 12 {
                            for i in 12..(len as usize).min(raw.len()) {
                                vec.push(raw[i]);
                            }
                        }
                        vec
                    },
                }
            }),
            UVCControlInterfaceSubclass::INPUT_TERMINAL => {
                Self::InputTerminal(UVCControlInterfaceInputTerminal {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type,
                    terminal_id: if raw.len() > 3 { raw[3] } else { 0 },
                    terminal_type: if raw.len() > 5 { 
                        u16::from_ne_bytes(raw[4..=5].try_into().unwrap()) 
                    } else { 0 },
                    associated_terminal: if raw.len() > 6 { raw[6] } else { 0 },
                    string_index_terminal: if raw.len() > 7 { raw[7] } else { 0 },
                    reserved: if raw.len() > 8 && (len as usize) > 8 {
                        raw[8..(len as usize).min(raw.len())].to_vec()
                    } else {
                        Vec::new()
                    },
                })
            }
            UVCControlInterfaceSubclass::OUTPUT_TERMINAL => {
                Self::OutputTerminal(UVCControlInterfaceOutputTerminal {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type,
                    terminal_id: if raw.len() > 3 { raw[3] } else { 0 },
                    terminal_type: if raw.len() > 5 { 
                        u16::from_ne_bytes(raw[4..=5].try_into().unwrap()) 
                    } else { 0 },
                    associated_terminal: if raw.len() > 6 { raw[6] } else { 0 },
                    source_id: if raw.len() > 7 { raw[7] } else { 0 },
                    string_index_terminal: if raw.len() > 8 { raw[8] } else { 0 },
                    reserved: if raw.len() > 9 && (len as usize) > 9 {
                        raw[9..(len as usize).min(raw.len())].to_vec()
                    } else {
                        Vec::new()
                    },
                })
            }
            UVCControlInterfaceSubclass::SELECTOR_UNIT => todo!(),
            UVCControlInterfaceSubclass::PROCESSING_UNIT => {
                if raw.len() >= 11 { // 最小长度检查
                    Self::ProcessingUnit(UVCControlInterfaceProcessingUnit {
                        length: len,
                        descriptor_type,
                        descriptor_sub_type,
                        unit_id: if raw.len() > 3 { raw[3] } else { 0 },
                        source_id: if raw.len() > 4 { raw[4] } else { 0 },
                        max_multiplier: if raw.len() > 6 { 
                            u16::from_ne_bytes(raw[5..=6].try_into().unwrap())
                        } else { 0 },
                        control_size: if raw.len() > 7 { raw[7] } else { 0 },
                        controls: if raw.len() > 10 {
                            [
                                raw[8], 
                                if raw.len() > 9 { raw[9] } else { 0 }, 
                                raw[10]
                            ]
                        } else {
                            [0, 0, 0]
                        },
                        processing: if raw.len() > 11 { raw[11] } else { 0 },
                        video_standards: if raw.len() > 12 { raw[12] } else { 0 },
                    })
                } else {
                    info!("处理单元描述符不完整，创建默认值");
                    Self::ProcessingUnit(UVCControlInterfaceProcessingUnit {
                        length: len,
                        descriptor_type,
                        descriptor_sub_type,
                        unit_id: if raw.len() > 3 { raw[3] } else { 0 },
                        source_id: 0,
                        max_multiplier: 0,
                        control_size: 0,
                        controls: [0, 0, 0],
                        processing: 0,
                        video_standards: 0,
                    })
                }
            }
            UVCControlInterfaceSubclass::EXTENSION_UNIT => Self::ExtensionUnit({
                if raw.len() < 22 {
                    return Self::ExtensionUnit(UVCControlInterfaceExtensionUnit {
                        length: len,
                        descriptor_type,
                        descriptor_sub_type,
                        unit_id: if raw.len() > 3 { raw[3] } else { 0 },
                        guid_extension_code: [0; 16],
                        num_controls: 0,
                        nr_in_pins: 0,
                        source_ids: Vec::new(),
                        control_size: 0,
                        controls: Vec::new(),
                        extension: 0,
                    });
                }
                
                let nr_in_pins = raw[21];
                let last_in_pin = 22 + nr_in_pins as usize;
                
                if last_in_pin > raw.len() {
                    return Self::ExtensionUnit(UVCControlInterfaceExtensionUnit {
                        length: len,
                        descriptor_type,
                        descriptor_sub_type,
                        unit_id: raw[3],
                        guid_extension_code: {
                            let mut codes = [0u8; 16];
                            if raw.len() >= 20 {
                                codes.copy_from_slice(&raw[4..20]);
                            }
                            codes
                        },
                        num_controls: raw[20],
                        nr_in_pins,
                        source_ids: Vec::new(),
                        control_size: 0,
                        controls: Vec::new(),
                        extension: 0,
                    });
                }
                
                let in_pins = raw[22..last_in_pin].to_vec();

                let control_size = if last_in_pin < raw.len() { raw[last_in_pin] } else { 0 };
                let last_control = last_in_pin + 1 + control_size as usize;
                
                let controls = if last_in_pin + 1 < raw.len() && last_control <= raw.len() {
                    raw[last_in_pin + 1..last_control].to_vec()
                } else {
                    Vec::new()
                };

                UVCControlInterfaceExtensionUnit {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type,
                    unit_id: raw[3],
                    guid_extension_code: {
                        let mut codes = [0u8; 16];
                        if raw.len() >= 20 {
                            codes.copy_from_slice(&raw[4..20]);
                        }
                        codes
                    },
                    num_controls: raw[20],
                    nr_in_pins,
                    source_ids: in_pins,
                    control_size,
                    controls,
                    extension: if last_control < raw.len() { raw[last_control] } else { 0 },
                }
            }),
            UVCControlInterfaceSubclass::ENCODING_UNIT => todo!(),
        }
    }
}

impl UVCStreamingInterface {
    pub fn from_u8_array(raw: &[u8]) -> Self {
        info!("buffer:{:?}", raw);
        let len = raw[0];
        let descriptor_type = raw[1];
        let descriptor_sub_type = UVCVSInterfaceSubclass::from(raw[2]);
        info!("subtype{:?}", descriptor_sub_type);
        match descriptor_sub_type {
            UVCVSInterfaceSubclass::INPUT_HEADER => Self::InputHeader({
                if raw.len() < 13 {
                    error!("UVC输入标头描述符太短: 长度{}, 需要至少13字节", raw.len());
                    return Self::Error(descriptor_sub_type.into());
                }
                
                let interface_count = raw[12];
                let mut interfaces = Vec::new();
                
                if 13 + interface_count as usize <= raw.len() {
                    for i in 0..interface_count {
                        interfaces.push(raw[13 + i as usize]);
                    }
                } else {
                    error!("UVC输入标头描述符接口数组不完整: 需要{}个接口，但只有{}字节", interface_count, raw.len() - 13);
                }
                
                UVCVSInterfaceInputHeader {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type: descriptor_sub_type.into(),
                    num_formats: raw[3],
                    total_length: u16::from_ne_bytes(raw[4..=5].try_into().unwrap_or([0, 0])),
                    endpoint_address: raw[6],
                    info: raw[7],
                    terminal_link: raw[8],
                    still_capture_method: raw[9],
                    trigger_support: raw[10],
                    trigger_useage: raw[11],
                    control_size: interface_count,
                    interface_nr: interfaces,
                }
            }),
            UVCVSInterfaceSubclass::OUTPUT_HEADER => {
                info!("解析UVC输出标头: {:?}", raw);
                if raw.len() < 8 {
                    error!("UVC输出标头描述符太短: 长度{}, 需要至少8字节", raw.len());
                    return Self::Error(descriptor_sub_type.into());
                }
                
                let control_size = if raw.len() > 8 { raw[8] } else { 0 };
                Self::OutputHeader(UVCVSInterfaceOutputHeader {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type: descriptor_sub_type.into(),
                    num_formats: raw[3],
                    total_length: u16::from_ne_bytes(raw[4..=5].try_into().unwrap_or([0, 0])),
                    endpoint_address: raw[6],
                    terminal_link: raw[7],
                    control_size,
                    format_info: if raw.len() > 9 && (len as usize) > 9 {
                        raw[9..(len as usize).min(raw.len())].to_vec()
                    } else {
                        Vec::new()
                    },
                })
            },
            UVCVSInterfaceSubclass::STILL_IMAGE_FRAME => {
                let num_image_size_paterns = if raw.len() > 4 { raw[4] } else { 0 };
                let loc_num_compression_pattern = 5 + 4 * num_image_size_paterns as usize;
                
                let width_heights = if raw.len() > 5 && loc_num_compression_pattern <= raw.len() {
                    raw[5..loc_num_compression_pattern]
                        .chunks(4)
                        .map(|t| {
                            if t.len() >= 4 {
                                (
                                    u16::from_ne_bytes(t[0..=1].try_into().unwrap()),
                                    u16::from_ne_bytes(t[2..=3].try_into().unwrap()),
                                )
                            } else {
                                (0, 0)
                            }
                        })
                        .collect()
                } else {
                    Vec::new()
                };

                Self::StillImageFrame(UVCVSInterfaceStillImageFrame {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type: descriptor_sub_type.into(),
                    endpoint_address: if raw.len() > 3 { raw[3] } else { 0 },
                    num_image_size_paterns,
                    width_heights,
                    num_compression_pattern: if raw.len() > loc_num_compression_pattern { 
                        raw[loc_num_compression_pattern] 
                    } else { 0 },
                    compressions: if raw.len() > loc_num_compression_pattern + 1 && (len as usize) > loc_num_compression_pattern + 1 {
                        raw[loc_num_compression_pattern + 1..(len as usize)].to_vec()
                    } else {
                        Vec::new()
                    },
                })
            }
            UVCVSInterfaceSubclass::FORMAT_UNCOMPRESSED => {
                if raw.len() >= 27 { // FORMAT_UNCOMPRESSED结构体至少需要27字节(包括16字节GUID)
                    Self::FormatUncompressed(UVCVSInterfaceFormatUncompressed {
                        length: len,
                        descriptor_type,
                        descriptor_sub_type: descriptor_sub_type.into(),
                        format_index: if raw.len() > 3 { raw[3] } else { 0 },
                        number_frame_descriptor: if raw.len() > 4 { raw[4] } else { 0 },
                        guid_format: {
                            let mut guid = [0u8; 16];
                            if raw.len() >= 21 {
                                for i in 0..16 {
                                    if 5 + i < raw.len() {
                                        guid[i] = raw[5 + i];
                                    }
                                }
                            }
                            guid
                        },
                        bits_per_pixel: if raw.len() > 21 { raw[21] } else { 0 },
                        default_frame_index: if raw.len() > 22 { raw[22] } else { 0 },
                        aspect_ratio_x: if raw.len() > 23 { raw[23] } else { 0 },
                        aspect_ratio_y: if raw.len() > 24 { raw[24] } else { 0 },
                        m_interlace_flags: if raw.len() > 25 { raw[25] } else { 0 },
                        is_copy_protect: if raw.len() > 26 { raw[26] } else { 0 },
                    })
                } else {
                    error!("未压缩格式描述符不完整: 长度{}, 需要至少27字节", raw.len());
                    return Self::Error(descriptor_sub_type.into());
                }
            }
            UVCVSInterfaceSubclass::FRAME_UNCOMPRESSED => {
                let frame_interval_type = if raw.len() > 25 { raw[25] } else { 0 };
                
                let frame_interval = if raw.len() <= 26 {
                    FrameInterval::Discrete(Vec::new())
                } else {
                    match frame_interval_type {
                        0 => {
                            if raw.len() >= 38 { // 26 + 12 (3个u32需要12字节)
                                FrameInterval::Continuous((
                                    u32::from_ne_bytes(raw[26..30].try_into().unwrap()),
                                    u32::from_ne_bytes(raw[30..34].try_into().unwrap()),
                                    u32::from_ne_bytes(raw[34..38].try_into().unwrap()),
                                ))
                            } else {
                                info!("警告: 未压缩帧连续间隔字段不完整");
                                FrameInterval::Continuous((0, 0, 0))
                            }
                        },
                        other => {
                            let end_index = (26 + other * 4) as usize;
                            if end_index <= raw.len() {
                                FrameInterval::Discrete(
                                    raw[26..end_index]
                                        .chunks(4)
                                        .map(|c| {
                                            if c.len() == 4 {
                                                u32::from_ne_bytes(c.try_into().unwrap())
                                            } else {
                                                0
                                            }
                                        })
                                        .collect(),
                                )
                            } else {
                                info!("警告: 未压缩帧离散间隔数据不完整");
                                FrameInterval::Discrete(Vec::new())
                            }
                        }
                    }
                };

                Self::FrameUncompressed(UVCVSInterfaceFrameUncompressed {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type: descriptor_sub_type.into(),
                    frame_index: if raw.len() > 3 { raw[3] } else { 0 },
                    capabilities: if raw.len() > 4 { raw[4] } else { 0 },
                    width: if raw.len() > 6 { u16::from_ne_bytes(raw[5..=6].try_into().unwrap()) } else { 0 },
                    height: if raw.len() > 8 { u16::from_ne_bytes(raw[7..=8].try_into().unwrap()) } else { 0 },
                    min_bit_rate: if raw.len() > 12 { u32::from_ne_bytes(raw[9..13].try_into().unwrap()) } else { 0 },
                    max_bit_rate: if raw.len() > 16 { u32::from_ne_bytes(raw[13..17].try_into().unwrap()) } else { 0 },
                    max_video_frame_buffer_size: if raw.len() > 20 { 
                        u32::from_ne_bytes(raw[17..21].try_into().unwrap())
                    } else { 0 },
                    default_frame_interval: if raw.len() > 24 { 
                        u32::from_ne_bytes(raw[21..25].try_into().unwrap())
                    } else { 0 },
                    frame_interval_type,
                    frame_interval,
                })
            }
            UVCVSInterfaceSubclass::FORMAT_MJPEG => {
                if raw.len() < 11 {
                    warn!("MJPEG格式描述符不完整: 长度{}, 需要至少11字节", raw.len());
                    return Self::Error(descriptor_sub_type.into());
                }
                Self::FormatMjpeg(UVCVSInterfaceFormatMJPEG {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type: descriptor_sub_type.into(),
                    format_index: if raw.len() > 3 { raw[3] } else { 0 },
                    num_frame_descriptors: if raw.len() > 4 { raw[4] } else { 0 },
                    flags: if raw.len() > 5 { raw[5] } else { 0 },
                    default_frame_index: if raw.len() > 6 { raw[6] } else { 0 },
                    aspect_ratio_x: if raw.len() > 7 { raw[7] } else { 0 },
                    aspect_ratio_y: if raw.len() > 8 { raw[8] } else { 0 },
                    interlace_flags: if raw.len() > 9 { raw[9] } else { 0 },
                    is_copy_protect: if raw.len() > 10 { raw[10] } else { 0 },
                })
            }
            UVCVSInterfaceSubclass::FRAME_MJPEG => {
                let frame_interval_type = if raw.len() > 25 { raw[25] } else { 0 };
                
                let frame_interval = if raw.len() <= 26 {
                    FrameInterval::Discrete(Vec::new())
                } else {
                    match frame_interval_type {
                        0 => {
                            if raw.len() >= 38 { // 26 + 12 (3个u32需要12字节)
                                FrameInterval::Continuous((
                                    u32::from_ne_bytes(raw[26..30].try_into().unwrap()),
                                    u32::from_ne_bytes(raw[30..34].try_into().unwrap()),
                                    u32::from_ne_bytes(raw[34..38].try_into().unwrap()),
                                ))
                            } else {
                                info!("警告: MJPEG帧连续间隔字段不完整");
                                FrameInterval::Continuous((0, 0, 0))
                            }
                        },
                        other => {
                            let end_index = (26 + other * 4) as usize;
                            if end_index <= raw.len() {
                                FrameInterval::Discrete(
                                    raw[26..end_index]
                                        .chunks(4)
                                        .map(|c| {
                                            if c.len() == 4 {
                                                u32::from_ne_bytes(c.try_into().unwrap())
                                            } else {
                                                0
                                            }
                                        })
                                        .collect(),
                                )
                            } else {
                                info!("警告: MJPEG帧离散间隔数据不完整");
                                FrameInterval::Discrete(Vec::new())
                            }
                        }
                    }
                };

                Self::FrameMjpeg(UVCVSInterfaceFrameMJPEG {
                    length: len,
                    descriptor_type,
                    descriptor_sub_type: descriptor_sub_type.into(),
                    frame_index: if raw.len() > 3 { raw[3] } else { 0 },
                    capabilities: if raw.len() > 4 { raw[4] } else { 0 },
                    width: if raw.len() > 6 { u16::from_ne_bytes(raw[5..=6].try_into().unwrap()) } else { 0 },
                    height: if raw.len() > 8 { u16::from_ne_bytes(raw[7..=8].try_into().unwrap()) } else { 0 },
                    min_bit_rate: if raw.len() > 12 { u32::from_ne_bytes(raw[9..13].try_into().unwrap()) } else { 0 },
                    max_bit_rate: if raw.len() > 16 { u32::from_ne_bytes(raw[13..17].try_into().unwrap()) } else { 0 },
                    max_video_frame_buffer_size: if raw.len() > 20 { 
                        u32::from_ne_bytes(raw[17..21].try_into().unwrap())
                    } else { 0 },
                    default_frame_interval: if raw.len() > 24 { 
                        u32::from_ne_bytes(raw[21..25].try_into().unwrap())
                    } else { 0 },
                    frame_interval_type,
                    frame_interval,
                })
            }
            UVCVSInterfaceSubclass::COLORFORMAT => {
                if raw.len() >= 6 { // COLORFORMAT结构体至少需要6字节
                    Self::COLORFORMAT(UVCVSInterfaceColorFormat {
                        length: len,
                        descriptor_type,
                        descriptor_sub_type: descriptor_sub_type.into(),
                        color_primaries: if raw.len() > 3 { raw[3] } else { 0 },
                        transfer_characteristics: if raw.len() > 4 { raw[4] } else { 0 },
                        matrix_coefficients: if raw.len() > 5 { raw[5] } else { 0 },
                    })
                } else {
                    error!("颜色格式描述符不完整: 长度{}, 需要至少6字节", raw.len());
                    return Self::Error(descriptor_sub_type.into());
                }
            }
            UVCVSInterfaceSubclass::FORMAT_FRAME_BASED => todo!(),
            UVCVSInterfaceSubclass::FRAME_FRAME_BASED => todo!(),
            UVCVSInterfaceSubclass::FORMAT_STREAM_BASED => todo!(),
            UVCVSInterfaceSubclass::FORMAT_H264 => todo!(),
            UVCVSInterfaceSubclass::FRAME_H264 => todo!(),
            UVCVSInterfaceSubclass::FORMAT_H264_SIMULCAST => todo!(),
            UVCVSInterfaceSubclass::FORMAT_VP8 => todo!(),
            UVCVSInterfaceSubclass::FRAME_VP8 => todo!(),
            UVCVSInterfaceSubclass::FORMAT_VP8_SIMULCAST => todo!(),
            todo => {
                info!("未实现的UVC流接口子类型: {:?}", todo);
                Self::Error(todo.into())
            }
        }
    }
}
