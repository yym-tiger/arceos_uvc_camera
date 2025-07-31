//
use core::ptr;

use alloc::{collections, vec, vec::Vec};
use const_enum::ConstEnum;
use desc_configuration::Configuration;
use desc_device::Device;
use desc_endpoint::Endpoint;
use desc_hid::{HIDDescriptorTypes, Hid};
use desc_interface::{Interface, InterfaceAssociation};
use desc_str::Str;
use desc_uvc::{
    uvc_endpoints::UVCVideoControlInterruptEndpoint,
    uvc_interfaces::{
        UVCControlInterface, UVCControlInterfaceHeader, UVCInterface, UVCInterfaceSubclass,
        UVCStreamingInterface,
    },
    UVCDescriptorTypes,
};
use log::{debug, info, warn};
use num_derive::{FromPrimitive, ToPrimitive};
use num_traits::FromPrimitive;
use parser::{Error, ParserMetaData};
use tock_registers::interfaces;

use crate::abstractions::{dma::DMA, PlatformAbstractions};

pub mod parser;
pub mod topological_desc;

pub mod desc_configuration;
pub mod desc_device;
pub mod desc_endpoint;
pub mod desc_hid;
pub mod desc_interface;
pub mod desc_str;
pub mod desc_uvc;

#[allow(non_camel_case_types)]
#[derive(FromPrimitive, ToPrimitive, Copy, Clone, Debug, PartialEq, ConstEnum)]
#[repr(u8)]
pub(crate) enum USBStandardDescriptorTypes {
    //USB 1.1: 9.4 Standard Device Requests, Table 9-5. Descriptor Types
    Device = 0x01,
    Configuration = 0x02,
    String = 0x03,
    Interface = 0x04,
    Endpoint = 0x05,
    // USB 2.0: 9.4 Standard Device Requests, Table 9-5. Descriptor Types
    DeviceQualifier = 0x06,
    OtherSpeedConfiguration = 0x07,
    InterfacePower1 = 0x08,
    // USB 3.0+: 9.4 Standard Device Requests, Table 9-5. Descriptor Types
    OTG = 0x09,
    Debug = 0x0a,
    InterfaceAssociation = 0x0b,
    Bos = 0x0f,
    DeviceCapability = 0x10,
    SuperSpeedEndpointCompanion = 0x30,
    SuperSpeedPlusIsochEndpointCompanion = 0x31,
}

pub(crate) fn construct_control_transfer_type(
    desc: u8,
    index: u8,
) -> DescriptionTypeIndexPairForControlTransfer {
    DescriptionTypeIndexPairForControlTransfer { ty: desc, i: index }
}

pub(crate) struct DescriptionTypeIndexPairForControlTransfer {
    ty: u8,
    i: u8,
}

impl DescriptionTypeIndexPairForControlTransfer {
    pub(crate) fn bits(self) -> u16 {
        (self.ty as u16) << 8 | u16::from(self.i)
    }
}

#[derive(Clone, Debug)]
pub(crate) enum USBDescriptor {
    Device(Device),
    Configuration(Configuration),
    Str(Str),
    Interface(Interface),
    InterfaceAssociation(InterfaceAssociation),
    Endpoint(Endpoint),
    Hid(Hid),
    UVCInterface(UVCInterface),
    UVCClassSpecVideoControlInterruptEndpoint(UVCVideoControlInterruptEndpoint),
}

impl USBDescriptor {
    pub(crate) fn from_slice(raw: &[u8], metadata: ParserMetaData) -> Result<Self, Error> {
        if raw.is_empty() {
            return Err(Error::UnrecognizedType(0));
        }
        
        // 检查描述符长度是否合理
        let claimed_len = raw[0] as usize;
        if claimed_len == 0 || claimed_len > raw.len() {
            info!("描述符长度不合理: 声明长度 {} 实际长度 {}", claimed_len, raw.len());
            return Err(Error::UnrecognizedType(raw[0]));
        }
        
        match Self::from_slice_standard_usb(raw) {
            Ok(okay) => Ok(okay),
            Err(_) if let ParserMetaData::HID = metadata => Self::from_slice_hid(raw),
            Err(_) if let ParserMetaData::UVC(flag) = metadata => Self::from_slice_uvc(raw, flag),
            Err(any) => panic!("unknown situation {:?},{:?}", any, metadata),
        }
    }

    pub(crate) fn from_slice_uvc(raw: &[u8], flag: u8) -> Result<Self, Error> {
        info!("from slice uvc!{:?}", raw);
        
        if raw.len() < 3 {
            info!("UVC描述符太短，至少需要3字节");
            return Err(Error::UnrecognizedType(raw.len() as u8));
        }
        
        match UVCDescriptorTypes::from(raw[1]) {
            UVCDescriptorTypes::UVCClassSpecUnderfined => panic!("underfined!"),
            UVCDescriptorTypes::UVCClassSpecDevice => todo!(),
            UVCDescriptorTypes::UVCClassSpecConfiguration => todo!(),
            UVCDescriptorTypes::UVCClassSpecString => todo!(),
            UVCDescriptorTypes::UVCClassSpecInterface => {
                match UVCInterfaceSubclass::from(if flag == 0 { raw[2] } else { flag }) {
                    UVCInterfaceSubclass::UNDEFINED => panic!("impossible!"),
                    UVCInterfaceSubclass::VIDEOCONTROL => Ok(Self::UVCInterface(
                        UVCInterface::Control(UVCControlInterface::from_u8_array(raw)),
                    )),
                    UVCInterfaceSubclass::VIDEOSTREAMING => Ok(Self::UVCInterface(
                        UVCInterface::Streaming(UVCStreamingInterface::from_u8_array(raw)),
                    )),
                    UVCInterfaceSubclass::VIDEO_INTERFACE_COLLECTION => {
                        panic!("this subclass only appear in iac, impossible here!");
                    }
                }
            }
            UVCDescriptorTypes::UVCClassSpecVideoControlInterruptEndpoint => {
                // 使用原始数据直接解析，而不创建默认值
                let actual_size = raw.len();
                
                // 检查是否至少有5字节，这是最小的必要长度
                if actual_size < 5 {
                    // 如果长度小于5字节，则确实太短，无法解析
                    warn!("UVC端点描述符太短无法解析: {} 字节, 最少需要5字节", actual_size);
                    return Err(Error::UnrecognizedType(raw.len() as u8));
                }
                
                // 安全地从原始数据创建端点描述符结构
                let mut endpoint = UVCVideoControlInterruptEndpoint {
                    len: raw[0],
                    descriptor_type: raw[1],
                    descriptor_sub_type: raw[2],
                    max_transfer_size_low: raw[3],
                    max_transfer_size_high: if actual_size >= 6 { raw[4] } else { 0 },
                };
                
                // 如果实际长度和声明长度不匹配，进行警告但继续处理
                if endpoint.len as usize != actual_size {
                    warn!("UVC端点描述符声明长度({})与实际长度({})不匹配", 
                         endpoint.len, actual_size);
                }
                
                info!("解析UVC端点描述符: 长度={}, 类型={}, 子类型={}, 最大传输大小={}", 
                      endpoint.len, endpoint.descriptor_type, endpoint.descriptor_sub_type, 
                      endpoint.max_transfer_size());
                
                Ok(Self::UVCClassSpecVideoControlInterruptEndpoint(endpoint))
            }
        }
    }

    pub(crate) fn from_slice_hid(raw: &[u8]) -> Result<Self, Error> {
        match HIDDescriptorTypes::from(raw[1]) {
            HIDDescriptorTypes::Hid => {
                Ok(Self::Hid(unsafe { ptr::read((raw as *const [u8]).cast()) }))
            }
            HIDDescriptorTypes::HIDReport => todo!(),
            HIDDescriptorTypes::HIDPhysical => todo!(),
        }
    }

    pub(crate) fn from_slice_standard_usb(raw: &[u8]) -> Result<Self, Error> {
        match USBStandardDescriptorTypes::from_u8(raw[1]) {
            Some(t) => {
                let raw: *const [u8] = raw;
                match t {
                    // SAFETY: This operation is safe because the length of `raw` is equivalent to the
                    // one of the descriptor.
                    USBStandardDescriptorTypes::Device => {
                        Ok(Self::Device(unsafe { ptr::read(raw.cast()) }))
                    }
                    USBStandardDescriptorTypes::Configuration => {
                        Ok(Self::Configuration(unsafe { ptr::read(raw.cast()) }))
                    }
                    USBStandardDescriptorTypes::String => {
                        Ok(Self::Str(unsafe { ptr::read(raw.cast()) }))
                    }
                    USBStandardDescriptorTypes::Interface => {
                        Ok(Self::Interface(unsafe { ptr::read(raw.cast()) }))
                    }
                    USBStandardDescriptorTypes::Endpoint => {
                        Ok(Self::Endpoint(unsafe { ptr::read(raw.cast()) }))
                    }
                    USBStandardDescriptorTypes::InterfaceAssociation => {
                        Ok(Self::InterfaceAssociation(unsafe { ptr::read(raw.cast()) }))
                    }
                    other => {
                        unimplemented!("please implement descriptor type:{:?}", other)
                    }
                }
            }
            None => Err(Error::UnrecognizedType(raw[1])),
        }
    }
}

#[derive(Copy, Clone, ConstEnum)]
#[repr(u8)]
pub enum PortSpeed {
    FullSpeed = 1,
    LowSpeed = 2,
    HighSpeed = 3,
    SuperSpeed = 4,
    SuperSpeedPlus = 5,
}
