use alloc::string::String;
use core::fmt::{write, Display};
use xhci::ring::trb::event::CompletionCode;

#[derive(Debug, Clone)]
pub enum Error {
    /// 缺少DMA初始化
    Dma,
    /// USB设备无效状态
    InvalidUsbState,
    /// 管道错误
    Pip,
    /// 缺少描述符
    NoDesc,
    /// 在控制管道上做禁止的操作
    DontDoThatOnControlPipe,
    /// XHCI完成代码错误
    CMD(xhci::ring::trb::event::CompletionCode),
    /// 超时错误
    Timeout,
    /// 未知错误
    Unknown(String),
    /// 参数错误
    Param(String),
    /// 传输被取消
    Cancelled,
    /// 无效的槽ID
    InvalidSlot,
    /// 环满
    RingFull,
}

impl Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Unknown(msg) => write!(f, "unknown usb err: {}", msg),
            Error::Param(msg) => write!(f, "param err: {}", msg),
            Error::Timeout => write!(f, "timeout"),
            Error::CMD(cmd) => write!(f, "cmd fail: {:#?}", cmd),
            Error::Pip => write!(f, "piped"),
            Error::DontDoThatOnControlPipe => {
                write!(f, "don't do that on controller pipe! illegal operation!")
            },
            Error::Dma => write!(f, "DMA error"),
            Error::InvalidUsbState => write!(f, "invalid USB state"),
            Error::NoDesc => write!(f, "no descriptor"),
            Error::Cancelled => write!(f, "transfer cancelled"),
            Error::InvalidSlot => write!(f, "invalid slot ID"),
            Error::RingFull => write!(f, "transfer ring is full"),
        }
    }
}

pub type Result<T = ()> = core::result::Result<T, Error>;
