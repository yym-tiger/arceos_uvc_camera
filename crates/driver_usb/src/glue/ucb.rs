use core::marker::PhantomData;

use xhci::ring::trb::event::CompletionCode;

use crate::abstractions::PlatformAbstractions;

#[derive(Debug, Clone, Copy)]
pub struct UCB<O>
where
    O: PlatformAbstractions,
{
    //UCB A.K.A Usb Complete Block
    pub code: CompleteCode,
    _phantom_data: PhantomData<O>,
}

impl<O> UCB<O>
where
    O: PlatformAbstractions,
{
    pub fn new(code: CompleteCode) -> Self {
        Self {
            code,
            _phantom_data: PhantomData,
        }
    }

    // Helper method to extract TRB pointer if present in the event code
    pub fn get_trb_pointer(&self) -> Option<u64> {
        match &self.code {
            CompleteCode::Event(tec) => match tec {
                TransferEventCompleteCode::Success(ptr_opt) |
                TransferEventCompleteCode::Stall(ptr_opt) |
                TransferEventCompleteCode::Halt(ptr_opt) |
                TransferEventCompleteCode::Babble(ptr_opt) => *ptr_opt,
                // Add other cases if they also carry a TRB pointer
                _ => None, // Timeout, Unknown may not have a TRB pointer
            },
            CompleteCode::XHCICMDError(_) => None, // XHCI Command Errors don't carry the type of TRB pointer relevant here
            // Add other CompleteCode variants if they might wrap a TransferEventCompleteCode
            // For example, if you had EventWithData(TransferEventCompleteCode, _)
            // you would need to match that too.
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompleteCode {
    Event(TransferEventCompleteCode),
    XHCICMDError(xhci::ring::trb::event::CompletionCode),
    // If you have other variants like EventWithData that might wrap TransferEventCompleteCode,
    // they also need to be handled in UCB::get_trb_pointer
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferEventCompleteCode {
    Success(Option<u64>),
    Halt(Option<u64>),
    Stall(Option<u64>),
    Babble(Option<u64>),
    Timeout,
    Unknown(u8),
}
