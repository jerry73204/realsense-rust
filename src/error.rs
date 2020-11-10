//! Defines the error type used by the crate.

use crate::common::*;

#[derive(Debug)]
pub(crate) struct ErrorChecker {
    checked: bool,
    ptr: *mut sys::rs2_error,
}

impl ErrorChecker {
    pub fn new() -> ErrorChecker {
        ErrorChecker {
            checked: false,
            ptr: ptr::null_mut(),
        }
    }

    pub fn inner_mut_ptr(&mut self) -> *mut *mut sys::rs2_error {
        &mut self.ptr as *mut _
    }

    pub fn check(mut self) -> Result<()> {
        self.checked = true;
        match NonNull::new(self.ptr) {
            Some(nonnull) => {
                let msg = get_error_message(nonnull);
                let err = if msg.starts_with("Frame didn't arrive within ") {
                    Error::Timeout(nonnull)
                } else if msg.starts_with("object doesn\'t support option #") {
                    Error::UnsupportedOption(nonnull)
                } else {
                    Error::Other(nonnull)
                };
                Err(err)
            }
            None => Ok(()),
        }
    }
}

impl Drop for ErrorChecker {
    fn drop(&mut self) {
        if !self.checked {
            panic!("internal error: forget to call check()");
        }
    }
}

/// The error type wraps around underlying error thrown by librealsense library.
pub enum Error {
    Timeout(NonNull<sys::rs2_error>),
    UnsupportedOption(NonNull<sys::rs2_error>),
    Other(NonNull<sys::rs2_error>),
}

impl Error {
    pub fn error_message(&self) -> &str {
        get_error_message(self.ptr())
    }

    pub fn into_raw(self) -> *mut sys::rs2_error {
        let ptr = self.ptr().as_ptr();
        mem::forget(self);
        ptr
    }

    pub(crate) fn ptr(&self) -> NonNull<sys::rs2_error> {
        match *self {
            Error::Timeout(ptr) => ptr,
            Error::UnsupportedOption(ptr) => ptr,
            Error::Other(ptr) => ptr,
        }
    }
}

impl Display for Error {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        let message = self.error_message();
        write!(formatter, "RealSense error: {}", message)
    }
}

impl Debug for Error {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        let message = self.error_message();
        write!(formatter, "RealSense error: {}", message)
    }
}

impl StdError for Error {}

unsafe impl Send for Error {}

unsafe impl Sync for Error {}

impl Drop for Error {
    fn drop(&mut self) {
        let ptr = self.ptr();
        unsafe {
            sys::rs2_free_error(ptr.as_ptr());
        }
    }
}

/// A convenient alias Result type.
pub type Result<T> = result::Result<T, Error>;

fn get_error_message<'a>(ptr: NonNull<sys::rs2_error>) -> &'a str {
    unsafe {
        let ptr = sys::rs2_get_error_message(ptr.as_ptr());
        CStr::from_ptr(ptr).to_str().unwrap()
    }
}
