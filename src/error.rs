use std::{
    ffi::CStr,
    fmt::{Debug, Display, Formatter, Result as FormatResult},
    ptr::NonNull,
};

pub(crate) struct ErrorChecker {
    checked: bool,
    ptr: *mut realsense_sys::rs2_error,
}

impl ErrorChecker {
    pub fn new() -> ErrorChecker {
        ErrorChecker {
            checked: false,
            ptr: std::ptr::null_mut(),
        }
    }

    pub fn inner_mut_ptr(&mut self) -> *mut *mut realsense_sys::rs2_error {
        &mut self.ptr as *mut _
    }

    pub fn check(mut self) -> Result<()> {
        self.checked = true;
        match NonNull::new(self.ptr) {
            Some(nonnull) => Err(Error::from_ptr(nonnull)),
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

pub struct Error {
    ptr: NonNull<realsense_sys::rs2_error>,
}

impl Error {
    pub(crate) fn from_ptr(ptr: NonNull<realsense_sys::rs2_error>) -> Self {
        Self { ptr }
    }
}

impl Display for Error {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        let message = unsafe {
            let ptr = realsense_sys::rs2_get_error_message(self.ptr.as_ptr());
            CStr::from_ptr(ptr).to_str().unwrap()
        };

        write!(formatter, "{}", message)
    }
}

impl Debug for Error {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        let message = unsafe {
            let ptr = realsense_sys::rs2_get_error_message(self.ptr.as_ptr());
            CStr::from_ptr(ptr).to_str().unwrap()
        };

        write!(formatter, "{}", message)
    }
}

impl Drop for Error {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_free_error(self.ptr.as_ptr());
        }
    }
}

pub type Result<T> = std::result::Result<T, Error>;
