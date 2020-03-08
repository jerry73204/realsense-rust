//! Defines the queue type of frames.

use crate::{
    error::{ErrorChecker, Result as RsResult},
    frame::{
        marker::{Any, FrameKind},
        Frame, GenericFrame,
    },
};
use std::{
    os::raw::{c_int, c_uint, c_void},
    ptr::NonNull,
    time::Duration,
};

/// The queue of frames.
#[derive(Debug)]
pub struct FrameQueue {
    ptr: NonNull<realsense_sys::rs2_frame_queue>,
}

impl FrameQueue {
    /// Creates an instance with given capacity.
    pub fn with_capacity(capacity: usize) -> RsResult<Self> {
        let queue = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_create_frame_queue(capacity as c_int, checker.inner_mut_ptr());
            checker.check()?;
            Self::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(queue)
    }

    /// Push a frame to the queue.
    pub fn push<Kind>(&mut self, frame: Frame<Kind>)
    where
        Kind: FrameKind,
    {
        unsafe {
            realsense_sys::rs2_enqueue_frame(
                frame.ptr.as_ptr(),
                self.ptr.cast::<c_void>().as_ptr(),
            );
        }
    }

    /// Pops a frame from queue.
    ///
    /// The method blocks until a frame is available.
    pub fn wait(&mut self, timeout: Duration) -> RsResult<Frame<Any>> {
        let frame = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_wait_for_frame(
                self.ptr.as_ptr(),
                timeout.as_millis() as c_uint,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Frame::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(frame)
    }

    /// Try to pop a frame and returns immediately.
    pub fn try_wait(&mut self) -> RsResult<Option<Frame<Any>>> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut ptr: *mut realsense_sys::rs2_frame = std::ptr::null_mut();
            let ret = realsense_sys::rs2_poll_for_frame(
                self.ptr.as_ptr(),
                &mut ptr as *mut _,
                checker.inner_mut_ptr(),
            );
            checker.check()?;

            if ret != 0 {
                let frame = Frame::from_ptr(NonNull::new(ptr).unwrap());
                Ok(Some(frame))
            } else {
                Ok(None)
            }
        }
    }

    pub(crate) fn from_ptr(ptr: NonNull<realsense_sys::rs2_frame_queue>) -> Self {
        Self { ptr }
    }
}

impl Drop for FrameQueue {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_frame_queue(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for FrameQueue {}
