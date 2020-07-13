//! Defines the queue type of frames.

use crate::{
    base::DEFAULT_TIMEOUT,
    common::*,
    error::{Error as RsError, ErrorChecker, Result as RsResult},
    frame::{
        marker::{Any, FrameKind},
        Frame, GenericFrame,
    },
};

/// The queue of frames.
#[derive(Debug)]
pub struct FrameQueue {
    pub(crate) ptr: NonNull<realsense_sys::rs2_frame_queue>,
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
    pub fn enqueue<Kind>(&mut self, frame: Frame<Kind>)
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
    pub fn wait(&mut self, timeout: Option<Duration>) -> RsResult<Frame<Any>> {
        let timeout_ms = timeout.unwrap_or(DEFAULT_TIMEOUT).as_millis() as c_uint;

        let frame = loop {
            let mut checker = ErrorChecker::new();
            let ptr = unsafe {
                realsense_sys::rs2_wait_for_frame(
                    self.ptr.as_ptr(),
                    timeout_ms,
                    checker.inner_mut_ptr(),
                )
            };

            match (timeout, checker.check()) {
                (None, Err(RsError::Timeout(..))) => continue,
                tuple => {
                    let (_, result) = tuple;
                    result?;
                }
            }

            let frame = unsafe { Frame::from_ptr(NonNull::new(ptr).unwrap()) };
            break frame;
        };
        Ok(frame)
    }

    /// Wait for frame asynchronously. It is analogous to [FrameQueue::wait]
    pub async fn wait_async(&mut self, timeout: Option<Duration>) -> RsResult<Frame<Any>> {
        let timeout_ms = timeout
            .map(|duration| duration.as_millis() as c_uint)
            .unwrap_or(realsense_sys::RS2_DEFAULT_TIMEOUT as c_uint);
        let (tx, rx) = futures::channel::oneshot::channel();
        let queue_ptr = AtomicPtr::new(self.ptr.as_ptr());

        std::thread::spawn(move || {
            let result = unsafe {
                loop {
                    let mut checker = ErrorChecker::new();
                    let ptr = realsense_sys::rs2_wait_for_frame(
                        queue_ptr.load(Ordering::Relaxed),
                        timeout_ms,
                        checker.inner_mut_ptr(),
                    );
                    let result = match (timeout, checker.check()) {
                        (None, Err(RsError::Timeout(..))) => continue,
                        (_, result) => result.map(|_| Frame::from_ptr(NonNull::new(ptr).unwrap())),
                    };
                    break result;
                }
            };
            let _ = tx.send(result);
        });

        let frame = rx.await.unwrap()?;
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

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_frame_queue>) -> Self {
        Self { ptr }
    }

    pub(crate) unsafe fn unsafe_clone(&self) -> Self {
        Self { ptr: self.ptr }
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
