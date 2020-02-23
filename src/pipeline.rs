use crate::{
    config::Config,
    context::Context,
    error::{ErrorChecker, Result as RsResult},
    frame::{marker::Composite, Frame},
    pipeline_profile::PipelineProfile,
};
use std::{mem::MaybeUninit, os::raw::c_uint, ptr::NonNull, time::Duration};

pub mod marker {
    use super::*;

    pub trait PipelineState {}

    #[derive(Debug)]
    pub struct Active {
        pub profile: PipelineProfile,
        pub config: Option<Config>,
    }

    impl PipelineState for Active {}

    #[derive(Debug)]
    pub struct Inactive;

    impl PipelineState for Inactive {}
}

#[derive(Debug)]
pub struct Pipeline<State>
where
    State: marker::PipelineState,
{
    ptr: NonNull<realsense_sys::rs2_pipeline>,
    context: Context,
    state: State,
}

impl Pipeline<marker::Inactive> {
    pub fn new() -> RsResult<Self> {
        let context = Context::new()?;
        let pipeline = Self::from_context(context)?;
        Ok(pipeline)
    }

    pub fn from_context(context: Context) -> RsResult<Self> {
        let ptr = {
            let mut checker = ErrorChecker::new();
            let ptr = unsafe {
                realsense_sys::rs2_create_pipeline(context.ptr.as_ptr(), checker.inner_mut_ptr())
            };
            checker.check()?;
            ptr
        };

        let pipeline = Self {
            ptr: NonNull::new(ptr).unwrap(),
            context,
            state: marker::Inactive,
        };
        Ok(pipeline)
    }

    pub fn start(self, config: Option<Config>) -> RsResult<Pipeline<marker::Active>> {
        let ptr = match &config {
            Some(conf) => unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = realsense_sys::rs2_pipeline_start_with_config(
                    self.ptr.as_ptr(),
                    conf.ptr.as_ptr(),
                    checker.inner_mut_ptr(),
                );
                checker.check()?;
                ptr
            },
            None => unsafe {
                let mut checker = ErrorChecker::new();
                // let ptr = realsense_sys::rs2_pipeline_start_with_callback(
                //     self.ptr.as_ptr(),
                //     Some(callback),
                //     std::ptr::null_mut(),
                //     checker.inner_mut_ptr(),
                // );
                let ptr =
                    realsense_sys::rs2_pipeline_start(self.ptr.as_ptr(), checker.inner_mut_ptr());
                checker.check()?;
                ptr
            },
        };

        let profile = unsafe { PipelineProfile::from_ptr(NonNull::new(ptr).unwrap()) };
        let pipeline = unsafe {
            let (ptr, context) = self.take();
            Pipeline {
                ptr,
                context,
                state: marker::Active {
                    profile,
                    config,
                    // polling: Mutex::new(PollingState::None),
                },
            }
        };

        Ok(pipeline)
    }
}

impl Pipeline<marker::Active> {
    pub fn profile(&self) -> &PipelineProfile {
        &self.state.profile
    }

    pub fn wait(&self, timeout: Option<Duration>) -> RsResult<Frame<Composite>> {
        let timeout_ms = timeout
            .map(|duration| duration.as_millis() as c_uint)
            .unwrap_or(realsense_sys::RS2_DEFAULT_TIMEOUT as c_uint);
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_pipeline_wait_for_frames(
                self.ptr.as_ptr(),
                timeout_ms,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };

        let frame = Frame::from_ptr(NonNull::new(ptr).unwrap());
        Ok(frame)
    }

    pub fn try_wait(&self) -> RsResult<Option<Frame<Composite>>> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut ptr: *mut realsense_sys::rs2_frame = std::ptr::null_mut();
            let ret = realsense_sys::rs2_pipeline_poll_for_frames(
                self.ptr.as_ptr(),
                &mut ptr as *mut _,
                checker.inner_mut_ptr(),
            );

            if let Err(err) = checker.check() {
                return Err(err);
            }

            if ret != 0 {
                let frame = Frame::from_ptr(NonNull::new(ptr).unwrap());
                Ok(Some(frame))
            } else {
                Ok(None)
            }
        }
    }

    pub fn stop(self) -> RsResult<Pipeline<marker::Inactive>> {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_pipeline_stop(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
        }

        let pipeline = unsafe {
            let (ptr, context) = self.take();
            Pipeline {
                ptr,
                context,
                state: marker::Inactive,
            }
        };

        Ok(pipeline)
    }
}

impl<State> Pipeline<State>
where
    State: marker::PipelineState,
{
    unsafe fn take(mut self) -> (NonNull<realsense_sys::rs2_pipeline>, Context) {
        // take fields without invoking drop()
        let ptr = std::mem::replace(&mut self.ptr, { MaybeUninit::uninit().assume_init() });
        let context = std::mem::replace(&mut self.context, { MaybeUninit::uninit().assume_init() });
        std::mem::forget(self);

        (ptr, context)
    }
}

// impl Future for Pipeline<marker::Active> {
//     type Output = RsResult<Frame>;

//     fn poll(self: Pin<&mut Self>, cx: &mut AsyncContext) -> Poll<Self::Output> {
//         let waker = cx.waker();

//         std::thread::spawn(move || {
//             let result = self.wait();
//             waker.wake();
//         });
//     }
// }

impl<State> Drop for Pipeline<State>
where
    State: marker::PipelineState,
{
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_pipeline(self.ptr.as_ptr());
        }
    }
}

unsafe impl<State> Send for Pipeline<State> where State: marker::PipelineState {}

// extern "C" fn callback(frame_ptr: *mut realsense_sys::rs2_frame, arg2: *mut c_void) {
//     let frame = Frame::from_ptr(NonNull::new(frame_ptr).unwrap());
//     println!("{:?}\t{:?}", frame.number(), frame.timestamp());
// }
