use crate::{
    config::Config,
    context::Context,
    error::{ErrorChecker, Result as RsResult},
    frame::{marker::Composite, Frame},
    pipeline_profile::PipelineProfile,
};
use std::{
    mem::MaybeUninit,
    os::raw::c_uint,
    ptr::NonNull,
    sync::atomic::{AtomicPtr, Ordering},
    time::Duration,
};

/// Marker traits and types for [Pipeline].
pub mod marker {
    use super::*;

    /// Marker trait for pipeline marker types.
    pub trait PipelineState {}

    /// A marker type indicating the [Pipeline] is started.
    #[derive(Debug)]
    pub struct Active {
        pub profile: PipelineProfile,
        pub config: Option<Config>,
    }

    impl PipelineState for Active {}

    /// A marker type indicating the [Pipeline] is stopped.
    #[derive(Debug)]
    pub struct Inactive;

    impl PipelineState for Inactive {}
}

/// Represents the data pipeline from a RealSense device.
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
    /// Creates an instance.
    pub fn new() -> RsResult<Self> {
        let context = Context::new()?;
        let pipeline = Self::from_context(context)?;
        Ok(pipeline)
    }

    /// Consumes a context and creates an instance.
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

    /// Start the pipeline with optional config.
    ///
    /// The method consumes inactive pipeline itself, and returns the started pipeine.
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
                let ptr =
                    realsense_sys::rs2_pipeline_start(self.ptr.as_ptr(), checker.inner_mut_ptr());
                checker.check()?;
                ptr
            },
        };

        let profile = unsafe { PipelineProfile::from_ptr(NonNull::new(ptr).unwrap()) };
        let pipeline = unsafe {
            let (ptr, context, _) = self.take();
            Pipeline {
                ptr,
                context,
                state: marker::Active { profile, config },
            }
        };

        Ok(pipeline)
    }

    /// Start the pipeline asynchronously. It is analogous to [Pipeline::start].
    pub async fn start_async(self, config: Option<Config>) -> RsResult<Pipeline<marker::Active>> {
        let pipeline_ptr = AtomicPtr::new(self.ptr.as_ptr());
        let config_ptr_opt = config
            .as_ref()
            .map(|conf| AtomicPtr::new(conf.ptr.as_ptr()));
        let (tx, rx) = futures::channel::oneshot::channel();

        // start blocking thread
        std::thread::spawn(move || {
            let func = || unsafe {
                let profile_ptr = match config_ptr_opt {
                    Some(config_ptr) => {
                        let mut checker = ErrorChecker::new();
                        let ptr = realsense_sys::rs2_pipeline_start_with_config(
                            pipeline_ptr.load(Ordering::SeqCst),
                            config_ptr.load(Ordering::SeqCst),
                            checker.inner_mut_ptr(),
                        );
                        checker.check()?;
                        ptr
                    }
                    None => {
                        let mut checker = ErrorChecker::new();
                        let ptr = realsense_sys::rs2_pipeline_start(
                            pipeline_ptr.load(Ordering::SeqCst),
                            checker.inner_mut_ptr(),
                        );
                        checker.check()?;
                        ptr
                    }
                };
                Ok(AtomicPtr::new(profile_ptr))
            };
            let result = func();
            let _ = tx.send(result);
        });

        let profile_ptr = rx.await.unwrap()?;
        let profile = unsafe {
            PipelineProfile::from_ptr(NonNull::new(profile_ptr.load(Ordering::SeqCst)).unwrap())
        };
        let pipeline = unsafe {
            let (ptr, context, _) = self.take();
            Pipeline {
                ptr,
                context,
                state: marker::Active { profile, config },
            }
        };

        Ok(pipeline)
    }
}

impl Pipeline<marker::Active> {
    /// Gets the profile of pipeline.
    pub fn profile(&self) -> &PipelineProfile {
        &self.state.profile
    }

    /// Block and wait for next frame.
    pub fn wait(&mut self, timeout: Option<Duration>) -> RsResult<Frame<Composite>> {
        let timeout_ms = timeout
            .map(|duration| duration.as_millis() as c_uint)
            .unwrap_or(realsense_sys::RS2_DEFAULT_TIMEOUT as c_uint);
        let frame = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_pipeline_wait_for_frames(
                self.ptr.as_ptr(),
                timeout_ms,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            let frame = Frame::from_ptr(NonNull::new(ptr).unwrap());
            frame
        };

        Ok(frame)
    }

    /// Poll if next frame is immediately available.
    ///
    /// Unlike [Pipeline::start], the method does not block and returns None
    /// if next from is not available.
    pub fn try_wait(&mut self) -> RsResult<Option<Frame<Composite>>> {
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

    /// Wait for frame asynchronously. It is analogous to [Pipeline::wait]
    pub async fn wait_async(&mut self, timeout: Option<Duration>) -> RsResult<Frame<Composite>> {
        let timeout_ms = timeout
            .map(|duration| duration.as_millis() as c_uint)
            .unwrap_or(realsense_sys::RS2_DEFAULT_TIMEOUT as c_uint);
        let (tx, rx) = futures::channel::oneshot::channel();
        let pipeline_ptr = AtomicPtr::new(self.ptr.as_ptr());

        std::thread::spawn(move || {
            let func = || unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = realsense_sys::rs2_pipeline_wait_for_frames(
                    pipeline_ptr.load(Ordering::SeqCst),
                    timeout_ms,
                    checker.inner_mut_ptr(),
                );
                checker.check()?;
                let frame = Frame::from_ptr(NonNull::new(ptr).unwrap());
                Ok(frame)
            };
            let result = func();
            let _ = tx.send(result);
        });

        let frame = rx.await.unwrap()?;
        Ok(frame)
    }

    /// Stop the pipeline.
    ///
    /// This method consumes the pipeline instance and returns pipeline markered inactive.
    pub fn stop(self) -> RsResult<Pipeline<marker::Inactive>> {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_pipeline_stop(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
        }

        let pipeline = unsafe {
            let (ptr, context, _) = self.take();
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
    unsafe fn take(mut self) -> (NonNull<realsense_sys::rs2_pipeline>, Context, State) {
        // take fields without invoking drop()
        let ptr = std::mem::replace(&mut self.ptr, { MaybeUninit::uninit().assume_init() });
        let context = std::mem::replace(&mut self.context, { MaybeUninit::uninit().assume_init() });
        let state = std::mem::replace(&mut self.state, { MaybeUninit::uninit().assume_init() });
        std::mem::forget(self);

        (ptr, context, state)
    }
}

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
