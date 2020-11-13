//! Defines the pipeline type.

use crate::{
    base::DEFAULT_TIMEOUT,
    common::*,
    config::Config,
    context::Context,
    error::{Error as RsError, ErrorChecker, Result},
    frame::{CompositeFrame, Frame, GenericFrameEx},
    pipeline_kind::{self, PipelineState},
    pipeline_profile::PipelineProfile,
};

/// Represents the data pipeline from a RealSense device.
#[derive(Debug)]
pub struct Pipeline<State>
where
    State: pipeline_kind::PipelineState,
{
    pub(crate) ptr: NonNull<sys::rs2_pipeline>,
    context: Context,
    state: State,
}

// type aliases

pub type InactivePipeline = Pipeline<pipeline_kind::Inactive>;
pub type ActivePipeline = Pipeline<pipeline_kind::Active>;

impl InactivePipeline {
    /// Creates an instance.
    pub fn new() -> Result<Self> {
        let context = Context::new()?;
        let pipeline = Self::from_context(context)?;
        Ok(pipeline)
    }

    /// Consumes a context and creates an instance.
    pub fn from_context(context: Context) -> Result<Self> {
        let ptr = {
            let mut checker = ErrorChecker::new();
            let ptr =
                unsafe { sys::rs2_create_pipeline(context.ptr.as_ptr(), checker.inner_mut_ptr()) };
            checker.check()?;
            ptr
        };

        let pipeline = Self {
            ptr: NonNull::new(ptr).unwrap(),
            context,
            state: pipeline_kind::Inactive,
        };
        Ok(pipeline)
    }

    /// Start the pipeline with an optional config.
    ///
    /// The method consumes inactive pipeline itself, and returns the started pipeine.
    pub fn start<'a>(self, config: impl Into<Option<&'a Config>>) -> Result<ActivePipeline> {
        let config = config.into();
        let ptr = match config {
            Some(conf) => unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = sys::rs2_pipeline_start_with_config(
                    self.ptr.as_ptr(),
                    conf.ptr.as_ptr(),
                    checker.inner_mut_ptr(),
                );
                checker.check()?;
                ptr
            },
            None => unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = sys::rs2_pipeline_start(self.ptr.as_ptr(), checker.inner_mut_ptr());
                checker.check()?;
                ptr
            },
        };

        let profile = unsafe { PipelineProfile::from_raw(ptr) };
        let pipeline = {
            let (pipeline_ptr, context_ptr) = self.into_raw_parts();
            Pipeline {
                ptr: NonNull::new(pipeline_ptr).unwrap(),
                context: unsafe { Context::from_raw(context_ptr) },
                state: pipeline_kind::Active { profile },
            }
        };

        Ok(pipeline)
    }

    /// Start the pipeline asynchronously. It is analogous to [Pipeline::start].
    pub async fn start_async<'a>(
        self,
        config: impl Into<Option<&'a Config>>,
    ) -> Result<ActivePipeline> {
        let config = config.into();
        let pipeline_ptr = AtomicPtr::new(self.ptr.as_ptr());
        let config_ptr_opt = config.map(|conf| AtomicPtr::new(conf.ptr.as_ptr()));
        let (tx, rx) = futures::channel::oneshot::channel();

        // start blocking thread
        thread::spawn(move || {
            let func = || unsafe {
                let profile_ptr = match config_ptr_opt {
                    Some(config_ptr) => {
                        let mut checker = ErrorChecker::new();
                        let ptr = sys::rs2_pipeline_start_with_config(
                            pipeline_ptr.load(Ordering::SeqCst),
                            config_ptr.load(Ordering::SeqCst),
                            checker.inner_mut_ptr(),
                        );
                        checker.check()?;
                        ptr
                    }
                    None => {
                        let mut checker = ErrorChecker::new();
                        let ptr = sys::rs2_pipeline_start(
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
        let profile = unsafe { PipelineProfile::from_raw(profile_ptr.load(Ordering::SeqCst)) };
        let pipeline = {
            let (pipeline_ptr, context_ptr) = self.into_raw_parts();
            Pipeline {
                ptr: NonNull::new(pipeline_ptr).unwrap(),
                context: unsafe { Context::from_raw(context_ptr) },
                state: pipeline_kind::Active { profile },
            }
        };

        Ok(pipeline)
    }

    /// Unpack the pipeline into raw pointers.
    ///
    /// It returns the raw pointer along with the context pointer that the pipeline depends on.
    pub fn into_raw_parts(self) -> (*mut sys::rs2_pipeline, *mut sys::rs2_context) {
        // take fields without invoking drop()
        let ptr = self.ptr;
        let context = unsafe { self.context.unsafe_clone().into_raw() };
        mem::forget(self);
        (ptr.as_ptr(), context)
    }

    /// Construct an inactive pipeline from raw pointers.
    ///
    /// It assumes the pipeline pointers is built atop from the context pointer.
    pub unsafe fn from_raw_parts(
        pipeline_ptr: *mut sys::rs2_pipeline,
        context_ptr: *mut sys::rs2_context,
    ) -> Self {
        let context = Context::from_raw(context_ptr);
        Self {
            ptr: NonNull::new(pipeline_ptr).unwrap(),
            context,
            state: pipeline_kind::Inactive,
        }
    }
}

impl ActivePipeline {
    /// Gets the active profile of pipeline.
    pub fn profile(&self) -> &PipelineProfile {
        &self.state.profile
    }

    /// Block until the next frame is available.
    ///
    /// When the timeout is set, it returns `Ok(Some(frame))` if the frame is available,
    /// or returns `Ok(None)` when timeout occurs.
    ///
    /// If the timeout is `None`, it waits indefinitely before the next frame.
    pub fn wait(&mut self, timeout: impl Into<Option<Duration>>) -> Result<Option<CompositeFrame>> {
        let timeout = timeout.into();
        let timeout_ms = timeout.unwrap_or(DEFAULT_TIMEOUT).as_millis() as c_uint;

        let frame = loop {
            let mut checker = ErrorChecker::new();
            let ptr = unsafe {
                sys::rs2_pipeline_wait_for_frames(
                    self.ptr.as_ptr(),
                    timeout_ms,
                    checker.inner_mut_ptr(),
                )
            };

            match (timeout, checker.check()) {
                (None, Err(RsError::Timeout(_))) => continue,
                (Some(_), Err(RsError::Timeout(_))) => {
                    return Ok(None);
                }
                (_, result) => result?,
            }

            let frame = unsafe { Frame::from_raw(ptr) };
            break frame;
        };

        Ok(Some(frame))
    }

    /// Poll if next frame is immediately available.
    ///
    /// Unlike [Pipeline::start], the method does not block and returns None
    /// if next from is not available.
    pub fn try_wait(&mut self) -> Result<Option<CompositeFrame>> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut ptr: *mut sys::rs2_frame = ptr::null_mut();
            let ret = sys::rs2_pipeline_poll_for_frames(
                self.ptr.as_ptr(),
                &mut ptr as *mut _,
                checker.inner_mut_ptr(),
            );

            if let Err(err) = checker.check() {
                return Err(err);
            }

            if ret != 0 {
                let frame = Frame::from_raw(ptr);
                Ok(Some(frame))
            } else {
                Ok(None)
            }
        }
    }

    /// Wait for the next frame asynchronously.
    ///
    /// The method is analogous to [Pipeline::wait].
    ///
    /// When the timeout is set, it returns `Ok(Some(frame))` if the frame is available,
    /// or returns `Ok(None)` when timeout occurs.
    ///
    /// If the timeout is `None`, it waits indefinitely before the next frame.
    pub async fn wait_async(
        &mut self,
        timeout: impl Into<Option<Duration>>,
    ) -> Result<Option<CompositeFrame>> {
        let timeout = timeout.into();
        let timeout_ms = timeout
            .map(|duration| duration.as_millis() as c_uint)
            .unwrap_or(sys::RS2_DEFAULT_TIMEOUT as c_uint);
        let (tx, rx) = futures::channel::oneshot::channel();
        let pipeline_ptr = AtomicPtr::new(self.ptr.as_ptr());

        thread::spawn(move || {
            let result = unsafe {
                loop {
                    let mut checker = ErrorChecker::new();
                    let ptr = sys::rs2_pipeline_wait_for_frames(
                        pipeline_ptr.load(Ordering::Relaxed),
                        timeout_ms,
                        checker.inner_mut_ptr(),
                    );
                    let result = match (timeout, checker.check()) {
                        (None, Err(RsError::Timeout(_))) => continue,
                        (Some(_), Err(RsError::Timeout(_))) => Ok(None),
                        (_, result) => result.map(|_| Some(Frame::from_raw(ptr))),
                    };
                    break result;
                }
            };
            let _ = tx.send(result);
        });

        let frame = rx.await.unwrap()?;
        Ok(frame)
    }

    /// Stop the pipeline.
    ///
    /// This method consumes the pipeline instance and returns pipeline markered inactive.
    pub fn stop(self) -> Result<InactivePipeline> {
        unsafe {
            let mut checker = ErrorChecker::new();
            sys::rs2_pipeline_stop(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
        }

        let pipeline = {
            let (pipeline_ptr, context_ptr, profile_ptr) = self.into_raw_parts();

            mem::drop(unsafe { pipeline_kind::Active::from_raw_parts(profile_ptr) });

            Pipeline {
                ptr: NonNull::new(pipeline_ptr).unwrap(),
                context: unsafe { Context::from_raw(context_ptr) },
                state: pipeline_kind::Inactive,
            }
        };

        Ok(pipeline)
    }

    /// Unpack the pipeline into raw pointers.
    ///
    /// After calling this method, you have to take care of their lifetime manually.
    pub fn into_raw_parts(
        self,
    ) -> (
        *mut sys::rs2_pipeline,
        *mut sys::rs2_context,
        *mut sys::rs2_pipeline_profile,
    ) {
        // take fields without invoking drop()
        let ptr = self.ptr;
        let context = unsafe { self.context.unsafe_clone().into_raw() };
        let pipeline_profile = unsafe { self.state.unsafe_clone().into_raw_parts() };
        mem::forget(self);
        (ptr.as_ptr(), context, pipeline_profile)
    }

    /// Construct an active pipeline from raw pointers.
    ///
    /// It assumes the pipeline pointer is built from the context pointer, and profile pointer
    /// is the active profile of the pipeline.
    pub unsafe fn from_raw_parts(
        pipeline_ptr: *mut sys::rs2_pipeline,
        context_ptr: *mut sys::rs2_context,
        profile_ptr: *mut sys::rs2_pipeline_profile,
    ) -> Self {
        let context = Context::from_raw(context_ptr);
        let state = pipeline_kind::Active::from_raw_parts(profile_ptr);
        Self {
            ptr: NonNull::new(pipeline_ptr).unwrap(),
            context,
            state,
        }
    }
}

impl<State> Drop for Pipeline<State>
where
    State: pipeline_kind::PipelineState,
{
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_pipeline(self.ptr.as_ptr());
        }
    }
}

unsafe impl<State> Send for Pipeline<State> where State: pipeline_kind::PipelineState {}
