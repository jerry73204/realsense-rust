use crate::{
    config::Config,
    context::Context,
    error::{ErrorChecker, Result as RsResult},
    frame::{marker::Composite, Frame},
    pipeline_profile::PipelineProfile,
};
use futures::channel::oneshot::Receiver as OneShotReceiver;
use std::{
    future::Future,
    mem::MaybeUninit,
    os::raw::c_uint,
    pin::Pin,
    ptr::NonNull,
    task::{Context as AsyncContext, Poll},
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

    /// Start the pipeline asynchronously. It is analogous to [Pipeline::start].
    pub fn start_async(self, config: Option<Config>) -> PipelineStartFuture {
        PipelineStartFuture::new(config, self)
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
    pub fn wait_async(self) -> PipelineWaitFuture {
        PipelineWaitFuture::new(self)
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

type PipelineWaitFutureOutput = RsResult<(Pipeline<marker::Active>, Frame<Composite>)>;

/// Future type returned by [wait_async()](Pipeline::wait_async).
#[derive(Debug)]
pub struct PipelineWaitFuture {
    pipeline_opt: Option<Pipeline<marker::Active>>,
    rx_opt: Option<OneShotReceiver<PipelineWaitFutureOutput>>,
}

impl PipelineWaitFuture {
    pub(crate) fn new(pipeline: Pipeline<marker::Active>) -> Self {
        Self {
            pipeline_opt: Some(pipeline),
            rx_opt: None,
        }
    }
}

impl<'a> Future for PipelineWaitFuture {
    type Output = PipelineWaitFutureOutput;

    fn poll(self: Pin<&mut Self>, cx: &mut AsyncContext) -> Poll<Self::Output> {
        let self_mut = self.get_mut();

        if let Some(mut pipeline) = self_mut.pipeline_opt.take() {
            let waker = cx.waker().clone();
            let (tx, rx) = futures::channel::oneshot::channel();
            self_mut.rx_opt = Some(rx);

            std::thread::spawn(move || {
                let result = pipeline.wait(None).map(|frame| (pipeline, frame));
                if let Ok(()) = tx.send(result) {
                    waker.wake();
                }
            });
            Poll::Pending
        } else if let Some(rx) = &mut self_mut.rx_opt {
            match rx.try_recv().unwrap() {
                Some(result) => Poll::Ready(result),
                None => Poll::Pending,
            }
        } else {
            unreachable!();
        }
    }
}

type PipelineStartFutureOutput = RsResult<Pipeline<marker::Active>>;

/// Future type returned by [start_async()](Pipeline::start_async).
#[derive(Debug)]
pub struct PipelineStartFuture {
    pipeline_opt: Option<(Option<Config>, Pipeline<marker::Inactive>)>,
    rx_opt: Option<OneShotReceiver<PipelineStartFutureOutput>>,
}

impl PipelineStartFuture {
    pub(crate) fn new(config_opt: Option<Config>, pipeline: Pipeline<marker::Inactive>) -> Self {
        Self {
            pipeline_opt: Some((config_opt, pipeline)),
            rx_opt: None,
        }
    }
}
impl<'a> Future for PipelineStartFuture {
    type Output = PipelineStartFutureOutput;

    fn poll(self: Pin<&mut Self>, cx: &mut AsyncContext) -> Poll<Self::Output> {
        let self_mut = self.get_mut();

        if let Some((config_opt, pipeline)) = self_mut.pipeline_opt.take() {
            let waker = cx.waker().clone();
            let (tx, rx) = futures::channel::oneshot::channel();
            self_mut.rx_opt = Some(rx);

            std::thread::spawn(move || {
                let result = pipeline.start(config_opt);
                if let Ok(()) = tx.send(result) {
                    waker.wake();
                }
            });
            Poll::Pending
        } else if let Some(rx) = &mut self_mut.rx_opt {
            match rx.try_recv().unwrap() {
                Some(result) => Poll::Ready(result),
                None => Poll::Pending,
            }
        } else {
            unreachable!();
        }
    }
}
