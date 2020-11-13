pub use crate::kind::{Format, StreamKind};
#[cfg(feature = "with-image")]
pub use image::{
    buffer::ConvertBuffer,
    flat::{FlatSamples, SampleLayout},
    Bgr, Bgra, ColorType, DynamicImage, ImageBuffer, Luma, Rgb, Rgba,
};
#[cfg(feature = "with-nalgebra")]
pub use nalgebra::{
    Isometry3, MatrixMN, Quaternion, Translation3, Unit, UnitQuaternion, Vector3, U3,
};
pub use num_derive::FromPrimitive;
pub use num_traits::FromPrimitive;
pub use realsense_sys as sys;
pub use safe_transmute::guard::PedanticGuard;
#[cfg(any(unix))]
pub use std::os::unix::ffi::OsStrExt;
#[cfg(any(windows))]
pub use std::os::windows::ffi::OsStrExt;
pub use std::{
    borrow::{Borrow, Cow},
    collections::HashMap,
    convert::{AsMut, AsRef},
    error::Error as StdError,
    ffi::{CStr, CString},
    fmt::{Debug, Display, Formatter, Result as FormatResult},
    iter::FusedIterator,
    marker::PhantomData,
    mem::{self, MaybeUninit},
    num::NonZeroU8,
    ops::{Deref, DerefMut},
    os::raw::{c_int, c_uchar, c_uint, c_void},
    path::Path,
    ptr::{self, NonNull},
    result, slice,
    sync::atomic::{AtomicPtr, Ordering},
    thread,
    time::Duration,
};
