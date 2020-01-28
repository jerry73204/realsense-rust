use crate::error::{ErrorChecker, Result as RsResult};
use nalgebra::{Isometry3, MatrixMN, Translation3, UnitQuaternion, U3};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
#[cfg(any(unix, target_os = "redox"))]
use std::os::unix::ffi::OsStrExt;
#[cfg(any(windows))]
use std::os::windows::ffi::OsStrExt;
use std::{
    borrow::Borrow,
    ffi::{CStr, CString},
    mem::MaybeUninit,
    os::raw::{c_int, c_uint, c_void},
    path::Path,
    ptr::NonNull,
    task::Poll,
    time::Duration,
};

#[repr(u32)]
#[derive(FromPrimitive)]
pub enum TimestampDomain {
    HardwareClock = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
    SystemTime = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
    GlobalTime = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME,
    Count = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_COUNT,
}

#[repr(u32)]
#[derive(FromPrimitive)]
pub enum FrameMetaDataValue {
    FrameCounter = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_COUNTER,
    FrameTimestamp = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_TIMESTAMP,
    SensorTimestamp = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SENSOR_TIMESTAMP,
    ActualExposure = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_EXPOSURE,
    GainLevel = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_GAIN_LEVEL,
    AutoExposure = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_AUTO_EXPOSURE,
    WhiteBalance = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_WHITE_BALANCE,
    TimeOfArrival = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_TIME_OF_ARRIVAL,
    Temperature = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_TEMPERATURE,
    BackendTimestamp = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BACKEND_TIMESTAMP,
    ActualFps = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_FPS,
    FrameLaserPower = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LASER_POWER,
    FrameLaserPowerMode =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE,
    ExposurePriority = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_PRIORITY,
    ExposureRoiLeft = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_LEFT,
    ExposureRoiRight =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_RIGHT,
    ExposureRoiTop = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_TOP,
    ExposureRoiBottom =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_BOTTOM,
    Brightness = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BRIGHTNESS,
    Contrast = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_CONTRAST,
    Saturation = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SATURATION,
    Sharpness = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SHARPNESS,
    AutoWhiteBalanceTemperature =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_AUTO_WHITE_BALANCE_TEMPERATURE,
    BacklightCompensation =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BACKLIGHT_COMPENSATION,
    Hue = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_HUE,
    Gamma = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_GAMMA,
    ManualWhiteBalance =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_MANUAL_WHITE_BALANCE,
    PowerLineFrequency =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_POWER_LINE_FREQUENCY,
    LowLightCompensation =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_LOW_LIGHT_COMPENSATION,
    FrameEmitterMode =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_EMITTER_MODE,
    FrameLedPower = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LED_POWER,
    Count = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_COUNT,
}

#[repr(u32)]
#[derive(FromPrimitive)]
pub enum Extension {
    Unknown = realsense_sys::rs2_extension_RS2_EXTENSION_UNKNOWN,
    Debug = realsense_sys::rs2_extension_RS2_EXTENSION_DEBUG,
    Info = realsense_sys::rs2_extension_RS2_EXTENSION_INFO,
    Motion = realsense_sys::rs2_extension_RS2_EXTENSION_MOTION,
    Options = realsense_sys::rs2_extension_RS2_EXTENSION_OPTIONS,
    Video = realsense_sys::rs2_extension_RS2_EXTENSION_VIDEO,
    Roi = realsense_sys::rs2_extension_RS2_EXTENSION_ROI,
    DepthSensor = realsense_sys::rs2_extension_RS2_EXTENSION_DEPTH_SENSOR,
    VideoFrame = realsense_sys::rs2_extension_RS2_EXTENSION_VIDEO_FRAME,
    MotionFrame = realsense_sys::rs2_extension_RS2_EXTENSION_MOTION_FRAME,
    CompositeFrame = realsense_sys::rs2_extension_RS2_EXTENSION_COMPOSITE_FRAME,
    Points = realsense_sys::rs2_extension_RS2_EXTENSION_POINTS,
    DepthFrame = realsense_sys::rs2_extension_RS2_EXTENSION_DEPTH_FRAME,
    AdvancedMode = realsense_sys::rs2_extension_RS2_EXTENSION_ADVANCED_MODE,
    Record = realsense_sys::rs2_extension_RS2_EXTENSION_RECORD,
    VideoProfile = realsense_sys::rs2_extension_RS2_EXTENSION_VIDEO_PROFILE,
    Playback = realsense_sys::rs2_extension_RS2_EXTENSION_PLAYBACK,
    DepthStereoSensor = realsense_sys::rs2_extension_RS2_EXTENSION_DEPTH_STEREO_SENSOR,
    DisparityFrame = realsense_sys::rs2_extension_RS2_EXTENSION_DISPARITY_FRAME,
    MotionProfile = realsense_sys::rs2_extension_RS2_EXTENSION_MOTION_PROFILE,
    PoseFrame = realsense_sys::rs2_extension_RS2_EXTENSION_POSE_FRAME,
    PoseProfile = realsense_sys::rs2_extension_RS2_EXTENSION_POSE_PROFILE,
    SoftwareDevice = realsense_sys::rs2_extension_RS2_EXTENSION_SOFTWARE_DEVICE,
    SoftwareSensor = realsense_sys::rs2_extension_RS2_EXTENSION_SOFTWARE_SENSOR,
    DecimationFilter = realsense_sys::rs2_extension_RS2_EXTENSION_DECIMATION_FILTER,
    ThresholdFilter = realsense_sys::rs2_extension_RS2_EXTENSION_THRESHOLD_FILTER,
    DisparityFilter = realsense_sys::rs2_extension_RS2_EXTENSION_DISPARITY_FILTER,
    SpatialFilter = realsense_sys::rs2_extension_RS2_EXTENSION_SPATIAL_FILTER,
    TemporalFilter = realsense_sys::rs2_extension_RS2_EXTENSION_TEMPORAL_FILTER,
    HoleFillingFilter = realsense_sys::rs2_extension_RS2_EXTENSION_HOLE_FILLING_FILTER,
    ZeroOrderFilter = realsense_sys::rs2_extension_RS2_EXTENSION_ZERO_ORDER_FILTER,
    RecommendedFilters = realsense_sys::rs2_extension_RS2_EXTENSION_RECOMMENDED_FILTERS,
    Pose = realsense_sys::rs2_extension_RS2_EXTENSION_POSE,
    PoseSensor = realsense_sys::rs2_extension_RS2_EXTENSION_POSE_SENSOR,
    WheelOdometer = realsense_sys::rs2_extension_RS2_EXTENSION_WHEEL_ODOMETER,
    GlobalTimer = realsense_sys::rs2_extension_RS2_EXTENSION_GLOBAL_TIMER,
    Updatable = realsense_sys::rs2_extension_RS2_EXTENSION_UPDATABLE,
    UpdateDevice = realsense_sys::rs2_extension_RS2_EXTENSION_UPDATE_DEVICE,
    AutoCalibratedDevice = realsense_sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATED_DEVICE,
    Count = realsense_sys::rs2_extension_RS2_EXTENSION_COUNT,
    L500DepthSensor = realsense_sys::rs2_extension_RS2_EXTENSION_L500_DEPTH_SENSOR,
    Tm2 = realsense_sys::rs2_extension_RS2_EXTENSION_TM2,
    Tm2Sensor = realsense_sys::rs2_extension_RS2_EXTENSION_TM2_SENSOR,
}

#[repr(u32)]
#[derive(FromPrimitive)]
pub enum CameraInfo {
    Name = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_NAME,
    SerialNumber = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_SERIAL_NUMBER,
    FirmwareVersion = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_VERSION,
    RecommendedFirmwareVersion =
        realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION,
    PhysicalPort = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_PHYSICAL_PORT,
    DebugOpCode = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_DEBUG_OP_CODE,
    AdvancedMode = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_ADVANCED_MODE,
    ProductId = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_ID,
    CameraLocked = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_CAMERA_LOCKED,
    UsbTypeDescriptor = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR,
    ProductLine = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_LINE,
    AsicSerialNumber = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER,
    FirmwareUpdateId = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID,
    Count = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_COUNT,
}

#[repr(u32)]
#[derive(FromPrimitive)]
pub enum StreamKind {
    Any = realsense_sys::rs2_stream_RS2_STREAM_ANY,
    Depth = realsense_sys::rs2_stream_RS2_STREAM_DEPTH,
    Color = realsense_sys::rs2_stream_RS2_STREAM_COLOR,
    Infrared = realsense_sys::rs2_stream_RS2_STREAM_INFRARED,
    Fisheye = realsense_sys::rs2_stream_RS2_STREAM_FISHEYE,
    Gyro = realsense_sys::rs2_stream_RS2_STREAM_GYRO,
    Accel = realsense_sys::rs2_stream_RS2_STREAM_ACCEL,
    Gpio = realsense_sys::rs2_stream_RS2_STREAM_GPIO,
    Pose = realsense_sys::rs2_stream_RS2_STREAM_POSE,
    Confidence = realsense_sys::rs2_stream_RS2_STREAM_CONFIDENCE,
    Count = realsense_sys::rs2_stream_RS2_STREAM_COUNT,
}

#[repr(u32)]
#[derive(FromPrimitive)]
pub enum Format {
    Any = realsense_sys::rs2_format_RS2_FORMAT_ANY,
    Yuyv = realsense_sys::rs2_format_RS2_FORMAT_YUYV,
    Uyvy = realsense_sys::rs2_format_RS2_FORMAT_UYVY,
    MotionRaw = realsense_sys::rs2_format_RS2_FORMAT_MOTION_RAW,
    GpioRaw = realsense_sys::rs2_format_RS2_FORMAT_GPIO_RAW,
    Distance = realsense_sys::rs2_format_RS2_FORMAT_DISTANCE,
    Mjpeg = realsense_sys::rs2_format_RS2_FORMAT_MJPEG,
    Inzi = realsense_sys::rs2_format_RS2_FORMAT_INZI,
    Invi = realsense_sys::rs2_format_RS2_FORMAT_INVI,
    Count = realsense_sys::rs2_format_RS2_FORMAT_COUNT,
    _6Dof = realsense_sys::rs2_format_RS2_FORMAT_6DOF,
    Bgr8 = realsense_sys::rs2_format_RS2_FORMAT_BGR8,
    Bgra8 = realsense_sys::rs2_format_RS2_FORMAT_BGRA8,
    Disparity16 = realsense_sys::rs2_format_RS2_FORMAT_DISPARITY16,
    Disparity32 = realsense_sys::rs2_format_RS2_FORMAT_DISPARITY32,
    MotionXyz32F = realsense_sys::rs2_format_RS2_FORMAT_MOTION_XYZ32F,
    Raw8 = realsense_sys::rs2_format_RS2_FORMAT_RAW8,
    Raw10 = realsense_sys::rs2_format_RS2_FORMAT_RAW10,
    Raw16 = realsense_sys::rs2_format_RS2_FORMAT_RAW16,
    Rgb8 = realsense_sys::rs2_format_RS2_FORMAT_RGB8,
    Rgba8 = realsense_sys::rs2_format_RS2_FORMAT_RGBA8,
    W10 = realsense_sys::rs2_format_RS2_FORMAT_W10,
    Xyz32F = realsense_sys::rs2_format_RS2_FORMAT_XYZ32F,
    Y8 = realsense_sys::rs2_format_RS2_FORMAT_Y8,
    Y8I = realsense_sys::rs2_format_RS2_FORMAT_Y8I,
    Y10Bpack = realsense_sys::rs2_format_RS2_FORMAT_Y10BPACK,
    Y12I = realsense_sys::rs2_format_RS2_FORMAT_Y12I,
    Y16 = realsense_sys::rs2_format_RS2_FORMAT_Y16,
    Z16 = realsense_sys::rs2_format_RS2_FORMAT_Z16,
}

pub struct Context {
    ptr: NonNull<realsense_sys::rs2_context>,
}

impl Context {
    pub fn new() -> RsResult<Self> {
        let ptr = {
            let mut checker = ErrorChecker::new();
            let context = unsafe {
                realsense_sys::rs2_create_context(
                    realsense_sys::RS2_API_VERSION as i32,
                    checker.inner_mut_ptr(),
                )
            };
            checker.check()?;
            context
        };

        let context = Self {
            ptr: NonNull::new(ptr).unwrap(),
        };

        Ok(context)
    }

    pub fn create_device_hub(&mut self) -> RsResult<DeviceHub> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_create_device_hub(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            ptr
        };

        let hub = DeviceHub {
            ptr: NonNull::new(ptr).unwrap(),
        };
        Ok(hub)
    }

    // pub fn create_processing_block<F>(&mut self, callback: F) -> ProcessingBlock
    // where
    //     F: FnMut(&mut Frame, &mut Source, &mut Context)
    // {
    //     unsafe {

    //     }
    // }

    pub fn query_devices(&mut self) -> RsResult<DeviceList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let list = realsense_sys::rs2_query_devices(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            DeviceList::from_ptr(NonNull::new(list).unwrap())
        };
        Ok(list)
    }

    pub fn query_devices_ex(&mut self, product_mask: c_int) -> RsResult<DeviceList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let list = realsense_sys::rs2_query_devices_ex(
                self.ptr.as_ptr(),
                product_mask,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            DeviceList::from_ptr(NonNull::new(list).unwrap())
        };
        Ok(list)
    }

    #[cfg(any(unix, target_os = "redox"))]
    pub fn add_device<P>(&mut self, file: P) -> RsResult<()>
    where
        P: AsRef<Path>,
    {
        let cstring = CString::new(file.as_ref().as_os_str().as_bytes()).unwrap();
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_context_add_device(
                self.ptr.as_ptr(),
                cstring.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        Ok(())
    }

    #[cfg(any(windows))]
    pub fn add_device<P>(&mut self, file: P) -> RsResult<()>
    where
        P: AsRef<Path>,
    {
        todo!();
    }

    #[cfg(any(unix, target_os = "redox"))]
    pub fn remove_device<P>(&mut self, file: P) -> RsResult<()>
    where
        P: AsRef<Path>,
    {
        let cstring = CString::new(file.as_ref().as_os_str().as_bytes()).unwrap();
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_context_remove_device(
                self.ptr.as_ptr(),
                cstring.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        Ok(())
    }

    #[cfg(any(windows))]
    pub fn remove_device<P>(&mut self, file: P) -> RsResult<()>
    where
        P: AsRef<Path>,
    {
        todo!();
    }

    // #[cfg(any(unix, target_os = "redox"))]
    // pub fn add_software_device<P>(&mut self, file: P) -> RsResult<()>
    // where
    //     P: AsRef<Path>,
    // {
    //     let cstring = CString::new(file.as_ref().as_os_str().as_bytes()).unwrap();
    //     unsafe {
    //         let mut checker = ErrorChecker::new();
    //         realsense_sys::rs2_context_add_software_device(
    //             self.ptr.as_ptr(),
    //             cstring.as_ptr(),
    //             checker.inner_mut_ptr(),
    //         );
    //         checker.check()?;
    //     }
    //     Ok(())
    // }

    // #[cfg(any(windows))]
    // pub fn add_software_device<P>(&mut self, file: P) -> RsResult<()>
    // where
    //     P: AsRef<Path>,
    // {
    //     todo!();
    // }
}

impl Drop for Context {
    fn drop(&mut self) {
        unsafe { realsense_sys::rs2_delete_context(self.ptr.as_ptr()) }
    }
}

pub struct DeviceList {
    ptr: NonNull<realsense_sys::rs2_device_list>,
}

impl DeviceList {
    pub fn get(&mut self, index: usize) -> RsResult<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_device(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Device::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(device)
    }

    pub fn len(&mut self) -> RsResult<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len =
                realsense_sys::rs2_get_device_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            len
        };
        Ok(len as usize)
    }

    pub fn try_into_iter(mut self) -> RsResult<DeviceListIter> {
        let len = self.len()?;
        let iter = DeviceListIter {
            ptr: self.ptr,
            index: 0,
            len,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_device_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for DeviceList {
    type Item = RsResult<Device>;
    type IntoIter = DeviceListIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl Drop for DeviceList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_device_list(self.ptr.as_ptr());
        }
    }
}

pub struct DeviceListIter {
    index: usize,
    len: usize,
    ptr: NonNull<realsense_sys::rs2_device_list>,
}

impl Iterator for DeviceListIter {
    type Item = RsResult<Device>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.len {
            let ptr = unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = realsense_sys::rs2_create_device(
                    self.ptr.as_ptr(),
                    self.index as c_int,
                    checker.inner_mut_ptr(),
                );
                match checker.check() {
                    Ok(()) => ptr,
                    Err(err) => return Some(Err(err)),
                }
            };

            let device = Device {
                ptr: NonNull::new(ptr).unwrap(),
            };

            self.index += 1;
            Some(Ok(device))
        } else {
            None
        }
    }
}

pub struct Device {
    ptr: NonNull<realsense_sys::rs2_device>,
}

impl Device {
    pub fn query_sensors(&mut self) -> RsResult<SensorList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_query_sensors(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            SensorList::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(list)
    }

    pub fn name<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::Name)
    }

    pub fn serial_number<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::SerialNumber)
    }

    pub fn recommended_firmware_version<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::RecommendedFirmwareVersion)
    }

    pub fn physical_port<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::PhysicalPort)
    }

    pub fn debug_op_code<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::DebugOpCode)
    }

    pub fn advanced_mode<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::AdvancedMode)
    }

    pub fn product_id<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::ProductId)
    }

    pub fn camera_locked<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::CameraLocked)
    }

    pub fn usb_type_descriptor<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::UsbTypeDescriptor)
    }

    pub fn product_line<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::ProductLine)
    }

    pub fn asic_serial_number<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::AsicSerialNumber)
    }

    pub fn firmware_update_id<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::FirmwareUpdateId)
    }

    pub fn count<'a>(&'a mut self) -> RsResult<&'a str> {
        self.info(CameraInfo::Count)
    }

    pub fn info<'a>(&'a mut self, kind: CameraInfo) -> RsResult<&'a str> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_device_info(
                self.ptr.as_ptr(),
                kind as realsense_sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };

        // TODO: deallicate this CStr?
        let string = unsafe { CStr::from_ptr(ptr).to_str().unwrap() };
        Ok(string)
    }

    pub fn is_info_supported(&mut self, kind: CameraInfo) -> RsResult<bool> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_supports_device_info(
                self.ptr.as_ptr(),
                kind as realsense_sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        Ok(val != 0)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_device>) -> Self {
        Self { ptr }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_device(self.ptr.as_ptr());
        }
    }
}

pub struct DeviceHub {
    ptr: NonNull<realsense_sys::rs2_device_hub>,
}

impl DeviceHub {
    pub fn wait_for_device(&mut self) -> RsResult<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_device_hub_wait_for_device(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Device::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(device)
    }

    pub fn is_device_connected(&mut self, device: &mut Device) -> RsResult<bool> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_device_hub_is_device_connected(
                self.ptr.as_ptr(),
                device.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        Ok(val != 0)
    }
}

impl Drop for DeviceHub {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_device_hub(self.ptr.as_ptr());
        }
    }
}

pub struct SensorList {
    ptr: NonNull<realsense_sys::rs2_sensor_list>,
}

impl SensorList {
    pub fn get(&mut self, index: usize) -> RsResult<Self> {
        let sensor = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_sensor(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Self::from_ptr(NonNull::new(ptr as *mut _).unwrap())
        };
        Ok(sensor)
    }

    pub fn len(&mut self) -> RsResult<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len =
                realsense_sys::rs2_get_sensors_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            len
        };
        Ok(len as usize)
    }

    pub fn try_into_iter(mut self) -> RsResult<SensorListIter> {
        let len = self.len()?;
        let iter = SensorListIter {
            ptr: self.ptr,
            len,
            index: 0,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_sensor_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for SensorList {
    type Item = RsResult<Sensor>;
    type IntoIter = SensorListIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

pub struct SensorListIter {
    ptr: NonNull<realsense_sys::rs2_sensor_list>,
    len: usize,
    index: usize,
}

impl Iterator for SensorListIter {
    type Item = RsResult<Sensor>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.len {
            let sensor = unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = realsense_sys::rs2_create_sensor(
                    self.ptr.as_ptr(),
                    self.index as c_int,
                    checker.inner_mut_ptr(),
                );
                match checker.check() {
                    Ok(()) => Sensor::from_ptr(NonNull::new(ptr).unwrap()),
                    Err(err) => return Some(Err(err)),
                }
            };
            self.index += 1;
            Some(Ok(sensor))
        } else {
            None
        }
    }
}

impl Drop for SensorList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_sensor_list(self.ptr.as_ptr());
        }
    }
}

pub struct Sensor {
    ptr: NonNull<realsense_sys::rs2_sensor>,
}

impl Sensor {
    pub fn device(&mut self) -> RsResult<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_create_device_from_sensor(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Device::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(device)
    }

    pub fn get_stream_profiles(&mut self) -> RsResult<StreamProfileList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_get_stream_profiles(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            StreamProfileList::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(list)
    }

    pub fn get_recommended_processing_blocks(&mut self) -> RsResult<ProcessingBlockList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_recommended_processing_blocks(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ProcessingBlockList::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(list)
    }

    // pub fn start<F>(&mut self, callback: F) -> RsResult<()>
    // where
    //     F: FnMut(Frame),
    // {
    //     unsafe {
    //         let mut checker = ErrorChecker::new();
    //         realsense_sys::rs2_start(
    //             self.ptr.as_ptr(),
    //             Some(|frame_ptr, _| {
    //                 let frame = Frame::from_ptr(NonNull::new(frame_ptr).unwrap());
    //                 callback(frame);
    //             }),
    //             std::ptr::null_mut(),
    //             checker.inner_mut_ptr(),
    //         );
    //         checker.check()?;
    //     }
    //     Ok(())
    // }

    pub fn name(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::Name)
    }

    pub fn serial_number(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::SerialNumber)
    }

    pub fn recommended_firmware_version(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::RecommendedFirmwareVersion)
    }

    pub fn physical_port(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::PhysicalPort)
    }

    pub fn debug_op_code(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::DebugOpCode)
    }

    pub fn advanced_mode(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::AdvancedMode)
    }

    pub fn product_id(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::ProductId)
    }

    pub fn camera_locked(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::CameraLocked)
    }

    pub fn usb_type_descriptor(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::UsbTypeDescriptor)
    }

    pub fn product_line(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::ProductLine)
    }

    pub fn asic_serial_number(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::AsicSerialNumber)
    }

    pub fn firmware_update_id(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::FirmwareUpdateId)
    }

    pub fn count(&mut self) -> RsResult<&CStr> {
        self.info(CameraInfo::Count)
    }

    pub fn info(&mut self, kind: CameraInfo) -> RsResult<&CStr> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_sensor_info(
                self.ptr.as_ptr(),
                kind as realsense_sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };
        let string = unsafe { CStr::from_ptr(ptr) };
        Ok(string)
    }

    pub fn is_info_supported(&mut self, kind: CameraInfo) -> RsResult<bool> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_supports_sensor_info(
                self.ptr.as_ptr(),
                kind as realsense_sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        Ok(val != 0)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_sensor>) -> Self {
        Self { ptr }
    }
}

impl Drop for Sensor {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_sensor(self.ptr.as_ptr());
        }
    }
}

pub struct StreamProfileList {
    ptr: NonNull<realsense_sys::rs2_stream_profile_list>,
}

impl StreamProfileList {
    pub fn get(&mut self, index: usize) -> RsResult<StreamProfile> {
        let profile = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_stream_profile(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            StreamProfile::from_ptr(NonNull::new(ptr as *mut _).unwrap())
        };
        Ok(profile)
    }

    pub fn len(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let len = realsense_sys::rs2_get_stream_profiles_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(len as usize)
        }
    }

    pub fn try_into_iter(mut self) -> RsResult<StreamProfileListIter> {
        let len = self.len()?;
        let iter = StreamProfileListIter {
            ptr: self.ptr,
            len,
            index: 0,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_stream_profile_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for StreamProfileList {
    type Item = RsResult<StreamProfile>;
    type IntoIter = StreamProfileListIter;

    fn into_iter(mut self) -> Self::IntoIter {
        let len = self.len().unwrap();
        StreamProfileListIter {
            ptr: self.ptr,
            len,
            index: 0,
        }
    }
}

impl Drop for StreamProfileList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_stream_profiles_list(self.ptr.as_ptr());
        }
    }
}

pub struct StreamProfileListIter {
    ptr: NonNull<realsense_sys::rs2_stream_profile_list>,
    len: usize,
    index: usize,
}

impl Iterator for StreamProfileListIter {
    type Item = RsResult<StreamProfile>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.len {
            let profile = unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = realsense_sys::rs2_get_stream_profile(
                    self.ptr.as_ptr(),
                    self.index as c_int,
                    checker.inner_mut_ptr(),
                );
                match checker.check() {
                    Ok(()) => StreamProfile::from_ptr(NonNull::new(ptr as *mut _).unwrap()),
                    Err(err) => return Some(Err(err)),
                }
            };
            Some(Ok(profile))
        } else {
            None
        }
    }
}

pub struct StreamProfile {
    ptr: NonNull<realsense_sys::rs2_stream_profile>,
}

impl StreamProfile {
    pub fn is_default(&mut self) -> RsResult<bool> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_is_stream_profile_default(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    pub fn extend(&mut self, extension: Extension) -> RsResult<()> {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_stream_profile_is(
                self.ptr.as_ptr(),
                extension as realsense_sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        Ok(())
    }

    pub fn get_data(&mut self) -> RsResult<StreamProfileData> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut stream = MaybeUninit::uninit();
            let mut format = MaybeUninit::uninit();
            let mut index = MaybeUninit::uninit();
            let mut unique_id = MaybeUninit::uninit();
            let mut framerate = MaybeUninit::uninit();

            realsense_sys::rs2_get_stream_profile_data(
                self.ptr.as_ptr(),
                stream.as_mut_ptr(),
                format.as_mut_ptr(),
                index.as_mut_ptr(),
                unique_id.as_mut_ptr(),
                framerate.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;

            let data = StreamProfileData {
                stream: StreamKind::from_u32(stream.assume_init()).unwrap(),
                format: Format::from_u32(format.assume_init()).unwrap(),
                index: index.assume_init() as usize,
                unique_id: unique_id.assume_init(),
                framerate: framerate.assume_init(),
            };
            Ok(data)
        }
    }

    pub fn set_data(&mut self) {
        todo!();
    }

    pub fn extrinsics_to<P>(&mut self, other: P) -> RsResult<Isometry3<f32>>
    where
        P: Borrow<StreamProfile>,
    {
        let mut extrinsics = MaybeUninit::<realsense_sys::rs2_extrinsics>::uninit();
        let (raw_rotation, raw_translation) = unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_get_extrinsics(
                self.ptr.as_ptr(),
                other.borrow().ptr.as_ptr(),
                extrinsics.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;

            let realsense_sys::rs2_extrinsics {
                rotation,
                translation,
            } = extrinsics.assume_init();
            (rotation, translation)
        };

        let rotation = {
            let matrix =
                MatrixMN::<f32, U3, U3>::from_iterator(raw_rotation.iter().map(|val| *val));
            UnitQuaternion::from_matrix(&matrix)
        };

        let translation =
            { Translation3::new(raw_translation[0], raw_translation[1], raw_translation[2]) };

        let isometry = Isometry3::from_parts(translation, rotation);

        Ok(isometry)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_stream_profile>) -> Self {
        Self { ptr }
    }
}

impl Drop for StreamProfile {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_stream_profile(self.ptr.as_ptr());
        }
    }
}

pub struct StreamProfileData {
    pub stream: StreamKind,
    pub format: Format,
    pub index: usize,
    pub unique_id: i32,
    pub framerate: i32,
}

pub struct FrameQueue {
    ptr: NonNull<realsense_sys::rs2_frame_queue>,
}

impl FrameQueue {
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

    pub fn push(&mut self, frame: Frame) {
        unsafe {
            realsense_sys::rs2_enqueue_frame(
                frame.ptr.as_ptr(),
                self.ptr.cast::<c_void>().as_ptr(),
            );
        }
    }

    pub fn wait(&mut self, timeout: Duration) -> RsResult<Frame> {
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

    pub fn poll(&mut self) -> Poll<RsResult<Frame>> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut ptr: *mut realsense_sys::rs2_frame = std::ptr::null_mut();
            let ret = realsense_sys::rs2_poll_for_frame(
                self.ptr.as_ptr(),
                &mut ptr as *mut _,
                checker.inner_mut_ptr(),
            );

            if let Err(err) = checker.check() {
                return Poll::Ready(Err(err));
            }

            if ret != 0 {
                let frame = Frame::from_ptr(NonNull::new(ptr).unwrap());
                Poll::Ready(Ok(frame))
            } else {
                Poll::Pending
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

pub struct Frame {
    ptr: NonNull<realsense_sys::rs2_frame>,
}

impl Frame {
    pub fn metadata(&mut self, kind: FrameMetaDataValue) -> RsResult<u64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_metadata(
                self.ptr.as_ptr(),
                kind as realsense_sys::rs2_frame_metadata_value,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as u64)
        }
    }

    pub fn number(&mut self) -> RsResult<u64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_number(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as u64)
        }
    }

    pub fn width(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_width(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn height(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_height(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn data_size(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_data_size(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn timestamp(&mut self) -> RsResult<f64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_timestamp(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as f64)
        }
    }

    pub fn timestamp_domain(&mut self) -> RsResult<TimestampDomain> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_timestamp_domain(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        let domain = TimestampDomain::from_u32(val).unwrap();
        Ok(domain)
    }

    pub fn bits_per_pixel(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_bits_per_pixel(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn stride_in_bytes(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_stride_in_bytes(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn vertices<'a>(&'a mut self) -> RsResult<&'a [realsense_sys::rs2_vertex]> {
        let n_points = self.points_count()?;
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_get_frame_vertices(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            let slice = std::slice::from_raw_parts::<'a, realsense_sys::rs2_vertex>(ptr, n_points);
            Ok(slice)
        }
    }

    pub fn points_count(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_points_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn texture_coordinates<'a>(&'a mut self) -> RsResult<&'a [realsense_sys::rs2_pixel]> {
        let width = self.width()?;
        let height = self.height()?;
        let n_pixels = width * height;

        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_frame_texture_coordinates(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            let slice = std::slice::from_raw_parts::<'a, realsense_sys::rs2_pixel>(ptr, n_pixels);
            Ok(slice)
        }
    }

    pub fn data<'a>(&'a mut self) -> RsResult<&'a [u8]> {
        let size = self.data_size()?;
        let slice = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_frame_data(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            std::slice::from_raw_parts::<'a, u8>(ptr.cast::<u8>(), size)
        };
        Ok(slice)
    }

    pub fn sensor(&mut self) -> RsResult<Sensor> {
        let sensor = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_get_frame_sensor(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Sensor::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(sensor)
    }

    pub fn stream_profile(&mut self) -> RsResult<StreamProfile> {
        let profile = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_frame_stream_profile(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            StreamProfile::from_ptr(NonNull::new(ptr as *mut _).unwrap())
        };
        Ok(profile)
    }

    pub(crate) fn from_ptr(ptr: NonNull<realsense_sys::rs2_frame>) -> Self {
        Self { ptr }
    }
}

impl Drop for Frame {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_release_frame(self.ptr.as_ptr());
        }
    }
}

pub struct ProcessingBlockList {
    ptr: NonNull<realsense_sys::rs2_processing_block_list>,
}

impl ProcessingBlockList {
    pub fn get(&mut self, index: usize) -> RsResult<ProcessingBlock> {
        let block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_processing_block(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ProcessingBlock::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(block)
    }

    pub fn len(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_recommended_processing_blocks_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn try_into_iter(mut self) -> RsResult<ProcessingBlockListIter> {
        let len = self.len()?;
        let iter = ProcessingBlockListIter {
            ptr: self.ptr,
            len,
            index: 0,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_processing_block_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for ProcessingBlockList {
    type Item = RsResult<ProcessingBlock>;
    type IntoIter = ProcessingBlockListIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl Drop for ProcessingBlockList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_recommended_processing_blocks(self.ptr.as_ptr());
        }
    }
}

pub struct ProcessingBlockListIter {
    ptr: NonNull<realsense_sys::rs2_processing_block_list>,
    len: usize,
    index: usize,
}

impl Iterator for ProcessingBlockListIter {
    type Item = RsResult<ProcessingBlock>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.len {
            let block = unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = realsense_sys::rs2_get_processing_block(
                    self.ptr.as_ptr(),
                    self.index as c_int,
                    checker.inner_mut_ptr(),
                );
                match checker.check() {
                    Ok(()) => ProcessingBlock::from_ptr(NonNull::new(ptr).unwrap()),
                    Err(err) => return Some(Err(err)),
                }
            };
            self.index += 1;
            Some(Ok(block))
        } else {
            None
        }
    }
}

pub struct ProcessingBlock {
    ptr: NonNull<realsense_sys::rs2_processing_block>,
}

impl ProcessingBlock {
    // pub fn start<F, C>(&mut self, callback: F)
    // where
    //     F: FnMut(Frame, C),
    // {
    // }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_processing_block>) -> Self {
        Self { ptr }
    }
}

impl Drop for ProcessingBlock {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_processing_block(self.ptr.as_ptr());
        }
    }
}
