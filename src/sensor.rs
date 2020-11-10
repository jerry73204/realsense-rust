//! Defines the sensor type.

use crate::{
    common::*,
    device::Device,
    error::{ErrorChecker, Result as RsResult},
    kind::{CameraInfo, Rs2Option},
    options::ToOptions,
    processing_block_list::ProcessingBlockList,
    sensor_kind,
    stream_profile_list::StreamProfileList,
};

/// The enumeration of extended sensor type returned by [Sensor::try_extend](Sensor::try_extend).
#[derive(Debug)]
pub enum ExtendedSensor {
    Color(Sensor<sensor_kind::Color>),
    Depth(Sensor<sensor_kind::Depth>),
    DepthStereo(Sensor<sensor_kind::DepthStereo>),
    L500Depth(Sensor<sensor_kind::L500Depth>),
    Motion(Sensor<sensor_kind::Motion>),
    FishEye(Sensor<sensor_kind::FishEye>),
    Software(Sensor<sensor_kind::Software>),
    Pose(Sensor<sensor_kind::Pose>),
    Tm2(Sensor<sensor_kind::Tm2>),
    Other(Sensor<sensor_kind::Any>),
}

/// Represents a sensor on device.
#[derive(Debug)]
pub struct Sensor<Kind>
where
    Kind: sensor_kind::SensorKind,
{
    pub(crate) ptr: NonNull<realsense_sys::rs2_sensor>,
    _phantom: PhantomData<Kind>,
}

// type aliases

pub type ColorSensor = Sensor<sensor_kind::Color>;
pub type DepthSensor = Sensor<sensor_kind::Depth>;
pub type DepthStereoSensor = Sensor<sensor_kind::DepthStereo>;
pub type L500DepthSensor = Sensor<sensor_kind::L500Depth>;
pub type MotionSensor = Sensor<sensor_kind::Motion>;
pub type FishEyeSensor = Sensor<sensor_kind::FishEye>;
pub type SoftwareSensor = Sensor<sensor_kind::Software>;
pub type PoseSensor = Sensor<sensor_kind::Pose>;
pub type Tm2Sensor = Sensor<sensor_kind::Tm2>;
pub type AnySensor = Sensor<sensor_kind::Any>;

impl<Kind> Sensor<Kind>
where
    Kind: sensor_kind::SensorKind,
{
    /// Gets the corresponding device for sensor.
    pub fn device(&self) -> RsResult<Device> {
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

    /// Gets an attribute on sensor.
    ///
    /// It will return error if the attribute is not available on sensor.
    pub fn get_option(&self, option: Rs2Option) -> RsResult<f32> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_option(
                self.ptr.as_ptr().cast::<realsense_sys::rs2_options>(),
                option as realsense_sys::rs2_option,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val)
        }
    }

    // pub fn set_option(&mut self, option: Rs2Option, value: f32) -> RsResult<()> {
    //     unsafe {
    //         let mut checker = ErrorChecker::new();
    //         let val = realsense_sys::rs2_set_option(
    //             self.ptr.as_ptr().cast::<realsense_sys::rs2_options>(),
    //             option as realsense_sys::rs2_option,
    //             value,
    //             checker.inner_mut_ptr(),
    //         );
    //         checker.check()?;
    //     }
    //     Ok(())
    // }

    /// List stream profiles on sensor.
    pub fn stream_profiles(&self) -> RsResult<StreamProfileList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_get_stream_profiles(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            StreamProfileList::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(list)
    }

    /// Retrieves list of recommended processing blocks.
    pub fn recommended_processing_blocks(&self) -> RsResult<ProcessingBlockList> {
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

    pub fn name(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::Name)
    }

    pub fn serial_number(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::SerialNumber)
    }

    pub fn recommended_firmware_version(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::RecommendedFirmwareVersion)
    }

    pub fn physical_port(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::PhysicalPort)
    }

    pub fn debug_op_code(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::DebugOpCode)
    }

    pub fn advanced_mode(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::AdvancedMode)
    }

    pub fn product_id(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::ProductId)
    }

    pub fn camera_locked(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::CameraLocked)
    }

    pub fn usb_type_descriptor(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::UsbTypeDescriptor)
    }

    pub fn product_line(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::ProductLine)
    }

    pub fn asic_serial_number(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::AsicSerialNumber)
    }

    pub fn firmware_update_id(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::FirmwareUpdateId)
    }

    pub fn count(&self) -> RsResult<&CStr> {
        self.info(CameraInfo::Count)
    }

    pub fn info(&self, kind: CameraInfo) -> RsResult<&CStr> {
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

    pub fn is_info_supported(&self, kind: CameraInfo) -> RsResult<bool> {
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

    pub(crate) unsafe fn take(self) -> NonNull<realsense_sys::rs2_sensor> {
        let ptr = self.ptr;
        std::mem::forget(self);
        ptr
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_sensor>) -> Self {
        Self {
            ptr,
            _phantom: PhantomData,
        }
    }
}

impl AnySensor {
    pub fn is_extendable_to<Kind>(&self) -> RsResult<bool>
    where
        Kind: sensor_kind::NonAnySensorKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_is_sensor_extendable_to(
                self.ptr.as_ptr(),
                Kind::EXTENSION as realsense_sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    /// Extends to a specific sensor subtype.
    pub fn try_extend_to<Kind>(self) -> RsResult<Result<Sensor<Kind>, Self>>
    where
        Kind: sensor_kind::NonAnySensorKind,
    {
        if self.is_extendable_to::<Kind>()? {
            let ptr = unsafe { self.take() };
            let sensor = Sensor {
                ptr,
                _phantom: PhantomData,
            };
            Ok(Ok(sensor))
        } else {
            Ok(Err(self))
        }
    }

    /// Extends to one of a sensor subtype.
    pub fn try_extend(self) -> RsResult<ExtendedSensor> {
        let sensor_any = self;

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::DepthStereo>()? {
            Ok(sensor) => return Ok(ExtendedSensor::DepthStereo(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::Depth>()? {
            Ok(sensor) => return Ok(ExtendedSensor::Depth(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::L500Depth>()? {
            Ok(sensor) => return Ok(ExtendedSensor::L500Depth(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::Color>()? {
            Ok(sensor) => return Ok(ExtendedSensor::Color(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::Motion>()? {
            Ok(sensor) => return Ok(ExtendedSensor::Motion(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::FishEye>()? {
            Ok(sensor) => return Ok(ExtendedSensor::FishEye(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::Software>()? {
            Ok(sensor) => return Ok(ExtendedSensor::Software(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::Pose>()? {
            Ok(sensor) => return Ok(ExtendedSensor::Pose(sensor)),
            Err(sensor) => sensor,
        };

        let sensor_any = match sensor_any.try_extend_to::<sensor_kind::Tm2>()? {
            Ok(sensor) => return Ok(ExtendedSensor::Tm2(sensor)),
            Err(sensor) => sensor,
        };

        Ok(ExtendedSensor::Other(sensor_any))
    }
}

impl DepthSensor {
    /// Gets the depth units of depth sensor.
    pub fn depth_units(&self) -> RsResult<f32> {
        self.get_option(Rs2Option::DepthUnits)
    }
}

impl<Kind> ToOptions for Sensor<Kind>
where
    Kind: sensor_kind::SensorKind,
{
    fn get_options_ptr(&self) -> NonNull<realsense_sys::rs2_options> {
        self.ptr.cast::<realsense_sys::rs2_options>()
    }
}

unsafe impl<Kind> Send for Sensor<Kind> where Kind: sensor_kind::SensorKind {}

impl<Kind> Drop for Sensor<Kind>
where
    Kind: sensor_kind::SensorKind,
{
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_sensor(self.ptr.as_ptr());
        }
    }
}
