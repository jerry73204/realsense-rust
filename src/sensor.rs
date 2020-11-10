//! Defines the sensor type.

use crate::{
    common::*,
    device::Device,
    error::{ErrorChecker, Result},
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
    pub(crate) ptr: NonNull<sys::rs2_sensor>,
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
    pub fn device(&self) -> Result<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_create_device_from_sensor(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Device::from_raw(ptr)
        };
        Ok(device)
    }

    /// Gets an attribute on sensor.
    ///
    /// It will return error if the attribute is not available on sensor.
    pub fn get_option(&self, option: Rs2Option) -> Result<f32> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_option(
                self.ptr.as_ptr().cast::<sys::rs2_options>(),
                option as sys::rs2_option,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val)
        }
    }

    // pub fn set_option(&mut self, option: Rs2Option, value: f32) -> Result<()> {
    //     unsafe {
    //         let mut checker = ErrorChecker::new();
    //         let val = sys::rs2_set_option(
    //             self.ptr.as_ptr().cast::<sys::rs2_options>(),
    //             option as sys::rs2_option,
    //             value,
    //             checker.inner_mut_ptr(),
    //         );
    //         checker.check()?;
    //     }
    //     Ok(())
    // }

    /// List stream profiles on sensor.
    pub fn stream_profiles(&self) -> Result<StreamProfileList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_get_stream_profiles(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            StreamProfileList::from_raw(ptr)
        };
        Ok(list)
    }

    /// Retrieves list of recommended processing blocks.
    pub fn recommended_processing_blocks(&self) -> Result<ProcessingBlockList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_get_recommended_processing_blocks(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ProcessingBlockList::from_raw(ptr)
        };
        Ok(list)
    }

    pub fn name(&self) -> Result<&str> {
        self.info(CameraInfo::Name)
    }

    pub fn serial_number(&self) -> Result<&str> {
        self.info(CameraInfo::SerialNumber)
    }

    pub fn recommended_firmware_version(&self) -> Result<&str> {
        self.info(CameraInfo::RecommendedFirmwareVersion)
    }

    pub fn physical_port(&self) -> Result<&str> {
        self.info(CameraInfo::PhysicalPort)
    }

    pub fn debug_op_code(&self) -> Result<&str> {
        self.info(CameraInfo::DebugOpCode)
    }

    pub fn advanced_mode(&self) -> Result<&str> {
        self.info(CameraInfo::AdvancedMode)
    }

    pub fn product_id(&self) -> Result<&str> {
        self.info(CameraInfo::ProductId)
    }

    pub fn camera_locked(&self) -> Result<&str> {
        self.info(CameraInfo::CameraLocked)
    }

    pub fn usb_type_descriptor(&self) -> Result<&str> {
        self.info(CameraInfo::UsbTypeDescriptor)
    }

    pub fn product_line(&self) -> Result<&str> {
        self.info(CameraInfo::ProductLine)
    }

    pub fn asic_serial_number(&self) -> Result<&str> {
        self.info(CameraInfo::AsicSerialNumber)
    }

    pub fn firmware_update_id(&self) -> Result<&str> {
        self.info(CameraInfo::FirmwareUpdateId)
    }

    pub fn count(&self) -> Result<&str> {
        self.info(CameraInfo::Count)
    }

    pub fn info(&self, kind: CameraInfo) -> Result<&str> {
        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_get_sensor_info(
                self.ptr.as_ptr(),
                kind as sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };
        // TODO: deallicate this CStr?
        let string = unsafe { CStr::from_ptr(ptr).to_str().unwrap() };
        Ok(string)
    }

    pub fn is_info_supported(&self, kind: CameraInfo) -> Result<bool> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_supports_sensor_info(
                self.ptr.as_ptr(),
                kind as sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        Ok(val != 0)
    }

    pub fn into_raw(self) -> *mut sys::rs2_sensor {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    pub unsafe fn from_raw(ptr: *mut sys::rs2_sensor) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
            _phantom: PhantomData,
        }
    }
}

impl AnySensor {
    pub fn is_extendable_to<Kind>(&self) -> Result<bool>
    where
        Kind: sensor_kind::NonAnySensorKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_is_sensor_extendable_to(
                self.ptr.as_ptr(),
                Kind::EXTENSION as sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    /// Extends to a specific sensor subtype.
    pub fn try_extend_to<Kind>(self) -> Result<result::Result<Sensor<Kind>, Self>>
    where
        Kind: sensor_kind::NonAnySensorKind,
    {
        if self.is_extendable_to::<Kind>()? {
            let ptr = self.into_raw();
            let sensor = Sensor {
                ptr: NonNull::new(ptr).unwrap(),
                _phantom: PhantomData,
            };
            Ok(Ok(sensor))
        } else {
            Ok(Err(self))
        }
    }

    /// Extends to one of a sensor subtype.
    pub fn try_extend(self) -> Result<ExtendedSensor> {
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
    pub fn depth_units(&self) -> Result<f32> {
        self.get_option(Rs2Option::DepthUnits)
    }
}

impl<Kind> ToOptions for Sensor<Kind>
where
    Kind: sensor_kind::SensorKind,
{
    fn get_options_ptr(&self) -> NonNull<sys::rs2_options> {
        self.ptr.cast::<sys::rs2_options>()
    }
}

unsafe impl<Kind> Send for Sensor<Kind> where Kind: sensor_kind::SensorKind {}

impl<Kind> Drop for Sensor<Kind>
where
    Kind: sensor_kind::SensorKind,
{
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_sensor(self.ptr.as_ptr());
        }
    }
}
