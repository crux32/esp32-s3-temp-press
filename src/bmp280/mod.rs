pub mod calibration;
pub mod config;
pub mod registers;

use defmt::info;
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::AnyPin,
    i2c::master::{AnyI2c, Config, I2c},
    time::Rate,
};

use crate::bmp280::{
    calibration::Bmp280Calib,
    config::Bmp280Config,
    registers::{BMP280_CHIP_ID, Bmp280Register},
};

/// Possible errors during BMP280 initialization.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Bmp280Error {
    /// Soft reset command failed
    ResetFailed,
    /// Failed to read chip ID register
    ReadChipIdFailed,
    /// Chip ID is not 0x58 (not a BMP280)
    ReadChipIdMismatch,
    /// Failed to read calibration coefficients
    ReadCalibrationRegFailed,
    /// Failed to write configuration register (0xF5)
    SetConfFailed,
    /// Failed to write measurement control register (0xF4)
    SetMeasConfFailed,
    /// Failed to bulk read starting from register (0xF7)
    ReadFailed,
}

/// BMP280 driver instance (blocking I²C mode).
///
/// Owns the I²C peripheral, device address, calibration data, and delay provider.
pub struct Bmp280<'d> {
    /// Blocking I²C interface
    pub i2c: I2c<'d, Blocking>,
    /// BMP280 I²C slave address (0x76 or 0x77)
    pub haddr: u8,
    /// Loaded factory calibration coefficients
    pub calib: Bmp280Calib,
    /// Delay provider (used for reset timing)
    delay: Delay,
}

impl<'d> Bmp280<'d> {
    /// Creates a new BMP280 driver instance.
    ///
    /// # Arguments
    /// * `sda_pin` - SDA GPIO pin
    /// * `scl_pin` - SCL GPIO pin
    /// * `i2c` - AnyI2c peripheral instance
    /// * `sdo_gnd` - `true` if SDO pin is connected to GND (address 0x76), `false` otherwise (0x77)
    pub fn new(sda_pin: AnyPin<'d>, scl_pin: AnyPin<'d>, i2c: AnyI2c<'d>, sdo_gnd: bool) -> Self {
        let cfg = Config::default().with_frequency(Rate::from_khz(100));
        let i2c_interface: I2c<'_, esp_hal::Blocking> = I2c::new(i2c, cfg)
            .unwrap()
            .with_sda(sda_pin)
            .with_scl(scl_pin);
        Self {
            i2c: i2c_interface,
            haddr: if sdo_gnd { 0x76 } else { 0x77 },
            calib: Bmp280Calib::default(),
            delay: Delay::new(),
        }
    }

    /// Initializes the BMP280 sensor.
    ///
    /// Sequence:
    /// 1. Soft reset (0xE0 ← 0xB6)
    /// 2. Wait ~10 ms
    /// 3. Verify chip ID (0xD0 == 0x58)
    /// 4. Read calibration coefficients
    /// 5. Apply default HHDeviceDyn preset configuration (Normal mode, strong IIR, 1 s standby)
    ///
    /// # Errors
    /// Returns `Bmp280Error` on any I²C failure or chip ID mismatch.
    pub async fn init(&mut self) -> Result<(), Bmp280Error> {
        // Perform a soft-reset
        let mut cfg = Bmp280Config::default();
        let reset_cmd = cfg.make_reg_val(config::RegValType::Reset);
        let reset_result: Result<(), esp_hal::i2c::master::Error> =
            self.i2c.write(self.haddr, &reset_cmd);

        // Wait on success
        match reset_result {
            Err(_) => {
                info!("Failed to perform soft reset");
                return Err(Bmp280Error::ResetFailed);
            }
            _ => self.delay.delay_millis(10),
        }

        // Verify chip ID
        let mut chip_id: [u8; 1] = [0u8];
        let chip_id_result =
            self.i2c
                .write_read(self.haddr, &[Bmp280Register::Id as u8], &mut chip_id);
        match chip_id_result {
            Err(_) => {
                info!("Failed to read cheap ID");
                return Err(Bmp280Error::ReadChipIdFailed);
            }
            _ => {
                if chip_id[0] != BMP280_CHIP_ID {
                    {
                        info!("Chip ID mismatch");
                        return Err(Bmp280Error::ReadChipIdMismatch);
                    };
                }
            }
        }

        // Read calibration data
        let calib_read_result: Result<(), Bmp280Error> = Bmp280Calib::read_calib_data(self);
        match calib_read_result {
            Err(e) => {
                info!("Failed to read calibration data");
                return Err(e);
            }
            _ => (),
        }

        // Configure
        let mut bmp280_config: Bmp280Config = Bmp280Config::default_with_preset(
            config::Bmp280ConfigPreset::HHDeviceDyn,
            config::StdByTime::StdBy1000,
        );
        let config_result = self.i2c.write(
            self.haddr,
            &bmp280_config.make_reg_val(config::RegValType::Config),
        );
        match config_result {
            Err(_) => {
                info!("Failed to set configuration setting");
                return Err(Bmp280Error::SetConfFailed);
            }
            _ => (),
        }
        let config_meas = self.i2c.write(
            self.haddr,
            &bmp280_config.make_reg_val(config::RegValType::Measurement),
        );
        match config_meas {
            Err(_) => {
                info!("Failed to set measurement setting");
                return Err(Bmp280Error::SetMeasConfFailed);
            }
            _ => (),
        }

        Ok(())
    }

    pub fn read_raw(&mut self) -> Result<[u8; 6], Bmp280Error> {
        let mut raw_data: [u8; 6] = [0; 6];
        let read_result: Result<(), esp_hal::i2c::master::Error> =
            self.i2c
                .write_read(self.haddr, &[Bmp280Register::PressMsb as u8], &mut raw_data);

        match read_result {
            Err(e) => {
                info!("Read failed: {:?}", e);
                return Err(Bmp280Error::ReadFailed);
            }
            _ => return Ok(raw_data),
        }
    }

    pub fn read_data(&mut self) -> Result<[i32; 2], Bmp280Error> {
        let raw_data: Result<[u8; 6], Bmp280Error> = self.read_raw();
        match raw_data {
            Err(e) => return Err(e),
            Ok(raw) => {
                // Convert readings to 32 bit
                let adc_t: i32 =
                    (raw[3] as i32) << 12 | (raw[4] as i32) << 4 | (raw[5] as i32) >> 4;
                let adc_p: i32 =
                    (raw[0] as i32) << 12 | (raw[1] as i32) << 4 | (raw[2] as i32) >> 4;

                let temp: [i32; 2] = Bmp280Calib::bmp280_compensate_t_i32(adc_t, self);
                let press: i32 = Bmp280Calib::bmp280_compensate_p_i32(adc_p, &temp[0], self);
                return Ok([temp[1], press]);
            }
        }
    }
}
