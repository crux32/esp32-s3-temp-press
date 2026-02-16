//! BMP280 driver for ESP32-S3 (blocking I²C mode).
//!
//! This crate provides a safe, type-safe driver for the Bosch BMP280 digital pressure
//! and temperature sensor over I²C (7-bit addressing: 0x76 or 0x77).
//!
//! Key features:
//! - Soft reset and chip ID verification
//! - Factory calibration coefficient loading (registers 0x88–0xA1)
//! - Configurable oversampling, filter, standby time, and power mode
//! - Raw measurement reading and fixed-point compensation (temperature °C×100, pressure Pa)
//!
//! Datasheet reference: Bosch BMP280 Data Sheet (BST-BMP280-DS001 rev 1.26, October 2021)
//!
//! Typical usage:
//! ```rust
//! let mut bmp = Bmp280::new(sda, scl, i2c_periph, true); // address 0x76
//! bmp.init()?;
//! let cfg = Bmp280Config::default_with_preset(Bmp280ConfigPreset::HHDeviceDyn, StdByTime::StdBy625);
//! bmp.with_config(cfg)?;
//! let [temp, press] = bmp.read_data()?;
//! // temp in °C×100, press in Pa
//! ```

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

/// Errors that can occur during BMP280 operation.
///
/// All I²C failures are mapped to specific variants for easier debugging.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Bmp280Error {
    /// Soft reset (write 0xB6 to 0xE0) failed
    ResetFailed,
    /// Failed to read chip ID register (0xD0)
    ReadChipIdFailed,
    /// Chip ID ≠ 0x58 (device is not a BMP280)
    ReadChipIdMismatch,
    /// Failed to read calibration block (0x88–0xA1)
    ReadCalibrationRegFailed,
    /// Failed to write configuration register (0xF5)
    SetConfFailed,
    /// Failed to write measurement control register (0xF4)
    SetMeasConfFailed,
    /// Failed to read measurement block (0xF7–0xFC)
    ReadFailed,
}

/// BMP280 driver (blocking I²C).
///
/// Manages a single BMP280 sensor instance over I²C.
/// Owns the I²C bus, device address, calibration data, and delay provider.
///
/// # Lifetimes
/// `'d` ties to the lifetime of the GPIO pins and I²C peripheral.
pub struct Bmp280<'d> {
    /// Blocking I²C master interface
    pub i2c: I2c<'d, Blocking>,
    /// I²C slave address (0x76 if SDO=GND, 0x77 if SDO=VCC)
    pub haddr: u8,
    /// Factory calibration coefficients (loaded during init)
    pub calib: calibration::Bmp280Calib,
    /// Delay provider for reset and timing waits
    delay: Delay,
}

impl<'d> Bmp280<'d> {
    /// Creates a new BMP280 driver instance.
    ///
    /// # Arguments
    /// * `sda_pin`   – SDA GPIO pin
    /// * `scl_pin`   – SCL GPIO pin
    /// * `i2c`       – AnyI2c peripheral instance (e.g. I2C0)
    /// * `sdo_gnd`   – `true` → address 0x76 (SDO tied to GND), `false` → 0x77 (SDO tied to VCC)
    ///
    /// # Panics
    /// Panics if I²C initialization fails (rare).
    pub fn new(sda_pin: AnyPin<'d>, scl_pin: AnyPin<'d>, i2c: AnyI2c<'d>, sdo_gnd: bool) -> Self {
        let cfg = Config::default().with_frequency(Rate::from_khz(100));
        let i2c_interface = I2c::new(i2c, cfg)
            .unwrap()
            .with_sda(sda_pin)
            .with_scl(scl_pin);

        Self {
            i2c: i2c_interface,
            haddr: if sdo_gnd { 0x76 } else { 0x77 },
            calib: calibration::Bmp280Calib::default(),
            delay: Delay::new(),
        }
    }

    /// Initializes the BMP280 sensor.
    ///
    /// Execution sequence:
    /// 1. Soft reset (write 0xB6 to register 0xE0)
    /// 2. Wait ~10 ms for reset to complete
    /// 3. Verify chip ID (register 0xD0 must return 0x58)
    /// 4. Read factory calibration coefficients (registers 0x88–0xA1)
    ///
    /// # Errors
    /// Returns `Bmp280Error` on any I²C failure or chip ID mismatch.
    pub fn init(&mut self) -> Result<(), Bmp280Error> {
        // Soft reset
        let mut cfg = config::Bmp280Config::default();
        let reset_cmd = cfg.make_reg_val(config::RegValType::Reset);
        self.i2c.write(self.haddr, &reset_cmd).map_err(|_| {
            info!("Soft reset failed");
            Bmp280Error::ResetFailed
        })?;

        self.delay.delay_millis(10);

        // Verify chip ID
        let mut chip_id = [0u8; 1];
        self.i2c
            .write_read(
                self.haddr,
                &[registers::Bmp280Register::Id as u8],
                &mut chip_id,
            )
            .map_err(|_| {
                info!("Failed to read chip ID");
                Bmp280Error::ReadChipIdFailed
            })?;

        if chip_id[0] != registers::BMP280_CHIP_ID {
            info!("Chip ID mismatch: got 0x{:02x}, expected 0x58", chip_id[0]);
            return Err(Bmp280Error::ReadChipIdMismatch);
        }

        // Load calibration coefficients
        calibration::Bmp280Calib::read_calib_data(self)?;

        info!("BMP280 initialized successfully");
        Ok(())
    }

    /// Applies a configuration to the sensor.
    ///
    /// Writes the config register (0xF5) and measurement control register (0xF4).
    /// Use with `Bmp280Config::default_with_preset()` or manual setup.
    ///
    /// # Errors
    /// Returns `Bmp280Error` on I²C write failure.
    pub fn with_config(
        &mut self,
        mut bmp280_config: config::Bmp280Config,
    ) -> Result<(), Bmp280Error> {
        let config_cmd = bmp280_config.make_reg_val(config::RegValType::Config);
        self.i2c.write(self.haddr, &config_cmd).map_err(|_| {
            info!("Failed to write config register (0xF5)");
            Bmp280Error::SetConfFailed
        })?;

        let meas_cmd = bmp280_config.make_reg_val(config::RegValType::Measurement);
        self.i2c.write(self.haddr, &meas_cmd).map_err(|_| {
            info!("Failed to write measurement control register (0xF4)");
            Bmp280Error::SetMeasConfFailed
        })?;

        Ok(())
    }

    /// Reads the raw 6-byte measurement block (registers 0xF7–0xFC).
    ///
    /// Returns `[press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb]`.
    ///
    /// # Errors
    /// Returns `Bmp280Error::ReadFailed` on I²C read failure.
    pub fn read_raw(&mut self) -> Result<[u8; 6], Bmp280Error> {
        let mut raw_data = [0u8; 6];
        self.i2c
            .write_read(
                self.haddr,
                &[registers::Bmp280Register::PressMsb as u8],
                &mut raw_data,
            )
            .map_err(|e| {
                info!("Raw measurement read failed: {:?}", e);
                Bmp280Error::ReadFailed
            })?;

        Ok(raw_data)
    }

    /// Reads raw data, converts to 20-bit ADC values, and applies compensation.
    ///
    /// Returns `[temperature °C × 100, pressure Pa]`.
    ///
    /// # Errors
    /// Propagates any error from `read_raw()`.
    pub fn read_data(&mut self) -> Result<[i32; 2], Bmp280Error> {
        let raw = self.read_raw()?;

        // Extract 20-bit ADC values (datasheet §5.4.7 & §5.4.8)
        let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
        let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);

        let temp_array = calibration::Bmp280Calib::bmp280_compensate_t_i32(adc_t, self);
        let t_fine = temp_array[0];
        let temp = temp_array[1];

        let press = calibration::Bmp280Calib::bmp280_compensate_p_i32(adc_p, &t_fine, self);

        Ok([temp, press])
    }
}
