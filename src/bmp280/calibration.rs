//! BMP280 calibration coefficients and compensation functions.
//!
//! This module handles loading and applying the factory-trimmed compensation coefficients
//! from the BMP280 sensor (registers 0x88–0xA1), as described in the Bosch BMP280 datasheet
//! (BST-BMP280-DS001 rev 1.26, section 3.11 "Compensation formula" and Appendix 8.2).

use crate::bmp280::{Bmp280, Bmp280Error, registers::Bmp280Register};

/// Factory-trimmed calibration coefficients (dig_T* and dig_P*) for temperature
/// and pressure compensation.
///
/// Loaded from registers 0x88–0xA1 (24 bytes, little-endian).
/// These values are used in the fixed-point compensation formulas (datasheet §3.11.3).
#[derive(Clone, Copy, Default)]
pub struct Bmp280Calib {
    /// Temperature coefficient 1 (unsigned, typical ~27000–28000)
    pub dig_t1: u16,
    /// Temperature coefficient 2 (signed)
    pub dig_t2: i16,
    /// Temperature coefficient 3 (signed)
    pub dig_t3: i16,
    /// Pressure coefficient 1 (unsigned, typical ~30000–37000)
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
}

impl Bmp280Calib {
    /// Reads the 24-byte calibration block from registers 0x88–0xA1 and populates
    /// the coefficients in the provided `Bmp280` instance.
    ///
    /// Must be called once after soft reset and chip ID verification.
    ///
    /// # Errors
    /// - `ReadCalibrationRegFailed` if the I²C read fails (NACK, timeout, etc.)
    pub fn read_calib_data(device: &mut Bmp280) -> Result<(), Bmp280Error> {
        let mut buffer: [u8; 24] = [0u8; 24];
        let calib_read_result = device.i2c.write_read(
            device.haddr,
            &[Bmp280Register::CalibStart as u8],
            &mut buffer,
        );

        match calib_read_result {
            Err(_) => Err(Bmp280Error::ReadCalibrationRegFailed),
            Ok(_) => {
                device.calib = Bmp280Calib {
                    dig_t1: u16::from_le_bytes([buffer[0], buffer[1]]),
                    dig_t2: i16::from_le_bytes([buffer[2], buffer[3]]),
                    dig_t3: i16::from_le_bytes([buffer[4], buffer[5]]),
                    dig_p1: u16::from_le_bytes([buffer[6], buffer[7]]),
                    dig_p2: i16::from_le_bytes([buffer[8], buffer[9]]),
                    dig_p3: i16::from_le_bytes([buffer[10], buffer[11]]),
                    dig_p4: i16::from_le_bytes([buffer[12], buffer[13]]),
                    dig_p5: i16::from_le_bytes([buffer[14], buffer[15]]),
                    dig_p6: i16::from_le_bytes([buffer[16], buffer[17]]),
                    dig_p7: i16::from_le_bytes([buffer[18], buffer[19]]),
                    dig_p8: i16::from_le_bytes([buffer[20], buffer[21]]),
                    dig_p9: i16::from_le_bytes([buffer[22], buffer[23]]),
                };
                Ok(())
            }
        }
    }

    /// Compensates raw 20-bit temperature ADC value (adc_T) using the calibration coefficients.
    ///
    /// Implements the first part of the temperature compensation formula (datasheet §3.11.3).
    ///
    /// Returns `[t_fine, temperature × 100]` where:
    /// - `t_fine` is the intermediate fine temperature value (used for pressure compensation)
    /// - `temperature × 100` is the final temperature in °C (e.g. 2358 = 23.58 °C)
    pub fn bmp280_compensate_t_i32(adc_t: i32, device: &mut Bmp280) -> [i32; 2] {
        let var1 = (((adc_t >> 3) - ((device.calib.dig_t1 as i32) << 1))
            * (device.calib.dig_t2 as i32))
            >> 11;
        let var2 = (((((adc_t >> 4) - (device.calib.dig_t1 as i32))
            * ((adc_t >> 4) - (device.calib.dig_t1 as i32)))
            >> 12)
            * (device.calib.dig_t3 as i32))
            >> 14;

        let t_fine = var1 + var2;
        let t = (t_fine * 5 + 128) >> 8;
        [t_fine, t]
    }

    /// Compensates raw 20-bit pressure ADC value (adc_P) using t_fine and calibration coefficients.
    ///
    /// Implements the full pressure compensation formula (datasheet §3.11.3, Appendix 8.2).
    ///
    /// Returns pressure in Pa (e.g. 101325 = 1013.25 hPa at sea level).
    pub fn bmp280_compensate_p_i32(adc_p: i32, t_fine: &i32, device: &mut Bmp280) -> i32 {
        let mut var1 = (*t_fine >> 1) - 64000;
        let mut var2 = ((var1 >> 2) * (var1 >> 2) >> 11) * (device.calib.dig_p6 as i32);
        var2 += (var1 * (device.calib.dig_p5 as i32)) << 1;
        var2 = (var2 >> 2) + ((device.calib.dig_p4 as i32) << 16);
        var1 = (((device.calib.dig_p3 as i32) * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3)
            + (((device.calib.dig_p2 as i32) * var1) >> 1);
        var1 = (var1 >> 18) + ((device.calib.dig_p1 as i32) << 5);

        if var1 == 0 {
            return 0; // avoid division by zero
        }

        let mut p = ((1048576 - adc_p) - (var2 >> 12)) * 3125;

        if p < 0x80000000u32 as i32 {
            p = (p << 1) / var1;
        } else {
            p = (p / var1) * 2;
        }

        var1 = ((device.calib.dig_p9 as i32) * (((p >> 3) * (p >> 3)) >> 13)) >> 12;
        var2 = (p >> 2) * ((device.calib.dig_p8 as i32) >> 13);
        p += (var1 + var2 + (device.calib.dig_p7 as i32)) >> 4;

        p
    }
}
