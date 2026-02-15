/// BMP280 register map (Bosch Sensortec BMP280 datasheet, revision 1.26).
///
/// This enum provides type-safe, zero-cost access to the most commonly used registers
/// of the BMP280 pressure + temperature sensor.
///
/// All addresses are 8-bit (7-bit I²C slave address + R/W bit handled by HAL).
/// Most registers support sequential read/write (auto-incrementing pointer).
///
/// Key groups:
/// - **Measurement results** - 0xF7–0xFC (6 bytes): pressure (20-bit) + temperature (20-bit)
/// - **Control registers** - 0xF4 (measurement config), 0xF5 (IIR + standby)
/// - **Status** - 0xF3 (measuring / updating bits)
/// - **Reset & ID** - 0xE0 (soft reset), 0xD0 (chip ID)
/// - **Calibration** - 0x88–0xA1 (24 bytes, read-only, factory trimmed)
///
/// Usage example:
/// ```rust
/// let reg_addr = Bmp280Register::CtrlMeas as u8;
/// i2c.write(BMP280_ADDR, &[reg_addr, value])?;
/// ```
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Bmp280Register {
    TempXlsb = 0xFC,
    TempLsb = 0xFB,
    TempMsb = 0xFA,
    PressXlsb = 0xF9,
    PressLsb = 0xF8,
    PressMsb = 0xF7,
    Config = 0xF5,
    CtrlMeas = 0xF4,
    // bit 3 - conversion, bit 0 - updating registers
    Status = 0xF3,
    // If 0xB6 is written to the register,
    // the device is reset using the complete power-on-reset procedure
    Reset = 0xE0,
    // Chip identification number
    // Must be 0x58 after start up
    Id = 0xD0,
    // Calibration values start address
    CalibStart = 0x88,
}

pub const BMP280_RESET_REG_VALUE: u8 = 0xB6;
pub const BMP280_CHIP_ID: u8 = 0x58;
