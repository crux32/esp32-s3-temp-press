use crate::bmp280::registers::{BMP280_RESET_REG_VALUE, Bmp280Register};

/// Oversampling setting for pressure (osrs_p[2:0] in ctrl_meas 0xF4, bits 4:2).
///
/// Controls pressure resolution, RMS noise, and conversion time.
/// Higher oversampling improves resolution/noise at cost of power/time.
///
/// | Variant     | osrs_p | Bits | Resolution | RMS Noise (typ) | Conversion time (typ) | Typical use                  |
/// |-------------|--------|------|------------|-----------------|-----------------------|------------------------------|
/// | Px1UL       | 001    | 0x04 | 16 bit     | ~3.3 Pa         | ~5–6 ms               | Weather (lowest power)       |
/// | Px2L        | 010    | 0x08 | 17 bit     | ~2.6 Pa         | ~10 ms                | Low-power handheld           |
/// | Px4STD      | 011    | 0x0C | 18 bit     | ~2.1 Pa         | ~18 ms                | Standard / balanced          |
/// | Px8H        | 100    | 0x10 | 19 bit     | ~1.6 Pa         | ~34 ms                | High resolution              |
/// | Px16UH      | 101    | 0x14 | 20 bit     | ~1.3 Pa         | ~66 ms                | Ultra-high (indoor nav)      |
///
/// Note: ×16 is officially 101–111 (all treated as ×16); 0x14 is common.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PressOversampling {
    Px1UL = 0x04,
    Px2L = 0x08,
    Px4STD = 0x0C,
    Px8H = 0x10,
    Px16UH = 0x14,
}

/// Represents the oversampling resolution setting for temperature.
///
/// osrs_t[2:0] bits (ctrl_meas register 0xF4, bits 7:5).
///
/// Controls temperature resolution, RMS noise, and conversion time.
/// Higher oversampling improves resolution and reduces noise at the cost of increased power consumption
/// and longer measurement duration.
///
/// | Variant     | osrs_t | Bits  | Resolution | RMS Noise (typ) | Conversion time (typ) | Typical use case                     |
/// |-------------|--------|-------|------------|-----------------|-----------------------|--------------------------------------|
/// | Tx1UL       | 001    | 0x20  | 16 bit     | ~0.0050 °C      | ~2 ms                 | Ultra-low power, weather monitoring  |
/// | Tx2L        | 010    | 0x40  | 17 bit     | ~0.0025 °C      | ~4 ms                 | Low-power handheld devices           |
/// | Tx4STD      | 011    | 0x60  | 18 bit     | ~0.0012 °C      | ~7 ms                 | Standard / balanced applications     |
/// | Tx8H        | 100    | 0x80  | 19 bit     | ~0.0006 °C      | ~13 ms                | High-resolution needs                |
/// | Tx16UH      | 101    | 0xA0  | 20 bit     | ~0.0003 °C      | ~25 ms                | Ultra-high precision, indoor navigation |
///
/// Notes:
/// - Settings 110 and 111 are reserved / treated as ×16 (same as 101).
/// - Temperature measurements are always performed before pressure in a measurement cycle.
/// - Even in low-oversampling modes, temperature is usually accurate enough for pressure compensation.
/// - Conversion times are approximate and scale with oversampling setting (datasheet Table 15).
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum TempOversampling {
    Tx1UL = 0x20,
    Tx2L = 0x40,
    Tx4STD = 0x60,
    Tx8H = 0x80,
    Tx16UH = 0xA0,
}

/// Power mode (mode[1:0] in ctrl_meas 0xF4, bits 1:0).
///
/// | Variant   | Value | Behavior                                                                 |
/// |-----------|-------|--------------------------------------------------------------------------|
/// | SLEEP     | 0x00  | No measurements; lowest power (~0.1–0.3 μA); registers readable          |
/// | FORCED0/1 | 0x01/0x02 | One full measurement cycle, then auto-return to sleep; host must re-trigger |
/// | NORMAL    | 0x03  | Continuous: measure → standby (t_sb) → measure; uses IIR filter effectively |
///
/// Forced mode is ideal for low sampling rates (e.g. 1/min) or event-driven reads.
/// Normal mode is best when using IIR filter or needing regular updates.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PowerMode {
    SLEEP = 0x0,
    FORCED0 = 0x1,
    FORCED1 = 0x2,
    NORMAL = 0x3,
}

/// IIR filter coefficient (filter[2:0] in config 0xF5, bits 4:2).
///
/// Smooths short-term pressure fluctuations (e.g. wind, vibrations).
/// Higher coefficient → stronger smoothing, slower response to real changes.
///
/// | Variant   | filter | Samples for ≥75% step response | Typical use case                  |
/// |-----------|--------|--------------------------------|-----------------------------------|
/// | IIROff    | 000    | 1                              | Fast response (drop/elevator)     |
/// | IIRx2     | 001    | 2                              | Moderate smoothing                |
/// | IIRx4     | 010    | 4                              | Handheld, weather                 |
/// | IIRx8     | 011    | 5                              | —                                 |
/// | IIRx16    | 100    | 8                              | Indoor navigation (strong filter) |
///
/// Note: 101–111 reserved; filter state persists across sleeps/forces.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum IIRFilter {
    IIROff = 0x0,
    IIRx2 = 0x04,
    IIRx4 = 0x08,
    IIRx8 = 0x0C,
    IIRx16 = 0x10,
}

/// Standby duration in Normal mode (t_sb[2:0] in config 0xF5, bits 7:5).
///
/// Time between end of measurement and start of next (Normal mode only).
/// Standby current ~0.2–0.5 μA (25 °C).
///
/// Shorter t_sb → higher data rate but higher average power.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum StdByTime {
    StdByOff = 0x0,
    StdBy625 = 0x20,
    StdBy125 = 0x40,
    StdBy250 = 0x60,
    StdBy500 = 0x80,
    StdBy1000 = 0xA0,
    StdBy2000 = 0xC0,
    StdBy4000 = 0xE0,
}

/// Pre-defined configurations matching Bosch datasheet recommendations (Table 7).
///
/// These presets balance power, noise, response time, and use-case needs.
/// Standby time is passed separately as it depends on desired ODR (output data rate).
///
/// See BMP280 datasheet section 3.5 "Recommended modes of operation".
#[derive(Clone, Copy)]
pub enum Bmp280ConfigPreset {
    HHDeviceLP,
    HHDeviceDyn,
    Weather,
    Elevator,
    Drop,
    Indoor,
}

#[derive(Clone, Copy)]
pub enum RegValType {
    Config,
    Measurement,
    Reset,
}

#[derive(Clone, Copy)]
pub struct Bmp280Config {
    pub tovrs: TempOversampling,
    pub povrs: PressOversampling,
    pub pmode: PowerMode,
    pub iir: IIRFilter,
    pub stdby: StdByTime,
}

impl Bmp280Config {
    /// Creates a new `Bmp280Config` with the specified settings.
    ///
    /// This is the primary constructor when you want full control over every parameter.
    /// For simpler cases, consider [`default()`] or [`default_with_preset()`].
    ///
    /// # Parameters
    /// - `tovrs`: Temperature oversampling setting (see [`TempOversampling`])
    /// - `povrs`: Pressure oversampling setting (see [`PressOversampling`])
    /// - `pmode`: Power mode (Sleep, Forced, or Normal; see [`PowerMode`])
    /// - `iir`: IIR filter coefficient for pressure smoothing (see [`IIRFilter`])
    /// - `stdby`: Standby time between measurements in Normal mode (see [`StdByTime`])
    ///
    /// # Example
    /// ```rust
    /// let cfg = Bmp280Config::new(
    ///     TempOversampling::Tx4STD,
    ///     PressOversampling::Px8H,
    ///     PowerMode::NORMAL,
    ///     IIRFilter::IIRx8,
    ///     StdByTime::StdBy500,
    /// );
    /// ```
    pub fn new(
        tovrs: TempOversampling,
        povrs: PressOversampling,
        pmode: PowerMode,
        iir: IIRFilter,
        stdby: StdByTime,
    ) -> Self {
        Self {
            povrs,
            tovrs,
            pmode,
            iir,
            stdby,
        }
    }

    /// Returns a reasonable default configuration suitable for general-purpose use.
    ///
    /// Chosen values:
    /// - Temperature oversampling: ×1 (ultra-low power, sufficient for most compensation)
    /// - Pressure oversampling: ×4 (standard resolution, balanced noise/power)
    /// - Power mode: Normal (continuous measurements with standby)
    /// - IIR filter: ×16 (strong smoothing for indoor/outdoor stability)
    /// - Standby time: 62.5 ms (moderate data rate ~16 Hz effective)
    ///
    /// This matches a good starting point for many IoT/weather/handheld applications.
    pub fn default() -> Self {
        Self {
            tovrs: TempOversampling::Tx1UL,
            povrs: PressOversampling::Px4STD,
            pmode: PowerMode::NORMAL,
            iir: IIRFilter::IIRx16,
            stdby: StdByTime::StdBy625,
        }
    }

    /// Fluent setter for pressure oversampling.
    ///
    /// ```rust
    /// let mut cfg = Bmp280Config::default();
    /// cfg.with_povrs(PressOversampling::Px16UH);
    /// ```
    pub fn with_povrs(&mut self, povrs: PressOversampling) {
        self.povrs = povrs;
    }

    /// Fluent setter for temperature oversampling.
    pub fn with_tovrs(&mut self, tovrs: TempOversampling) {
        self.tovrs = tovrs;
    }

    /// Fluent setter for power mode.
    pub fn with_pmode(&mut self, pmode: PowerMode) {
        self.pmode = pmode;
    }

    /// Fluent setter for IIR filter coefficient.
    pub fn with_filter(&mut self, iir: IIRFilter) {
        self.iir = iir;
    }

    /// Fluent setter for standby time (used in Normal mode).
    pub fn with_stdby_time(&mut self, stdby: StdByTime) {
        self.stdby = stdby;
    }

    /// Creates a configuration based on one of Bosch's recommended presets.
    ///
    /// These presets follow the guidelines in the BMP280 datasheet (section 3.5,
    /// Table 7 and Table 15) for balancing noise, power, response time, and
    /// application needs. Standby time is provided separately since it depends
    /// on desired output data rate.
    ///
    /// | Preset          | Typical Use Case                  | Oversampling (T/P) | Mode    | Filter | Notes                              |
    /// |-----------------|-----------------------------------|---------------------|---------|--------|------------------------------------|
    /// | HHDeviceLP      | Handheld low-power                | ×2 / ×16            | Normal  | ×4     | Good balance for battery life      |
    /// | HHDeviceDyn     | Handheld dynamic/response         | ×1 / ×4             | Normal  | ×16    | Fast response, strong smoothing    |
    /// | Weather         | Weather station (low freq)        | ×1 / ×1             | Forced  | Off    | Lowest power, manual trigger       |
    /// | Elevator        | Elevator / floor change detection | ×1 / ×4             | Normal  | ×4     | Moderate smoothing                 |
    /// | Drop            | Drop / impact detection           | ×1 / ×2             | Normal  | Off    | Fastest response, no smoothing     |
    /// | Indoor          | Indoor navigation / altitude      | ×2 / ×16            | Normal  | ×16    | Highest resolution & smoothing     |
    ///
    /// # Parameters
    /// - `preset`: One of the predefined use-case configurations
    /// - `stdby`: Desired standby duration (ignored in Forced mode)
    pub fn default_with_preset(preset: Bmp280ConfigPreset, stdby: StdByTime) -> Self {
        match preset {
            Bmp280ConfigPreset::HHDeviceLP => Self {
                tovrs: TempOversampling::Tx2L,
                povrs: PressOversampling::Px16UH,
                pmode: PowerMode::NORMAL,
                iir: IIRFilter::IIRx4,
                stdby,
            },

            Bmp280ConfigPreset::HHDeviceDyn => Self {
                tovrs: TempOversampling::Tx1UL,
                povrs: PressOversampling::Px4STD,
                pmode: PowerMode::NORMAL,
                iir: IIRFilter::IIRx16,
                stdby,
            },

            Bmp280ConfigPreset::Weather => Self {
                tovrs: TempOversampling::Tx1UL,
                povrs: PressOversampling::Px1UL,
                pmode: PowerMode::FORCED0,
                iir: IIRFilter::IIROff,
                stdby,
            },

            Bmp280ConfigPreset::Elevator => Self {
                tovrs: TempOversampling::Tx1UL,
                povrs: PressOversampling::Px4STD,
                pmode: PowerMode::NORMAL,
                iir: IIRFilter::IIRx4,
                stdby,
            },

            Bmp280ConfigPreset::Drop => Self {
                tovrs: TempOversampling::Tx1UL,
                povrs: PressOversampling::Px2L,
                pmode: PowerMode::NORMAL,
                iir: IIRFilter::IIROff,
                stdby,
            },

            Bmp280ConfigPreset::Indoor => Self {
                tovrs: TempOversampling::Tx2L,
                povrs: PressOversampling::Px16UH,
                pmode: PowerMode::NORMAL,
                iir: IIRFilter::IIRx16,
                stdby,
            },
        }
    }

    /// Generates a register address + value pair ready for I²C/SPI write.
    ///
    /// Returns a 2-byte array: `[register_address, value_to_write]`.
    ///
    /// # Supported register types
    /// - `RegValType::Config`     → Writes to `0xF5` (t_sb + filter)
    /// - `RegValType::Measurement` → Writes to `0xF4` (osrs_t + osrs_p + mode)
    /// - `RegValType::Reset`      → Writes `0xB6` to `0xE0` (soft reset)
    ///
    /// # Usage example
    /// ```rust
    /// let mut cfg = Bmp280Config::default();
    /// let meas = cfg.make_reg_val(RegValType::Measurement);
    /// // i2c.write(BMP280_ADDR, &meas);
    /// ```
    ///
    /// Note: This method takes `&mut self` only for consistency with fluent setters,
    /// but does not mutate the config.
    pub fn make_reg_val(&mut self, reg_val_type: RegValType) -> [u8; 2] {
        let value: [u8; 2] = match reg_val_type {
            RegValType::Config => [
                Bmp280Register::Config as u8,
                0x0 | self.stdby as u8 | self.iir as u8,
            ],
            RegValType::Measurement => [
                Bmp280Register::CtrlMeas as u8,
                0x0 | self.tovrs as u8 | self.povrs as u8 | self.pmode as u8,
            ],
            RegValType::Reset => [Bmp280Register::Reset as u8, BMP280_RESET_REG_VALUE],
        };
        value
    }
}

impl Default for Bmp280Config {
    fn default() -> Self {
        Self::default()
    }
}
