use crate::bmp280::{Bmp280, Bmp280Error, registers::Bmp280Register};

#[derive(Clone, Copy, Default)]
pub struct Bmp280Calib {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
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
    pub fn read_calib_data(device: &mut Bmp280) -> Result<(), Bmp280Error> {
        let mut buffer: [u8; _] = [0u8; 24];
        let calib_read_result: Result<(), esp_hal::i2c::master::Error> = device.i2c.write_read(
            device.haddr,
            &[Bmp280Register::CalibStart as u8],
            &mut buffer,
        );

        match calib_read_result {
            Err(_) => return Err(Bmp280Error::ReadCalibrationRegFailed),
            _ => {
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
                return Ok(());
            }
        }
    }

    pub fn bmp280_compensate_t_i32(adc_t: i32, device: &mut Bmp280) -> i32 {
        let var1: i32 = (((adc_t >> 3) - ((device.calib.dig_t1 as i32) << 1))
            * (device.calib.dig_t2 as i32))
            >> 11;

        let var2: i32 = (((((adc_t >> 4) - (device.calib.dig_t1 as i32))
            * ((adc_t >> 4) - (device.calib.dig_t1 as i32)))
            >> 12)
            * (device.calib.dig_t3 as i32))
            >> 14;

        let t_fine: i32 = var1 + var2;
        let t: i32 = (t_fine * 5 + 128) >> 8;
        t
    }

    pub fn bmp280_compensate_p_i32(adc_p: i32, temp: &i32, device: &mut Bmp280) -> i32 {
        let mut var1: i32 = (temp >> 1) - 64000;
        let mut var2: i32 = ((var1 >> 2) * (var1 >> 2) >> 11) * device.calib.dig_p6 as i32;
        var2 = var2 + ((var1 * device.calib.dig_p5 as i32) << 1);
        var2 = (var2 >> 2) + ((device.calib.dig_p4 as i32) << 16);
        var1 = (device.calib.dig_p3 as i32 * (((var1 >> 2) * (var1 >> 2) >> 13) >> 3))
            + (((device.calib.dig_p2 as i32 * var1) >> 1) >> 18);
        var1 = (32768 + var1) * (device.calib.dig_p1 as i32 >> 15);
        if var1 == 0 {
            return 0;
        }
        let mut p: i32 = ((1048576 - adc_p) - (var2 >> 12)) * 3125;
        if p < 0x8 as i32 {
            p = (p << 1) / var1;
        } else {
            p = (p / var1) * 2;
        }
        var1 = device.calib.dig_p9 as i32 * (((p >> 3) * (p >> 3) >> 13) >> 12);
        var2 = (p >> 2) * (device.calib.dig_p8 as i32 >> 13);
        p = p + ((var1 + var2 + device.calib.dig_p7 as i32) >> 4);
        p
    }
}
