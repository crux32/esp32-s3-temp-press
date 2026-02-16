#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, StackResources};
use embassy_time::Timer;
use esp_hal::gpio::AnyPin;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, i2c::master::AnyI2c};
use esp_println as _;
use esp32s3_temp_press::bmp280::config::{Bmp280Config, Bmp280ConfigPreset};
use esp32s3_temp_press::bmp280::{self, Bmp280};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // Wi-Fi controller initialization
    let radio_init: &esp_radio::Controller<'_> = &*mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );

    // Wi-Fi controller and associated network interface creation
    let (mut wifi_controller, _wifi_interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // Station mode Wi-Fi device
    let wifi_interface: esp_radio::wifi::WifiDevice<'_> = _wifi_interfaces.sta;

    // Switch off power saving mode
    wifi_controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();

    let rng = esp_hal::rng::Rng::new();
    let seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    let dhcp_config = DhcpConfig::default();
    let config = embassy_net::Config::dhcpv4(dhcp_config);

    // Network stack
    let (network_stack, network_runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    // Wi-Fi connection task
    spawner
        .spawn(esp32s3_temp_press::wifi::wifi_connection(
            wifi_controller,
            peripherals.GPIO4.into(),
        ))
        .ok();

    // Networking device
    spawner
        .spawn(esp32s3_temp_press::wifi::net_runner(network_runner))
        .ok();

    // IP address
    esp32s3_temp_press::wifi::get_ip_addr(network_stack).await;

    spawner
        .spawn(read_temp_press(
            peripherals.GPIO21.into(),
            peripherals.GPIO20.into(),
            peripherals.I2C0.into(),
            true,
        ))
        .ok();

    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn read_temp_press(
    sda_pin: AnyPin<'static>,
    scl_pin: AnyPin<'static>,
    i2c: AnyI2c<'static>,
    sdo_gnd: bool,
) {
    info!("'read_temp_press' has been started");
    let mut device: Bmp280<'_> = Bmp280::new(sda_pin, scl_pin, i2c, sdo_gnd);

    // Initialization
    let init_result = device.init();
    match init_result {
        Err(_) => info!("Initialization failed!"),
        _ => (),
    }

    // Set configuration
    let device_cfg: Bmp280Config = Bmp280Config::default_with_preset(
        Bmp280ConfigPreset::HHDeviceDyn,
        bmp280::config::StdByTime::StdBy625,
    );
    let set_conf_result: Result<(), bmp280::Bmp280Error> = device.with_config(device_cfg);
    match set_conf_result {
        Err(_) => {
            info!("Set config failed");
        }
        _ => (),
    }
    info!("BMP280 has been initialized");
    loop {
        Timer::after_secs(3).await;
        let read_result: Result<[i32; 2], esp32s3_temp_press::bmp280::Bmp280Error> =
            device.read_data();
        match read_result {
            Err(_) => info!("Error occured"),
            Ok(res) => info!("T={:?}, P={:?}", res[0], res[1]),
        }
    }
}
