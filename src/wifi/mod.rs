use defmt::info;
use embassy_net::{Runner, Stack};
use embassy_time::Timer;
use esp_hal::gpio::{AnyPin, Output, OutputConfig};
use esp_println as _;
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiStaState};

// Establish Wi-fi connection and reconnect
#[embassy_executor::task]
pub async fn wifi_connection(
    mut wifi_controller: WifiController<'static>,
    led_pin: AnyPin<'static>,
) {
    info!("'wifi_connection' task has been started");
    let mut wifi_led_pin: Output<'_> =
        Output::new(led_pin, esp_hal::gpio::Level::Low, OutputConfig::default());
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                wifi_controller
                    .wait_for_event(esp_radio::wifi::WifiEvent::StaDisconnected)
                    .await;
                Timer::after_millis(5000).await;
            }
            _ => {}
        }

        if !matches!(wifi_controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid("".into())
                    .with_password("".into()),
            );
            wifi_controller.set_config(&client_config).unwrap();
            wifi_controller.start_async().await.unwrap();
        }

        match wifi_controller.connect_async().await {
            Ok(_) => {
                info!("Wi-Fi connected!");
                wifi_led_pin.set_high();
            }
            Err(_) => {
                info!("Failed to connect to Wi-Fi");
                wifi_led_pin.set_low();
                Timer::after_millis(5000).await;
            }
        }
    }
}

// Run the networking device
#[embassy_executor::task]
pub async fn net_runner(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

// Get IP address
pub async fn get_ip_addr(stack: Stack<'static>) {
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(500).await;
    }

    loop {
        if let Some(config) = stack.config_v4() {
            info!("IP address acquired: {=?}!", config.address);
            config.address;
            break;
        }
        Timer::after_millis(500).await;
    }
}
