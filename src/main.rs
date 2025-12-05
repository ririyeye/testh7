#![no_std]
#![no_main]

use panic_halt as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // 初始化外设
    let p = embassy_stm32::init(Default::default());

    // LED0 - PB1, LED1 - PB0 (低电平点亮)
    // 初始状态：LED 熄灭（高电平）
    let mut led0 = Output::new(p.PB1, Level::High, Speed::Low);
    let mut led1 = Output::new(p.PB0, Level::High, Speed::Low);

    // 主循环：LED 1秒闪烁
    loop {
        // 点亮 LED（低电平）
        led0.set_low();
        led1.set_low();
        Timer::after_millis(500).await;

        // 熄灭 LED（高电平）
        led0.set_high();
        led1.set_high();
        Timer::after_millis(500).await;
    }
}
