#![no_std]
#![no_main]

use panic_halt as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pull, Speed};
use embassy_time::Timer;

/// LED1 + KEY1 异步任务：按下 KEY1 时 LED1 闪烁
#[embassy_executor::task]
async fn led1_task(led1_pin: AnyPin, key1_pin: AnyPin) {
    let mut led1 = Output::new(led1_pin, Level::High, Speed::Low);
    let key1 = Input::new(key1_pin, Pull::Up);

    let mut led1_state = false;

    loop {
        // KEY1 按下时（低电平）LED1 闪烁，否则熄灭
        if key1.is_low() {
            if led1_state {
                led1.set_low(); // 点亮
            } else {
                led1.set_high(); // 熄灭
            }
            led1_state = !led1_state;
        } else {
            led1.set_high(); // KEY1 未按下，LED1 熄灭
            led1_state = false;
        }

        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 初始化外设
    let p = embassy_stm32::init(Default::default());

    // LED0 - PB1 (低电平点亮)
    let mut led0 = Output::new(p.PB1, Level::High, Speed::Low);

    // 启动 LED1 + KEY1 异步任务
    // LED1 - PB0, KEY1 - PH2
    spawner
        .spawn(led1_task(p.PB0.into(), p.PH2.into()))
        .unwrap();

    // 主循环：LED0 一直闪烁
    loop {
        led0.set_low(); // 点亮
        Timer::after_millis(500).await;

        led0.set_high(); // 熄灭
        Timer::after_millis(500).await;
    }
}
