#![no_std]
#![no_main]

use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{PB0, PH2};
use embassy_stm32::Peri;
use embassy_time::Timer;

/// LED1 + KEY1 异步任务：按下 KEY1 时 LED1 闪烁
#[embassy_executor::task]
async fn led1_task(led1_pin: Peri<'static, PB0>, key1_pin: Peri<'static, PH2>) {
    let mut led1 = Output::new(led1_pin, Level::High, Speed::Low);
    let key1 = Input::new(key1_pin, Pull::Up);

    let mut led1_state = false;

    loop {
        // KEY1 按下时（低电平）LED1 闪烁，否则熄灭
        if key1.is_low() {
            if !led1_state {
                rprintln!("KEY1 pressed!");
            }
            if led1_state {
                led1.set_low(); // 点亮
            } else {
                led1.set_high(); // 熄灭
            }
            led1_state = !led1_state;
        } else {
            if led1_state {
                rprintln!("KEY1 released!");
            }
            led1.set_high(); // KEY1 未按下，LED1 熄灭
            led1_state = false;
        }

        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 初始化 RTT
    rtt_init_print!();

    // 初始化外设
    let p = embassy_stm32::init(Default::default());

    rprintln!("STM32H743 started!");

    // LED0 - PB1 (低电平点亮)
    let mut led0 = Output::new(p.PB1, Level::High, Speed::Low);

    // 启动 LED1 + KEY1 异步任务
    // LED1 - PB0, KEY1 - PH2
    spawner.spawn(led1_task(p.PB0, p.PH2)).unwrap();

    // 主循环：LED0 一直闪烁
    loop {
        led0.set_low(); // 点亮
        Timer::after_millis(500).await;

        led0.set_high(); // 熄灭
        Timer::after_millis(500).await;
    }
}
