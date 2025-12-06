#![no_std]
#![no_main]

mod shell;

use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{PB0, PH2};
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, usart, Config, Peri};
use embassy_time::Timer;

/// RX DMA 环形缓冲区大小
const RX_BUF_SIZE: usize = 256;

// 绑定 USART1 中断
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

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

    // 配置时钟：25MHz HSE + PLL -> 480MHz
    let mut config = Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(25),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV5,  // 25MHz / 5 = 5MHz
        mul: PllMul::MUL192,      // 5MHz * 192 = 960MHz VCO
        divp: Some(PllDiv::DIV2), // 960MHz / 2 = 480MHz -> SYSCLK
        divq: Some(PllDiv::DIV4), // 960MHz / 4 = 240MHz
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P; // SYSCLK = 480MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // AHB = 240MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // APB1 = 120MHz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // APB2 = 120MHz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // APB3 = 120MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // APB4 = 120MHz

    // 初始化外设
    let p = embassy_stm32::init(config);

    rprintln!("STM32H743 started @ 480MHz!");

    // LED0 - PB1 (低电平点亮)
    let mut led0 = Output::new(p.PB1, Level::High, Speed::Low);

    // 启动 LED1 + KEY1 异步任务
    // LED1 - PB0, KEY1 - PH2
    spawner.spawn(led1_task(p.PB0, p.PH2)).unwrap();

    // 启动串口终端 - USART1: PA9(TX), PA10(RX)
    // 使用 RingBuffered 模式，DMA 循环接收，不丢数据
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200;
    let uart = Uart::new(
        p.USART1,
        p.PA10, // RX
        p.PA9,  // TX
        Irqs,
        p.DMA1_CH0, // TX DMA
        p.DMA1_CH1, // RX DMA
        uart_config,
    )
    .unwrap();

    // 拆分 TX 和 RX，RX 转换为环形缓冲区模式
    let (tx, rx) = uart.split();
    static mut RX_BUF: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];
    let rx = rx.into_ring_buffered(unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF) });

    spawner.spawn(shell_task(tx, rx)).unwrap();
    rprintln!("Serial shell started on USART1 (PA9/PA10) @ 115200");

    // 主循环：LED0 一直闪烁
    loop {
        led0.set_low(); // 点亮
        Timer::after_millis(500).await;

        led0.set_high(); // 熄灭
        Timer::after_millis(500).await;
    }
}

/// 串口终端任务
#[embassy_executor::task]
async fn shell_task(
    tx: UartTx<'static, embassy_stm32::mode::Async>,
    rx: RingBufferedUartRx<'static>,
) {
    shell::run(tx, rx).await;
}
