#![no_std]
#![no_main]

mod mpu9250;
mod shell;
mod usb_bulk;

use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::peripherals::{PB0, PH2};
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, usart, Config, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use mpu9250::{ImuData, Mpu9250};
use static_cell::StaticCell;

/// RX DMA 环形缓冲区大小
const RX_BUF_SIZE: usize = 256;

// 绑定中断
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    OTG_FS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>;
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

/// 全局 IMU 数据共享
static IMU_DATA: StaticCell<Mutex<CriticalSectionRawMutex, Option<ImuData>>> = StaticCell::new();

/// IMU 数据的全局引用 (在 main 中初始化后设置)
static mut IMU_DATA_REF: Option<&'static Mutex<CriticalSectionRawMutex, Option<ImuData>>> = None;

/// 获取 IMU 数据的引用
pub fn imu_data() -> &'static Mutex<CriticalSectionRawMutex, Option<ImuData>> {
    // 安全: 在 main 中初始化后才会被调用
    unsafe { IMU_DATA_REF.unwrap_unchecked() }
}

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

    // 配置时钟：25MHz HSE + PLL -> 480MHz，USB需要48MHz
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
    // PLL3 用于 USB 48MHz
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV5, // 25MHz / 5 = 5MHz
        mul: PllMul::MUL48,      // 5MHz * 48 = 240MHz VCO
        divp: None,
        divq: Some(PllDiv::DIV5), // 240MHz / 5 = 48MHz -> USB
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P; // SYSCLK = 480MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // AHB = 240MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // APB1 = 120MHz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // APB2 = 120MHz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // APB3 = 120MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // APB4 = 120MHz

    // 关键：设置 USB 时钟源为 PLL3_Q (48MHz)
    config.rcc.mux.usbsel = mux::Usbsel::PLL3_Q;

    // 初始化外设
    let p = embassy_stm32::init(config);

    rprintln!("STM32H743 started @ 480MHz!");

    // LED0 - PB1 (低电平点亮)
    let mut led0 = Output::new(p.PB1, Level::High, Speed::Low);

    // 启动 LED1 + KEY1 异步任务
    // LED1 - PB0, KEY1 - PH2
    spawner.spawn(led1_task(p.PB0, p.PH2).unwrap());

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

    spawner.spawn(shell_task(tx, rx).unwrap());
    rprintln!("Serial shell started on USART1 (PA9/PA10) @ 115200");

    // ========== USB Bulk 初始化 ==========
    let usb_periph = usb_bulk::UsbPeripherals {
        usb_otg_fs: p.USB_OTG_FS,
        dp: p.PA12,
        dm: p.PA11,
    };
    spawner.spawn(usb_bulk::start(usb_periph, Irqs).unwrap());
    rprintln!("USB Bulk started on PA11(D-)/PA12(D+)");

    // ========== I2C2 + MPU-9250 IMU 初始化 ==========
    // MPU-9250 连接: PH4 = I2C2_SCL, PH5 = I2C2_SDA
    let mut i2c_config = embassy_stm32::i2c::Config::default();
    // 先用低速模式调试
    i2c_config.frequency = Hertz::khz(100);

    // 检查 I2C2 时钟
    rprintln!("Creating I2C2 instance...");

    let i2c2 = I2c::new(
        p.I2C2, p.PH4, // SCL
        p.PH5, // SDA
        Irqs, p.DMA1_CH2, // TX DMA
        p.DMA1_CH3, // RX DMA
        i2c_config,
    );
    rprintln!("I2C2 initialized @ 100kHz (PH4=SCL, PH5=SDA)");

    // 初始化全局 IMU 数据
    let imu_mutex = IMU_DATA.init(Mutex::new(None));
    unsafe {
        IMU_DATA_REF = Some(imu_mutex);
    }

    // 启动 MPU-9250 任务
    spawner.spawn(mpu9250_task(i2c2).unwrap());

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

/// MPU-9250 IMU 任务：持续读取传感器数据
#[embassy_executor::task]
async fn mpu9250_task(i2c: I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>) {
    // 等待一下让 I2C 总线稳定
    Timer::after_millis(100).await;

    // 创建 MPU-9250 驱动 (地址 0x68)
    let mut imu = Mpu9250::new(i2c);

    // 初始化传感器
    match imu.init().await {
        Ok(_) => rprintln!("MPU-9250 IMU initialized successfully!"),
        Err(e) => {
            rprintln!("MPU-9250 init failed: {:?}", e);
            // 进入空闲循环
            loop {
                Timer::after_secs(1).await;
            }
        }
    }

    // 持续读取数据
    let mut print_counter = 0u32;
    loop {
        match imu.read_raw().await {
            Ok(data) => {
                // 每秒打印一次 (100次 * 10ms = 1秒)
                if print_counter % 100 == 0 {
                    let acc_s = mpu9250::AccRange::G8.sensitivity();
                    let gyro_s = mpu9250::GyroRange::Dps2000.sensitivity();
                    rprintln!(
                        "IMU: Acc({:.2},{:.2},{:.2})g Gyro({:.1},{:.1},{:.1})dps T={:.1}C",
                        data.acc.x as f32 * acc_s,
                        data.acc.y as f32 * acc_s,
                        data.acc.z as f32 * acc_s,
                        data.gyro.x as f32 * gyro_s,
                        data.gyro.y as f32 * gyro_s,
                        data.gyro.z as f32 * gyro_s,
                        data.temp_celsius()
                    );
                }
                print_counter = print_counter.wrapping_add(1);

                // 更新全局共享数据
                let mut guard = imu_data().lock().await;
                *guard = Some(data);
            }
            Err(e) => {
                rprintln!("MPU-9250 read error: {:?}", e);
            }
        }
        Timer::after_millis(10).await; // 100Hz 采样
    }
}
