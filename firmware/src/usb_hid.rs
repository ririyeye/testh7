//! USB HID 模块
//!
//! 提供 USB HID 设备功能，包括：
//! - 64 字节 Vendor HID 报告
//! - USB 设备状态处理
//! - HID 读写任务

use embassy_stm32::peripherals;
use embassy_stm32::usb::Driver;
use embassy_stm32::Peri;
use embassy_time::Timer;
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Handler};
use rtt_target::rprintln;
use static_cell::StaticCell;

// 使用共享的协议定义
pub use protocol::commands::*;
pub use protocol::{HID_REPORT_DESCRIPTOR, USB_PID, USB_VID};

/// USB 驱动类型别名
pub type UsbDriver = Driver<'static, peripherals::USB_OTG_FS>;

/// HID 读写器类型别名
pub type HidRW = HidReaderWriter<'static, UsbDriver, 64, 64>;

// ==================== HID 请求处理器 ====================

/// HID 请求处理器
pub struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        rprintln!("[HID] Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        rprintln!("[HID] Set report for {:?}: {:02X?}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, dur: u32) {
        // dur=0 表示无限（只在数据变化时报告），这是正常的
        if dur == 0 {
            rprintln!("[HID] Set idle: 无限 (只在变化时报告)");
        } else {
            rprintln!("[HID] Set idle: {} ms", dur);
        }
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        rprintln!("[HID] Get idle rate for {:?}", id);
        None
    }
}

// ==================== USB 设备处理器 ====================

/// USB 设备状态处理器
pub struct MyDeviceHandler {
    configured: bool,
}

impl MyDeviceHandler {
    pub fn new() -> Self {
        Self { configured: false }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        if enabled {
            rprintln!("========================================");
            rprintln!("[USB] 设备已启用 - 检测到USB连接");
            rprintln!("========================================");
        } else {
            rprintln!("========================================");
            rprintln!("[USB] 设备已禁用 - USB已断开");
            rprintln!("========================================");
        }
    }

    fn reset(&mut self) {
        rprintln!("[USB] 总线复位 - 开始枚举");
        self.configured = false;
    }

    fn addressed(&mut self, addr: u8) {
        rprintln!("[USB] 已分配地址: {} - 枚举进行中", addr);
    }

    fn configured(&mut self, configured: bool) {
        if configured {
            rprintln!("========================================");
            rprintln!("[USB] 设备已配置 - 枚举完成!");
            rprintln!("[USB] HID 端点已就绪，可以通信");
            rprintln!("========================================");
        } else {
            rprintln!("[USB] 设备未配置");
        }
        self.configured = configured;
    }

    fn suspended(&mut self, suspended: bool) {
        if suspended {
            rprintln!("[USB] 进入挂起状态 (省电模式)");
        } else {
            rprintln!("[USB] 从挂起状态恢复");
        }
    }
}

// ==================== USB 任务 ====================

/// USB HID 主任务 - 同时运行 USB 设备和 HID 读写
#[embassy_executor::task]
pub async fn task(mut device: embassy_usb::UsbDevice<'static, UsbDriver>, hid: HidRW) {
    let usb_fut = device.run();

    let hid_fut = async {
        let (mut reader, mut writer) = hid.split();

        // 发送任务
        let write_fut = async {
            let mut counter: u32 = 0;
            loop {
                // 等待端点准备好
                writer.ready().await;

                let mut report = [0u8; 64];
                // 填充测试数据
                report[0] = 0xAA; // 标识头
                report[1] = 0x55;
                report[2..6].copy_from_slice(&counter.to_le_bytes());
                report[6] = b'H';
                report[7] = b'7';
                report[8] = b'4';
                report[9] = b'3';

                match writer.write(&report).await {
                    Ok(_) => {
                        rprintln!("USB HID sent: counter={}", counter);
                        counter = counter.wrapping_add(1);
                    }
                    Err(e) => {
                        rprintln!("USB HID write error: {:?}", e);
                    }
                }
                Timer::after_millis(1000).await;
            }
        };

        // 接收任务
        let read_fut = async {
            let mut buf = [0u8; 64];
            loop {
                // 等待端点准备好
                reader.ready().await;

                match reader.read(&mut buf).await {
                    Ok(n) => {
                        rprintln!("USB HID received {} bytes: {:02X?}", n, &buf[..n.min(16)]);
                        // 处理收到的命令
                        if buf[0] == CMD_PING {
                            rprintln!("  -> Command: PING");
                        } else if buf[0] == CMD_GET_STATUS {
                            rprintln!("  -> Command: GET_STATUS");
                        }
                    }
                    Err(e) => {
                        rprintln!("USB HID read error: {:?}", e);
                        // 出错时等待一下再重试
                        Timer::after_millis(100).await;
                    }
                }
            }
        };

        // 同时运行读写
        embassy_futures::join::join(write_fut, read_fut).await
    };

    // 同时运行 USB 设备和 HID 任务
    embassy_futures::join::join(usb_fut, hid_fut).await;
}

// ==================== USB 初始化 ====================

/// USB HID 初始化所需的外设
pub struct UsbPeripherals {
    pub usb_otg_fs: Peri<'static, peripherals::USB_OTG_FS>,
    pub dp: Peri<'static, peripherals::PA12>,
    pub dm: Peri<'static, peripherals::PA11>,
}

/// 初始化并启动 USB HID
///
/// 返回 SpawnToken，调用方只需 spawn 一次
pub fn start(
    usb_periph: UsbPeripherals,
    irqs: impl embassy_stm32::interrupt::typelevel::Binding<
            embassy_stm32::interrupt::typelevel::OTG_FS,
            embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>,
        > + Copy
        + 'static,
) -> Result<embassy_executor::SpawnToken<impl Sized>, embassy_executor::SpawnError> {
    // 创建 USB 驱动
    static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; 256]);

    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false;

    let driver = Driver::new_fs(
        usb_periph.usb_otg_fs,
        irqs,
        usb_periph.dp,
        usb_periph.dm,
        ep_out_buffer,
        usb_config,
    );

    // 配置 USB 设备
    let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("STM32H743");
    config.product = Some("HID Bulk Device");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // 创建 USB builder
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 128]),
    );

    // 设置设备处理器
    static DEVICE_HANDLER: StaticCell<MyDeviceHandler> = StaticCell::new();
    builder.handler(DEVICE_HANDLER.init(MyDeviceHandler::new()));

    // 创建 HID 类
    static REQUEST_HANDLER: StaticCell<MyRequestHandler> = StaticCell::new();
    let request_handler = REQUEST_HANDLER.init(MyRequestHandler {});

    static HID_STATE: StaticCell<State> = StaticCell::new();
    let hid_state = HID_STATE.init(State::new());

    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: HID_REPORT_DESCRIPTOR,
        request_handler: Some(request_handler),
        poll_ms: 1,
        max_packet_size: 64,
        hid_subclass: embassy_usb::class::hid::HidSubclass::No,
        hid_boot_protocol: embassy_usb::class::hid::HidBootProtocol::None,
    };

    let hid = HidReaderWriter::<_, 64, 64>::new(&mut builder, hid_state, hid_config);

    // 构建 USB 设备
    let usb = builder.build();

    task(usb, hid)
}
