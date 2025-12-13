//! USB Bulk 模块
//!
//! 提供 USB Bulk 设备功能，用于高速数据传输：
//! - 64 字节 Bulk 端点 (Full Speed 最大包大小)
//! - Vendor Class 设备
//! - Echo 测试功能

use embassy_stm32::peripherals;
use embassy_stm32::usb::Driver;
use embassy_stm32::Peri;
use embassy_time::Timer;
use embassy_usb::driver::{Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::{Builder, Handler};
use rtt_target::rprintln;
use static_cell::StaticCell;

// 使用共享的协议定义
pub use protocol::commands::*;
pub use protocol::{USB_PID, USB_VID};

/// Bulk 端点最大包大小 (Full Speed = 64)
pub const BULK_PACKET_SIZE: u16 = 64;

/// USB 驱动类型别名
pub type UsbDriver = Driver<'static, peripherals::USB_OTG_FS>;

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
            rprintln!("[USB] Bulk 端点已就绪，可以通信");
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

/// Echo数据通道 - 用于读写任务之间传递数据
static ECHO_CHANNEL: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    ([u8; 64], usize), // (数据, 长度)
    8,                 // 8个槽位的缓冲，实现流水线
> = embassy_sync::channel::Channel::new();

/// Bulk 端点类型
#[allow(dead_code)]
pub struct BulkEndpoints<'d, D: embassy_usb::driver::Driver<'d>> {
    pub read_ep: D::EndpointOut,
    pub write_ep: D::EndpointIn,
}

/// USB Bulk 主任务
#[embassy_executor::task]
pub async fn task(
    mut device: embassy_usb::UsbDevice<'static, UsbDriver>,
    mut read_ep: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
    mut write_ep: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
) {
    let usb_fut = device.run();

    let bulk_fut = async {
        // 等待USB配置完成
        loop {
            read_ep.wait_enabled().await;
            rprintln!("[Bulk] 端点已启用，开始通信");

            // 读取任务
            let read_fut = async {
                let mut buf = [0u8; 64];
                loop {
                    match read_ep.read(&mut buf).await {
                        Ok(n) if n > 0 => {
                            // Echo测试模式
                            if buf[0] == CMD_ECHO_PERF {
                                let mut data = [0u8; 64];
                                data[..n].copy_from_slice(&buf[..n]);
                                let _ = ECHO_CHANNEL.try_send((data, n));
                                continue;
                            }

                            // 其他命令
                            rprintln!("USB Bulk received {} bytes: {:02X?}", n, &buf[..n.min(16)]);
                            if buf[0] == CMD_PING {
                                rprintln!("  -> Command: PING");
                            }
                        }
                        Ok(_) => {}
                        Err(EndpointError::Disabled) => {
                            rprintln!("[Bulk] 端点已禁用");
                            break;
                        }
                        Err(e) => {
                            rprintln!("[Bulk] 读取错误: {:?}", e);
                            Timer::after_millis(10).await;
                        }
                    }
                }
            };

            // 写入任务
            let write_fut = async {
                loop {
                    let (data, len) = ECHO_CHANNEL.receive().await;
                    match write_ep.write(&data[..len]).await {
                        Ok(_) => {}
                        Err(EndpointError::Disabled) => {
                            rprintln!("[Bulk] 写端点已禁用");
                            break;
                        }
                        Err(e) => {
                            rprintln!("[Bulk] 写入错误: {:?}", e);
                        }
                    }
                }
            };

            // 同时运行读写
            embassy_futures::select::select(read_fut, write_fut).await;
        }
    };

    embassy_futures::join::join(usb_fut, bulk_fut).await;
}

// ==================== USB 初始化 ====================

/// USB Bulk 初始化所需的外设
pub struct UsbPeripherals {
    pub usb_otg_fs: Peri<'static, peripherals::USB_OTG_FS>,
    pub dp: Peri<'static, peripherals::PA12>,
    pub dm: Peri<'static, peripherals::PA11>,
}

/// 初始化并启动 USB Bulk
pub fn start(
    usb_periph: UsbPeripherals,
    irqs: impl embassy_stm32::interrupt::typelevel::Binding<
            embassy_stm32::interrupt::typelevel::OTG_FS,
            embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>,
        > + Copy
        + 'static,
) -> Result<embassy_executor::SpawnToken<impl Sized>, embassy_executor::SpawnError> {
    // 创建 USB 驱动 - 增大缓冲区
    static EP_OUT_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; 512]);

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
    config.product = Some("Bulk Device");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // 设置为 Vendor Class (非复合设备)
    config.device_class = 0xFF;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;
    config.composite_with_iads = false; // 必须关闭，否则要求 device_class = 0xEF

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

    // 添加 MS OS 2.0 描述符，让 Windows 自动安装 WinUSB 驱动
    builder.msos_descriptor(embassy_usb::msos::windows_version::WIN8_1, 2);
    builder.msos_feature(embassy_usb::msos::CompatibleIdFeatureDescriptor::new(
        "WINUSB", "",
    ));

    // 创建 Vendor 接口和 Bulk 端点
    let mut func = builder.function(0xFF, 0x00, 0x00);
    let mut iface = func.interface();
    let mut alt = iface.alt_setting(0xFF, 0x00, 0x00, None);

    // 创建 Bulk OUT 端点 (主机 -> 设备)
    let read_ep = alt.endpoint_bulk_out(None, BULK_PACKET_SIZE);
    // 创建 Bulk IN 端点 (设备 -> 主机)
    let write_ep = alt.endpoint_bulk_in(None, BULK_PACKET_SIZE);

    drop(func);

    // 构建 USB 设备
    let usb = builder.build();

    task(usb, read_ep, write_ep)
}
