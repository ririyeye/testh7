//! STM32H743 USB HID 上位机工具
//!
//! 提供与 STM32H743 固件通信的命令行工具。
//!
//! 用法:
//!   cargo run -p host -- --help
//!   cargo run -p host -- list      # 列出所有 HID 设备
//!   cargo run -p host -- monitor   # 监听设备数据
//!   cargo run -p host -- ping      # 发送 PING 命令

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use hidapi::HidApi;
use protocol::commands::*;
use protocol::{EP_IN, EP_OUT, HID_REPORT_SIZE, USB_PID, USB_VID};
use rusb;
use std::thread;
use std::time::Duration; // Import rusb crate

/// 通用设备抽象，兼容 HID 驱动与 WinUSB/libusb
enum Device {
    Hid(hidapi::HidDevice),
    Libusb {
        handle: rusb::DeviceHandle<rusb::GlobalContext>,
        timeout: Duration,
    },
}

impl Device {
    fn write_report(&self, report: &[u8]) -> Result<usize> {
        match self {
            Device::Hid(dev) => Ok(dev.write(report)?),
            Device::Libusb { handle, timeout } => {
                // 对 HID 端点使用中断写
                Ok(handle.write_interrupt(EP_OUT, report, *timeout)? as usize)
            }
        }
    }

    fn read_timeout(&self, buf: &mut [u8], timeout_ms: i32) -> Result<usize> {
        match self {
            Device::Hid(dev) => Ok(dev.read_timeout(buf, timeout_ms)?),
            Device::Libusb { handle, timeout } => {
                let effective = if timeout_ms >= 0 {
                    Duration::from_millis(timeout_ms as u64)
                } else {
                    *timeout
                };
                match handle.read_interrupt(EP_IN, buf, effective) {
                    Ok(n) => Ok(n as usize),
                    // libusb 返回超时时抛错，这里转为 0 字节以与 HID 行为一致
                    Err(rusb::Error::Timeout) => Ok(0),
                    Err(e) => Err(e.into()),
                }
            }
        }
    }
}

#[derive(Parser)]
#[command(name = "host")]
#[command(author = "STM32H743 Project")]
#[command(version = "1.0")]
#[command(about = "STM32H743 USB HID 上位机工具", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// 列出所有设备（HID + libusb）
    List,
    /// 监听设备数据
    Monitor {
        /// 监听时长（秒），0 表示无限
        #[arg(short, long, default_value = "10")]
        duration: u64,
    },
    /// 发送 PING 命令
    Ping,
    /// 发送 LED 开命令
    LedOn,
    /// 发送 LED 关命令
    LedOff,
    /// 获取设备状态
    Status,
    /// 交互式测试
    Interactive,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    let api = HidApi::new().context("无法初始化 HID API")?;

    match cli.command {
        Commands::List => list_devices(&api),
        Commands::Monitor { duration } => monitor_device(duration),
        Commands::Ping => send_command(CMD_PING, "PING"),
        Commands::LedOn => send_command(CMD_LED_ON, "LED_ON"),
        Commands::LedOff => send_command(CMD_LED_OFF, "LED_OFF"),
        Commands::Status => send_command(CMD_GET_STATUS, "GET_STATUS"),
        Commands::Interactive => interactive_mode(),
    }
}

/// 列出所有 HID 设备
fn list_devices(api: &HidApi) -> Result<()> {
    println!("===== HID 设备列表 (hidapi) =====\n");
    for (i, device) in api.device_list().enumerate() {
        let manufacturer = device.manufacturer_string().unwrap_or("N/A");
        let product = device.product_string().unwrap_or("N/A");
        let serial = device.serial_number().unwrap_or("N/A");

        println!("设备 #{}:", i + 1);
        println!(
            "  VID:PID      = {:04X}:{:04X}",
            device.vendor_id(),
            device.product_id()
        );
        println!("  制造商       = {}", manufacturer);
        println!("  产品名       = {}", product);
        println!("  序列号       = {}", serial);

        if device.vendor_id() == USB_VID && device.product_id() == USB_PID {
            println!("  ★★★ 这是目标设备! ★★★");
        }
        println!();
    }

    println!("===== USB 设备列表 (libusb) =====\n");
    let devices = rusb::devices()?;
    for (i, device) in devices.iter().enumerate() {
        let desc = device.device_descriptor()?;
        let vid = desc.vendor_id();
        let pid = desc.product_id();

        println!("设备 #{}:", i + 1);
        println!("  VID:PID      = {:04X}:{:04X}", vid, pid);

        let mut handle = device.open();
        if let Ok(ref mut h) = handle {
            // 尝试读取字符串描述符
            if let Ok(lang) = h.read_languages(Duration::from_millis(100)) {
                if let Some(&lang0) = lang.first() {
                    if let Ok(m) =
                        h.read_manufacturer_string(lang0, &desc, Duration::from_millis(100))
                    {
                        println!("  制造商       = {}", m);
                    }
                    if let Ok(p) = h.read_product_string(lang0, &desc, Duration::from_millis(100)) {
                        println!("  产品名       = {}", p);
                    }
                    if let Ok(s) =
                        h.read_serial_number_string(lang0, &desc, Duration::from_millis(100))
                    {
                        println!("  序列号       = {}", s);
                    }
                }
            }
        }

        if vid == USB_VID && pid == USB_PID {
            println!("  ★★★ 这是目标设备! ★★★");
        }
        println!();
    }

    Ok(())
}

/// 打开目标设备，优先 HID，失败则尝试 libusb (WinUSB)
fn open_device_auto() -> Result<Device> {
    println!("搜索设备 VID={:04X}, PID={:04X}...", USB_VID, USB_PID);

    // 尝试 HID
    if let Ok(api) = HidApi::new() {
        if let Ok(dev) = api.open(USB_VID, USB_PID) {
            println!("✅ 使用 HID 驱动");
            if let Ok(Some(m)) = dev.get_manufacturer_string() {
                println!("  制造商: {}", m);
            }
            if let Ok(Some(p)) = dev.get_product_string() {
                println!("  产品名: {}", p);
            }
            if let Ok(Some(s)) = dev.get_serial_number_string() {
                println!("  序列号: {}", s);
            }
            return Ok(Device::Hid(dev));
        }
    }

    // 尝试 libusb (WinUSB)
    let devices = rusb::devices()?;
    for device in devices.iter() {
        let desc = device.device_descriptor()?;
        if desc.vendor_id() == USB_VID && desc.product_id() == USB_PID {
            let handle = device.open().context("无法打开设备 (libusb)")?;

            // 如有内核驱动，先分离
            let iface = 0;
            let _ = handle.detach_kernel_driver(iface);

            handle.set_active_configuration(1)?;
            handle.claim_interface(iface)?;

            println!("✅ 使用 WinUSB/libusb 驱动");
            if let Ok(lang) = handle.read_languages(Duration::from_millis(100)) {
                if let Some(&lang0) = lang.first() {
                    if let Ok(m) =
                        handle.read_manufacturer_string(lang0, &desc, Duration::from_millis(100))
                    {
                        println!("  制造商: {}", m);
                    }
                    if let Ok(p) =
                        handle.read_product_string(lang0, &desc, Duration::from_millis(100))
                    {
                        println!("  产品名: {}", p);
                    }
                    if let Ok(s) =
                        handle.read_serial_number_string(lang0, &desc, Duration::from_millis(100))
                    {
                        println!("  序列号: {}", s);
                    }
                }
            }

            return Ok(Device::Libusb {
                handle,
                timeout: Duration::from_millis(1000),
            });
        }
    }

    anyhow::bail!("未找到设备，请检查连接或驱动 (HID/WinUSB)")
}

/// 监听设备数据
fn monitor_device(duration: u64) -> Result<()> {
    let device = open_device_auto()?;

    println!("\n===== 监听设备数据 =====");
    if duration > 0 {
        println!("持续时间: {} 秒", duration);
    } else {
        println!("持续监听中... (Ctrl+C 停止)");
    }
    println!();

    let start = std::time::Instant::now();
    let mut rx_count = 0u32;
    let mut buf = [0u8; HID_REPORT_SIZE];

    loop {
        if duration > 0 && start.elapsed().as_secs() >= duration {
            break;
        }

        match device.read_timeout(&mut buf, 2000)? {
            0 => {
                // 超时，无数据
                print!(".");
                std::io::Write::flush(&mut std::io::stdout())?;
            }
            n => {
                rx_count += 1;
                println!("\n📥 收到 {} 字节:", n);

                // 解析数据包
                if let Some(counter) = protocol::extract_counter(&buf) {
                    if let Some(device_id) = protocol::extract_device_id(&buf) {
                        let id_str = String::from_utf8_lossy(&device_id);
                        println!("  标识: 0x{:02X} 0x{:02X}", buf[0], buf[1]);
                        println!("  计数: {}", counter);
                        println!("  芯片: {}", id_str);
                    }
                } else {
                    println!("  原始数据: {:02X?}", &buf[..n.min(16)]);
                }
            }
        }

        thread::sleep(Duration::from_millis(100));
    }

    println!("\n\n测试完成: 接收 {} 个数据包", rx_count);
    Ok(())
}

/// 发送命令
fn send_command(cmd: u8, name: &str) -> Result<()> {
    let device = open_device_auto()?;

    let mut report = [0u8; HID_REPORT_SIZE + 1]; // +1 for report ID
    report[0] = 0x00; // Report ID
    report[1] = cmd;

    println!("\n📤 发送命令: {} (0x{:02X})", name, cmd);

    let written = device.write_report(&report)?;
    println!("已发送 {} 字节", written);

    // 等待响应
    let mut buf = [0u8; HID_REPORT_SIZE];
    println!("等待响应...");
    thread::sleep(Duration::from_millis(100));

    match device.read_timeout(&mut buf, 2000)? {
        0 => println!("未收到响应"),
        n => {
            println!("📥 收到响应 {} 字节: {:02X?}", n, &buf[..n.min(16)]);
        }
    }

    Ok(())
}

/// 交互式测试模式
fn interactive_mode() -> Result<()> {
    let device = open_device_auto()?;

    println!("\n===== 交互式测试 =====");
    println!("命令:");
    println!("  1 - 发送 PING");
    println!("  2 - 获取状态");
    println!("  3 - LED 开");
    println!("  4 - LED 关");
    println!("  r - 接收数据");
    println!("  l - 持续监听");
    println!("  q - 退出");

    let mut buf = [0u8; HID_REPORT_SIZE];

    loop {
        print!("\n请输入命令: ");
        std::io::Write::flush(&mut std::io::stdout())?;

        let mut input = String::new();
        std::io::stdin().read_line(&mut input)?;
        let cmd = input.trim().to_lowercase();

        match cmd.as_str() {
            "q" => break,
            "1" => {
                let mut report = [0u8; HID_REPORT_SIZE + 1];
                report[0] = 0x00;
                report[1] = CMD_PING;
                let _ = device.write_report(&report);
                println!("📤 已发送 PING");
            }
            "2" => {
                let mut report = [0u8; HID_REPORT_SIZE + 1];
                report[0] = 0x00;
                report[1] = CMD_GET_STATUS;
                let _ = device.write_report(&report);
                println!("📤 已发送 GET_STATUS");
            }
            "3" => {
                let mut report = [0u8; HID_REPORT_SIZE + 1];
                report[0] = 0x00;
                report[1] = CMD_LED_ON;
                let _ = device.write_report(&report);
                println!("📤 已发送 LED_ON");
            }
            "4" => {
                let mut report = [0u8; HID_REPORT_SIZE + 1];
                report[0] = 0x00;
                report[1] = CMD_LED_OFF;
                let _ = device.write_report(&report);
                println!("📤 已发送 LED_OFF");
            }
            "r" => match device.read_timeout(&mut buf, 2000)? {
                0 => println!("无数据"),
                n => {
                    println!("📥 收到 {} 字节: {:02X?}", n, &buf[..n.min(16)]);
                    if let Some(counter) = protocol::extract_counter(&buf) {
                        println!("  计数: {}", counter);
                    }
                }
            },
            "l" => {
                println!("持续监听中... (按 Enter 停止)");
                let (tx, rx) = std::sync::mpsc::channel();
                thread::spawn(move || {
                    let mut line = String::new();
                    let _ = std::io::stdin().read_line(&mut line);
                    let _ = tx.send(());
                });

                loop {
                    if rx.try_recv().is_ok() {
                        println!("停止监听");
                        break;
                    }

                    match device.read_timeout(&mut buf, 200)? {
                        0 => {}
                        n => {
                            if let Some(counter) = protocol::extract_counter(&buf) {
                                println!("📥 计数: {}", counter);
                            } else {
                                println!("📥 {} 字节", n);
                            }
                        }
                    }

                    thread::sleep(Duration::from_millis(50));
                }
            }
            _ => println!("未知命令"),
        }
    }

    println!("退出");
    Ok(())
}
