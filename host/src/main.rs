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
use protocol::{USB_VID, USB_PID, HID_REPORT_SIZE};
use protocol::commands::*;
use std::time::Duration;
use std::thread;

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
    /// 列出所有 HID 设备
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
        Commands::Monitor { duration } => monitor_device(&api, duration),
        Commands::Ping => send_command(&api, CMD_PING, "PING"),
        Commands::LedOn => send_command(&api, CMD_LED_ON, "LED_ON"),
        Commands::LedOff => send_command(&api, CMD_LED_OFF, "LED_OFF"),
        Commands::Status => send_command(&api, CMD_GET_STATUS, "GET_STATUS"),
        Commands::Interactive => interactive_mode(&api),
    }
}

/// 列出所有 HID 设备
fn list_devices(api: &HidApi) -> Result<()> {
    println!("===== 所有 HID 设备列表 =====\n");
    
    for (i, device) in api.device_list().enumerate() {
        let manufacturer = device.manufacturer_string().unwrap_or("N/A");
        let product = device.product_string().unwrap_or("N/A");
        let serial = device.serial_number().unwrap_or("N/A");
        
        println!("设备 #{}:", i + 1);
        println!("  VID:PID      = {:04X}:{:04X}", device.vendor_id(), device.product_id());
        println!("  制造商       = {}", manufacturer);
        println!("  产品名       = {}", product);
        println!("  序列号       = {}", serial);
        
        if device.vendor_id() == USB_VID && device.product_id() == USB_PID {
            println!("  ★★★ 这是目标设备! ★★★");
        }
        println!();
    }
    
    Ok(())
}

/// 打开目标设备
fn open_device(api: &HidApi) -> Result<hidapi::HidDevice> {
    println!("搜索设备 VID={:04X}, PID={:04X}...", USB_VID, USB_PID);
    
    let device = api
        .open(USB_VID, USB_PID)
        .context("无法打开目标设备，请检查设备是否连接")?;
    
    println!("✅ 设备已打开");
    
    if let Some(manufacturer) = device.get_manufacturer_string()? {
        println!("  制造商: {}", manufacturer);
    }
    if let Some(product) = device.get_product_string()? {
        println!("  产品名: {}", product);
    }
    if let Some(serial) = device.get_serial_number_string()? {
        println!("  序列号: {}", serial);
    }
    
    Ok(device)
}

/// 监听设备数据
fn monitor_device(api: &HidApi, duration: u64) -> Result<()> {
    let device = open_device(api)?;
    
    // 设置非阻塞模式
    device.set_blocking_mode(false)?;
    
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
        
        match device.read_timeout(&mut buf, 500)? {
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
fn send_command(api: &HidApi, cmd: u8, name: &str) -> Result<()> {
    let device = open_device(api)?;
    
    let mut report = [0u8; HID_REPORT_SIZE + 1]; // +1 for report ID
    report[0] = 0x00; // Report ID
    report[1] = cmd;
    
    println!("\n📤 发送命令: {} (0x{:02X})", name, cmd);
    
    let written = device.write(&report)?;
    println!("已发送 {} 字节", written);
    
    // 等待响应
    let mut buf = [0u8; HID_REPORT_SIZE];
    device.set_blocking_mode(false)?;
    
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
fn interactive_mode(api: &HidApi) -> Result<()> {
    let device = open_device(api)?;
    device.set_blocking_mode(false)?;
    
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
                let _ = device.write(&report);
                println!("📤 已发送 PING");
            }
            "2" => {
                let mut report = [0u8; HID_REPORT_SIZE + 1];
                report[0] = 0x00;
                report[1] = CMD_GET_STATUS;
                let _ = device.write(&report);
                println!("📤 已发送 GET_STATUS");
            }
            "3" => {
                let mut report = [0u8; HID_REPORT_SIZE + 1];
                report[0] = 0x00;
                report[1] = CMD_LED_ON;
                let _ = device.write(&report);
                println!("📤 已发送 LED_ON");
            }
            "4" => {
                let mut report = [0u8; HID_REPORT_SIZE + 1];
                report[0] = 0x00;
                report[1] = CMD_LED_OFF;
                let _ = device.write(&report);
                println!("📤 已发送 LED_OFF");
            }
            "r" => {
                match device.read_timeout(&mut buf, 2000)? {
                    0 => println!("无数据"),
                    n => {
                        println!("📥 收到 {} 字节: {:02X?}", n, &buf[..n.min(16)]);
                        if let Some(counter) = protocol::extract_counter(&buf) {
                            println!("  计数: {}", counter);
                        }
                    }
                }
            }
            "l" => {
                println!("持续监听中... (按 Enter 停止)");
                loop {
                    match device.read_timeout(&mut buf, 100)? {
                        0 => {}
                        n => {
                            if let Some(counter) = protocol::extract_counter(&buf) {
                                println!("📥 计数: {}", counter);
                            } else {
                                println!("📥 {} 字节", n);
                            }
                        }
                    }
                    
                    // 检查是否有输入
                    // 简化处理：固定监听 5 秒
                    thread::sleep(Duration::from_millis(100));
                }
            }
            _ => println!("未知命令"),
        }
    }
    
    println!("退出");
    Ok(())
}
