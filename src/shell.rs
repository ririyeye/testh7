//! 串口终端 Shell 模块
//!
//! 提供类似 Linux 终端的交互式命令行界面
//! 使用 USART1 (PA9/PA10) @ 115200 波特率

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embedded_io_async::Write;
use heapless::String;

/// 向串口写入字符串
async fn write_str(tx: &mut UartTx<'_, Async>, s: &str) {
    let _ = tx.write_all(s.as_bytes()).await;
}

/// 向串口写入字节
async fn write_bytes(tx: &mut UartTx<'_, Async>, bytes: &[u8]) {
    let _ = tx.write_all(bytes).await;
}

/// 处理命令
async fn process_command(tx: &mut UartTx<'_, Async>, cmd: &str) {
    let cmd = cmd.trim();
    let parts: heapless::Vec<&str, 8> = cmd.split_whitespace().collect();

    if parts.is_empty() {
        return;
    }

    match parts[0] {
        "help" => {
            write_str(
                tx,
                "\
Available commands:\r\n\
  help        - Show this help\r\n\
  info        - Show system info\r\n\
  led <0|1>   - Turn LED0 on/off\r\n\
  uptime      - Show uptime\r\n\
  echo <text> - Echo text back\r\n\
  clear       - Clear screen\r\n\
  reboot      - Reboot system\r\n",
            )
            .await;
        }
        "info" => {
            write_str(
                tx,
                "\
System: STM32H743\r\n\
Core: Cortex-M7 @ 480MHz\r\n\
Framework: Embassy\r\n\
UART: 115200 8N1\r\n",
            )
            .await;
        }
        "uptime" => {
            let ticks = embassy_time::Instant::now().as_millis();
            let secs = ticks / 1000;
            let mins = secs / 60;
            let hours = mins / 60;

            let mut buf: String<64> = String::new();
            let _ = core::fmt::write(
                &mut buf,
                format_args!("Uptime: {}h {}m {}s\r\n", hours, mins % 60, secs % 60),
            );
            write_str(tx, buf.as_str()).await;
        }
        "echo" => {
            if parts.len() > 1 {
                for (i, part) in parts[1..].iter().enumerate() {
                    if i > 0 {
                        write_bytes(tx, b" ").await;
                    }
                    write_str(tx, part).await;
                }
                write_bytes(tx, b"\r\n").await;
            }
        }
        "clear" => {
            write_bytes(tx, b"\x1B[2J\x1B[H").await; // ANSI 清屏
        }
        "led" => {
            if parts.len() > 1 {
                match parts[1] {
                    "0" | "off" => {
                        write_str(tx, "LED command received (off)\r\n").await;
                        // TODO: 实际控制需要通过 channel 等机制
                    }
                    "1" | "on" => {
                        write_str(tx, "LED command received (on)\r\n").await;
                    }
                    _ => {
                        write_str(tx, "Usage: led <0|1|on|off>\r\n").await;
                    }
                }
            } else {
                write_str(tx, "Usage: led <0|1|on|off>\r\n").await;
            }
        }
        "reboot" => {
            write_str(tx, "Rebooting...\r\n").await;
            embassy_time::Timer::after_millis(100).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        _ => {
            write_str(tx, "Unknown command: ").await;
            write_str(tx, cmd).await;
            write_str(tx, "\r\nType 'help' for available commands\r\n").await;
        }
    }
}

/// 串口 Shell 主循环
///
/// 使用 RingBuffered RX，DMA 循环接收，不会丢失数据
/// 支持：
/// - 字符回显
/// - Backspace 删除
/// - Ctrl+C 取消当前输入
/// - Enter 执行命令
pub async fn run(mut tx: UartTx<'_, Async>, mut rx: RingBufferedUartRx<'_>) {
    // 发送欢迎信息
    write_str(
        &mut tx,
        "\r\n\
====================================\r\n\
  STM32H743 Embedded Shell v1.0\r\n\
  Type 'help' for available commands\r\n\
====================================\r\n\
> ",
    )
    .await;

    let mut cmd_buf: String<128> = String::new();
    let mut rx_buf = [0u8; 32]; // 可以一次读取多个字节

    loop {
        // 异步读取（RingBuffered 模式，有数据就返回，不会丢失）
        match rx.read(&mut rx_buf).await {
            Ok(n) if n > 0 => {
                // 处理接收到的每个字节
                for &ch in &rx_buf[..n] {
                    match ch {
                        // Enter 键 - 执行命令
                        b'\r' | b'\n' => {
                            write_bytes(&mut tx, b"\r\n").await;

                            if !cmd_buf.is_empty() {
                                process_command(&mut tx, cmd_buf.as_str()).await;
                                cmd_buf.clear();
                            }

                            write_bytes(&mut tx, b"> ").await;
                        }
                        // Backspace
                        0x7F | 0x08 => {
                            if !cmd_buf.is_empty() {
                                cmd_buf.pop();
                                write_bytes(&mut tx, b"\x08 \x08").await;
                            }
                        }
                        // 可打印字符
                        0x20..=0x7E => {
                            if cmd_buf.push(ch as char).is_ok() {
                                write_bytes(&mut tx, &[ch]).await; // 回显
                            }
                        }
                        // Ctrl+C
                        0x03 => {
                            cmd_buf.clear();
                            write_bytes(&mut tx, b"^C\r\n> ").await;
                        }
                        _ => {}
                    }
                }
            }
            Ok(_) => {
                // 没有数据，继续等待
            }
            Err(_e) => {
                // 发生错误（如 overrun），打印错误并继续
                write_str(&mut tx, "\r\n[RX Error]\r\n> ").await;
                cmd_buf.clear();
            }
        }
    }
}
