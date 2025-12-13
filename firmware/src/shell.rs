//! 串口终端 Shell 模块
//!
//! 提供类似 Linux 终端的交互式命令行界面
//! 使用 USART1 (PA9/PA10) @ 115200 波特率

use crate::fusion::{AhrsAlgorithm, CalMode, Calibration, Vec3};
use crate::{ahrs_state, cal_state, calib, imu_data, mpu9250};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embedded_io_async::Write;
use heapless::String;

fn parse_u32(s: &str) -> Option<u32> {
    if s.is_empty() {
        return None;
    }
    u32::from_str_radix(s, 10).ok()
}

fn parse_f32(s: &str) -> Option<f32> {
    let mut int_part: i32 = 0;
    let mut frac_part: i32 = 0;
    let mut frac_div: i32 = 1;
    let mut neg = false;
    let mut seen_dot = false;

    for (i, ch) in s.bytes().enumerate() {
        if i == 0 && ch == b'-' {
            neg = true;
            continue;
        }
        if ch == b'.' {
            if seen_dot {
                return None;
            }
            seen_dot = true;
            continue;
        }
        if !(b'0'..=b'9').contains(&ch) {
            return None;
        }
        let d = (ch - b'0') as i32;
        if !seen_dot {
            int_part = int_part.saturating_mul(10).saturating_add(d);
        } else {
            frac_part = frac_part.saturating_mul(10).saturating_add(d);
            frac_div = frac_div.saturating_mul(10);
        }
    }

    let mut v = int_part as f32 + (frac_part as f32) / (frac_div as f32);
    if neg {
        v = -v;
    }
    Some(v)
}

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
  imu         - Show MPU-9250 IMU data\r\n\
    att         - Show attitude quaternion\r\n\
    cal         - Calibrate gyro/acc/mag\r\n\
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
        "imu" => {
            // 读取共享的 IMU 数据
            let data = {
                let guard = imu_data().lock().await;
                *guard
            };

            match data {
                Some(imu) => {
                    // 转换为实际物理量 (MPU-9250: ±8g, ±2000°/s)
                    let acc_s = mpu9250::AccRange::G8.sensitivity();
                    let gyro_s = mpu9250::GyroRange::Dps2000.sensitivity();

                    let ax = imu.acc.x as f32 * acc_s;
                    let ay = imu.acc.y as f32 * acc_s;
                    let az = imu.acc.z as f32 * acc_s;

                    let gx = imu.gyro.x as f32 * gyro_s;
                    let gy = imu.gyro.y as f32 * gyro_s;
                    let gz = imu.gyro.z as f32 * gyro_s;

                    let temp = imu.temp_celsius();

                    let mag_str = if let Some(m) = imu.mag {
                        let mut s: String<96> = String::new();
                        let _ = core::fmt::write(
                            &mut s,
                            format_args!("  Mag:  X={:6}  Y={:6}  Z={:6}\r\n", m.x, m.y, m.z),
                        );
                        s
                    } else {
                        String::new()
                    };

                    let mut buf: String<256> = String::new();
                    let _ = core::fmt::write(
                        &mut buf,
                        format_args!(
                            "MPU-9250 IMU Data:\r\n\
  Acc:  X={:7.3}g  Y={:7.3}g  Z={:7.3}g\r\n\
  Gyro: X={:8.2}/s Y={:8.2}/s Z={:8.2}/s\r\n\
{}\
  Temp: {:.1}C\r\n\
  Raw:  ACC({},{},{}) GYRO({},{},{})\r\n",
                            ax,
                            ay,
                            az,
                            gx,
                            gy,
                            gz,
                            mag_str.as_str(),
                            temp,
                            imu.acc.x,
                            imu.acc.y,
                            imu.acc.z,
                            imu.gyro.x,
                            imu.gyro.y,
                            imu.gyro.z,
                        ),
                    );
                    write_str(tx, buf.as_str()).await;
                }
                None => {
                    write_str(tx, "IMU not initialized or no data available\r\n").await;
                }
            }
        }
        "att" => {
            if parts.len() >= 3 && parts[1] == "algo" {
                let mut ahrs = ahrs_state().lock().await;
                match parts[2] {
                    "madgwick" => {
                        ahrs.set_algo(AhrsAlgorithm::Madgwick);
                        write_str(tx, "AHRS algorithm set to Madgwick\r\n").await;
                    }
                    "mahony" => {
                        ahrs.set_algo(AhrsAlgorithm::Mahony);
                        write_str(tx, "AHRS algorithm set to Mahony\r\n").await;
                    }
                    _ => {
                        write_str(tx, "Usage: att algo <madgwick|mahony>\r\n").await;
                    }
                }
                return;
            }

            if parts.len() >= 3 && parts[1] == "beta" {
                if let Some(v) = parse_f32(parts[2]) {
                    let mut ahrs = ahrs_state().lock().await;
                    ahrs.madgwick.beta = v;
                    write_str(tx, "Madgwick beta updated\r\n").await;
                } else {
                    write_str(tx, "Usage: att beta <float>\r\n").await;
                }
                return;
            }

            if parts.len() >= 3 && parts[1] == "kp" {
                if let Some(v) = parse_f32(parts[2]) {
                    let mut ahrs = ahrs_state().lock().await;
                    ahrs.mahony.kp = v;
                    write_str(tx, "Mahony kp updated\r\n").await;
                } else {
                    write_str(tx, "Usage: att kp <float>\r\n").await;
                }
                return;
            }

            if parts.len() >= 3 && parts[1] == "ki" {
                if let Some(v) = parse_f32(parts[2]) {
                    let mut ahrs = ahrs_state().lock().await;
                    ahrs.mahony.ki = v;
                    write_str(tx, "Mahony ki updated\r\n").await;
                } else {
                    write_str(tx, "Usage: att ki <float>\r\n").await;
                }
                return;
            }

            let ahrs = { *ahrs_state().lock().await };
            let q = ahrs.quaternion();
            let (roll, pitch, yaw) = q.to_euler_deg();

            let mut buf: String<256> = String::new();
            let algo_name = match ahrs.algo {
                AhrsAlgorithm::Madgwick => "Madgwick",
                AhrsAlgorithm::Mahony => "Mahony",
            };
            let _ = core::fmt::write(
                &mut buf,
                format_args!(
                    "Attitude ({})\r\n  q = [{:.6}, {:.6}, {:.6}, {:.6}]\r\n  rpy = [{:.2}, {:.2}, {:.2}] deg\r\n",
                    algo_name, q.w, q.x, q.y, q.z, roll, pitch, yaw
                ),
            );
            write_str(tx, buf.as_str()).await;
        }
        "cal" => {
            if parts.len() == 1 || parts[1] == "help" {
                write_str(
                    tx,
                    "\
Calibration commands:\r\n\
  cal status              - Show calibration status\r\n\
  cal show                - Show current calibration values\r\n\
  cal reset               - Reset calibration to defaults\r\n\
  cal gyro <secs>         - Estimate gyro bias (keep still)\r\n\
  cal acc <secs>          - Acc min/max calib (slowly rotate to cover all axes)\r\n\
  cal mag <secs>          - Mag min/max calib (figure-8 / rotate all axes)\r\n",
                )
                .await;
                return;
            }

            match parts[1] {
                "reset" => {
                    let mut c = calib().lock().await;
                    *c = Calibration::default();
                    let mut st = cal_state().lock().await;
                    st.mode = CalMode::None;
                    write_str(tx, "Calibration reset\r\n").await;
                }
                "show" => {
                    let c = { *calib().lock().await };
                    let mut buf: String<256> = String::new();
                    let _ = core::fmt::write(
                        &mut buf,
                        format_args!(
                            "Calibration:\r\n  gyro_bias_dps = [{:.4},{:.4},{:.4}]\r\n  acc_bias_g    = [{:.4},{:.4},{:.4}]\r\n  acc_scale     = [{:.4},{:.4},{:.4}]\r\n  mag_bias_uT   = [{:.1},{:.1},{:.1}]\r\n  mag_scale     = [{:.4},{:.4},{:.4}]\r\n",
                            c.gyro_bias_dps.x,
                            c.gyro_bias_dps.y,
                            c.gyro_bias_dps.z,
                            c.acc_bias_g.x,
                            c.acc_bias_g.y,
                            c.acc_bias_g.z,
                            c.acc_scale.x,
                            c.acc_scale.y,
                            c.acc_scale.z,
                            c.mag_bias_u_t.x,
                            c.mag_bias_u_t.y,
                            c.mag_bias_u_t.z,
                            c.mag_scale.x,
                            c.mag_scale.y,
                            c.mag_scale.z,
                        ),
                    );
                    write_str(tx, buf.as_str()).await;
                }
                "status" => {
                    let st = { *cal_state().lock().await };
                    let mut buf: String<128> = String::new();
                    match st.mode {
                        CalMode::None => {
                            write_str(tx, "Calibration: idle\r\n").await;
                        }
                        CalMode::GyroBias { remaining, .. } => {
                            let _ = core::fmt::write(
                                &mut buf,
                                format_args!(
                                    "Calibration: gyro bias running ({} samples left)\r\n",
                                    remaining
                                ),
                            );
                            write_str(tx, buf.as_str()).await;
                        }
                        CalMode::AccMinMax { remaining, .. } => {
                            let _ = core::fmt::write(
                                &mut buf,
                                format_args!(
                                    "Calibration: acc min/max running ({} samples left)\r\n",
                                    remaining
                                ),
                            );
                            write_str(tx, buf.as_str()).await;
                        }
                        CalMode::MagMinMax { remaining, .. } => {
                            let _ = core::fmt::write(
                                &mut buf,
                                format_args!(
                                    "Calibration: mag min/max running ({} samples left)\r\n",
                                    remaining
                                ),
                            );
                            write_str(tx, buf.as_str()).await;
                        }
                    }
                }
                "gyro" => {
                    let secs = parts.get(2).and_then(|s| parse_u32(s)).unwrap_or(3);
                    let samples = secs.saturating_mul(100);
                    let mut st = cal_state().lock().await;
                    st.mode = CalMode::GyroBias {
                        remaining: samples,
                        sum: Vec3::new(0.0, 0.0, 0.0),
                        count: 0,
                    };
                    write_str(tx, "Gyro bias calibration started (keep still)\r\n").await;
                }
                "acc" => {
                    let secs = parts.get(2).and_then(|s| parse_u32(s)).unwrap_or(10);
                    let samples = secs.saturating_mul(100);
                    let mut st = cal_state().lock().await;
                    st.mode = CalMode::AccMinMax {
                        remaining: samples,
                        min: Vec3::new(0.0, 0.0, 0.0),
                        max: Vec3::new(0.0, 0.0, 0.0),
                        init: false,
                    };
                    write_str(tx, "Acc min/max calibration started (rotate slowly)\r\n").await;
                }
                "mag" => {
                    let secs = parts.get(2).and_then(|s| parse_u32(s)).unwrap_or(15);
                    let samples = secs.saturating_mul(100);
                    let mut st = cal_state().lock().await;
                    st.mode = CalMode::MagMinMax {
                        remaining: samples,
                        min: Vec3::new(0.0, 0.0, 0.0),
                        max: Vec3::new(0.0, 0.0, 0.0),
                        init: false,
                    };
                    write_str(tx, "Mag min/max calibration started (figure-8)\r\n").await;
                }
                _ => {
                    write_str(tx, "Type 'cal help' for usage\r\n").await;
                }
            }
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
