//! USB HID 通信协议定义
//!
//! 此 crate 定义了固件和上位机之间共享的协议常量和数据结构。
//! 支持 `no_std` 环境（固件）和 `std` 环境（上位机）。

#![cfg_attr(not(feature = "std"), no_std)]

// no_std 环境下需要手动导入 core prelude
#[cfg(not(feature = "std"))]
use core::prelude::rust_2021::*;

pub mod commands;

/// USB 设备 VID (pid.codes test VID)
pub const USB_VID: u16 = 0x1209;
/// USB 设备 PID
pub const USB_PID: u16 = 0x0001;

/// HID 报告大小（字节）
pub const HID_REPORT_SIZE: usize = 64;

/// HID IN 端点地址 (设备 -> 主机)
pub const EP_IN: u8 = 0x81;
/// HID OUT 端点地址 (主机 -> 设备)
pub const EP_OUT: u8 = 0x01;

// 自定义 HID 报告描述符 - 64字节 Vendor Defined
pub const HID_REPORT_DESCRIPTOR: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01, // Usage (0x01)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x02, //   Usage (0x02) - Input
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x40, //   Report Count (64)
    0x81, 0x02, //   Input (Data, Variable, Absolute)
    0x09, 0x03, //   Usage (0x03) - Output
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x40, //   Report Count (64)
    0x91, 0x02, //   Output (Data, Variable, Absolute)
    0xC0, // End Collection
];

/// 设备数据包头部标识
pub const PACKET_HEADER: [u8; 2] = [0xAA, 0x55];

/// 检查数据包头部是否有效
pub fn is_valid_packet(data: &[u8]) -> bool {
    data.len() >= 2 && data[0] == PACKET_HEADER[0] && data[1] == PACKET_HEADER[1]
}

/// 从数据包中提取计数器值
pub fn extract_counter(data: &[u8]) -> Option<u32> {
    if data.len() >= 6 && is_valid_packet(data) {
        Some(u32::from_le_bytes([data[2], data[3], data[4], data[5]]))
    } else {
        None
    }
}

/// 从数据包中提取设备标识
pub fn extract_device_id(data: &[u8]) -> Option<[u8; 4]> {
    if data.len() >= 10 && is_valid_packet(data) {
        Some([data[6], data[7], data[8], data[9]])
    } else {
        None
    }
}
