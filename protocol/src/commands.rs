//! 命令定义
//!
//! 定义上位机到固件的命令码

/// PING 命令 - 测试连接
pub const CMD_PING: u8 = 0x01;

/// 获取状态命令
pub const CMD_GET_STATUS: u8 = 0x02;

/// LED 开命令
pub const CMD_LED_ON: u8 = 0x03;

/// LED 关命令
pub const CMD_LED_OFF: u8 = 0x04;

/// Echo 吞吐测试命令
pub const CMD_ECHO_PERF: u8 = 0x10;

/// 响应成功
pub const RSP_OK: u8 = 0x00;

/// 响应失败
pub const RSP_ERROR: u8 = 0xFF;
