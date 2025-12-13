// MPU-9250 9轴 IMU 传感器驱动 (加速度计 + 陀螺仪 + 磁力计)
// I2C 地址: 0x68 (AD0 接地) 或 0x69 (AD0 接 VDD)
// WHO_AM_I = 0x71

#![allow(dead_code)]

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use rtt_target::rprintln;

/// MPU-9250 I2C 地址 (AD0 接地)
pub const MPU9250_ADDR: u8 = 0x68;
/// MPU-9250 I2C 备用地址 (AD0 接 VDD)
pub const MPU9250_ADDR_ALT: u8 = 0x69;
/// MPU-9250 WHO_AM_I 值
pub const MPU9250_WHO_AM_I: u8 = 0x71;

// ========== MPU-9250 寄存器地址 ==========
mod reg {
    // 配置寄存器
    pub const SMPLRT_DIV: u8 = 0x19; // 采样率分频器
    pub const CONFIG: u8 = 0x1A; // 配置
    pub const GYRO_CONFIG: u8 = 0x1B; // 陀螺仪配置
    pub const ACCEL_CONFIG: u8 = 0x1C; // 加速度计配置
    pub const ACCEL_CONFIG2: u8 = 0x1D; // 加速度计配置2

    // 中断配置
    pub const INT_PIN_CFG: u8 = 0x37;
    pub const INT_ENABLE: u8 = 0x38;
    pub const INT_STATUS: u8 = 0x3A;

    // 加速度计数据 (高字节在前)
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    pub const ACCEL_XOUT_L: u8 = 0x3C;
    pub const ACCEL_YOUT_H: u8 = 0x3D;
    pub const ACCEL_YOUT_L: u8 = 0x3E;
    pub const ACCEL_ZOUT_H: u8 = 0x3F;
    pub const ACCEL_ZOUT_L: u8 = 0x40;

    // 温度数据
    pub const TEMP_OUT_H: u8 = 0x41;
    pub const TEMP_OUT_L: u8 = 0x42;

    // 陀螺仪数据 (高字节在前)
    pub const GYRO_XOUT_H: u8 = 0x43;
    pub const GYRO_XOUT_L: u8 = 0x44;
    pub const GYRO_YOUT_H: u8 = 0x45;
    pub const GYRO_YOUT_L: u8 = 0x46;
    pub const GYRO_ZOUT_H: u8 = 0x47;
    pub const GYRO_ZOUT_L: u8 = 0x48;

    // 电源管理
    pub const PWR_MGMT_1: u8 = 0x6B;
    pub const PWR_MGMT_2: u8 = 0x6C;

    // 芯片 ID
    pub const WHO_AM_I: u8 = 0x75;
}

/// 加速度计量程
#[derive(Clone, Copy, Debug)]
pub enum AccRange {
    G2 = 0x00,  // ±2g
    G4 = 0x08,  // ±4g
    G8 = 0x10,  // ±8g
    G16 = 0x18, // ±16g
}

impl AccRange {
    /// 返回 LSB 到 g 的转换系数
    pub fn sensitivity(self) -> f32 {
        match self {
            AccRange::G2 => 2.0 / 32768.0,
            AccRange::G4 => 4.0 / 32768.0,
            AccRange::G8 => 8.0 / 32768.0,
            AccRange::G16 => 16.0 / 32768.0,
        }
    }
}

/// 陀螺仪量程
#[derive(Clone, Copy, Debug)]
pub enum GyroRange {
    Dps250 = 0x00,  // ±250 °/s
    Dps500 = 0x08,  // ±500 °/s
    Dps1000 = 0x10, // ±1000 °/s
    Dps2000 = 0x18, // ±2000 °/s
}

impl GyroRange {
    /// 返回 LSB 到 °/s 的转换系数
    pub fn sensitivity(self) -> f32 {
        match self {
            GyroRange::Dps250 => 250.0 / 32768.0,
            GyroRange::Dps500 => 500.0 / 32768.0,
            GyroRange::Dps1000 => 1000.0 / 32768.0,
            GyroRange::Dps2000 => 2000.0 / 32768.0,
        }
    }
}

/// 数字低通滤波器带宽
#[derive(Clone, Copy, Debug)]
pub enum Dlpf {
    Hz260 = 0,
    Hz184 = 1,
    Hz94 = 2,
    Hz44 = 3,
    Hz21 = 4,
    Hz10 = 5,
    Hz5 = 6,
}

/// 3 轴数据
#[derive(Clone, Copy, Debug, Default)]
pub struct AxisData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// MPU-9250 完整 IMU 数据
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuData {
    pub acc: AxisData,
    pub gyro: AxisData,
    pub temp_raw: i16,
}

impl ImuData {
    /// 温度（摄氏度）
    /// 公式: Temperature = (TEMP_OUT / 333.87) + 21.0
    pub fn temp_celsius(&self) -> f32 {
        (self.temp_raw as f32) / 333.87 + 21.0
    }
}

/// MPU-9250 驱动
pub struct Mpu9250<I2C> {
    i2c: I2C,
    addr: u8,
    acc_range: AccRange,
    gyro_range: GyroRange,
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    InvalidChipId(u8),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

impl<I2C, E> Mpu9250<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 创建 MPU-9250 驱动（使用默认地址 0x68）
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: MPU9250_ADDR,
            acc_range: AccRange::G8,
            gyro_range: GyroRange::Dps2000,
        }
    }

    /// 创建 MPU-9250 驱动（指定地址）
    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            acc_range: AccRange::G8,
            gyro_range: GyroRange::Dps2000,
        }
    }

    /// 读取单个寄存器
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut buf = [0u8];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    /// 读取多个寄存器
    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c.write_read(self.addr, &[reg], buf).await?;
        Ok(())
    }

    /// 写单个寄存器
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<E>> {
        self.i2c.write(self.addr, &[reg, val]).await?;
        Ok(())
    }

    /// 读取 WHO_AM_I (应返回 0x71)
    pub async fn who_am_i(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(reg::WHO_AM_I).await
    }

    /// 初始化传感器
    pub async fn init(&mut self) -> Result<(), Error<E>> {
        // 读取并验证芯片 ID
        let id = self.who_am_i().await?;
        if id != MPU9250_WHO_AM_I {
            rprintln!("MPU9250: unexpected WHO_AM_I: 0x{:02X}, expected 0x71", id);
            // 不返回错误，可能是兼容芯片
        }
        rprintln!("MPU9250: WHO_AM_I = 0x{:02X}", id);

        // 复位设备
        self.write_reg(reg::PWR_MGMT_1, 0x80).await?;
        Timer::after_millis(100).await;

        // 唤醒设备，使用最佳时钟源 (PLL with X-axis gyro)
        self.write_reg(reg::PWR_MGMT_1, 0x01).await?;
        Timer::after_millis(10).await;

        // 启用所有轴
        self.write_reg(reg::PWR_MGMT_2, 0x00).await?;

        // 配置采样率分频器 (1kHz / (1 + 4) = 200Hz)
        self.write_reg(reg::SMPLRT_DIV, 4).await?;

        // 配置低通滤波器 (44Hz)
        self.write_reg(reg::CONFIG, Dlpf::Hz44 as u8).await?;

        // 配置陀螺仪: ±2000°/s
        self.write_reg(reg::GYRO_CONFIG, GyroRange::Dps2000 as u8)
            .await?;
        self.gyro_range = GyroRange::Dps2000;

        // 配置加速度计: ±8g
        self.write_reg(reg::ACCEL_CONFIG, AccRange::G8 as u8)
            .await?;
        self.acc_range = AccRange::G8;

        // 配置加速度计低通滤波器
        self.write_reg(reg::ACCEL_CONFIG2, 0x03).await?; // 44.8Hz

        Timer::after_millis(10).await;
        rprintln!("MPU9250: initialized (Acc=±8g, Gyro=±2000°/s, ODR=200Hz)");
        Ok(())
    }

    /// 设置加速度计量程
    pub async fn set_acc_range(&mut self, range: AccRange) -> Result<(), Error<E>> {
        self.write_reg(reg::ACCEL_CONFIG, range as u8).await?;
        self.acc_range = range;
        Ok(())
    }

    /// 设置陀螺仪量程
    pub async fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error<E>> {
        self.write_reg(reg::GYRO_CONFIG, range as u8).await?;
        self.gyro_range = range;
        Ok(())
    }

    /// 获取当前加速度计量程
    pub fn acc_range(&self) -> AccRange {
        self.acc_range
    }

    /// 获取当前陀螺仪量程
    pub fn gyro_range(&self) -> GyroRange {
        self.gyro_range
    }

    /// 读取原始 IMU 数据 (加速度 + 温度 + 陀螺仪，共14字节)
    pub async fn read_raw(&mut self) -> Result<ImuData, Error<E>> {
        let mut buf = [0u8; 14]; // 6 acc + 2 temp + 6 gyro
        self.read_regs(reg::ACCEL_XOUT_H, &mut buf).await?;

        // MPU-9250 是大端序 (高字节在前)
        Ok(ImuData {
            acc: AxisData {
                x: i16::from_be_bytes([buf[0], buf[1]]),
                y: i16::from_be_bytes([buf[2], buf[3]]),
                z: i16::from_be_bytes([buf[4], buf[5]]),
            },
            temp_raw: i16::from_be_bytes([buf[6], buf[7]]),
            gyro: AxisData {
                x: i16::from_be_bytes([buf[8], buf[9]]),
                y: i16::from_be_bytes([buf[10], buf[11]]),
                z: i16::from_be_bytes([buf[12], buf[13]]),
            },
        })
    }

    /// 读取加速度 (单位: g)
    pub async fn read_acc_g(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let data = self.read_raw().await?;
        let s = self.acc_range.sensitivity();
        Ok((
            data.acc.x as f32 * s,
            data.acc.y as f32 * s,
            data.acc.z as f32 * s,
        ))
    }

    /// 读取角速度 (单位: °/s)
    pub async fn read_gyro_dps(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let data = self.read_raw().await?;
        let s = self.gyro_range.sensitivity();
        Ok((
            data.gyro.x as f32 * s,
            data.gyro.y as f32 * s,
            data.gyro.z as f32 * s,
        ))
    }

    /// 读取温度 (摄氏度)
    pub async fn read_temp(&mut self) -> Result<f32, Error<E>> {
        let data = self.read_raw().await?;
        Ok(data.temp_celsius())
    }
}

/// 扫描 I2C 总线，查找所有响应的设备（带超时）
pub async fn i2c_scan<I2C, E>(i2c: &mut I2C) -> heapless::Vec<u8, 16>
where
    I2C: I2c<Error = E>,
{
    use embassy_time::with_timeout;
    use embassy_time::Duration;

    let mut found = heapless::Vec::new();
    rprintln!("I2C scan starting...");

    for addr in 0x08..0x78 {
        // 尝试读取一个字节来检测设备，带 10ms 超时
        let mut buf = [0u8];
        let result = with_timeout(Duration::from_millis(10), i2c.read(addr, &mut buf)).await;
        match result {
            Ok(Ok(_)) => {
                rprintln!("  Found device at 0x{:02X}", addr);
                let _ = found.push(addr);
            }
            Ok(Err(_)) => {
                // NACK - 没有设备
            }
            Err(_) => {
                // 超时
                rprintln!("  Timeout at 0x{:02X}", addr);
            }
        }
    }

    if found.is_empty() {
        rprintln!("I2C scan: no devices found!");
    } else {
        rprintln!("I2C scan: found {} device(s)", found.len());
    }

    found
}
