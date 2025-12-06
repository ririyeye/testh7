# STM32H743 项目

基于 Embassy 异步框架的 STM32H743 开发项目，包含单片机固件和 Windows 上位机。

## 项目结构

```
testh7/
├── Cargo.toml              # Workspace 根配置
├── .cargo/config.toml      # Cargo 别名和 ARM target 配置
│
├── firmware/               # 单片机固件 (no_std, Cortex-M7)
│   ├── Cargo.toml
│   ├── .cargo/config.toml  # 默认 ARM target
│   ├── build.rs
│   ├── memory.x            # 链接脚本
│   └── src/
│       ├── main.rs         # 主程序入口
│       ├── shell.rs        # 串口命令行
│       └── usb_hid.rs      # USB HID 通信
│
├── protocol/               # 共享协议库 (no_std + 可选 std)
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs          # USB VID/PID, HID 描述符
│       └── commands.rs     # 命令码定义
│
├── host/                   # Windows 上位机 (std)
│   ├── Cargo.toml
│   └── src/
│       └── main.rs         # CLI 工具
│
└── tools/                  # Python 测试脚本 (遗留)
    └── usb_hid_test.py
```

## 快速开始

### 环境准备

1. 安装 Rust：https://rustup.rs/
2. 安装 ARM target：
   ```powershell
   rustup target add thumbv7em-none-eabihf
   ```
3. 安装 probe-rs 工具（包含 cargo-embed/cargo-flash）：
   ```powershell
   irm https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.ps1 | iex
   ```
4. 安装 size/objdump 等工具（cargo-binutils + LLVM 工具）：
   ```powershell
   cargo install cargo-binutils
   rustup component add llvm-tools-preview
   ```
   安装后即可使用 `cargo size` / `cargo objdump` 等命令查看 Flash/RAM 使用。

### 编译固件

```powershell
# 方式1：在 firmware 目录（推荐）
cd firmware
cargo build              # Debug 版本
cargo build --release    # Release 版本

# 方式2：在根目录用别名
cargo fw                 # Debug
cargo fwr                # Release
cargo fwd                # Release-debuggable (可调试的优化版)
```

### 烧录运行

```powershell
# 在 firmware 目录
cd firmware
cargo run --release      # 编译 + 烧录 + RTT日志
```

按 `Ctrl+C` 退出 RTT 日志监控。

### 编译上位机

```powershell
# 在根目录
cargo host               # Debug
cargo hostr              # Release

# 运行上位机
cargo hostrun list       # 列出 HID 设备
cargo hostrun monitor    # 监听设备数据
cargo hostrun ping       # 发送 PING 命令
```

## 常用命令

### Cargo 别名（在根目录使用）

| 命令 | 说明 |
|------|------|
| `cargo fw` | 编译固件 (Debug) |
| `cargo fwr` | 编译固件 (Release) |
| `cargo fwd` | 编译固件 (Release-debuggable) |
| `cargo fwcheck` | 检查固件语法 |
| `cargo host` | 编译上位机 |
| `cargo hostr` | 编译上位机 (Release) |
| `cargo hostrun` | 运行上位机 |
| `cargo hostcheck` | 检查上位机语法 |

### 查看 Flash/RAM 使用

```powershell
# 查看固件大小（需要先编译）
cd firmware
cargo size --release

# 详细 section 信息
cargo size --release -- -A

# 或者使用 arm-none-eabi-size（如果安装了 ARM 工具链）
arm-none-eabi-size target/thumbv7em-none-eabihf/release/stm32h743-firmware
```

如果没有 `cargo-size`，先安装：
```powershell
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

输出示例：
```
   text    data     bss     dec     hex filename
  44032     120    8192   52344    cc78 stm32h743-firmware
```
- **text**: 代码段 (Flash)
- **data**: 初始化数据 (Flash + RAM)
- **bss**: 未初始化数据 (RAM)

### probe-rs 命令

```powershell
# 烧录固件
probe-rs run --chip STM32H743IITx target/thumbv7em-none-eabihf/release/stm32h743-firmware

# 只烧录不运行
probe-rs download --chip STM32H743IITx target/thumbv7em-none-eabihf/release/stm32h743-firmware

# 擦除芯片
probe-rs erase --chip STM32H743IITx

# 复位芯片
probe-rs reset --chip STM32H743IITx

# 查看支持的芯片
probe-rs chip list | Select-String "STM32H7"
```

## 硬件配置

### MCU
- **型号**: STM32H743IIT6
- **内核**: Cortex-M7 @ 480MHz
- **Flash**: 2MB
- **RAM**: 512KB (AXI SRAM)

### 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| LED0 | PB1 | 低电平点亮，主循环闪烁 |
| LED1 | PB0 | 低电平点亮，KEY1 控制 |
| KEY1 | PH2 | 上拉输入，按下为低 |
| USART1 TX | PA9 | 串口终端 115200 |
| USART1 RX | PA10 | 串口终端 |
| USB D+ | PA12 | USB HID |
| USB D- | PA11 | USB HID |

### USB 配置
- **VID**: 0x1209 (pid.codes test VID)
- **PID**: 0x0001
- **类型**: HID Vendor Defined (64 字节报告)

## 串口终端命令

通过 USART1 (115200 8N1) 连接后可用的命令：

| 命令 | 说明 |
|------|------|
| `help` | 显示帮助 |
| `info` | 显示系统信息 |
| `uptime` | 显示运行时间 |
| `led <on\|off>` | 控制 LED |
| `echo <text>` | 回显文本 |
| `clear` | 清屏 |
| `reboot` | 重启系统 |

## 开发说明

### 添加共享类型

在 `protocol/src/` 中定义的类型可以同时被固件和上位机使用：

```rust
// protocol/src/lib.rs
pub const MY_CONSTANT: u8 = 0x42;

// firmware 中使用
use protocol::MY_CONSTANT;

// host 中使用
use protocol::MY_CONSTANT;
```

### 编译配置

| Profile | 优化 | 调试信息 | 用途 |
|---------|------|----------|------|
| `dev` | 无 | 完整 | 开发调试 |
| `release` | 最大 (z) | 有 | 最终发布 |
| `release-debuggable` | 中等 (1) | 完整 | 调试优化代码 |

### 注意事项

1. **在 firmware 目录编译固件**：子目录有默认 ARM target 配置
2. **在根目录编译上位机**：使用 `cargo host` 或 `cargo hostrun`
3. **不要在根目录直接 `cargo build`**：会尝试编译所有包，固件会因为 target 错误而失败

## 故障排除

### 编译报错 `invalid instruction mnemonic 'sev'`

原因：在错误的目录编译固件，没有使用 ARM target。

解决：
```powershell
cd firmware
cargo build
```

### 链接报错 `region 'FLASH' already defined`

原因：`memory.x` 被包含了两次。

解决：确保只有 `firmware/memory.x` 存在，根目录不要有 `memory.x`。

### probe-rs 找不到设备

1. 检查调试器连接
2. 安装驱动（WinUSB/libusb）
3. 尝试：`probe-rs list`

### USB HID 设备不识别

1. 检查 USB 线连接
2. 设备管理器查看是否有未知设备
3. 可能需要用 Zadig 安装 WinUSB 驱动
