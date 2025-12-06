#!/usr/bin/env python3
"""
STM32H743 USB HID 设备检测和通信工具

支持两种模式:
1. HID 模式 (Windows 原生驱动) - 使用 hidapi
2. WinUSB 模式 (Zadig 安装驱动后) - 使用 pyusb/libusb

使用方法:
    pip install hidapi pyusb libusb
    python usb_hid_test.py
"""

import sys
import time
import struct

# USB 设备 VID/PID (与固件中配置一致)
VENDOR_ID = 0x1209   # pid.codes test VID
PRODUCT_ID = 0x0001  # 自定义 PID

# HID 端点地址 (根据 USB 描述符)
EP_IN = 0x81   # IN 端点 (设备 -> 主机)
EP_OUT = 0x01  # OUT 端点 (主机 -> 设备)

# 命令定义
CMD_PING = 0x01
CMD_GET_STATUS = 0x02
CMD_LED_ON = 0x03
CMD_LED_OFF = 0x04


def try_import_hidapi():
    """尝试导入 hidapi"""
    try:
        import hid
        return hid
    except ImportError:
        return None


def try_import_pyusb():
    """尝试导入 pyusb"""
    try:
        import usb.core
        import usb.util
        # 尝试加载 libusb 后端
        try:
            import libusb_package
            import usb.backend.libusb1 as libusb1
            backend = libusb1.get_backend(find_library=libusb_package.find_library)
            if backend:
                usb.core._backend = backend
                print("  [DEBUG] 使用 libusb-package 后端")
        except ImportError:
            pass
        return usb
    except ImportError:
        return None


# ==================== PyUSB (WinUSB/libusb) 模式 ====================

class PyUSBDevice:
    """使用 PyUSB 访问 USB 设备 (WinUSB 驱动)"""
    
    def __init__(self, dev):
        self.dev = dev
        self.ep_in = EP_IN
        self.ep_out = EP_OUT
        self.ep_in_obj = None
        self.ep_out_obj = None
        
    @staticmethod
    def find():
        """查找设备"""
        usb = try_import_pyusb()
        if not usb:
            return None
        
        # 尝试使用 libusb-package 后端
        backend = None
        try:
            import libusb_package
            import usb.backend.libusb1 as libusb1
            backend = libusb1.get_backend(find_library=libusb_package.find_library)
        except ImportError:
            pass
            
        dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=backend)
        if dev is None:
            return None
        return PyUSBDevice(dev)
    
    @staticmethod
    def list_all():
        """列出所有 USB 设备"""
        usb = try_import_pyusb()
        if not usb:
            print("pyusb 未安装，无法列出 USB 设备")
            return
        
        # 尝试使用 libusb-package 后端
        backend = None
        try:
            import libusb_package
            import usb.backend.libusb1 as libusb1
            backend = libusb1.get_backend(find_library=libusb_package.find_library)
            print("  [使用 libusb-package 后端]")
        except ImportError:
            print("  [使用默认后端]")
            
        print("\n===== 所有 USB 设备列表 (libusb) =====")
        devices = list(usb.core.find(find_all=True, backend=backend))
        
        if not devices:
            print("未检测到任何 USB 设备")
            return
            
        for i, dev in enumerate(devices):
            try:
                manufacturer = dev.manufacturer or "N/A"
            except:
                manufacturer = "N/A"
            try:
                product = dev.product or "N/A"
            except:
                product = "N/A"
                
            print(f"\n设备 #{i + 1}:")
            print(f"  VID:PID      = {dev.idVendor:04X}:{dev.idProduct:04X}")
            print(f"  制造商       = {manufacturer}")
            print(f"  产品名       = {product}")
            
            # 高亮显示目标设备
            if dev.idVendor == VENDOR_ID and dev.idProduct == PRODUCT_ID:
                print(f"  ★★★ 这是目标设备! ★★★")
    
    def open(self):
        """打开设备"""
        try:
            # 如果有内核驱动，先分离
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except:
            pass
            
        # 设置配置
        try:
            self.dev.set_configuration()
        except:
            pass
            
        # 获取接口
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, 0)]
        
        # 查找端点
        import usb.util
        self.ep_in_obj = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        self.ep_out_obj = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        )
        
        if self.ep_in_obj:
            self.ep_in = self.ep_in_obj.bEndpointAddress
            print(f"  IN 端点: 0x{self.ep_in:02X}")
        if self.ep_out_obj:
            self.ep_out = self.ep_out_obj.bEndpointAddress
            print(f"  OUT 端点: 0x{self.ep_out:02X}")
            
        print(f"\n✅ 设备已打开 (PyUSB/WinUSB 模式)")
        try:
            print(f"  制造商: {self.dev.manufacturer}")
            print(f"  产品名: {self.dev.product}")
            print(f"  序列号: {self.dev.serial_number}")
        except:
            pass
        return True
        
    def write(self, data):
        """写入数据"""
        try:
            # 对于 HID，可能需要通过控制传输发送
            # 先尝试 OUT 端点
            if self.ep_out_obj:
                written = self.dev.write(self.ep_out, data, timeout=1000)
                return written
            else:
                # 使用控制传输 (SET_REPORT)
                # bmRequestType: 0x21 (Host to Device, Class, Interface)
                # bRequest: 0x09 (SET_REPORT)
                # wValue: 0x0200 (Report Type: Output, Report ID: 0)
                # wIndex: 0 (Interface)
                self.dev.ctrl_transfer(0x21, 0x09, 0x0200, 0, data, timeout=1000)
                return len(data)
        except Exception as e:
            print(f"写入错误: {e}")
            return 0
            
    def read(self, size=64, timeout=1000):
        """读取数据"""
        try:
            if self.ep_in_obj:
                data = self.dev.read(self.ep_in, size, timeout=timeout)
                return bytes(data)
            else:
                # 使用控制传输 (GET_REPORT)
                data = self.dev.ctrl_transfer(0xA1, 0x01, 0x0100, 0, size, timeout=timeout)
                return bytes(data)
        except Exception as e:
            # 超时是正常的，不打印
            if "timeout" in str(e).lower() or "10060" in str(e):
                return None
            print(f"读取错误: {e}")
            return None
            
    def close(self):
        """关闭设备"""
        try:
            import usb.util
            usb.util.dispose_resources(self.dev)
        except:
            pass


# ==================== HID API 模式 ====================

class HIDAPIDevice:
    """使用 hidapi 访问 HID 设备 (Windows 原生驱动)"""
    
    def __init__(self):
        self.dev = None
        self.hid = try_import_hidapi()
        
    @staticmethod
    def find():
        """查找设备"""
        hid = try_import_hidapi()
        if not hid:
            return None
            
        devices = hid.enumerate(VENDOR_ID, PRODUCT_ID)
        if not devices:
            return None
        return HIDAPIDevice()
    
    @staticmethod
    def list_all():
        """列出所有 HID 设备"""
        hid = try_import_hidapi()
        if not hid:
            print("hidapi 未安装，无法列出 HID 设备")
            return
            
        print("\n===== 所有 HID 设备列表 =====")
        devices = hid.enumerate()
        
        if not devices:
            print("未检测到任何 HID 设备")
            return
        
        for i, dev in enumerate(devices):
            print(f"\n设备 #{i + 1}:")
            print(f"  VID:PID      = {dev['vendor_id']:04X}:{dev['product_id']:04X}")
            print(f"  制造商       = {dev.get('manufacturer_string', 'N/A')}")
            print(f"  产品名       = {dev.get('product_string', 'N/A')}")
            print(f"  序列号       = {dev.get('serial_number', 'N/A')}")
            
            if dev['vendor_id'] == VENDOR_ID and dev['product_id'] == PRODUCT_ID:
                print(f"  ★★★ 这是目标设备! ★★★")
        
    def open(self):
        """打开设备"""
        try:
            self.dev = self.hid.device()
            self.dev.open(VENDOR_ID, PRODUCT_ID)
            self.dev.set_nonblocking(True)
            
            print(f"\n✅ 设备已打开 (HIDAPI 模式)")
            print(f"  制造商: {self.dev.get_manufacturer_string()}")
            print(f"  产品名: {self.dev.get_product_string()}")
            print(f"  序列号: {self.dev.get_serial_number_string()}")
            return True
        except Exception as e:
            print(f"❌ 无法打开设备: {e}")
            return False
            
    def write(self, data):
        """写入数据"""
        try:
            # HID 报告需要加上 Report ID (0x00)
            report = bytes([0x00]) + bytes(data)
            return self.dev.write(report)
        except Exception as e:
            print(f"写入错误: {e}")
            return 0
            
    def read(self, size=64, timeout=1000):
        """读取数据"""
        try:
            data = self.dev.read(size, timeout)
            if data:
                return bytes(data)
            return None
        except Exception as e:
            print(f"读取错误: {e}")
            return None
            
    def close(self):
        """关闭设备"""
        if self.dev:
            self.dev.close()


# ==================== 通用函数 ====================

def find_device():
    """查找设备，自动选择合适的驱动"""
    print(f"\n===== 搜索 STM32H743 设备 (VID={VENDOR_ID:04X}, PID={PRODUCT_ID:04X}) =====")
    
    # 先尝试 PyUSB (WinUSB)
    dev = PyUSBDevice.find()
    if dev:
        print("✅ 通过 PyUSB/WinUSB 找到设备")
        return dev
        
    # 再尝试 HIDAPI
    dev = HIDAPIDevice.find()
    if dev:
        print("✅ 通过 HIDAPI 找到设备")
        return dev
        
    print("❌ 未找到 STM32H743 USB 设备")
    return None


def send_command(dev, cmd, data=None):
    """发送命令到设备"""
    report = [0x00] * 64
    report[0] = cmd
    
    if data:
        for i, b in enumerate(data[:63]):
            report[i + 1] = b
    
    try:
        written = dev.write(bytes(report))
        print(f"📤 发送命令 0x{cmd:02X}, 写入 {written} 字节")
        return True
    except Exception as e:
        print(f"❌ 发送失败: {e}")
        return False


def receive_data(dev, timeout_ms=1000):
    """接收设备数据"""
    data = dev.read(64, timeout_ms)
    if data:
        print(f"📥 收到 {len(data)} 字节:")
        # 解析头部
        if len(data) >= 10 and data[0] == 0xAA and data[1] == 0x55:
            counter = struct.unpack('<I', bytes(data[2:6]))[0]
            ident = bytes(data[6:10]).decode('utf-8', errors='ignore')
            print(f"  标识: 0x{data[0]:02X} 0x{data[1]:02X}")
            print(f"  计数: {counter}")
            print(f"  芯片: {ident}")
        else:
            print(f"  原始数据: {' '.join(f'{b:02X}' for b in data[:16])}...")
        return data
    return None


def interactive_test(dev):
    """交互式测试"""
    print("\n===== 交互式测试 =====")
    print("命令:")
    print("  1 - 发送 PING")
    print("  2 - 获取状态")
    print("  3 - LED 开")
    print("  4 - LED 关")
    print("  r - 接收数据")
    print("  l - 持续监听")
    print("  q - 退出")
    
    while True:
        try:
            cmd = input("\n请输入命令: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == '1':
                send_command(dev, CMD_PING)
            elif cmd == '2':
                send_command(dev, CMD_GET_STATUS)
            elif cmd == '3':
                send_command(dev, CMD_LED_ON)
            elif cmd == '4':
                send_command(dev, CMD_LED_OFF)
            elif cmd == 'r':
                receive_data(dev, 2000)
            elif cmd == 'l':
                print("持续监听中... (Ctrl+C 停止)")
                try:
                    while True:
                        data = receive_data(dev, 100)
                        if not data:
                            time.sleep(0.1)
                except KeyboardInterrupt:
                    print("\n监听已停止")
            else:
                print("未知命令")
                
        except KeyboardInterrupt:
            break


def auto_test(dev, duration=10):
    """自动测试：持续接收数据"""
    print(f"\n===== 自动测试 ({duration}秒) =====")
    print("持续监听设备数据...")
    
    start_time = time.time()
    rx_count = 0
    
    while time.time() - start_time < duration:
        data = receive_data(dev, 500)
        if data:
            rx_count += 1
        else:
            print(".", end="", flush=True)
        time.sleep(0.1)
    
    print(f"\n\n测试完成: 接收 {rx_count} 个数据包")


def main():
    print("=" * 50)
    print("    STM32H743 USB HID 测试工具")
    print("    支持: HIDAPI (原生HID) / PyUSB (WinUSB)")
    print("=" * 50)
    
    # 检查可用的库
    hid = try_import_hidapi()
    usb = try_import_pyusb()
    
    print("\n可用库:")
    print(f"  hidapi: {'✅ 已安装' if hid else '❌ 未安装 (pip install hidapi)'}")
    print(f"  pyusb:  {'✅ 已安装' if usb else '❌ 未安装 (pip install pyusb)'}")
    
    if not hid and not usb:
        print("\n❌ 错误: 需要安装至少一个 USB 库")
        print("   pip install hidapi pyusb")
        return
    
    # 列出所有设备
    if usb:
        PyUSBDevice.list_all()
    if hid:
        HIDAPIDevice.list_all()
    
    # 查找目标设备
    dev = find_device()
    
    if not dev:
        print("\n提示:")
        print("  1. 检查设备是否连接")
        print("  2. 检查设备管理器中的设备状态")
        print("  3. 如果用了 Zadig，确保安装的是 WinUSB 驱动")
        print("  4. 尝试重新插拔设备")
        return
    
    # 打开设备
    if not dev.open():
        return
    
    try:
        # 选择测试模式
        print("\n选择测试模式:")
        print("  1 - 自动测试 (持续接收 10秒)")
        print("  2 - 交互式测试")
        print("  3 - 仅监听接收")
        
        choice = input("请选择 (1/2/3): ").strip()
        
        if choice == '1':
            auto_test(dev)
        elif choice == '2':
            interactive_test(dev)
        elif choice == '3':
            print("\n持续监听中... (Ctrl+C 停止)")
            try:
                while True:
                    data = receive_data(dev, 1000)
                    if not data:
                        print(".", end="", flush=True)
            except KeyboardInterrupt:
                print("\n监听已停止")
        else:
            print("使用默认: 自动测试")
            auto_test(dev)
            
    finally:
        dev.close()
        print("\n设备已关闭")


if __name__ == "__main__":
    main()
