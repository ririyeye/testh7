@echo off
REM 下载并自动复位运行
pyocd load --target stm32h743xx --format elf %1 && pyocd cmd -t stm32h743xx -c "reset"
