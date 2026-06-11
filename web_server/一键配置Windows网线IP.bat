@echo off
chcp 65001 >nul
echo 正在以管理员身份配置有线网卡 IP...
powershell -ExecutionPolicy Bypass -File "%~dp0setup_windows_eth.ps1"
pause
