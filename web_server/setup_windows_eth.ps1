# 一键配置 Windows 有线网卡 IP（网线直连树莓派）
# 用法：右键此文件 -> 使用 PowerShell 运行
# 或在管理员 PowerShell 中：powershell -ExecutionPolicy Bypass -File setup_windows_eth.ps1

$PiIP   = "192.168.50.1"
$WinIP  = "192.168.50.2"
$Prefix = 24

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  树莓派网线直连 - Windows IP 自动配置" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  树莓派 IP : $PiIP"
Write-Host "  本机 IP   : $WinIP"
Write-Host "  控制网页  : http://${PiIP}:8080"
Write-Host ""

# 需要管理员权限
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "[!] 需要管理员权限，正在提权..." -ForegroundColor Yellow
    Start-Process powershell.exe -Verb RunAs -ArgumentList "-ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

# 自动查找已连接的有线网卡（排除 Wi-Fi / 蓝牙 / 虚拟网卡）
$adapters = Get-NetAdapter | Where-Object {
    $_.Status -eq 'Up' -and
    $_.InterfaceDescription -notmatch 'Wi-Fi|Wireless|WLAN|Bluetooth|Virtual|Hyper-V|VPN|TAP|TUN|Loopback'
}

if (-not $adapters) {
    Write-Host "[错误] 未找到已连接的有线网卡。请确认：" -ForegroundColor Red
    Write-Host "  1. 网线已插好（树莓派 <-> 电脑）"
    Write-Host "  2. 网卡驱动已安装（USB 网卡需先装好驱动）"
    Read-Host "按回车键退出"
    exit 1
}

if ($adapters.Count -gt 1) {
    Write-Host "检测到多个有线网卡，请选择：" -ForegroundColor Yellow
    for ($i = 0; $i -lt $adapters.Count; $i++) {
        Write-Host "  [$i] $($adapters[$i].Name) - $($adapters[$i].InterfaceDescription)"
    }
    $choice = Read-Host "输入编号 (默认 0)"
    if ([string]::IsNullOrWhiteSpace($choice)) { $choice = 0 }
    $adapter = $adapters[[int]$choice]
} else {
    $adapter = $adapters[0]
}

$alias = $adapter.Name
Write-Host ""
Write-Host ">>> 配置网卡: $alias ($($adapter.InterfaceDescription))" -ForegroundColor Green

try {
    # 清除旧 IP，避免冲突
    Get-NetIPAddress -InterfaceAlias $alias -ErrorAction SilentlyContinue | Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
    Get-NetRoute -InterfaceAlias $alias -ErrorAction SilentlyContinue | Remove-NetRoute -Confirm:$false -ErrorAction SilentlyContinue

    New-NetIPAddress -InterfaceAlias $alias -IPAddress $WinIP -PrefixLength $Prefix -ErrorAction Stop | Out-Null
    Write-Host "[OK] IP 已设置为 $WinIP/$Prefix" -ForegroundColor Green
} catch {
    Write-Host "[错误] 设置 IP 失败: $_" -ForegroundColor Red
    Read-Host "按回车键退出"
    exit 1
}

Write-Host ""
Write-Host ">>> 测试连接树莓派..." -ForegroundColor Cyan
$ping = Test-Connection -ComputerName $PiIP -Count 3 -Quiet
if ($ping) {
    Write-Host "[OK] 树莓派 $PiIP 连接成功！" -ForegroundColor Green
    Write-Host ""
    Write-Host "请在浏览器打开: http://${PiIP}:8080" -ForegroundColor Yellow
    Start-Process "http://${PiIP}:8080"
} else {
    Write-Host "[警告] ping 不通 $PiIP，可能原因：" -ForegroundColor Yellow
    Write-Host "  - 树莓派未开机"
    Write-Host "  - 网线未插好"
    Write-Host "  - 树莓派 eth0 未配置（需在树莓派运行 setup_eth_direct.sh）"
}

Write-Host ""
Read-Host "按回车键退出"
