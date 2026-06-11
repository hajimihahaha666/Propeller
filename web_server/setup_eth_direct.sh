#!/bin/bash
# 配置树莓派 eth0 网线直连固定 IP（仅需执行一次，需要 sudo）
#
# 树莓派 eth0 固定 IP: 192.168.50.1
# 笔记本电脑 eth   固定 IP: 192.168.50.2
# 网页地址: http://192.168.50.1:8080

set -e

PI_IP="192.168.50.1"
PI_PREFIX=24
CONN_UUID="626dd384-8b3d-3690-9511-192b2c79b3fd"
CONN_NAME="eth-direct"

if [ "$(id -u)" -ne 0 ]; then
    echo "请使用 sudo 运行: sudo bash $0"
    exit 1
fi

echo ">>> 配置 eth0 固定 IP: ${PI_IP}/${PI_PREFIX}"

nmcli connection modify "${CONN_UUID}" \
    connection.id "${CONN_NAME}" \
    connection.interface-name eth0 \
    ipv4.method manual \
    ipv4.addresses "${PI_IP}/${PI_PREFIX}" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv4.never-default yes \
    ipv6.method disabled \
    connection.autoconnect yes \
    connection.autoconnect-priority 100

echo ">>> 写入 netplan 持久化配置..."
cat > /etc/netplan/99-eth-direct.yaml << EOF
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - ${PI_IP}/${PI_PREFIX}
      optional: true
EOF
netplan apply

echo ">>> 激活有线连接..."
nmcli connection up "${CONN_NAME}" 2>/dev/null || true

echo ""
echo "=========================================="
echo "  网线直连配置完成"
echo "=========================================="
echo "  树莓派 eth0 固定 IP : ${PI_IP}"
echo "  笔记本 eth   固定 IP : 192.168.50.2"
echo "  子网掩码             : 255.255.255.0"
echo "  网关/DNS             : 留空"
echo ""
echo "  网页控制地址         : http://${PI_IP}:8080"
echo "=========================================="
echo ""
ip -br addr show eth0 2>/dev/null || true
echo ""
echo "【笔记本设置】插上网线后，把电脑有线网卡设为："
echo "  IP: 192.168.50.2  掩码: 255.255.255.0  网关: 无"
echo "  然后浏览器打开 http://${PI_IP}:8080"
echo ""
echo "  Windows(管理员 PowerShell) 示例："
echo "  Get-NetAdapter | Format-Table Name,Status,InterfaceDescription"
echo "  New-NetIPAddress -InterfaceAlias \"以太网\" -IPAddress 192.168.50.2 -PrefixLength 24"
echo ""
