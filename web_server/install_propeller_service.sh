#!/bin/bash
# 安装并启用 propeller 用户服务（开机自启，无需登录、无需外网）
set -e

UNIT_SRC="${HOME}/imu_ws/web_server/propeller.service"
UNIT_DST="${HOME}/.config/systemd/user/propeller.service"

mkdir -p "${HOME}/.config/systemd/user"
cp "${UNIT_SRC}" "${UNIT_DST}"

chmod +x "${HOME}/imu_ws/web_server/start_propeller_daemon.sh"

systemctl --user daemon-reload
systemctl --user enable propeller.service
systemctl --user restart propeller.service

echo ""
echo "=========================================="
echo "  propeller.service 已安装并启动"
echo "=========================================="
systemctl --user --no-pager status propeller.service || true
echo ""
echo "Linger: $(loginctl show-user "${USER}" -p Linger --value)"
if [ "$(loginctl show-user "${USER}" -p Linger --value)" != "yes" ]; then
    echo ""
    echo "[!] 需要开启 Linger 才能在未登录时自启，请执行："
    echo "    sudo loginctl enable-linger ${USER}"
fi
echo ""
echo "树莓派 eth0 : 192.168.50.1"
echo "电脑 eth    : 192.168.50.2"
echo "控制网页    : http://192.168.50.1:8080"
echo "=========================================="
