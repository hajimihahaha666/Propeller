#!/bin/bash
set -euo pipefail
BASE="http://127.0.0.1:8080"
stop() {
  curl -s -m1 -X POST -H "Content-Type: application/json" -d '{"surge":0}' "$BASE/api/control" >/dev/null || true
}
trap stop EXIT

for _ in $(seq 1 80); do
  curl -s -m1 -X POST -H "Content-Type: application/json" \
    -d '{"surge":1.0,"speed_mode":"fast"}' "$BASE/api/control" >/dev/null &
  sleep 0.02
done
wait
sleep 0.3
curl -s "$BASE/api/status" | python3 -c "import sys,json;d=json.load(sys.stdin);print('channels',d['last_spi_channels'],'surge',d['thruster_cmd_surge'])"
echo "200655" | sudo -S timeout 12 openocd \
  -f /usr/share/openocd/scripts/interface/stlink.cfg \
  -f /usr/share/openocd/scripts/target/stm32f1x.cfg \
  -c "init" -c "mdh 0x20000112 8" -c "mdw 0x40000834 4" -c "shutdown" 2>&1 | grep -E "0x20000112|0x40000834"
