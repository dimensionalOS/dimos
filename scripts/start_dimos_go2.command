#!/bin/zsh

# Double-click launcher for the real Unitree Go2.
# Opens the native DimOS app. The app owns runtime startup so a second hidden
# DimOS/MCP instance cannot steal its control ports.

set -u

SCRIPT_DIR="${0:A:h}"
PROJECT_DIR="${SCRIPT_DIR:h}"
DIMOS_BIN="${PROJECT_DIR}/.venv/bin/dimos"
NATIVE_APP="${PROJECT_DIR}/apps/DimOS Native.app"
ROBOT_IP="${DIMOS_GO2_IP:-192.168.12.1}"
LOG_DIR="${PROJECT_DIR}/logs"
LOG_FILE="${LOG_DIR}/dimos-go2-native.log"

function fail() {
  echo
  echo "启动失败：$1"
  echo "日志：${LOG_FILE}"
  echo
  read -r "?按回车关闭窗口…"
  exit 1
}

echo "DimOS Go2 一键启动"
echo "=================="
echo "机器狗：${ROBOT_IP}"
echo "模式：由 DimOS Native 统一管理"
echo

[[ -x "${DIMOS_BIN}" ]] || fail "找不到 ${DIMOS_BIN}"
[[ -d "${NATIVE_APP}" ]] || fail "找不到 ${NATIVE_APP}"

mkdir -p "${LOG_DIR}"

echo "1/2 检查机器狗网络…"
if ! nc -z -G 2 -w 2 "${ROBOT_IP}" 9991 >/dev/null 2>&1; then
  echo "${ROBOT_IP}:9991 暂不可达，正在自动发现同一局域网中的 Go2…"
  DISCOVERY_OUTPUT="$("${DIMOS_BIN}" go2tool discover --lan --timeout 8 2>/dev/null || true)"
  DISCOVERED_IP="$(
    echo "${DISCOVERY_OUTPUT}" |
      awk '$1 == "LAN" && $3 ~ /^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$/ { print $3; exit }'
  )"
  if [[ -z "${DISCOVERED_IP}" ]] ||
    ! nc -z -G 2 -w 2 "${DISCOVERED_IP}" 9991 >/dev/null 2>&1; then
    fail "没有发现可连接的 Go2。请先开机，并让电脑连到机器狗所在网络。"
  fi
  ROBOT_IP="${DISCOVERED_IP}"
  echo "自动发现 Go2：${ROBOT_IP}"
fi

export NO_PROXY="${NO_PROXY:+${NO_PROXY},}localhost,127.0.0.1,${ROBOT_IP}"
export no_proxy="${NO_PROXY}"
echo "网络正常。"

echo "2/2 打开 DimOS 原生 App…"
export DIMOS_PROJECT_ROOT="${PROJECT_DIR}"
open "${NATIVE_APP}"

echo
echo "DimOS Native 已打开。"
echo "只查看地图：点击「只读连接（不能遥控）」。"
echo "需要键盘遥控：直接点击「连接并启用遥控」。"
echo "原生 Viewer 打开后，先点击 Viewer 画面，再用 WASD。"
echo
exit 0
