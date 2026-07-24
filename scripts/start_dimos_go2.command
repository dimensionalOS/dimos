#!/bin/zsh

# Double-click launcher for the real Unitree Go2.
# Starts the native DimOS app, connects sensors, builds the lidar map, and
# exposes the first-party DimOS MCP server. Physical motion stays locked.

set -u

SCRIPT_DIR="${0:A:h}"
PROJECT_DIR="${SCRIPT_DIR:h}"
DIMOS_BIN="${PROJECT_DIR}/.venv/bin/dimos"
NATIVE_APP="${PROJECT_DIR}/apps/DimOS Native.app"
ROBOT_IP="${DIMOS_GO2_IP:-192.168.12.1}"
MCP_URL="http://127.0.0.1:9990/mcp"
LOG_DIR="${PROJECT_DIR}/logs"
LOG_FILE="${LOG_DIR}/dimos-go2-native.log"
PID_FILE="${LOG_DIR}/dimos-go2-native.pid"

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
echo "模式：真实硬件只读连接（行走已锁定）"
echo "将启动：原生 Viewer、雷达建图、官方 MCP"
echo

[[ -x "${DIMOS_BIN}" ]] || fail "找不到 ${DIMOS_BIN}"
[[ -d "${NATIVE_APP}" ]] || fail "找不到 ${NATIVE_APP}"

mkdir -p "${LOG_DIR}"

if "${DIMOS_BIN}" status 2>/dev/null | grep -q "Run ID:"; then
  echo "检测到 DimOS 已在运行，不重复启动。"
  open "${NATIVE_APP}"
  echo "MCP：${MCP_URL}"
  exit 0
fi

echo "1/4 检查机器狗网络…"
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

echo "2/4 打开 DimOS 原生 App…"
open "${NATIVE_APP}"

echo "3/4 连接机器狗并启动建图、原生 Viewer、MCP…"
nohup "${DIMOS_BIN}" \
  --robot-ip "${ROBOT_IP}" \
  --viewer rerun \
  --rerun-open native \
  --no-rerun-web \
  --obstacle-avoidance \
  run dimos-go2-studio.go2 \
  --option go2connection.movement_enabled=false \
  --option go2connection.auto_stand=false \
  >>"${LOG_FILE}" 2>&1 &

DIMOS_PID=$!
echo "${DIMOS_PID}" >"${PID_FILE}"

echo "4/4 等待 MCP 工具注册…"
MCP_READY=0
for _ in {1..180}; do
  if ! kill -0 "${DIMOS_PID}" 2>/dev/null; then
    tail -n 30 "${LOG_FILE}"
    fail "DimOS 进程提前退出。"
  fi
  if "${DIMOS_BIN}" mcp list-tools 2>/dev/null | grep -q '"studio_ready"'; then
    MCP_READY=1
    break
  fi
  sleep 0.5
done

if [[ "${MCP_READY}" -ne 1 ]]; then
  fail "90 秒内未等到 MCP 准备完成。"
fi

echo
echo "启动成功。"
echo "原生地图/相机：dimos-viewer"
echo "官方 MCP：${MCP_URL}"
echo "机器狗已连接，雷达建图已开始，行走仍处于锁定状态。"
echo "停止方式：在 DimOS Native 中点击「停止 DimOS」。"
echo
exit 0
