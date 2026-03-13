#!/bin/bash
# Container entrypoint for the DimOS navigation DockerModule.
# Sets up the ROS environment, starts background navigation services based on
# ENV vars, then hands off to the DimOS docker_runner which instantiates the
# actual Python module (ROSNav) via the --payload argument.
#
# This script is mounted at /usr/local/bin/dimos_module_entrypoint.sh so that
# entrypoint changes never require a container rebuild.
#
# Environment variables:
#   MODE                  simulation (default) | unity_sim | hardware | bagfile
#                         Falls back to HARDWARE_MODE=true → hardware
#                         unity_sim: same as simulation but also starts the Unity exe;
#                         crashes at startup if the Unity binary is missing.
#   BAGFILE_PATH          Path inside the container to a ROS 2 bag directory/
#                         mcap file.  When set, the bag is played automatically
#                         with --clock after the nav stack initialises.
#                         MODE should be set to "bagfile" when using this.
#   USE_ROUTE_PLANNER     Use the FAR route-planner launch file
#                         (default: true for simulation, false for hardware/bagfile)
#   USE_RVIZ              Launch RViz2 when a display socket is present (default: false)
#   LOCALIZATION_METHOD   SLAM backend: arise_slam (default) | fastlio
#   ROS_DISTRO            ROS distribution (default: humble)
#   HARDWARE_MODE         Legacy: "true" → hardware mode. Prefer MODE instead.

# NOTE: no set -e — individual step failures must not abort the whole entrypoint.

# ── X11 diagnostics ───────────────────────────────────────────────────────
# Usage (from host):
#   docker exec <container> bash -c 'source /usr/local/bin/dimos_module_entrypoint.sh; why_is_x11_not_working'
# Or inside the container just run:
#   why_is_x11_not_working
why_is_x11_not_working() {
    local ok=true
    echo "=== X11 Doctor ==="

    # 1. DISPLAY
    echo ""
    echo "--- DISPLAY ---"
    if [ -z "${DISPLAY:-}" ]; then
        echo "  FAIL  DISPLAY is not set"
        ok=false
    else
        echo "  OK    DISPLAY=${DISPLAY}"
    fi

    # 2. X11 unix socket directory
    echo ""
    echo "--- /tmp/.X11-unix socket directory ---"
    if [ ! -d /tmp/.X11-unix ]; then
        echo "  FAIL  /tmp/.X11-unix does not exist (volume not mounted?)"
        ok=false
    else
        local sockets
        sockets=$(ls /tmp/.X11-unix 2>/dev/null)
        if [ -z "$sockets" ]; then
            echo "  WARN  /tmp/.X11-unix exists but is empty (no display sockets)"
            ok=false
        else
            echo "  OK    /tmp/.X11-unix contents: $sockets"
        fi
    fi

    # 3. Socket for the specific DISPLAY
    echo ""
    echo "--- Display socket for DISPLAY=${DISPLAY:-<unset>} ---"
    if [ -n "${DISPLAY:-}" ]; then
        local display_num
        display_num=$(echo "${DISPLAY}" | sed 's/.*:\([0-9]*\).*/\1/')
        local sock="/tmp/.X11-unix/X${display_num}"
        if [ -S "$sock" ]; then
            echo "  OK    $sock exists and is a socket"
            ls -la "$sock"
        else
            echo "  FAIL  $sock not found or not a socket"
            ok=false
        fi
    else
        echo "  SKIP  (DISPLAY not set)"
    fi

    # 4. XAUTHORITY file
    echo ""
    echo "--- XAUTHORITY ---"
    local xauth_file="${XAUTHORITY:-$HOME/.Xauthority}"
    if [ -z "${XAUTHORITY:-}" ]; then
        echo "  WARN  XAUTHORITY env var not set; defaulting to $xauth_file"
    else
        echo "  OK    XAUTHORITY=${XAUTHORITY}"
    fi
    if [ -f "$xauth_file" ]; then
        echo "  OK    $xauth_file exists ($(wc -c < "$xauth_file") bytes)"
        ls -la "$xauth_file"
    else
        echo "  FAIL  $xauth_file not found (Xauthority not mounted?)"
        ok=false
    fi

    # 5. xauth cookie list
    echo ""
    echo "--- xauth cookie entries ---"
    if command -v xauth >/dev/null 2>&1; then
        local cookie_out
        cookie_out=$(XAUTHORITY="$xauth_file" xauth list 2>&1)
        if [ -z "$cookie_out" ]; then
            echo "  WARN  xauth list returned no entries (cookie file empty or wrong display)"
            ok=false
        else
            echo "  OK    cookies found:"
            echo "$cookie_out" | sed 's/^/        /'
        fi
    else
        echo "  WARN  xauth not installed; cannot check cookies"
    fi

    # 6. Live connection test
    echo ""
    echo "--- Live connection test ---"
    if command -v xdpyinfo >/dev/null 2>&1; then
        if DISPLAY="${DISPLAY:-:0}" XAUTHORITY="$xauth_file" xdpyinfo >/dev/null 2>&1; then
            echo "  OK    xdpyinfo connected to ${DISPLAY:-:0} successfully"
        else
            echo "  FAIL  xdpyinfo could not connect to ${DISPLAY:-:0}"
            DISPLAY="${DISPLAY:-:0}" XAUTHORITY="$xauth_file" xdpyinfo 2>&1 | head -5 | sed 's/^/        /'
            ok=false
        fi
    elif command -v xclock >/dev/null 2>&1; then
        if DISPLAY="${DISPLAY:-:0}" XAUTHORITY="$xauth_file" xclock -display "${DISPLAY:-:0}" &
           sleep 1 && kill %1 2>/dev/null; then
            echo "  OK    xclock launched on ${DISPLAY:-:0}"
        else
            echo "  FAIL  xclock could not connect"
            ok=false
        fi
    else
        echo "  SKIP  neither xdpyinfo nor xclock installed; skipping live test"
        echo "        Install with: apt-get install -y x11-utils"
    fi

    # 7. Summary
    echo ""
    echo "=== Summary ==="
    if $ok; then
        echo "  All checks passed — X11 should work."
    else
        echo "  One or more checks failed."
        echo ""
        echo "  Common fixes:"
        echo "    • Mount the socket:   -v /tmp/.X11-unix:/tmp/.X11-unix"
        echo "    • Mount the cookie:   -v \${XAUTHORITY:-\$HOME/.Xauthority}:/tmp/.Xauthority:ro"
        echo "    • Set env vars:       -e DISPLAY -e XAUTHORITY=/tmp/.Xauthority"
        echo "    • Allow local X:      xhost +local:  (run on host, less safe)"
    fi
    echo ""
}
echo '
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash
source /opt/dimos-venv/bin/activate
' > /usr/bin/upp
chmod +x /usr/bin/upp
echo '
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash
source /opt/dimos-venv/bin/activate
rosspy
' > /usr/bin/rosspy
chmod +x /usr/bin/rosspy

# ── Git safe directories ──────────────────────────────────────────────────
git config --global --add safe.directory /workspace/dimos 2>/dev/null || true
git config --global --add safe.directory /ros2_ws/src/ros-navigation-autonomy-stack 2>/dev/null || true

# ── Multicast (required for DimOS LCM pubsub) ─────────────────────────────
# MulticastConfiguratorLinux.critical=True → SystemExit(1) if not configured
echo "[entrypoint] Configuring multicast for LCM..."
ip link set lo multicast on 2>/dev/null \
    || echo "[entrypoint] NOTE: multicast already enabled on lo"
ip route add 224.0.0.0/4 dev lo 2>/dev/null \
    || echo "[entrypoint] NOTE: multicast route already present"

# ── Source ROS ────────────────────────────────────────────────────────────
echo "[entrypoint] Sourcing ROS ${ROS_DISTRO:-humble}..."
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash

# ── Python venv ───────────────────────────────────────────────────────────
source /opt/dimos-venv/bin/activate

# ── Install dimos from live source ────────────────────────────────────────
# Keeps the container in sync with host edits without a rebuild.
if [ -d "/workspace/dimos" ]; then
    echo "[entrypoint] pip install -e /workspace/dimos..."
    pip install -e /workspace/dimos >/tmp/dimos_pip_install.log 2>&1 || {
        echo "[entrypoint] WARNING: pip install failed — see /tmp/dimos_pip_install.log"
    }
fi

# ── DDS / FastDDS config ──────────────────────────────────────────────────
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml
if [ -f "/ros2_ws/config/custom_fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/custom_fastdds.xml
fi

# ── Determine operating mode ──────────────────────────────────────────────
# Priority: MODE env var > HARDWARE_MODE legacy var > default simulation
if [ -z "${MODE:-}" ]; then
    if [ "${HARDWARE_MODE:-false}" = "true" ]; then
        MODE="hardware"
    else
        MODE="unity_sim"
    fi
fi
export MODE
echo "[entrypoint] MODE=${MODE}"

# ── Optionally launch Unity vehicle_simulator exe ─────────────────────────
# simulation  — try Unity; silently skip if binary absent (mirrors run_both.sh)
# unity_sim   — same but hard-exit if binary absent (useful for CI / explicit test)
UNITY_EXECUTABLE="/ros2_ws/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
if [ "$MODE" = "unity_sim" ] || [ "$MODE" = "simulation" ]; then
    if [ -f "$UNITY_EXECUTABLE" ]; then
        echo "[entrypoint] Starting Unity: $UNITY_EXECUTABLE"
        "$UNITY_EXECUTABLE" &
    else
        echo "[entrypoint] ERROR: Unity executable not found at $UNITY_EXECUTABLE"
        echo "[entrypoint] Build or download the Unity environment and place it at that path."
        exit 1
    fi
else
    echo "[entrypoint] not running Unity simulator"
fi

# ── Select and launch ROS navigation stack ────────────────────────────────
# Always started. Launch file is chosen from MODE and USE_ROUTE_PLANNER.
# Simulation defaults USE_ROUTE_PLANNER=true (mirrors run_both.sh behaviour).
# Hardware/bagfile default to false; user opts in with USE_ROUTE_PLANNER=true.
LOCALIZATION_METHOD="${LOCALIZATION_METHOD:-arise_slam}"

case "$MODE" in
    hardware)
        if [ "${USE_ROUTE_PLANNER:-false}" = "true" ]; then
            LAUNCH_FILE="system_real_robot_with_route_planner.launch.py"
        else
            LAUNCH_FILE="system_real_robot.launch.py"
        fi
        ;;
    bagfile)
        if [ "${USE_ROUTE_PLANNER:-false}" = "true" ]; then
            LAUNCH_FILE="system_bagfile_with_route_planner.launch.py"
        else
            LAUNCH_FILE="system_bagfile.launch.py"
        fi
        ;;
    simulation|unity_sim|*)  # simulation / unity_sim / unrecognised → simulation launch
        if [ "${USE_ROUTE_PLANNER:-true}" = "true" ]; then
            LAUNCH_FILE="system_simulation_with_route_planner.launch.py"
        else
            LAUNCH_FILE="system_simulation.launch.py"
        fi
        ;;
esac

# Build extra launch arguments
LAUNCH_ARGS=""
[ "$LOCALIZATION_METHOD" = "fastlio" ] && LAUNCH_ARGS="use_fastlio2:=true"
# Disable foxglove bridge in non-hardware modes (started separately for
# hardware; unnecessary in simulation/bagfile and wastes resources)
[ "$MODE" != "hardware" ] && LAUNCH_ARGS="${LAUNCH_ARGS} enable_bridge:=false"

echo "[entrypoint] Launching: ros2 launch vehicle_simulator ${LAUNCH_FILE} ${LAUNCH_ARGS}"
cd /ros2_ws/src/ros-navigation-autonomy-stack
setsid bash -c "
    source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 launch vehicle_simulator ${LAUNCH_FILE} ${LAUNCH_ARGS}
" &
echo "[entrypoint] ROS nav stack started (PID: $!), waiting for init (5s)..."
sleep 5

# ── Optionally play a bag file ─────────────────────────────────────────────
# Set BAGFILE_PATH to a bag directory or .mcap file inside the container.
# The nav stack should be MODE=bagfile so use_sim_time is enabled.
if [ -n "${BAGFILE_PATH:-}" ]; then
    if [ ! -e "$BAGFILE_PATH" ]; then
        echo "[entrypoint] ERROR: BAGFILE_PATH set but not found: $BAGFILE_PATH"
        exit 1
    fi
    if [ "$MODE" != "bagfile" ]; then
        echo "[entrypoint] WARNING: BAGFILE_PATH is set but MODE=${MODE} (expected bagfile)."
        echo "[entrypoint]          use_sim_time may not be enabled in the nav stack."
    fi
    echo "[entrypoint] Playing bag: ros2 bag play --clock $BAGFILE_PATH"
    ros2 bag play "$BAGFILE_PATH" --clock &
fi

# ── Optionally launch RViz2 ───────────────────────────────────────────────
if [ "${USE_RVIZ:-false}" = "true" ]; then
    DISPLAY_SOCK="/tmp/.X11-unix/X$(echo "${DISPLAY:-:0}" | tr -dc '0-9')"
    if [ -S "$DISPLAY_SOCK" ]; then
        echo "[entrypoint] Starting RViz2..."
        if [ "${USE_ROUTE_PLANNER:-true}" = "true" ]; then
            RVIZ_CFG="/ros2_ws/src/ros-navigation-autonomy-stack/src/route_planner/far_planner/rviz/default.rviz"
        else
            RVIZ_CFG="/ros2_ws/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz"
        fi
        ros2 run rviz2 rviz2 -d "$RVIZ_CFG" &
    else
        echo "[entrypoint] No display socket at $DISPLAY_SOCK — skipping RViz"
    fi
fi

# ── Twist relay ───────────────────────────────────────────────────────────
# Converts /foxglove_teleop Twist → /cmd_vel TwistStamped
if [ -f "/usr/local/bin/twist_relay.py" ]; then
    python3 /usr/local/bin/twist_relay.py &
fi

# ── Nav demo (simulation only) ────────────────────────────────────────────
# Mirrors run_both.sh: sends an initial goal and keeps the ROS nav stack armed.
# Uses rclpy directly so /cmd_vel is subscribed with BEST_EFFORT QoS (matching
# the nav stack publisher) instead of the default RELIABLE used by ROSTransport.
if [ "$MODE" = "simulation" ] || [ "$MODE" = "unity_sim" ]; then
    DEMO="/workspace/dimos/dimos/navigation/demo_ros_navigation.py"
    if [ -f "$DEMO" ]; then
        echo "[entrypoint] Starting nav demo: $DEMO"
        python3 "$DEMO" &
    else
        echo "[entrypoint] WARNING: nav demo not found at $DEMO"
    fi
fi

# ── Hand off to the DimOS module runner ───────────────────────────────────
# The DockerModule framework injects: --payload '{"module_path":..., "args":..., "kwargs":...}'
exec python -m dimos.core.docker_runner run "$@"
