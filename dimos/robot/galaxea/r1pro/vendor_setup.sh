#!/usr/bin/env bash
# Boots the R1 Pro vendor stack in tmux unless it is already up. With no argument it starts Galaxea's
# ATCStandard/R1PROBody.d minus signal_camera and livox (dimos opens the head cameras and Mid-360
# itself); with a vendor profile, e.g. ATCStandard/R1PROVRTeleop, it boots that profile instead.
set -euo pipefail
script_dir="$HOME/galaxea-dimos/install/startup_config/share/startup_config/script"

if tmux has-session -t =hdas 2>/dev/null && tmux has-session -t =mobiman 2>/dev/null; then
    exit 0
fi
cd "$script_dir"
if [ -n "${1:-}" ]; then
    exec ./robot_startup.sh boot "../sessions.d/$1.d"
fi

# A tmux session named like the vendor's, with one window per command; `exec bash` keeps a crashed one open.
run() {
    local session=$1 dir=$script_dir/boot/modules/$2
    tmux has-session -t "=$session" 2>/dev/null && return
    tmux new-session -d -s "$session" -c "$dir" "$3; exec bash"
    for command in "${@:4}"; do
        tmux new-window -t "=$session:" -c "$dir" "$command; exec bash"
    done
}

# extremely cursed sleeps - straight out of the galaxea stock profile though (mobiman waits for HDAS, the gatekeeper for mobiman)
run ros_discovery system "sleep 1; ./start_discover_server.sh"
run hdas hdas "sleep 1; ./start_hdas_r1pro.sh" "sleep 1; ./start_zed_camera.sh" "sleep 1; ./start_realsense_camera_r1pro.sh"
run mobiman mobiman \
    "sleep 10; ./start_mobiman_r1pro_chassis.sh" \
    "sleep 10; ./start_mobiman_r1pro_joint_pid.sh" \
    "sleep 10; ./start_mobiman_r1_gripper_controller.sh R1PRO" \
    "sleep 3; ./start_mobiman_r1pro_pose_ee.sh" \
    "sleep 14; ./start_chassis_gatekeeper.sh"
