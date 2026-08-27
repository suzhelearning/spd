#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="spd_vr"
mode="detach"
mock=0
headless=0
duration="0"
scene="jenga"
task="handover_lr"
output=""
seed="0"

usage() {
  cat <<'EOF'
Usage: start_spd_vr.sh [--mock] [--headless] [--duration SECONDS]
  [--scene NAME] [--task NAME] [--seed N] [--output DIR] [--attach|--detach]
  start_spd_vr.sh --status|--stop

The mock path is deterministic and does not require ADB, ROS, or hardware.
Live mode requires the PICO ROS install and the Tianji controller build.
EOF
}

while (($#)); do
  case "$1" in
    --mock) mock=1; shift ;;
    --headless) headless=1; shift ;;
    --duration) duration="$2"; shift 2 ;;
    --scene) scene="$2"; shift 2 ;;
    --task) task="$2"; shift 2 ;;
    --seed) seed="$2"; shift 2 ;;
    --output) output="$2"; shift 2 ;;
    --attach) mode="attach"; shift ;;
    --detach) mode="detach"; shift ;;
    --status) mode="status"; shift ;;
    --stop) mode="stop"; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
command -v pixi >/dev/null || { echo "pixi is required" >&2; exit 1; }

session_exists() { tmux has-session -t "$session_name" 2>/dev/null; }

if [[ "$mode" == "status" ]]; then
  if session_exists; then
    tmux list-windows -t "$session_name" -F '#{window_name}:#{pane_current_command}'
    exit 0
  fi
  echo "spd-vr session is not running"
  exit 1
fi

if [[ "$mode" == "stop" ]]; then
  exec "$repo_root/scripts/stop_spd_vr.sh"
fi

if session_exists; then
  echo "refusing duplicate live session: $session_name" >&2
  exit 1
fi
if [[ "$mock" == 1 && -z "$output" ]]; then
  echo "--output is required with --mock" >&2
  exit 2
fi
if [[ "$mock" == 1 && "$duration" == "0" ]]; then
  echo "--duration must be positive with --mock" >&2
  exit 2
fi

pixi_path="$(command -v pixi)"
printf -v repo_quoted '%q' "$repo_root"
printf -v pixi_quoted '%q' "$pixi_path"
printf -v duration_q '%q' "$duration"
printf -v scene_q '%q' "$scene"
printf -v task_q '%q' "$task"
printf -v seed_q '%q' "$seed"
if [[ -z "$output" ]]; then
  output="$repo_root/data/spd_vr"
fi
printf -v output_q '%q' "$output"

if [[ "$mock" == 1 ]]; then
  # Keep a scoped session for the mock server and simulator labels, while the
  # foreground runtime provides a useful exit status to CI/smoke callers.
  tmux new-session -d -s "$session_name" -n mock
  tmux set-option -t "$session_name" remain-on-exit on >/dev/null
  mock_cmd="cd $repo_quoted && exec python3 src/pico_bridge/scripts/mock_pico_server.py --port 9999 --hand-case valid --duration $duration_q"
  printf -v mock_cmd_q '%q' "$mock_cmd"
  tmux send-keys -t "$session_name:mock" "exec bash -lc $mock_cmd_q" C-m
  tmux new-window -t "$session_name" -n simulator
  simulator_cmd="echo simulator foreground runtime; exec sleep $duration_q"
  printf -v simulator_cmd_q '%q' "$simulator_cmd"
  tmux send-keys -t "$session_name:simulator" "exec bash -lc $simulator_cmd_q" C-m
  tmux select-window -t "$session_name:mock"
  echo "Started mock SPD VR tmux session: $session_name"
  set +e
  PYTHONPATH="$repo_root/src/spd_vr" "$pixi_path" run python -m spd_vr.runtime \
    --mock --headless --duration "$duration" --scene "$scene" --task "$task" \
    --seed "$seed" --output "$output"
  result=$?
  set -e
  "$repo_root/scripts/stop_spd_vr.sh" || true
  exit "$result"
fi

if [[ ! -f "$repo_root/install/local_setup.bash" ]]; then
  echo "missing PICO install/local_setup.bash; run pixi run build-core first" >&2
  exit 1
fi
if [[ ! -x "$repo_root/../TJ_arm_control/build/tianji_qp_ik_viewer" ]]; then
  echo "missing TJ_arm_control/build/tianji_qp_ik_viewer; build Tianji first" >&2
  exit 1
fi
if [[ "$duration" == "0" ]]; then
  duration="3600"
  printf -v duration_q '%q' "$duration"
fi

ros_env="export ROS_DOMAIN_ID=120 EXO_REQUESTED_ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1"
driver_inner="cd $repo_quoted && $ros_env && exec ./scripts/start_pico_driver.sh"
m0_inner="cd $repo_quoted && $ros_env && exec ./scripts/start_pico_m0.sh --viewer"
bridge_inner="cd $repo_quoted && $ros_env && source install/local_setup.bash && exec ros2 launch pico_bridge start_tianji_mujoco_teleop.launch.py destination_address:=127.0.0.1 destination_port:=15000 position_retargeting_mode:=robot_arm_segments robot_arm_reach_scale:=0.95"
arm_inner="cd $repo_quoted/../TJ_arm_control && exec ./build/tianji_qp_ik_viewer --headless --duration $duration_q --config config/qp_ik_pico_teleop.yaml --model models/marvin_m6_qp_pico_fast.xml --pico-teleop --arm-target-host 127.0.0.1 --arm-target-port 15100"
sim_inner="cd $repo_quoted && $ros_env && exec $pixi_quoted run python -m spd_vr.runtime --duration $duration_q --scene $scene_q --task $task_q --seed $seed_q --output $output_q"
windows=(driver m0 bridge optical_hand arm_controller simulator)
commands=("$driver_inner" "$m0_inner" "$bridge_inner" "$optical_inner" "$arm_inner" "$sim_inner")
for index in "${!windows[@]}"; do
  window="${windows[$index]}"
  command="${commands[$index]}"
  if ((index == 0)); then
    tmux new-session -d -s "$session_name" -n "$window"
  else
    tmux new-window -t "$session_name" -n "$window"
  fi
  tmux set-option -t "$session_name" remain-on-exit on >/dev/null
  printf -v command_q '%q' "$command"
  tmux send-keys -t "$session_name:$window" "exec bash -lc $command_q" C-m
done
tmux select-window -t "$session_name:simulator"
echo "Started SPD VR tmux session: $session_name (palm_source=optical_hand)"
if [[ "$mode" == "attach" ]]; then
  exec tmux attach-session -t "$session_name"
fi
