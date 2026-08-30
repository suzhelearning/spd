#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="spd-teleop"
mode="detach"
action="start"
dry_run=0
endpoint="tcp/127.0.0.1:7447"
sdk_library="${PXREA_SDK_LIBRARY:-/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so}"
manifest="$repo_root/src/spd_vr/generated/model_manifest.yaml"
urdf="$repo_root/../assets/tianji_wuji2/tianji_wuji2.urdf"
metadata_dir="${XDG_RUNTIME_DIR:-/tmp}/spd-vr"
metadata_path="$metadata_dir/${session_name}.metadata"

usage() {
  cat <<'EOF'
Usage: start_spd_vr.sh [--dry-run] [--attach|--detach]
  [--endpoint ENDPOINT] [--sdk-library PATH] [--manifest PATH] [--urdf PATH]
EOF
}

while (($#)); do
  case "$1" in
    --dry-run) dry_run=1; shift ;;
    --attach) mode="attach"; shift ;;
    --detach) mode="detach"; shift ;;
    --endpoint) endpoint="$2"; shift 2 ;;
    --sdk-library) sdk_library="$2"; shift 2 ;;
    --manifest) manifest="$2"; shift 2 ;;
    --urdf) urdf="$2"; shift 2 ;;
    --status) action="status"; shift ;;
    --stop) action="stop"; shift ;;
    --help|-h) usage; exit 0 ;;
  esac
done

if [[ "$action" == "status" ]]; then
  command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
  if tmux has-session -t "$session_name" 2>/dev/null; then
    tmux list-windows -t "$session_name" -F '#{window_name}:#{pane_current_command}'
    exit 0
  fi
  echo "SPD teleoperation session is not running"
  exit 1
fi
if [[ "$action" == "stop" ]]; then
  exec "$repo_root/scripts/stop_spd_vr.sh" --endpoint "$endpoint"
fi

windows=(pxrea_bridge arm_ik viewer)
commands=(
  "python -m spd_vr.pxrea_bridge --sdk-library $sdk_library --endpoint $endpoint --listen"
  "python -m spd_vr.arm_ik --model ${manifest%/*}/arm_ik.xml --manifest $manifest --urdf $urdf --endpoint $endpoint"
  "python -m spd_vr.viewer --model ${manifest%/*}/unified_plant.xml --manifest $manifest --endpoint $endpoint"
)

if ((dry_run)); then
  printf 'session=%s\n' "$session_name"
  for index in "${!windows[@]}"; do
    printf '%s: %s\n' "${windows[$index]}" "${commands[$index]}"
  done
  exit 0
fi

command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
command -v pixi >/dev/null || { echo "pixi is required" >&2; exit 1; }
if tmux has-session -t "$session_name" 2>/dev/null; then
  echo "refusing duplicate session: $session_name" >&2
  exit 1
fi

preflight_args=(--repo-root "$repo_root" --endpoint "$endpoint" --sdk-library "$sdk_library" --manifest "$manifest" --urdf "$urdf" --session "$session_name")
if ! (cd "$repo_root" && pixi run spd-preflight "${preflight_args[@]}"); then
  echo "preflight failed; no session was created" >&2
  exit 1
fi

mkdir -p "$metadata_dir"
tmp_metadata="$(mktemp "$metadata_path.XXXXXX")"
cleanup_metadata() { rm -f "$tmp_metadata"; }
trap cleanup_metadata EXIT
session_id="$(date +%s%N)-$$"
printf 'session=%s\nid=%s\nendpoint=%s\n' "$session_name" "$session_id" "$endpoint" >"$tmp_metadata"

for index in "${!windows[@]}"; do
  window="${windows[$index]}"
  command_line="${commands[$index]}"
  if ((index == 0)); then
    tmux new-session -d -s "$session_name" -n "$window"
  else
    tmux new-window -t "$session_name" -n "$window"
  fi
  tmux set-option -t "$session_name" remain-on-exit on >/dev/null
  pane_id="$(tmux display-message -p -t "$session_name:$window" '#{pane_id}')"
  pane_pid="$(tmux display-message -p -t "$session_name:$window" '#{pane_pid}')"
  printf 'pane=%s\twindow=%s\tpid=%s\tmodule=spd_vr.%s\n' "$pane_id" "$window" "$pane_pid" "$window" >>"$tmp_metadata"
  printf -v quoted '%q' "$command_line"
  tmux send-keys -t "$session_name:$window" "cd $(printf '%q' "$repo_root") && exec bash -lc $quoted" C-m
done
mv -f "$tmp_metadata" "$metadata_path"
trap - EXIT

tmux select-window -t "$session_name:viewer"
echo "Started SPD teleoperation session: $session_name"
if [[ "$mode" == "attach" ]]; then
  exec tmux attach-session -t "$session_name"
fi
