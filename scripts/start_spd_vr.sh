#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="spd-teleop"
mode="detach"
action="start"
dry_run=0
endpoint="${SPD_VR_ZENOH_ENDPOINT:-tcp/127.0.0.1:8888}"
serial="${PICO_ADB_SERIAL:-}"
pico2_port="${PICO2_PORT:-10002}"
pico2_device_port="${PICO2_DEVICE_PORT:-$pico2_port}"
manifest="$repo_root/packages/spd-vr/generated/model_manifest.yaml"
urdf="$repo_root/assets/tianji_wuji2/tianji_wuji2.urdf"
metadata_dir="${XDG_RUNTIME_DIR:-/tmp}/spd-vr"
metadata_path="$metadata_dir/${session_name}.metadata"

usage() {
  cat <<'EOF'
Usage: start_spd_vr.sh [--dry-run] [--attach|--detach]
  [--endpoint ENDPOINT] [--serial SERIAL]
  [--manifest PATH] [--urdf PATH]

PICO2 uses ADB forward tcp:${PICO2_PORT:-10002} -> tcp:${PICO2_DEVICE_PORT:-10002}.
EOF
}

while (($#)); do
  case "$1" in
    --dry-run) dry_run=1; shift ;;
    --attach) mode="attach"; shift ;;
    --detach) mode="detach"; shift ;;
    --endpoint) endpoint="$2"; shift 2 ;;
    --serial)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { echo "--serial requires a value" >&2; exit 2; }
      serial="$2"
      shift 2
      ;;
    --manifest) manifest="$2"; shift 2 ;;
    --urdf) urdf="$2"; shift 2 ;;
    --status) action="status"; shift ;;
    --stop) action="stop"; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
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
windows=(pico2_bridge arm_ik viewer)
endpoint_q="$(printf '%q' "$endpoint")"
urdf_q="$(printf '%q' "$urdf")"
manifest_q="$(printf '%q' "$manifest")"
pico2_port_q="$(printf '%q' "$pico2_port")"
pico2_device_port_q="$(printf '%q' "$pico2_device_port")"
source_command="python -m spd_vr.pico2_bridge --port $pico2_port_q --device-port $pico2_device_port_q --endpoint $endpoint_q --listen"
if [[ -n "$serial" ]]; then
  source_command+=" --adb-serial $(printf '%q' "$serial")"
fi
commands=(
  "$source_command"
  "python -m spd_vr.arm_ik --model $(printf '%q' "${manifest%/*}/arm_ik.xml") --manifest $manifest_q --urdf $urdf_q --endpoint $endpoint_q"
  "python -m spd_vr.viewer --model $(printf '%q' "${manifest%/*}/unified_plant.xml") --manifest $manifest_q --urdf $urdf_q --endpoint $endpoint_q"
)
modules=(
  "spd_vr.pico2_bridge"
  "spd_vr.arm_ik"
  "spd_vr.viewer"
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

preflight_args=(--repo-root "$repo_root" --endpoint "$endpoint" --manifest "$manifest" --urdf "$urdf" --session "$session_name" --pico2-port "$pico2_port" --pico2-device-port "$pico2_device_port")
if [[ -n "$serial" ]]; then
  preflight_args+=(--serial "$serial")
fi
if ! (cd "$repo_root" && pixi run spd-preflight "${preflight_args[@]}"); then
  echo "preflight failed; no session was created" >&2
  exit 1
fi

mkdir -p "$metadata_dir"
tmp_metadata="$(mktemp "$metadata_path.XXXXXX")"
created_session=0
cleanup_start() {
  if ((created_session)) && tmux has-session -t "$session_name" 2>/dev/null; then
    tmux kill-session -t "$session_name" || true
  fi
  rm -f "$tmp_metadata"
}
trap cleanup_start EXIT
for index in "${!windows[@]}"; do
  window="${windows[$index]}"
  command_line="${commands[$index]}"
  module="${modules[$index]}"
  if ((index == 0)); then
    tmux new-session -d -s "$session_name" -n "$window"
    created_session=1
  else
    tmux new-window -t "$session_name" -n "$window"
  fi
  tmux set-option -t "$session_name" remain-on-exit on >/dev/null
  pane_id="$(tmux display-message -p -t "$session_name:$window" '#{pane_id}')"
  pane_pid="$(tmux display-message -p -t "$session_name:$window" '#{pane_pid}')"
  printf 'pane=%s\twindow=%s\tpid=%s\tmodule=%s\n' "$pane_id" "$window" "$pane_pid" "$module" >>"$tmp_metadata"
  tmux send-keys -t "$session_name:$window" "cd $(printf '%q' "$repo_root") && exec $command_line" C-m
done
mv -f "$tmp_metadata" "$metadata_path"
trap - EXIT

tmux select-window -t "$session_name:viewer"
echo "Started SPD teleoperation session: $session_name"
if [[ "$mode" == "attach" ]]; then
  exec tmux attach-session -t "$session_name"
fi
