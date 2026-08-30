#!/usr/bin/env bash
set -euo pipefail

session_name="spd-teleop"
endpoint="tcp/127.0.0.1:7447"
dry_run=0
metadata_path="${XDG_RUNTIME_DIR:-/tmp}/spd-vr/${session_name}.metadata"

usage() {
  cat <<'EOF'
Usage: stop_spd_vr.sh [--dry-run] [--endpoint ENDPOINT]
EOF
}

while (($#)); do
  case "$1" in
    --dry-run) dry_run=1; shift ;;
    --endpoint) endpoint="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ((dry_run)); then
  printf 'session=%s\n' "$session_name"
  printf 'spd-control shutdown --endpoint %s\n' "$endpoint"
  for window in viewer arm_ik pxrea_bridge; do
    printf '%s: wait\n' "$window"
  done
  exit 0
fi

command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }
if ! tmux has-session -t "$session_name" 2>/dev/null; then
  echo "SPD teleoperation session is not running"
  rm -f "$metadata_path"
  exit 0
fi
if [[ ! -s "$metadata_path" ]]; then
  echo "refusing to stop session without managed metadata: $metadata_path" >&2
  exit 1
fi

# Control is deliberately sent before any pane is interrupted.
if command -v pixi >/dev/null; then
  (cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)" && pixi run spd-control shutdown --endpoint "$endpoint") || echo "spd-control shutdown failed" >&2
else
  echo "pixi is required to publish spd-control shutdown" >&2
fi

# Read only panes created by the matching session metadata.  A pane PID and
# its command are checked again immediately before interrupt/escalation.
declare -A pane_for pid_for module_for
while IFS=$'\t' read -r field window pid module; do
  [[ "$field" == pane=* ]] || continue
  field="${field#pane=}"
  window="${window#window=}"
  pid="${pid#pid=}"
  pane_for["$window"]="$field"
  pid_for["$window"]="$pid"
  module_for["$window"]="${module#module=}"
done <"$metadata_path"
verified=1
wait_window() {
  local window="$1"
  local pane="${pane_for[$window]:-}" expected="${module_for[$window]:-}" recorded_pid="${pid_for[$window]:-}"
  if [[ -z "$pane" || -z "$expected" || -z "$recorded_pid" ]]; then
    echo "missing metadata for $window" >&2
    verified=0
    return
  fi
  local current_pid actual
  current_pid="$(tmux display-message -p -t "$pane" '#{pane_pid}' 2>/dev/null || true)"
  if [[ "$current_pid" != "$recorded_pid" ]]; then
    echo "refusing unverified pane $pane ($window)" >&2
    verified=0
    return
  fi
  actual="$(ps -p "$current_pid" -o args= 2>/dev/null || true)"
  if [[ "$actual" != *"python -m $expected"* ]]; then
    echo "refusing unexpected command for $window: $actual" >&2
    verified=0
    return
  fi
  tmux send-keys -t "$pane" C-c || true
  for _ in {1..50}; do
    if ! tmux list-panes -t "$pane" >/dev/null 2>&1; then
      return
    fi
    current_pid="$(tmux display-message -p -t "$pane" '#{pane_pid}' 2>/dev/null || true)"
    if [[ -z "$current_pid" ]] || ! kill -0 "$current_pid" 2>/dev/null; then
      tmux kill-pane -t "$pane" || true
      return
    fi
done
  # Escalate only after rechecking both identity fields.
  current_pid="$(tmux display-message -p -t "$pane" '#{pane_pid}' 2>/dev/null || true)"
  actual="$(ps -p "$current_pid" -o args= 2>/dev/null || true)"
  if [[ "$current_pid" == "$recorded_pid" && "$actual" == *"python -m $expected"* ]]; then
    tmux kill-pane -t "$pane" || true
  else
    echo "refusing escalation for unverified pane $pane" >&2
    verified=0
  fi
}

for window in viewer arm_ik pxrea_bridge; do
  wait_window "$window"
done
if ((verified)); then
  if tmux has-session -t "$session_name" 2>/dev/null; then
    echo "session retained because managed panes did not all close" >&2
    exit 1
  fi
  rm -f "$metadata_path"
  echo "Stopped SPD teleoperation session: $session_name"
  exit 0
fi
echo "session retained because one or more panes failed identity checks" >&2
exit 1
