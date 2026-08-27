#!/usr/bin/env bash
set -euo pipefail

session_name="spd_vr"
if ! tmux has-session -t "$session_name" 2>/dev/null; then
  echo "SPD VR session is not running"
  exit 0
fi

# Stop consumers before producers so recorder/model state can flush before
# arm control, ROS bridge, and PICO driver disappear.
for window in simulator arm_controller optical_hand bridge m0 driver; do
  if tmux list-windows -t "$session_name:$window" >/dev/null 2>&1; then
    tmux send-keys -t "$session_name:$window" C-c || true
  fi
done

for _ in {1..50}; do
  all_dead=1
  for window in simulator arm_controller optical_hand bridge m0 driver; do
    if ! tmux list-windows -t "$session_name:$window" >/dev/null 2>&1; then
      continue
    fi
    pane_dead="$(tmux list-panes -t "$session_name:$window" -F '#{pane_dead}' 2>/dev/null || printf '1')"
    if [[ "$pane_dead" != "1" ]]; then
      all_dead=0
    fi
  done
  if [[ "$all_dead" == 1 ]]; then
    tmux kill-session -t "$session_name"
    echo "Stopped SPD VR session: $session_name"
    exit 0
  fi
  sleep 0.1
done

# Only terminate the named session after the bounded recorder flush wait; never
# pkill unrelated ROS processes.
tmux kill-session -t "$session_name"
echo "Stopped SPD VR session: $session_name"
