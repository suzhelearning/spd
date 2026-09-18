#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="spd-ros"
socket="$repo_root/.pixi/spd-ros.tmux.sock"
tmux_bin="$(command -v tmux || true)"
if [[ -z "$tmux_bin" ]]; then
  tmux_bin="$repo_root/.pixi/envs/default/bin/tmux"
  export TERMINFO="$repo_root/.pixi/envs/default/share/terminfo"
fi
[[ -x "$tmux_bin" ]] || { echo "tmux is required; run pixi install" >&2; exit 1; }
if "$tmux_bin" -S "$socket" has-session -t "=$session_name" 2>/dev/null; then
  "$tmux_bin" -S "$socket" kill-session -t "=$session_name"
  echo "Stopped this project's SPD bridge/viewer session: $session_name"
else
  echo "This project's SPD bridge/viewer session is not running"
fi
