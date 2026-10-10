#!/usr/bin/env bash
# Quest localhost is a secure WebXR context; USB carries HTTP and WebSocket traffic.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
port=8080
has_height=0
help=0
arguments=("$@")
for ((i=0; i<${#arguments[@]}; i++)); do
  case "${arguments[i]}" in
    --height-m|--height-m=*) has_height=1 ;;
    --webxr-port)
      if (( i + 1 >= ${#arguments[@]} )); then echo '--webxr-port needs a value' >&2; exit 2; fi
      port="${arguments[i+1]}" ;;
    --webxr-port=*) port="${arguments[i]#*=}" ;;
    --help|-h) help=1 ;;
  esac
done
if (( help )); then exec bash "$root/bash/start_spd_sim.sh" "$@"; fi
if (( ! has_height )); then
  echo 'Specify operator height: pixi run spd-webxr --height-m HEIGHT' >&2
  exit 2
fi
if [[ ! "$port" =~ ^[0-9]{1,5}$ ]] || (( 10#$port < 1 || 10#$port > 65535 )); then
  echo 'WebXR port must be in 1..65535' >&2
  exit 2
fi
if [[ "${SPD_WEBXR_NO_ADB:-0}" != 1 ]]; then
  command -v adb >/dev/null || { echo 'Install adb, or use SPD_WEBXR_NO_ADB=1 for desktop inspection.' >&2; exit 1; }
  adb get-state >/dev/null
  adb reverse "tcp:$port" "tcp:$port"
  echo "Quest Browser: http://localhost:$port (USB; allow hand tracking, then enter VR)."
fi
arguments+=(--webxr-port "$port")
exec bash "$root/bash/start_spd_sim.sh" "${arguments[@]}"
