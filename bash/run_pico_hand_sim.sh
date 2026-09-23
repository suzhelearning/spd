#!/usr/bin/env bash
# One process owns headset input, relative binding, physics and collection keys.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
has_height=0
for argument in "$@"; do
  case "$argument" in
    --height-m|--height-m=*|--help|-h) has_height=1 ;;
  esac
done
if (( ! has_height )); then
  echo 'Specify the operator height: --height-m HEIGHT (metres).' >&2
  exit 2
fi
exec bash "$root/bash/start_spd_sim.sh" "$@"
