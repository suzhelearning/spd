#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  adb.sh [start]  [--offline] [--once] [--interval SEC] [--device-port PORT] [--host-port PORT] [--serial SERIAL]
  adb.sh status    [--device-port PORT] [--host-port PORT] [--serial SERIAL]
  adb.sh stop      [--device-port PORT] [--host-port PORT] [--serial SERIAL]

Default start runs in the foreground, prints ADB tunnel stability, and cleans
the reverse mapping on Ctrl-C. --offline starts a detached supervisor with
exponential retry backoff; use stop to terminate it and clean the mapping.
The host port is auto-detected from the non-loopback TCP listener owned by
RoboticsService when omitted. The device port defaults to the host port.
Pass --host-port explicitly when more than one service port is present.
--once establishes the reverse mapping and exits without foreground monitoring.

Environment:
  ADB_BIN               adb executable or absolute path (default: adb)
  SS_BIN                ss executable or absolute path (default: ss)
  PICO_ADB_SERIAL       serial to use when --serial is omitted
  PICO_DEVICE_PORT      default device-side reverse port
  PICO_HOST_PORT        default host-side reverse port
  PICO_ADB_STATE_DIR    supervisor pid/log directory
  PICO_ADB_MONITOR_INTERVAL  foreground health-check interval (default: 1)
EOF
}

die() {
  printf 'adb.sh: %s\n' "$*" >&2
  exit 2
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if (($# == 0)); then
  command_name="start"
elif [[ "$1" == -* ]]; then
  command_name="start"
elif [[ "$1" == "__monitor" ]]; then
  command_name="__monitor"
  shift
else
  command_name="$1"
  shift
fi

serial="${PICO_ADB_SERIAL:-}"
device_port="${PICO_DEVICE_PORT:-}"
host_port="${PICO_HOST_PORT:-}"
offline=0
once=0
monitor_interval="${PICO_ADB_MONITOR_INTERVAL:-1}"
monitor_mode=0
instance_token=""
sleep_pid=""
adb_child_pid=""
adb_output_file=""
managed_reverse=0
managed_serial=""
managed_device_port=""
managed_host_port=""
if [[ "$command_name" == "__monitor" ]]; then
  monitor_mode=1
fi

host_port_explicit=0
device_port_explicit=0
[[ -n "$host_port" ]] && host_port_explicit=1
[[ -n "$device_port" ]] && device_port_explicit=1

while (($#)); do
  case "$1" in
    --serial)
      (($# >= 2)) || die "--serial requires a value"
      serial="$2"
      shift 2
      ;;
    --device-port)
      (($# >= 2)) || die "--device-port requires a value"
      device_port="$2"
      device_port_explicit=1
      shift 2
      ;;
    --host-port)
      (($# >= 2)) || die "--host-port requires a value"
      host_port="$2"
      host_port_explicit=1
      shift 2
      ;;
    --offline)
      offline=1
      shift
      ;;
    --once)
      once=1
      shift
      ;;
    --interval)
      (($# >= 2)) || die "--interval requires a value"
      monitor_interval="$2"
      shift 2
      ;;
    --instance-token)
      (($# >= 2)) || die "--instance-token requires a value"
      instance_token="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

if [[ "$command_name" != "start" &&
      "$command_name" != "status" &&
      "$command_name" != "stop" &&
      "$command_name" != "__monitor" ]]; then
  die "unknown command: $command_name"
fi
if ((offline)) && [[ "$command_name" != "start" ]]; then
  die "--offline is only valid with start"
fi
if ((once)) && [[ "$command_name" != "start" ]]; then
  die "--once is only valid with start"
fi
if ((offline && once)); then
  die "--offline and --once are mutually exclusive"
fi

validate_port() {
  local name="$1"
  local value="$2"
  [[ "$value" =~ ^[0-9]+$ ]] || die "$name must be an integer in 1..65535"
  local numeric=$((10#$value))
  ((numeric >= 1 && numeric <= 65535)) || die "$name must be an integer in 1..65535"
}

validate_interval() {
  [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "--interval must be a positive number"
  awk -v value="$1" 'BEGIN { exit !(value > 0) }' \
    || die "--interval must be a positive number"
}

if [[ -n "$device_port" ]]; then
  validate_port "device port" "$device_port"
fi
if [[ -n "$host_port" ]]; then
  validate_port "host port" "$host_port"
fi
validate_interval "$monitor_interval"

adb_bin="${ADB_BIN:-adb}"
if [[ "$adb_bin" == */* ]]; then
  [[ -x "$adb_bin" ]] || die "ADB_BIN is not executable: $adb_bin"
else
  command -v "$adb_bin" >/dev/null 2>&1 \
    || die "adb executable not found; install Android Platform Tools or set ADB_BIN"
fi

state_dir="${PICO_ADB_STATE_DIR:-${XDG_RUNTIME_DIR:-/tmp}/pico-adb-$UID}"
pid_file="$state_dir/supervisor.pid"
owned_file="$state_dir/supervisor.reverse"
log_file="$state_dir/supervisor.log"
lock_file="$state_dir/supervisor.lock"
script_path="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/$(basename "${BASH_SOURCE[0]}")"

monitor_reason="PICO not connected"

process_start_time() {
  local pid="$1"
  local stat_line
  local stat_tail
  local -a stat_fields
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/stat" ]] || return 1
  IFS= read -r stat_line <"/proc/$pid/stat" || return 1
  stat_tail="${stat_line##*) }"
  read -r -a stat_fields <<<"$stat_tail"
  ((${#stat_fields[@]} > 19)) || return 1
  printf '%s\n' "${stat_fields[19]}"
}

process_matches_instance() {
  local pid="$1"
  local expected_start="$2"
  local expected_token="$3"
  local actual_start
  local arg
  local has_monitor=0
  local has_token=0
  local -a process_args
  actual_start="$(process_start_time "$pid" 2>/dev/null)" || return 1
  [[ "$actual_start" == "$expected_start" ]] || return 1
  mapfile -d '' -t process_args <"/proc/$pid/cmdline" 2>/dev/null || return 1
  for arg in "${process_args[@]}"; do
    [[ "$arg" == "__monitor" ]] && has_monitor=1
    [[ "$arg" == "$expected_token" ]] && has_token=1
  done
  ((has_monitor && has_token))
}

read_supervisor_state() {
  existing_pid=""
  existing_start=""
  existing_token=""
  [[ -f "$pid_file" ]] || return 1
  read -r existing_pid existing_start existing_token <"$pid_file" || return 1
  [[ "$existing_pid" =~ ^[0-9]+$ &&
     "$existing_start" =~ ^[0-9]+$ &&
     -n "$existing_token" ]]
}

acquire_supervisor_lock() {
  mkdir -p "$state_dir"
  command -v flock >/dev/null 2>&1 || die "flock executable not found"
  exec {supervisor_lock_fd}>"$lock_file"
  flock -x "$supervisor_lock_fd" || die "cannot lock supervisor state"
}

release_supervisor_lock() {
  flock -u "$supervisor_lock_fd" 2>/dev/null || true
  exec {supervisor_lock_fd}>&-
}

try_select_serial() {
  if [[ -n "$serial" ]]; then
    local state
    state="$("$adb_bin" -s "$serial" get-state 2>/dev/null || true)"
    if [[ "$state" == "device" ]]; then
      return 0
    fi
    monitor_reason="PICO not connected: ADB device is not online: $serial"
    return 1
  fi

  local devices_output
  if ! devices_output="$("$adb_bin" devices 2>&1)"; then
    monitor_reason="adb devices failed: $devices_output"
    return 1
  fi
  mapfile -t listed_serials < <(
    printf '%s\n' "$devices_output" | awk 'NR > 1 && $1 != "" {print $1}'
  )
  mapfile -t online_serials < <(
    printf '%s\n' "$devices_output" | awk 'NR > 1 && $2 == "device" {print $1}'
  )
  case "${#online_serials[@]}" in
    0)
      if ((${#listed_serials[@]} == 0)); then
        monitor_reason="PICO not connected"
      else
        monitor_reason="PICO not connected: ADB device is not authorized or online"
      fi
      return 1
      ;;
    1)
      serial="${online_serials[0]}"
      return 0
      ;;
    *)
      monitor_reason="multiple online adb devices; pass --serial"
      return 1
      ;;
  esac
}

select_serial() {
  try_select_serial || die "$monitor_reason"
}

try_detect_host_port() {
  local ss_bin="${SS_BIN:-ss}"
  if [[ "$ss_bin" == */* ]]; then
    if [[ ! -x "$ss_bin" ]]; then
      monitor_reason="SS_BIN is not executable: $ss_bin"
      return 1
    fi
  elif ! command -v "$ss_bin" >/dev/null 2>&1; then
    monitor_reason="ss executable not found; pass --host-port"
    return 1
  fi

  local ss_output
  if ! ss_output="$("$ss_bin" -H -ltnp 2>&1)"; then
    monitor_reason="ss failed while detecting RoboticsService port: $ss_output"
    return 1
  fi
  mapfile -t service_ports < <(
    printf '%s\n' "$ss_output" | awk '
      $0 ~ /users:\(\("RoboticsService"/ {
        address = $4
        if (address ~ /^127\./ ||
            address ~ /^\[::1\]/ ||
            address ~ /^\[::ffff:127\.0\.0\.1\]/) {
          next
        }
        sub(/^.*:/, "", address)
        if (address ~ /^[0-9]+$/) {
          print address
        }
      }
    ' | sort -nu
  )
  case "${#service_ports[@]}" in
    0)
      monitor_reason="cannot auto-detect a non-loopback RoboticsService TCP port; pass --host-port"
      return 1
      ;;
    1)
      host_port="${service_ports[0]}"
      return 0
      ;;
    *)
      monitor_reason="multiple non-loopback RoboticsService TCP ports; pass --host-port (${service_ports[*]})"
      return 1
      ;;
  esac
}

try_resolve_ports() {
  if ((host_port_explicit == 0)); then
    try_detect_host_port || return 1
  fi
  if ((device_port_explicit == 0)); then
    device_port="$host_port"
  fi
  if ! [[ "$device_port" =~ ^[0-9]+$ ]] ||
     ! [[ "$host_port" =~ ^[0-9]+$ ]]; then
    monitor_reason="detected port is not numeric"
    return 1
  fi
  if ((10#$device_port < 1 || 10#$device_port > 65535 ||
       10#$host_port < 1 || 10#$host_port > 65535)); then
    monitor_reason="detected port is outside 1..65535"
    return 1
  fi
  return 0
}

resolve_ports() {
  try_resolve_ports || die "$monitor_reason"
}

read_reverse_list() {
  local target_serial="${1:-$serial}"
  if ! reverse_output="$("$adb_bin" -s "$target_serial" reverse --list 2>&1)"; then
    monitor_reason="adb reverse --list failed for $target_serial: $reverse_output"
    return 1
  fi
  return 0
}

find_reverse() {
  local reverses="$1"
  local target_device_port="$2"
  local target_host_port="$3"
  printf '%s\n' "$reverses" |
    awk -v device="tcp:$target_device_port" -v host="tcp:$target_host_port" '
      $2 == device && $3 == host {print; exit}
    '
}

find_device_reverse() {
  local reverses="$1"
  local target_device_port="$2"
  printf '%s\n' "$reverses" | awk -v device="tcp:$target_device_port" '
    $2 == device {print $3; exit}
  '
}

persist_owned_reverse() {
  ((monitor_mode)) || return 0
  mkdir -p "$state_dir"
  printf '%s %s %s %s\n' \
    "$instance_token" "$managed_serial" "$managed_device_port" "$managed_host_port" \
    >"$owned_file"
}

clear_owned_file() {
  ((monitor_mode)) || return 0
  local owned_token=""
  if [[ -f "$owned_file" ]]; then
    read -r owned_token _ <"$owned_file" || true
    if [[ "$owned_token" == "$instance_token" ]]; then
      rm -f "$owned_file"
    fi
  fi
}

mark_owned_reverse() {
  managed_reverse=1
  managed_serial="$serial"
  managed_device_port="$device_port"
  managed_host_port="$host_port"
  persist_owned_reverse
}

forget_owned_reverse() {
  clear_owned_file
  managed_reverse=0
  managed_serial=""
  managed_device_port=""
  managed_host_port=""
}

remove_exact_reverse() {
  local target_serial="$1"
  local target_device_port="$2"
  local target_host_port="$3"
  local remove_error
  read_reverse_list "$target_serial" || return 1
  if [[ -z "$(find_reverse "$reverse_output" "$target_device_port" "$target_host_port")" ]]; then
    return 0
  fi
  if ! remove_error="$("$adb_bin" -s "$target_serial" reverse --remove \
      "tcp:$target_device_port" 2>&1)"; then
    monitor_reason="adb reverse --remove failed for $target_serial tcp:$target_device_port: $remove_error"
    return 1
  fi
  return 0
}

cleanup_owned_reverse() {
  ((managed_reverse)) || return 0
  remove_exact_reverse \
    "$managed_serial" "$managed_device_port" "$managed_host_port" || return 1
  forget_owned_reverse
}

cleanup_persisted_reverse() {
  local expected_token="$1"
  local owned_token=""
  local target_serial=""
  local target_device_port=""
  local target_host_port=""
  [[ -f "$owned_file" ]] || return 0
  read -r owned_token target_serial target_device_port target_host_port <"$owned_file" \
    || {
      monitor_reason="invalid persisted reverse state"
      return 1
    }
  if [[ -n "$expected_token" && "$owned_token" != "$expected_token" ]]; then
    monitor_reason="persisted reverse belongs to another supervisor instance"
    return 1
  fi
  remove_exact_reverse "$target_serial" "$target_device_port" "$target_host_port" \
    || return 1
  rm -f "$owned_file"
}

migrate_owned_reverse() {
  ((managed_reverse)) || return 0
  if [[ "$managed_serial" == "$serial" &&
        "$managed_device_port" == "$device_port" &&
        "$managed_host_port" == "$host_port" ]]; then
    return 0
  fi
  cleanup_owned_reverse
}

run_reverse_create() {
  local command_status
  adb_output_file="$(mktemp "${TMPDIR:-/tmp}/pico-adb-reverse.XXXXXX")" || {
    monitor_reason="cannot create temporary adb output file"
    return 1
  }
  "$adb_bin" -s "$serial" reverse --no-rebind \
    "tcp:$device_port" "tcp:$host_port" >"$adb_output_file" 2>&1 &
  adb_child_pid=$!
  if wait "$adb_child_pid"; then
    command_status=0
  else
    command_status=$?
  fi
  adb_child_pid=""
  reverse_error="$(<"$adb_output_file")"
  rm -f "$adb_output_file"
  adb_output_file=""
  return "$command_status"
}

ensure_reverse() {
  local failure_reason
  read_reverse_list "$serial" || return 1
  if [[ -n "$(find_reverse "$reverse_output" "$device_port" "$host_port")" ]]; then
    reverse_action="already-reversed"
    return 0
  fi
  existing_host="$(find_device_reverse "$reverse_output" "$device_port")"
  if [[ -n "$existing_host" ]]; then
    monitor_reason="device port $device_port is already reversed to $existing_host"
    return 1
  fi
  mark_owned_reverse
  if ! run_reverse_create; then
    failure_reason="adb reverse failed: $reverse_error"
    if read_reverse_list "$serial" &&
       [[ -z "$(find_reverse "$reverse_output" "$device_port" "$host_port")" ]]; then
      forget_owned_reverse
    fi
    monitor_reason="$failure_reason"
    return 1
  fi
  reverse_action="reversed"
  return 0
}

stability_percent() {
  if ((monitor_checks == 0)); then
    printf '0.0'
    return
  fi
  local tenths=$((monitor_healthy * 1000 / monitor_checks))
  printf '%d.%d' "$((tenths / 10))" "$((tenths % 10))"
}

print_health() {
  local state="$1"
  local reverse_state="$2"
  local retry_in="${3:-0}"
  local stability
  stability="$(stability_percent)"
  printf '[pico-adb] state=%s serial=%s reverse=%s stability=%s%% checks=%d healthy=%d drops=%d uptime=%ss' \
    "$state" "${serial:-none}" "$reverse_state" "$stability" \
    "$monitor_checks" "$monitor_healthy" "$monitor_drops" "$((SECONDS - monitor_started))"
  if ((retry_in > 0)); then
    printf ' retry_in=%ss' "$retry_in"
  fi
  if [[ "$state" != "online" && -n "$monitor_reason" ]]; then
    printf ' reason=%s' "$monitor_reason"
  fi
  printf '\n'
}

monitor_sleep() {
  sleep "$1" &
  sleep_pid=$!
  wait "$sleep_pid" 2>/dev/null || true
  sleep_pid=""
}

monitor_loop() {
  monitor_checks=0
  monitor_healthy=0
  monitor_drops=0
  monitor_started=$SECONDS
  last_healthy=0
  backoff_seconds=1
  monitor_reason="waiting for ADB tunnel"
  print_health starting inactive

  while :; do
    monitor_checks=$((monitor_checks + 1))
    monitor_reason="PICO not connected"
    if try_select_serial &&
       try_resolve_ports &&
       migrate_owned_reverse &&
       ensure_reverse; then
      monitor_healthy=$((monitor_healthy + 1))
      if ((last_healthy == 0)); then
        printf '%s serial=%s host_port=%s device_port=%s\n' \
          "$reverse_action" "$serial" "$host_port" "$device_port"
      fi
      print_health online active
      last_healthy=1
      backoff_seconds=1
      monitor_sleep "$monitor_interval"
    else
      if ((last_healthy == 1)); then
        monitor_drops=$((monitor_drops + 1))
      fi
      print_health offline inactive "$backoff_seconds"
      last_healthy=0
      monitor_sleep "$backoff_seconds"
      if ((backoff_seconds < 8)); then
        backoff_seconds=$((backoff_seconds * 2))
      fi
    fi
  done
}

cleanup_monitor() {
  if [[ -n "$adb_child_pid" ]]; then
    kill -TERM "$adb_child_pid" 2>/dev/null || true
    kill -KILL "$adb_child_pid" 2>/dev/null || true
    wait "$adb_child_pid" 2>/dev/null || true
    adb_child_pid=""
  fi
  if [[ -n "$adb_output_file" ]]; then
    rm -f "$adb_output_file"
    adb_output_file=""
  fi
  if [[ -n "$sleep_pid" ]]; then
    kill -TERM "$sleep_pid" 2>/dev/null || true
    wait "$sleep_pid" 2>/dev/null || true
    sleep_pid=""
  fi
  if ! cleanup_owned_reverse; then
    printf 'adb.sh: cleanup warning: %s\n' "$monitor_reason" >&2
  fi
  if ((monitor_mode)) &&
     read_supervisor_state &&
     [[ "$existing_pid" == "$$" && "$existing_token" == "$instance_token" ]]; then
    rm -f "$pid_file"
  fi
  printf 'stopped serial=%s host_port=%s device_port=%s\n' \
    "${serial:-none}" "${host_port:-auto}" "${device_port:-auto}"
}

interrupt_foreground() {
  # Stop accepting signals before EXIT cleanup.  In particular, do not leave
  # an inline `exit` trap to be parsed while Bash is unwinding a command or
  # process substitution from the fast monitor loop.
  trap - INT TERM
  exit 130
}

terminate_foreground() {
  trap - INT TERM
  exit 143
}

stop_background_monitor() {
  trap - INT TERM
  exit 0
}

run_foreground() {
  select_serial
  resolve_ports
  trap interrupt_foreground INT
  trap terminate_foreground TERM
  trap cleanup_monitor EXIT
  ensure_reverse || die "$monitor_reason"
  printf '%s serial=%s host_port=%s device_port=%s\n' \
    "$reverse_action" "$serial" "$host_port" "$device_port"
  if ((once)); then
    trap - INT TERM EXIT
    exit 0
  fi
  monitor_loop
}

start_background() {
  local stale_token=""
  local supervisor_start=""
  acquire_supervisor_lock
  if [[ -f "$pid_file" ]]; then
    if read_supervisor_state; then
      if process_matches_instance "$existing_pid" "$existing_start" "$existing_token"; then
        release_supervisor_lock
        die "offline supervisor already running: $existing_pid"
      fi
      stale_token="$existing_token"
      if ! cleanup_persisted_reverse "$stale_token"; then
        stale_error="$monitor_reason"
        release_supervisor_lock
        die "$stale_error"
      fi
      rm -f "$pid_file"
    elif [[ -f "$owned_file" ]]; then
      release_supervisor_lock
      die "invalid supervisor state with an owned reverse; run stop after inspection"
    else
      rm -f "$pid_file"
    fi
  elif [[ -f "$owned_file" ]]; then
    read -r stale_token _ <"$owned_file" || stale_token=""
    if ! cleanup_persisted_reverse "$stale_token"; then
      stale_error="$monitor_reason"
      release_supervisor_lock
      die "$stale_error"
    fi
  fi

  IFS= read -r instance_token </proc/sys/kernel/random/uuid \
    || instance_token="$$-$RANDOM-$RANDOM"
  monitor_args=(
    __monitor
    --interval "$monitor_interval"
    --instance-token "$instance_token"
  )
  [[ -n "$serial" ]] && monitor_args+=(--serial "$serial")
  [[ -n "$device_port" ]] && monitor_args+=(--device-port "$device_port")
  [[ -n "$host_port" ]] && monitor_args+=(--host-port "$host_port")
  nohup bash "$script_path" "${monitor_args[@]}" >"$log_file" 2>&1 < /dev/null &
  supervisor_pid=$!
  for _ in {1..50}; do
    supervisor_start="$(process_start_time "$supervisor_pid" 2>/dev/null || true)"
    [[ -n "$supervisor_start" ]] && break
    sleep 0.01
  done
  if [[ -z "$supervisor_start" ]]; then
    release_supervisor_lock
    die "offline supervisor failed to start"
  fi
  printf '%s %s %s\n' "$supervisor_pid" "$supervisor_start" "$instance_token" >"$pid_file"
  release_supervisor_lock
  printf 'offline-background pid=%s log=%s\n' "$supervisor_pid" "$log_file"
}

stop_background() {
  local had_owned=0
  local stale_token=""
  acquire_supervisor_lock
  [[ -f "$owned_file" ]] && had_owned=1
  if [[ ! -f "$pid_file" ]]; then
    if ((had_owned)); then
      read -r stale_token _ <"$owned_file" || stale_token=""
      if ! cleanup_persisted_reverse "$stale_token"; then
        stale_error="$monitor_reason"
        release_supervisor_lock
        die "$stale_error"
      fi
      release_supervisor_lock
      printf 'stale-supervisor-reverse-cleaned\n'
      return 0
    fi
    release_supervisor_lock
    return 1
  fi

  if ! read_supervisor_state; then
    if ((had_owned)); then
      release_supervisor_lock
      die "invalid supervisor state with an owned reverse"
    fi
    rm -f "$pid_file"
    release_supervisor_lock
    return 1
  fi

  if ! process_matches_instance "$existing_pid" "$existing_start" "$existing_token"; then
    if ! cleanup_persisted_reverse "$existing_token"; then
      stale_error="$monitor_reason"
      release_supervisor_lock
      die "$stale_error"
    fi
    rm -f "$pid_file"
    release_supervisor_lock
    if ((had_owned)); then
      printf 'stale-supervisor-reverse-cleaned pid=%s\n' "$existing_pid"
      return 0
    fi
    return 1
  fi

  kill -TERM "$existing_pid" 2>/dev/null || true
  for _ in {1..50}; do
    process_matches_instance "$existing_pid" "$existing_start" "$existing_token" || break
    sleep 0.1
  done
  if process_matches_instance "$existing_pid" "$existing_start" "$existing_token"; then
    kill -KILL "$existing_pid" 2>/dev/null || true
    for _ in {1..20}; do
      process_matches_instance "$existing_pid" "$existing_start" "$existing_token" || break
      sleep 0.1
    done
  fi
  if process_matches_instance "$existing_pid" "$existing_start" "$existing_token"; then
    release_supervisor_lock
    die "offline supervisor did not exit: $existing_pid"
  fi
  if ! cleanup_persisted_reverse "$existing_token"; then
    cleanup_error="$monitor_reason"
    rm -f "$pid_file"
    release_supervisor_lock
    die "$cleanup_error"
  fi
  rm -f "$pid_file"
  release_supervisor_lock
  printf 'offline-supervisor-stopped pid=%s\n' "$existing_pid"
  return 0
}

if [[ "$command_name" == "__monitor" ]]; then
  [[ -n "$instance_token" ]] || die "internal monitor requires an instance token"
  trap stop_background_monitor INT TERM
  trap cleanup_monitor EXIT
  monitor_loop
  exit 0
fi

case "$command_name" in
  start)
    if ((offline)); then
      start_background
    else
      run_foreground
    fi
    ;;
  status)
    select_serial
    resolve_ports
    if ! read_reverse_list "$serial"; then
      die "$monitor_reason"
    fi
    if [[ -n "$(find_reverse "$reverse_output" "$device_port" "$host_port")" ]]; then
      printf 'reversed serial=%s host_port=%s device_port=%s\n' "$serial" "$host_port" "$device_port"
      exit 0
    fi
    printf 'not-reversed serial=%s host_port=%s device_port=%s\n' "$serial" "$host_port" "$device_port"
    exit 1
    ;;
  stop)
    if stop_background; then
      exit 0
    fi
    if ! try_select_serial; then
      printf 'PICO not connected; reverse is unavailable\n' >&2
      exit 0
    fi
    if ! try_resolve_ports; then
      printf '%s\n' "$monitor_reason" >&2
      exit 0
    fi
    remove_exact_reverse "$serial" "$device_port" "$host_port" || die "$monitor_reason"
    printf 'stopped serial=%s host_port=%s device_port=%s\n' "$serial" "$host_port" "$device_port"
    ;;
esac
