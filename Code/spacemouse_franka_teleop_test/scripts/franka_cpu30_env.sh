#!/usr/bin/env bash

FRANKA_RT_CPU="${FRANKA_RT_CPU:-2}"
FRANKA_RT_MIN_FREQ_KHZ="${FRANKA_RT_MIN_FREQ_KHZ:-2000000}"
FRANKA_RT_OTHER_CPUS="${FRANKA_RT_OTHER_CPUS:-0-1,3-31}"
FRANKA_RT_REQUIRE_ISOLATED="${FRANKA_RT_REQUIRE_ISOLATED:-1}"

_franka_cpu_list_has_cpu() {
  local list="$1"
  local cpu="$2"
  local item start end

  list="${list// /}"
  IFS=',' read -ra _franka_cpu_items <<< "$list"
  for item in "${_franka_cpu_items[@]}"; do
    if [[ "$item" == *-* ]]; then
      start="${item%-*}"
      end="${item#*-}"
      if [[ "$start" =~ ^[0-9]+$ && "$end" =~ ^[0-9]+$ ]] &&
        (( cpu >= start && cpu <= end )); then
        return 0
      fi
    elif [[ "$item" =~ ^[0-9]+$ ]] && (( cpu == item )); then
      return 0
    fi
  done
  return 1
}

franka_cpu30_prepare() {
  if [[ ! -d "/sys/devices/system/cpu/cpu${FRANKA_RT_CPU}" ]]; then
    echo "ERROR: CPU${FRANKA_RT_CPU} does not exist on this machine." >&2
    return 1
  fi

  local cpufreq_dir="/sys/devices/system/cpu/cpu${FRANKA_RT_CPU}/cpufreq"
  local governor min_freq max_freq
  governor="$(cat "${cpufreq_dir}/scaling_governor" 2>/dev/null || true)"
  min_freq="$(cat "${cpufreq_dir}/scaling_min_freq" 2>/dev/null || echo 0)"
  max_freq="$(cat "${cpufreq_dir}/cpuinfo_max_freq" 2>/dev/null || echo unknown)"

  if [[ "${governor}" != "performance" ]] ||
    { [[ "${min_freq}" =~ ^[0-9]+$ ]] && (( min_freq < FRANKA_RT_MIN_FREQ_KHZ )); }; then
    if command -v cpufreq-set >/dev/null 2>&1; then
      sudo cpufreq-set -c "${FRANKA_RT_CPU}" -g performance
      sudo cpufreq-set -c "${FRANKA_RT_CPU}" -d "${FRANKA_RT_MIN_FREQ_KHZ}kHz"
      governor="$(cat "${cpufreq_dir}/scaling_governor" 2>/dev/null || true)"
      min_freq="$(cat "${cpufreq_dir}/scaling_min_freq" 2>/dev/null || echo 0)"
      max_freq="$(cat "${cpufreq_dir}/cpuinfo_max_freq" 2>/dev/null || echo unknown)"
    else
      echo "ERROR: cpufreq-set is not installed; cannot force CPU${FRANKA_RT_CPU} performance mode." >&2
      return 1
    fi
  fi

  if [[ "${governor}" != "performance" ]]; then
    echo "ERROR: CPU${FRANKA_RT_CPU} governor is '${governor}', expected 'performance'." >&2
    return 1
  fi

  if [[ "${min_freq}" =~ ^[0-9]+$ ]] && (( min_freq < FRANKA_RT_MIN_FREQ_KHZ )); then
    echo "ERROR: CPU${FRANKA_RT_CPU} min frequency is ${min_freq} kHz, expected >= ${FRANKA_RT_MIN_FREQ_KHZ} kHz." >&2
    return 1
  fi

  echo "CPU${FRANKA_RT_CPU}: governor=${governor}, min=${min_freq} kHz, max=${max_freq} kHz"

  local isolated=""
  if [[ -r /sys/devices/system/cpu/isolated ]]; then
    isolated="$(cat /sys/devices/system/cpu/isolated)"
  fi
  if [[ -z "${isolated}" ]] || ! _franka_cpu_list_has_cpu "${isolated}" "${FRANKA_RT_CPU}"; then
    echo "ERROR: CPU${FRANKA_RT_CPU} is not listed in /sys/devices/system/cpu/isolated." >&2
    echo "ERROR: refusing to start Franka control without CPU isolation; taskset alone cannot prevent other work/IRQs from using CPU${FRANKA_RT_CPU}." >&2
    echo "ERROR: configure kernel isolation first, or set FRANKA_RT_REQUIRE_ISOLATED=0 to bypass this safety check." >&2
    if [[ "${FRANKA_RT_REQUIRE_ISOLATED}" != "0" ]]; then
      return 1
    fi
  fi
}

franka_cpu30_exec() {
  franka_cpu30_prepare
  exec taskset -c "${FRANKA_RT_CPU}" "$@"
}
