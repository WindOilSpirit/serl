#!/usr/bin/env bash
set -eo pipefail

CPU="${FRANKA_RT_CPU:-2}"
CPUFREQ_DIR="/sys/devices/system/cpu/cpu${CPU}/cpufreq"

echo "CPU${CPU} frequency state:"
if [[ -d "${CPUFREQ_DIR}" ]]; then
  echo "  governor: $(cat "${CPUFREQ_DIR}/scaling_governor" 2>/dev/null || echo unknown)"
  echo "  min kHz:  $(cat "${CPUFREQ_DIR}/scaling_min_freq" 2>/dev/null || echo unknown)"
  echo "  max kHz:  $(cat "${CPUFREQ_DIR}/cpuinfo_max_freq" 2>/dev/null || echo unknown)"
  echo "  cur kHz:  $(cat "${CPUFREQ_DIR}/scaling_cur_freq" 2>/dev/null || echo unknown)"
else
  echo "  ERROR: ${CPUFREQ_DIR} not found"
fi

echo
echo "CPU isolation:"
if [[ -r /sys/devices/system/cpu/isolated ]]; then
  echo "  isolated: $(cat /sys/devices/system/cpu/isolated)"
else
  echo "  isolated: unavailable"
fi

echo
echo "Franka/teleop threads and their last CPU (PSR):"
mapfile -t PIDS < <(
  pgrep -f 'ros2_control_node|franka.*launch.py|serl_cartesian_impedance_controller|spacemouse_franka_teleop.launch.py|/spacemouse_franka_teleop_test/teleop_node' |
    grep -v "^$$$" || true
)

if [[ "${#PIDS[@]}" -eq 0 ]]; then
  echo "  no matching Franka/teleop processes found"
else
  ps -eLo pid,tid,psr,rtprio,cls,comm,args |
    awk -v cpu="${CPU}" -v pids=" ${PIDS[*]} " '
      index(pids, " " $1 " ") {
        print;
        if ($3 != cpu) bad = 1;
        found = 1;
      }
      END {
        if (found && bad) {
          print "WARNING: at least one matching thread was last observed outside CPU" cpu;
        }
      }'
fi
