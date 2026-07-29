#!/usr/bin/env bash
# Validate the production Livox/relay site configuration without exposing the
# nested catkin package source layout to operators.

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PACKAGE_DIR="${SCRIPT_DIR}/livox_ros_driver"
MANAGER="${PACKAGE_DIR}/livox_ros_driver/scripts/livox_power_cycle_manager.py"
SITE_VALIDATOR="${PACKAGE_DIR}/livox_ros_driver/scripts/validate_livox_power_cycle_site.py"
DRIVER_CONFIG="${PACKAGE_DIR}/config/livox_lidar_config_multi.json"
SITE_LAUNCH="${PACKAGE_DIR}/launch/livox_lidar_multi.launch"
CHILD_LAUNCH="${PACKAGE_DIR}/launch/livox_power_cycle.launch"
EXAMPLE_CONFIG="${PACKAGE_DIR}/config/livox_power_cycle.example.json"
MISPLACED_CONFIG="${PACKAGE_DIR}/config/livox_power_cycle.json"
RELAY_CONFIG="${HOME}/.config/livox/power_cycle.json"
CHECK_RELAYS=0

usage() {
  cat <<'EOF'
Usage: bash validate_livox_site.sh [--check-relays]

Without options, validate JSON syntax/safety and the Driver/relay/launch site
identity without ROS or network access.  --check-relays performs those checks
first, then sends one read-only B0 status query to each enabled relay group.
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

case "${1:-}" in
  "") ;;
  --check-relays) CHECK_RELAYS=1 ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    die "unknown argument: $1"
    ;;
esac
[[ $# -le 1 ]] || die "too many arguments"
[[ ${EUID} -ne 0 ]] || die "run this command as the normal workstation user, not root"
command -v python3 >/dev/null 2>&1 || die "python3 is not installed"

PREFLIGHT_FAILURE=0
for required in \
  "${MANAGER}" \
  "${SITE_VALIDATOR}" \
  "${DRIVER_CONFIG}" \
  "${SITE_LAUNCH}" \
  "${CHILD_LAUNCH}" \
  "${EXAMPLE_CONFIG}"; do
  if [[ ! -f "${required}" || -L "${required}" ]]; then
    printf 'ERROR: required new-Driver file is missing or is a symlink: %s\n' \
      "${required}" >&2
    PREFLIGHT_FAILURE=1
  fi
done

if [[ -e "${MISPLACED_CONFIG}" || -L "${MISPLACED_CONFIG}" ]]; then
  printf 'ERROR: production relay config is misplaced inside the Git repository: %s; use %s\n' \
    "${MISPLACED_CONFIG}" "${RELAY_CONFIG}" >&2
  PREFLIGHT_FAILURE=1
fi
if [[ ! -f "${RELAY_CONFIG}" || -L "${RELAY_CONFIG}" ]]; then
  printf 'ERROR: production relay config is missing or is a symlink: %s; run install_livox_power_cycle_service.sh after the staged build\n' \
    "${RELAY_CONFIG}" >&2
  PREFLIGHT_FAILURE=1
fi
((PREFLIGHT_FAILURE == 0)) || exit 2

printf 'Validating relay config: %s\n' "${RELAY_CONFIG}"
python3 "${MANAGER}" --config "${RELAY_CONFIG}" --validate-config
printf 'Validating Driver whitelist and launch identity\n'
python3 "${SITE_VALIDATOR}" \
  --relay-config "${RELAY_CONFIG}" \
  --driver-config "${DRIVER_CONFIG}" \
  --launch "${SITE_LAUNCH}"

if ((CHECK_RELAYS)); then
  printf 'Querying enabled relay groups (read-only; no output state is changed)\n'
  python3 "${MANAGER}" --config "${RELAY_CONFIG}" --check-relays
fi
