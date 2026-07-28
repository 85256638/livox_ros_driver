#!/usr/bin/env bash
# Install the relay safety hooks into the existing livox-ros-driver.service.
#
# The ROS power-cycle manager itself is started conditionally by the Driver
# launch file.  This installer deliberately does not start/restart the Driver
# and never changes an existing site's arming state.  The systemd hooks remain
# active even when the launch switch is disabled: a persisted must-be-ON
# obligation must always be repaired before start and after stop, without
# parsing the site JSON or requiring ROS/catkin to be available.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
CONFIG_DIR="${HOME}/.config/livox"
CONFIG_FILE="${CONFIG_DIR}/power_cycle.json"
STATE_DIR="${HOME}/.local/state/livox-power-cycle-manager"
STATE_DB="${STATE_DIR}/state.sqlite3"
EXAMPLE_FILE="${SCRIPT_DIR}/livox_ros_driver/config/livox_power_cycle.example.json"
MANAGER_SOURCE="${SCRIPT_DIR}/livox_ros_driver/livox_ros_driver/scripts/livox_power_cycle_manager.py"
MANAGER_RUNTIME_DIR="${HOME}/.local/libexec/livox-power-cycle-manager"
MANAGER_RUNTIME="${MANAGER_RUNTIME_DIR}/livox_power_cycle_manager.py"
SITE_VALIDATOR="${SCRIPT_DIR}/livox_ros_driver/livox_ros_driver/scripts/validate_livox_power_cycle_site.py"
DRIVER_CONFIG="${SCRIPT_DIR}/livox_ros_driver/config/livox_lidar_config_multi.json"
DRIVER_LAUNCH="${SCRIPT_DIR}/livox_ros_driver/launch/livox_lidar_multi.launch"
TEMPLATE_FILE="${SCRIPT_DIR}/systemd/livox-ros-driver-power-cycle.conf.in"
DRIVER_UNIT_NAME="livox-ros-driver.service"
LEGACY_UNIT_NAME="livox-power-cycle-manager.service"
LEGACY_UNIT_PATH="/etc/systemd/system/${LEGACY_UNIT_NAME}"
DROPIN_DIR="/etc/systemd/system/${DRIVER_UNIT_NAME}.d"
DROPIN_NAME="20-livox-power-cycle-safety.conf"
DROPIN_PATH="${DROPIN_DIR}/${DROPIN_NAME}"
LEGACY_WAS_ACTIVE=0
LEGACY_STOPPED_BY_INSTALLER=0
LEGACY_MIGRATION_COMPLETE=0
RENDERED_FILE=""
DROPIN_ROLLBACK_PATH=""
RUNTIME_TEMP=""

log() { printf '[livox-power-cycle-install] %s\n' "$*"; }
die() { log "ERROR: $*" >&2; exit 1; }

cleanup() {
  local exit_status=$?
  set +e
  if [[ -n "${RENDERED_FILE}" ]]; then
    rm -f -- "${RENDERED_FILE}"
  fi
  if [[ -n "${RUNTIME_TEMP}" ]]; then
    rm -f -- "${RUNTIME_TEMP}"
  fi
  if [[ -n "${DROPIN_ROLLBACK_PATH}" &&
        ( -e "${DROPIN_ROLLBACK_PATH}" || -L "${DROPIN_ROLLBACK_PATH}" ) &&
        ! -e "${DROPIN_PATH}" && ! -L "${DROPIN_PATH}" ]]; then
    log "卸载未完成，正在原子恢复 Driver 安全 drop-in。"
    if sudo mv -- "${DROPIN_ROLLBACK_PATH}" "${DROPIN_PATH}"; then
      sudo systemctl daemon-reload ||
        log "CRITICAL: drop-in 文件已恢复，但 systemd daemon-reload 失败，请立即人工检查。"
    else
      log "CRITICAL: 无法恢复 ${DROPIN_PATH}，请立即人工检查 ${DROPIN_ROLLBACK_PATH}。"
    fi
  fi
  if ((exit_status != 0 && LEGACY_WAS_ACTIVE && \
       LEGACY_STOPPED_BY_INSTALLER && !LEGACY_MIGRATION_COMPLETE)); then
    log "安装失败，正在恢复迁移前运行的旧 ${LEGACY_UNIT_NAME}，使其继续重试历史补 ON。"
    if ! sudo systemctl start "${LEGACY_UNIT_NAME}"; then
      log "CRITICAL: 旧 ${LEGACY_UNIT_NAME} 恢复启动失败；unit 文件仍保留，请立即人工检查。"
    fi
  fi
  trap - EXIT
  exit "${exit_status}"
}
trap cleanup EXIT

unit_load_state() {
  local listing=""
  local value=""
  if value="$(systemctl show --property=LoadState --value "$1" 2>/dev/null)"; then
    [[ -n "${value}" ]] && printf '%s\n' "${value}" || printf 'unknown\n'
  elif listing="$(systemctl list-unit-files --no-legend --no-pager "$1" 2>/dev/null)" &&
       [[ -z "${listing//[[:space:]]/}" ]]; then
    printf 'not-found\n'
  else
    printf 'unknown\n'
  fi
}

unit_active_state() {
  local listing=""
  local value=""
  if value="$(systemctl show --property=ActiveState --value "$1" 2>/dev/null)"; then
    [[ -n "${value}" ]] && printf '%s\n' "${value}" || printf 'unknown\n'
  elif listing="$(systemctl list-unit-files --no-legend --no-pager "$1" 2>/dev/null)" &&
       [[ -z "${listing//[[:space:]]/}" ]]; then
    printf 'inactive\n'
  else
    printf 'unknown\n'
  fi
}

unit_enable_state() {
  local listed_name=""
  local listed_state=""
  local listing=""
  local value=""
  if value="$(systemctl is-enabled "$1" 2>/dev/null)"; then
    :
  fi
  case "${value}" in
    enabled|enabled-runtime|linked|linked-runtime|alias|masked|masked-runtime|static|indirect|disabled|generated|transient|not-found)
      printf '%s\n' "${value}"
      ;;
    "")
      if ! listing="$(systemctl list-unit-files --no-legend --no-pager "$1" 2>/dev/null)"; then
        printf 'unknown\n'
      elif [[ -z "${listing//[[:space:]]/}" ]]; then
        printf 'not-found\n'
      elif [[ "${listing}" != *$'\n'* ]]; then
        IFS=$' \t' read -r listed_name listed_state _ <<<"${listing}"
        if [[ "${listed_name}" == "$1" && -n "${listed_state}" ]]; then
          printf '%s\n' "${listed_state}"
        else
          printf 'unknown\n'
        fi
      else
        printf 'unknown\n'
      fi
      ;;
    *)
      printf 'unknown\n'
      ;;
  esac
}

repair_obligations() {
  local helper="${MANAGER_RUNTIME}"
  [[ -f "${helper}" && ! -L "${helper}" ]] || helper="${MANAGER_SOURCE}"
  /usr/bin/python3 "${helper}" \
    --state-db "${STATE_DB}" --repair-obligations
}

for command_name in sudo systemctl sed install mktemp id mv tr cmp; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    die "缺少命令：${command_name}"
done
[[ -x /usr/bin/python3 ]] || die "找不到可执行文件：/usr/bin/python3"
[[ ${EUID} -ne 0 ]] || die "请使用普通用户运行；脚本仅通过 sudo 安装 systemd drop-in。"
[[ -f "${MANAGER_SOURCE}" ]] || die "找不到 relay manager 源文件：${MANAGER_SOURCE}"
[[ -f "${SITE_VALIDATOR}" ]] || die "找不到现场身份校验器：${SITE_VALIDATOR}"

for path in "${HOME}" "${CONFIG_FILE}" "${STATE_DIR}" "${STATE_DB}" \
  "${SCRIPT_DIR}" "${MANAGER_SOURCE}" "${MANAGER_RUNTIME_DIR}" \
  "${MANAGER_RUNTIME}" "${DROPIN_PATH}"; do
  [[ "${path}" != *[$'\n\r\t ']* ]] ||
    die "工业服务路径不能包含空白字符：${path}"
  [[ "${path}" =~ ^/[A-Za-z0-9._/-]+$ ]] ||
    die "工业服务路径含有不安全字符：${path}"
done

if [[ "${1:-}" == "--uninstall" ]]; then
  [[ $# -eq 1 ]] ||
    die "用法：bash install_livox_power_cycle_service.sh [--uninstall]"
  DRIVER_ACTIVE_STATE="$(unit_active_state "${DRIVER_UNIT_NAME}")"
  case "${DRIVER_ACTIVE_STATE}" in
    inactive|failed) ;;
    *)
      die "${DRIVER_UNIT_NAME} 当前为 ${DRIVER_ACTIVE_STATE}；为防 manager 在移除钩子后继续/重新启动，drop-in 已保留。请先安全停止 Driver，再重试卸载。"
      ;;
  esac
  log "移除安全 drop-in 前，先独立确认 SQLite 中所有历史通道均为 ON。"
  repair_obligations ||
    die "补上电失败；drop-in 已保留。若集成 manager 正在运行，请先安全停止 ${DRIVER_UNIT_NAME} 后重试。"
  if [[ -e "${DROPIN_PATH}" || -L "${DROPIN_PATH}" ]]; then
    DROPIN_ROLLBACK_PATH="${DROPIN_PATH}.uninstall-rollback.$$"
    sudo mv -- "${DROPIN_PATH}" "${DROPIN_ROLLBACK_PATH}"
    sudo systemctl daemon-reload ||
      die "daemon-reload 失败；退出清理会恢复原安全 drop-in。"
    if ! sudo rm -f -- "${DROPIN_ROLLBACK_PATH}"; then
      die "无法清理卸载回滚副本；退出清理会恢复原安全 drop-in。"
    fi
    DROPIN_ROLLBACK_PATH=""
    log "已移除 ${DROPIN_PATH}；未停止或重启 Driver。"
  else
    log "安全 drop-in 不存在，无需删除。"
  fi
  log "现场配置和 SQLite 审计记录均保留：${CONFIG_FILE}；${STATE_DB}"
  exit 0
fi
[[ $# -eq 0 ]] ||
  die "用法：bash install_livox_power_cycle_service.sh [--uninstall]"

[[ -f "${EXAMPLE_FILE}" ]] || die "找不到配置模板：${EXAMPLE_FILE}"
[[ -f "${TEMPLATE_FILE}" ]] || die "找不到 systemd drop-in 模板：${TEMPLATE_FILE}"

mkdir -p -- "${CONFIG_DIR}" "${STATE_DIR}"
chmod 700 "${CONFIG_DIR}" "${STATE_DIR}"
if [[ -e "${MANAGER_RUNTIME_DIR}" || -L "${MANAGER_RUNTIME_DIR}" ]]; then
  [[ -d "${MANAGER_RUNTIME_DIR}" && ! -L "${MANAGER_RUNTIME_DIR}" ]] ||
    die "稳定 helper 目录不是普通目录或是符号链接：${MANAGER_RUNTIME_DIR}"
fi
install -d -m 700 "${MANAGER_RUNTIME_DIR}"
RUNTIME_TEMP="$(mktemp "${MANAGER_RUNTIME}.new.XXXXXX")"
install -m 700 "${MANAGER_SOURCE}" "${RUNTIME_TEMP}"
cmp -s -- "${MANAGER_SOURCE}" "${RUNTIME_TEMP}" ||
  die "稳定 helper 临时副本校验失败：${RUNTIME_TEMP}"
mv -f -- "${RUNTIME_TEMP}" "${MANAGER_RUNTIME}"
RUNTIME_TEMP=""
[[ -f "${MANAGER_RUNTIME}" && ! -L "${MANAGER_RUNTIME}" ]] ||
  die "稳定 helper 安装结果无效：${MANAGER_RUNTIME}"
if [[ ! -e "${CONFIG_FILE}" ]]; then
  install -m 600 "${EXAMPLE_FILE}" "${CONFIG_FILE}"
  log "已创建安全配置（所有 power group 默认 disabled）：${CONFIG_FILE}"
else
  [[ -f "${CONFIG_FILE}" && ! -L "${CONFIG_FILE}" ]] ||
    die "现有配置不是普通文件或是符号链接，拒绝使用：${CONFIG_FILE}"
  log "保留现有配置，不覆盖、不改变 arming 状态：${CONFIG_FILE}"
fi
chmod 600 "${CONFIG_FILE}"

/usr/bin/python3 "${MANAGER_SOURCE}" \
  --config "${CONFIG_FILE}" --validate-config
/usr/bin/python3 "${SITE_VALIDATOR}" \
  --relay-config "${CONFIG_FILE}" \
  --driver-config "${DRIVER_CONFIG}" \
  --launch "${DRIVER_LAUNCH}"
CONFIG_STATE_DB="$(/usr/bin/python3 -c 'import json,os,sys; d=json.load(open(sys.argv[1], encoding="utf-8")); p=d.get("state_db", "~/.local/state/livox-power-cycle-manager/state.sqlite3"); print(os.path.abspath(os.path.expandvars(os.path.expanduser(p.strip()))))' "${CONFIG_FILE}")"
[[ "${CONFIG_STATE_DB}" == "${STATE_DB}" ]] ||
  die "生产安装只允许 state_db=${STATE_DB}；当前配置解析为 ${CONFIG_STATE_DB}。请先迁移并核对旧库中的补上电义务。"
CONFIG_MODE="$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("mode", "unset"))' "${CONFIG_FILE}")"
CONFIG_ENABLED_GROUPS="$(/usr/bin/python3 -c 'import json,sys; d=json.load(open(sys.argv[1], encoding="utf-8")); print(sum(1 for row in d.get("power_groups", {}).values() if row.get("enabled") is True))' "${CONFIG_FILE}")"
log "现场配置保持原状：legacy_mode=${CONFIG_MODE}（launch 才是唯一硬件授权），enabled_groups=${CONFIG_ENABLED_GROUPS}。本脚本不会启动 manager 或重启 Driver。"

sudo systemctl daemon-reload
DRIVER_LOAD_STATE="$(unit_load_state "${DRIVER_UNIT_NAME}")"
[[ "${DRIVER_LOAD_STATE}" == "loaded" ]] ||
  die "${DRIVER_UNIT_NAME} 当前 LoadState=${DRIVER_LOAD_STATE}；拒绝移除旧安全服务。请先正确安装 Driver systemd unit。"
CURRENT_USER="$(id -un)"
DRIVER_USER="$(systemctl show --property=User --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的运行用户。"
[[ "${DRIVER_USER}" == "${CURRENT_USER}" ]] ||
  die "${DRIVER_UNIT_NAME} 的 User=${DRIVER_USER:-root}，但当前安装用户为 ${CURRENT_USER}；为避免状态库和端点锁分裂，拒绝安装。"
DRIVER_RESTART_POLICY="$(systemctl show --property=Restart --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的 Restart 策略。"
[[ "${DRIVER_RESTART_POLICY}" == "always" ]] ||
  die "${DRIVER_UNIT_NAME} 的 Restart=${DRIVER_RESTART_POLICY:-unset}；Driver 节点仍使用 required=true，生产 unit 必须是 Restart=always 才能在 Driver 退出时整套自恢复。"
DRIVER_KILL_MODE="$(systemctl show --property=KillMode --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的 KillMode。"
[[ "${DRIVER_KILL_MODE}" == "control-group" ]] ||
  die "${DRIVER_UNIT_NAME} 的 KillMode=${DRIVER_KILL_MODE:-unset}；必须是 control-group，确保停止/重启时 Driver 与集成 manager 一起退出，随后才能无锁执行补 ON。"
DRIVER_SERVICE_TYPE="$(systemctl show --property=Type --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的 Type。"
[[ "${DRIVER_SERVICE_TYPE}" == "simple" ]] ||
  die "${DRIVER_UNIT_NAME} 的 Type=${DRIVER_SERVICE_TYPE:-unset}；集成版要求 Type=simple，让 roslaunch 退出能够成为明确的服务失败/重启边界。"
DRIVER_REMAIN_AFTER_EXIT="$(systemctl show --property=RemainAfterExit --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的 RemainAfterExit。"
[[ "${DRIVER_REMAIN_AFTER_EXIT}" == "no" ]] ||
  die "${DRIVER_UNIT_NAME} 的 RemainAfterExit=${DRIVER_REMAIN_AFTER_EXIT:-unset}；必须是 no，禁止 roslaunch/manager 退出后 unit 仍伪装 active。"

escape_sed() { printf '%s' "$1" | sed 's/[\\&|]/\\&/g'; }
rendered="$(mktemp "${TMPDIR:-/tmp}/livox-driver-power-cycle-dropin.XXXXXX")"
RENDERED_FILE="${rendered}"
sed \
  -e "s|@HOME@|$(escape_sed "${HOME}")|g" \
  -e "s|@MANAGER_RUNTIME@|$(escape_sed "${MANAGER_RUNTIME}")|g" \
  -e "s|@STATE_DIR@|$(escape_sed "${STATE_DIR}")|g" \
  -e "s|@STATE_DB@|$(escape_sed "${STATE_DB}")|g" \
  "${TEMPLATE_FILE}" >"${rendered}"

# Establish and verify the replacement safety boundary before touching the
# legacy unit.  Installing a drop-in does not start the launch manager, so it
# is safe for both active and inactive legacy deployments.
sudo install -d -m 755 "${DROPIN_DIR}"
sudo install -m 644 "${rendered}" "${DROPIN_PATH}"
sudo systemctl daemon-reload
DRIVER_DROPIN_PATHS="$(systemctl show --property=DropInPaths --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的 DropInPaths；安全 drop-in 文件已保留但安装未确认。"
[[ " ${DRIVER_DROPIN_PATHS} " == *" ${DROPIN_PATH} "* ]] ||
  die "systemd 未确认加载 ${DROPIN_PATH}；拒绝迁移旧 unit。"
EXPECTED_REPAIR_COMMAND="/usr/bin/python3 ${MANAGER_RUNTIME} --state-db ${STATE_DB} --repair-obligations"
DRIVER_EXEC_START_PRE="$(systemctl show --property=ExecStartPre --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的有效 ExecStartPre。"
DRIVER_EXEC_STOP_POST="$(systemctl show --property=ExecStopPost --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的有效 ExecStopPost。"
[[ "${DRIVER_EXEC_START_PRE}" == *"${EXPECTED_REPAIR_COMMAND}"* ]] ||
  die "systemd 的有效 ExecStartPre 未包含固定补 ON 命令；拒绝迁移旧 unit。"
[[ "${DRIVER_EXEC_STOP_POST}" == *"${EXPECTED_REPAIR_COMMAND}"* ]] ||
  die "systemd 的有效 ExecStopPost 未包含固定补 ON 命令；拒绝迁移旧 unit。"
DRIVER_SEND_SIGKILL="$(systemctl show --property=SendSIGKILL --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的有效 SendSIGKILL。"
[[ "${DRIVER_SEND_SIGKILL}" == "yes" ]] ||
  die "${DRIVER_UNIT_NAME} 的有效 SendSIGKILL=${DRIVER_SEND_SIGKILL:-unset}；必须是 yes，确保停止超时后旧 manager 不会存活并继续持锁。"
DRIVER_EFFECTIVE_ENVIRONMENT="$(systemctl show --property=Environment --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的有效 Environment。"
DRIVER_HOME_ENTRIES=0
while IFS= read -r environment_token; do
  if [[ "${environment_token}" == HOME=* ]]; then
    ((DRIVER_HOME_ENTRIES += 1))
    [[ "${environment_token}" == "HOME=${HOME}" ]] ||
      die "${DRIVER_UNIT_NAME} 的有效 ${environment_token} 与安装用户 HOME=${HOME} 不一致。"
  fi
done < <(printf '%s\n' "${DRIVER_EFFECTIVE_ENVIRONMENT}" | tr ' ' '\n')
[[ ${DRIVER_HOME_ENTRIES} -eq 1 ]] ||
  die "${DRIVER_UNIT_NAME} 必须且只能有一个有效 HOME=${HOME}；当前 Environment=${DRIVER_EFFECTIVE_ENVIRONMENT:-unset}。"
DRIVER_TIMEOUT_START="$(systemctl show --property=TimeoutStartUSec --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的有效 TimeoutStartUSec。"
DRIVER_TIMEOUT_STOP="$(systemctl show --property=TimeoutStopUSec --value "${DRIVER_UNIT_NAME}")" ||
  die "无法读取 ${DRIVER_UNIT_NAME} 的有效 TimeoutStopUSec。"
case "${DRIVER_TIMEOUT_START}" in
  10min|600s|600000000us|600000000) ;;
  *) die "${DRIVER_UNIT_NAME} 的有效 TimeoutStartUSec=${DRIVER_TIMEOUT_START:-unset}，预期 600 秒。" ;;
esac
case "${DRIVER_TIMEOUT_STOP}" in
  5min|300s|300000000us|300000000) ;;
  *) die "${DRIVER_UNIT_NAME} 的有效 TimeoutStopUSec=${DRIVER_TIMEOUT_STOP:-unset}，预期 300 秒。" ;;
esac

LEGACY_LOAD_STATE="$(unit_load_state "${LEGACY_UNIT_NAME}")"
LEGACY_INITIAL_ACTIVE_STATE="$(unit_active_state "${LEGACY_UNIT_NAME}")"
LEGACY_ENABLE_STATE="$(unit_enable_state "${LEGACY_UNIT_NAME}")"
if [[ "${LEGACY_LOAD_STATE}" == "unknown" ||
      "${LEGACY_INITIAL_ACTIVE_STATE}" == "unknown" ||
      "${LEGACY_ENABLE_STATE}" == "unknown" ]]; then
  die "无法可靠判断旧 ${LEGACY_UNIT_NAME}（load=${LEGACY_LOAD_STATE}, active=${LEGACY_INITIAL_ACTIVE_STATE}, enabled=${LEGACY_ENABLE_STATE}）；禁止迁移。"
fi
if [[ "${LEGACY_LOAD_STATE}" != "not-found" ||
      "${LEGACY_INITIAL_ACTIVE_STATE}" != "inactive" ||
      "${LEGACY_ENABLE_STATE}" != "not-found" ]]; then
  case "${LEGACY_INITIAL_ACTIVE_STATE}" in
    active|activating|reloading|deactivating) LEGACY_WAS_ACTIVE=1 ;;
  esac
  log "安全停止旧 ${LEGACY_UNIT_NAME}；若正在 OFF，其 shutdown/ExecStopPost 会先尝试恢复 ON。"
  if ! sudo systemctl stop "${LEGACY_UNIT_NAME}"; then
    # ExecStopPost returning non-zero also makes `systemctl stop` fail even
    # when the old process is already gone.  Do not delete anything yet, but
    # still allow the independent repair below to recover that exact case.
    log "WARNING: 旧服务 stop 返回失败；将先确认进程已退出，再独立补 ON。"
  fi
  LEGACY_ACTIVE_STATE="$(unit_active_state "${LEGACY_UNIT_NAME}")"
  case "${LEGACY_ACTIVE_STATE}" in
    inactive|failed) ;;
    *) die "旧服务 stop 后仍处于 ${LEGACY_ACTIVE_STATE}；禁止迁移。" ;;
  esac
  if ((LEGACY_WAS_ACTIVE)); then
    LEGACY_STOPPED_BY_INSTALLER=1
  fi
else
  log "旧 ${LEGACY_UNIT_NAME} 已严格确认不存在且未启用。"
fi

log "独立执行一次与 JSON/ROS 无关的 must-be-ON 修复。"
repair_obligations ||
  die "独立补上电失败；旧 unit 尚未删除，已安装的 Driver drop-in 不会自行启动 manager。若集成 manager 正在运行，请先安全停止 ${DRIVER_UNIT_NAME} 后重试。"

if [[ "${LEGACY_LOAD_STATE}" != "not-found" ||
      "${LEGACY_INITIAL_ACTIVE_STATE}" != "inactive" ||
      "${LEGACY_ENABLE_STATE}" != "not-found" ]]; then
  LEGACY_ACTIVE_STATE="$(unit_active_state "${LEGACY_UNIT_NAME}")"
  if [[ "${LEGACY_ACTIVE_STATE}" == "failed" ]]; then
    sudo systemctl reset-failed "${LEGACY_UNIT_NAME}"
    LEGACY_ACTIVE_STATE="$(unit_active_state "${LEGACY_UNIT_NAME}")"
  fi
  [[ "${LEGACY_ACTIVE_STATE}" == "inactive" ]] ||
    die "旧服务仍处于 ${LEGACY_ACTIVE_STATE}；禁止安装集成版。"
  sudo systemctl disable "${LEGACY_UNIT_NAME}" ||
    die "旧服务未能禁用；禁止安装集成版。"
  if [[ -e "${LEGACY_UNIT_PATH}" || -L "${LEGACY_UNIT_PATH}" ]]; then
    sudo rm -f -- "${LEGACY_UNIT_PATH}"
  fi
  sudo systemctl daemon-reload
  LEGACY_LOAD_STATE="$(unit_load_state "${LEGACY_UNIT_NAME}")"
  LEGACY_ACTIVE_STATE="$(unit_active_state "${LEGACY_UNIT_NAME}")"
  LEGACY_ENABLE_STATE="$(unit_enable_state "${LEGACY_UNIT_NAME}")"
  [[ "${LEGACY_LOAD_STATE}" == "not-found" &&
     "${LEGACY_ACTIVE_STATE}" == "inactive" &&
     "${LEGACY_ENABLE_STATE}" == "not-found" ]] ||
    die "旧 ${LEGACY_UNIT_NAME} 未能严格清除（load=${LEGACY_LOAD_STATE}, active=${LEGACY_ACTIVE_STATE}, enabled=${LEGACY_ENABLE_STATE}）；已保持停止，但拒绝报告迁移成功。"
  LEGACY_MIGRATION_COMPLETE=1
  log "旧独立 manager unit 已停止、补 ON、禁用并删除。"
fi

log "已安装 Driver 安全 drop-in：${DROPIN_PATH}"
log "稳定补 ON helper 已原子同步：${MANAGER_RUNTIME}"
log "ExecStartPre/ExecStopPost 将使用稳定 helper 和固定 SQLite 独立补 ON；TimeoutStartSec=600，TimeoutStopSec=300。"
log "未修改 launch 开关、未自动 armed、未停止或重启 ${DRIVER_UNIT_NAME}。"
log "请在完成 relay JSON、共享通道和电气验收后，再由维护人员决定何时重启 Driver 并启用 launch 开关。"
