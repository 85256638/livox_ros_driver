#!/usr/bin/env bash
# Install the independent Livox -> CORX relay manager as a hardened systemd
# service.  The generated configuration starts in observe mode and therefore
# cannot initiate a new OFF. A persisted must-be-ON repair remains mandatory.

set -Eeuo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
CATKIN_WS="${CATKIN_WS:-${HOME}/catkin_ws}"
CONFIG_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/livox"
CONFIG_FILE="${LIVOX_POWER_CYCLE_CONFIG:-${CONFIG_DIR}/power_cycle.json}"
STATE_DIR="${HOME}/.local/state/livox-power-cycle-manager"
STATE_DB="${STATE_DIR}/state.sqlite3"
EXAMPLE_FILE="${SCRIPT_DIR}/livox_ros_driver/config/livox_power_cycle.example.json"
TEMPLATE_FILE="${SCRIPT_DIR}/systemd/livox-power-cycle-manager.service.in"
UNIT_NAME="livox-power-cycle-manager.service"
UNIT_PATH="/etc/systemd/system/${UNIT_NAME}"

log() { printf '[livox-power-cycle-install] %s\n' "$*"; }
die() { log "ERROR: $*" >&2; exit 1; }

if [[ "${1:-}" == "--uninstall" ]]; then
  command -v python3 >/dev/null 2>&1 || die "缺少命令：python3"
  command -v sudo >/dev/null 2>&1 || die "缺少命令：sudo"
  command -v systemctl >/dev/null 2>&1 || die "缺少命令：systemctl"
  if [[ -e "${UNIT_PATH}" ]]; then
    log "先停止服务；停机后的 ExecStopPost 必须成功确认所有历史通道 ON。"
    sudo systemctl stop "${UNIT_NAME}" ||
      die "服务未能安全停止；unit 已保留，禁止卸载。"
    UNIT_ACTIVE_STATE="$(systemctl show --property=ActiveState --value "${UNIT_NAME}")" ||
      die "无法确认服务状态；unit 已保留，禁止卸载。"
    case "${UNIT_ACTIVE_STATE}" in
      inactive|failed) ;;
      *) die "服务仍处于 ${UNIT_ACTIVE_STATE}；unit 已保留，禁止卸载。" ;;
    esac
  fi
  log "卸载前再次独立确认 SQLite 中所有 must-be-ON obligation 已恢复。"
  python3 "${SCRIPT_DIR}/livox_ros_driver/livox_ros_driver/scripts/livox_power_cycle_manager.py" \
    --state-db "${STATE_DB}" --repair-obligations ||
    die "独立补上电失败；unit 已保留，禁止卸载。"
  if [[ -e "${UNIT_PATH}" ]]; then
    sudo systemctl disable "${UNIT_NAME}" ||
      die "服务未能禁用；unit 已保留，禁止卸载。"
    sudo rm -f -- "${UNIT_PATH}"
    sudo systemctl daemon-reload
  fi
  log "服务已安全移除；配置和 SQLite 审计记录均保留。"
  exit 0
fi
[[ $# -eq 0 ]] || die "用法：bash install_livox_power_cycle_service.sh [--uninstall]"
[[ ${EUID} -ne 0 ]] || die "请使用普通用户运行；脚本仅通过 sudo 安装 systemd unit。"
for command_name in python3 sudo systemctl sed install mktemp; do
  command -v "${command_name}" >/dev/null 2>&1 || die "缺少命令：${command_name}"
done
[[ -f "${EXAMPLE_FILE}" ]] || die "找不到配置模板：${EXAMPLE_FILE}"
[[ -f "${TEMPLATE_FILE}" ]] || die "找不到 systemd 模板：${TEMPLATE_FILE}"
[[ -f "/opt/ros/noetic/setup.bash" ]] || die "找不到 ROS Noetic。"
[[ -f "${CATKIN_WS}/devel/setup.bash" ]] || die "请先成功编译 catkin workspace。"
[[ -x "${CATKIN_WS}/devel/lib/livox_ros_driver/livox_power_cycle_manager.py" ]] ||
  die "找不到已编译的 livox_power_cycle_manager.py；请先运行 catkin_make。"
for path in "${HOME}" "${CATKIN_WS}" "${CONFIG_FILE}" "${STATE_DIR}" "${SCRIPT_DIR}"; do
  [[ "${path}" != *[$'\n\r\t ']* ]] || die "工业服务路径不能包含空白字符：${path}"
  [[ "${path}" =~ ^/[A-Za-z0-9._/-]+$ ]] ||
    die "工业服务路径含有不安全字符：${path}"
done

mkdir -p -- "${CONFIG_DIR}" "${STATE_DIR}" "${STATE_DIR}/ros" "${STATE_DIR}/ros-log"
chmod 700 "${CONFIG_DIR}" "${STATE_DIR}"
chmod 700 "${STATE_DIR}/ros" "${STATE_DIR}/ros-log"
if [[ ! -e "${CONFIG_FILE}" ]]; then
  install -m 600 "${EXAMPLE_FILE}" "${CONFIG_FILE}"
  log "已创建安全配置（mode=observe，不会发起新 OFF；历史补上电仍优先）：${CONFIG_FILE}"
else
  log "保留现有配置，不覆盖：${CONFIG_FILE}"
fi
chmod 600 "${CONFIG_FILE}"
python3 "${SCRIPT_DIR}/livox_ros_driver/livox_ros_driver/scripts/livox_power_cycle_manager.py" \
  --config "${CONFIG_FILE}" --validate-config
CONFIG_STATE_DB="$(python3 -c 'import json,os,sys; d=json.load(open(sys.argv[1], encoding="utf-8")); p=d.get("state_db", "~/.local/state/livox-power-cycle-manager/state.sqlite3"); print(os.path.abspath(os.path.expandvars(os.path.expanduser(p.strip()))))' "${CONFIG_FILE}")"
[[ "${CONFIG_STATE_DB}" == "${STATE_DB}" ]] ||
  die "生产安装只允许 state_db=${STATE_DB}；当前配置解析为 ${CONFIG_STATE_DB}。请迁移并核对旧库中的补上电义务后再安装。"
CONFIG_MODE="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8")).get("mode", "observe"))' "${CONFIG_FILE}")"
CONFIG_ENABLED_GROUPS="$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1], encoding="utf-8")); print(sum(1 for row in d.get("power_groups", {}).values() if row.get("enabled") is True))' "${CONFIG_FILE}")"
log "现场配置实际状态：mode=${CONFIG_MODE}，enabled_groups=${CONFIG_ENABLED_GROUPS}。"
if [[ "${CONFIG_MODE}" == "armed" && "${LIVOX_ALLOW_ARMED_INSTALL:-0}" != "1" ]]; then
  die "现有配置已是 armed；为防重装时意外恢复整组断电自动化，默认拒绝启动。确认共享通道映射、电气验收和无人值守恢复策略后，才可显式设置 LIVOX_ALLOW_ARMED_INSTALL=1 重跑。"
fi

escape_sed() { printf '%s' "$1" | sed 's/[\\&|]/\\&/g'; }
current_user="$(id -un)"
current_group="$(id -gn)"
[[ "${current_user}" =~ ^[A-Za-z0-9._-]+$ ]] || die "用户名含有不安全字符。"
[[ "${current_group}" =~ ^[A-Za-z0-9._-]+$ ]] || die "组名含有不安全字符。"
rendered="$(mktemp "${TMPDIR:-/tmp}/livox-power-cycle-unit.XXXXXX")"
trap 'rm -f -- "${rendered}"' EXIT
sed \
  -e "s|@USER@|$(escape_sed "${current_user}")|g" \
  -e "s|@GROUP@|$(escape_sed "${current_group}")|g" \
  -e "s|@HOME@|$(escape_sed "${HOME}")|g" \
  -e "s|@CATKIN_WS@|$(escape_sed "${CATKIN_WS}")|g" \
  -e "s|@CONFIG@|$(escape_sed "${CONFIG_FILE}")|g" \
  -e "s|@STATE_DIR@|$(escape_sed "${STATE_DIR}")|g" \
  -e "s|@STATE_DB@|$(escape_sed "${STATE_DB}")|g" \
  -e "s|@DRIVER_DIR@|$(escape_sed "${SCRIPT_DIR}")|g" \
  "${TEMPLATE_FILE}" >"${rendered}"

sudo install -m 644 "${rendered}" "${UNIT_PATH}"
sudo systemctl daemon-reload
sudo systemctl enable --now "${UNIT_NAME}"
log "服务已启动（mode=${CONFIG_MODE}，enabled_groups=${CONFIG_ENABLED_GROUPS}）：sudo systemctl status ${UNIT_NAME}"
log "任一或多名成员触发都会使同组 4 台执行一次整体断/上电；不依赖 PLC/上位机许可。先填写 4 个 members 和唯一继电器 IP/通道，完成只读核对、电气浪涌核算与受控实机验收后才允许 armed。"
