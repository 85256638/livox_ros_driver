#!/usr/bin/env bash

# Update the paired Livox-SDK and livox_ros_driver on Ubuntu/ROS Noetic.
# Every remote Git operation is routed through the local Geph SOCKS proxy.
# SDK is always built/installed before the Driver is updated and built.

set -Eeuo pipefail
IFS=$'\n\t'

PROXY_URL="${LIVOX_GEPH_PROXY:-socks5h://127.0.0.1:9909}"
SDK_URL="${LIVOX_SDK_URL:-https://github.com/85256638/Livox-SDK.git}"
SDK_BRANCH="${LIVOX_SDK_BRANCH:-network-relay-added}"
DRIVER_URL="${LIVOX_DRIVER_URL:-https://github.com/85256638/livox_ros_driver.git}"
DRIVER_BRANCH="${LIVOX_DRIVER_BRANCH:-network-relay-added}"
SDK_DIR="${LIVOX_SDK_DIR:-${HOME}/Livox-SDK}"
SDK_INSTALL_PREFIX="${LIVOX_SDK_INSTALL_PREFIX:-/usr/local}"
CATKIN_WS="${CATKIN_WS:-${HOME}/catkin_ws}"
DRIVER_DIR="${LIVOX_DRIVER_DIR:-${CATKIN_WS}/src/livox_ros_driver}"
STATE_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/livox-stack-updater"
SDK_STAMP="${STATE_DIR}/sdk-installed-commit"
DRIVER_STAMP="${STATE_DIR}/driver-built-revisions"
SITE_CONFIG_PENDING_FILE="${STATE_DIR}/site-config-pending"
ROS_SETUP="${LIVOX_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
PYTHON_EXECUTABLE="${LIVOX_PYTHON_EXECUTABLE:-/usr/bin/python3}"
JOBS="${LIVOX_JOBS:-}"
GIT_TIMEOUT_SEC="${LIVOX_GIT_TIMEOUT_SEC:-300}"

FORCE_REBUILD=0
RESTART_SERVICE=0
PRESERVE_SITE_CONFIG=0
FETCHED_HEAD=""
ACTIVE_CLONE_TEMP=""
SITE_CONFIG_PREPARED=0
SITE_CONFIG_BACKUP_DIR=""
SITE_CONFIG_PATCH=""
SITE_CONFIG_STASH_SHA=""
POWER_MANAGER_WAS_ACTIVE=0
POWER_MANAGER_STOPPED_BY_UPDATER=0
SITE_CONFIG_CHANGED_PATHS=()
SITE_CONFIG_PATHS=(
  "livox_ros_driver/config/livox_lidar_config_multi.json"
  "livox_ros_driver/launch/livox_lidar_multi.launch"
)
ORIGINAL_ARGS=("$@")

usage() {
  cat <<'EOF'
Usage: update_livox_geph.sh [--force] [--preserve-site-config] [--restart-service]

Default environment:
  LIVOX_GEPH_PROXY=socks5h://127.0.0.1:9909
  LIVOX_SDK_DIR=$HOME/Livox-SDK
  LIVOX_SDK_INSTALL_PREFIX=/usr/local
  CATKIN_WS=$HOME/catkin_ws

Options:
  --force                 Rebuild even when revisions are unchanged.
  --preserve-site-config  Preserve pit-specific multi-LiDAR JSON/launch edits.
  --restart-service       Apply services after success; safely stop/restart
                          an active/starting livox-power-cycle-manager.
  -h, --help              Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --force)
      FORCE_REBUILD=1
      ;;
    --restart-service)
      RESTART_SERVICE=1
      ;;
    --preserve-site-config)
      PRESERVE_SITE_CONFIG=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

log() {
  printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >&2
}

die() {
  log "ERROR: $*"
  exit 1
}

cleanup() {
  local exit_status=$?
  set +e
  if ((SITE_CONFIG_PREPARED)); then
    log "脚本提前退出，正在恢复工位 JSON/launch。"
    if ! restore_site_config; then
      exit_status=1
      log "ERROR: 自动恢复未完全成功；请从备份核对工位文件：${SITE_CONFIG_BACKUP_DIR}"
    fi
  fi
  if ((POWER_MANAGER_WAS_ACTIVE && POWER_MANAGER_STOPPED_BY_UPDATER)); then
    log "脚本异常退出，正在恢复此前运行的 livox-power-cycle-manager。"
    if sudo systemctl start livox-power-cycle-manager.service; then
      POWER_MANAGER_STOPPED_BY_UPDATER=0
    else
      exit_status=1
      log "ERROR: livox-power-cycle-manager 恢复启动失败，请立即人工检查。"
    fi
  fi
  if [[ -n "${LIVOX_UPDATER_TEMP_COPY:-}" ]]; then
    rm -f -- "${LIVOX_UPDATER_TEMP_COPY}"
  fi
  if [[ -n "${ACTIVE_CLONE_TEMP}" && -d "${ACTIVE_CLONE_TEMP}" ]]; then
    rm -rf -- "${ACTIVE_CLONE_TEMP}"
  fi
  trap - EXIT
  exit "${exit_status}"
}

# Run from a temporary copy so updating the Driver repository cannot replace
# this script while Bash is still reading it.
if [[ -z "${LIVOX_UPDATER_TEMP_COPY:-}" ]]; then
  temp_script="$(mktemp "${TMPDIR:-/tmp}/livox-geph-update.XXXXXX")"
  cp -- "${BASH_SOURCE[0]}" "${temp_script}"
  chmod 700 "${temp_script}"
  export LIVOX_UPDATER_TEMP_COPY="${temp_script}"
  exec "${temp_script}" "${ORIGINAL_ARGS[@]}"
fi
trap cleanup EXIT

GIT_PROXY=(
  git
  -c "http.proxy=${PROXY_URL}"
  -c "https.proxy=${PROXY_URL}"
)

normalize_url() {
  local value="${1%/}"
  printf '%s\n' "${value%.git}"
}

short_sha() {
  printf '%.7s' "$1"
}

atomic_write() {
  local target="$1"
  local value="$2"
  local temporary="${target}.tmp.$$"
  printf '%s\n' "${value}" >"${temporary}"
  mv -f -- "${temporary}" "${target}"
}

power_manager_active_state() {
  systemctl show livox-power-cycle-manager.service \
    --property=ActiveState --value 2>/dev/null || printf 'not-found\n'
}

check_clean_checkout() {
  local name="$1"
  local directory="$2"
  local dirty
  dirty="$(git -C "${directory}" status --porcelain --untracked-files=no)"
  [[ -z "${dirty}" ]] || die "${name} 存在 tracked 本地修改，拒绝覆盖：${directory}"
}

check_origin() {
  local name="$1"
  local directory="$2"
  local expected_url="$3"
  local actual_url
  actual_url="$(git -C "${directory}" remote get-url origin 2>/dev/null)" ||
    die "${name} 缺少 origin：${directory}"
  [[ "$(normalize_url "${actual_url}")" == "$(normalize_url "${expected_url}")" ]] ||
    die "${name} origin 不符合预期：${actual_url}"
}

site_config_other_changes() {
  git -C "${DRIVER_DIR}" diff --name-only HEAD -- . \
    ":(exclude)${SITE_CONFIG_PATHS[0]}" \
    ":(exclude)${SITE_CONFIG_PATHS[1]}"
}

site_path_is_tracked_regular() {
  local path="$1"
  local stage_entry
  local status_entry
  local mode

  stage_entry="$(git -C "${DRIVER_DIR}" ls-files --stage -- "${path}")"
  [[ -n "${stage_entry}" ]] || return 1
  mode="${stage_entry%% *}"
  [[ "${mode}" == "100644" || "${mode}" == "100755" ]] || return 1
  status_entry="$(git -C "${DRIVER_DIR}" ls-files -v -- "${path}")"
  [[ "${status_entry%% *}" == "H" ]] || return 1
  [[ -f "${DRIVER_DIR}/${path}" && ! -L "${DRIVER_DIR}/${path}" ]]
}

validate_site_config_scope() {
  local other_changes
  local path

  ((PRESERVE_SITE_CONFIG)) || return 0
  [[ -d "${DRIVER_DIR}/.git" ]] || return 0

  for path in "${SITE_CONFIG_PATHS[@]}"; do
    site_path_is_tracked_regular "${path}" ||
      die "工位配置文件未被跟踪、被删除、不是普通文件或设置了 skip-worktree/assume-unchanged，无法自动保留：${path}"
  done

  if ! git -C "${DRIVER_DIR}" diff --cached --quiet -- \
      "${SITE_CONFIG_PATHS[@]}"; then
    die "工位 JSON/launch 存在 staged 修改；请先取消暂存，避免丢失 index 中间版本。"
  fi

  other_changes="$(site_config_other_changes)"
  if [[ -n "${other_changes}" ]]; then
    log "以下 tracked 修改不属于允许自动保留的两份工位配置："
    printf '%s\n' "${other_changes}" >&2
    die "为避免覆盖源码，已停止更新。"
  fi
}

drop_site_config_stash_if_top() {
  local current_stash=""

  [[ -n "${SITE_CONFIG_STASH_SHA}" ]] || return 0
  current_stash="$(git -C "${DRIVER_DIR}" rev-parse -q --verify refs/stash 2>/dev/null || true)"
  if [[ "${current_stash}" == "${SITE_CONFIG_STASH_SHA}" ]]; then
    git -C "${DRIVER_DIR}" stash drop -q "stash@{0}" ||
      log "WARNING: 工位配置已恢复，但自动备份 stash 未能删除。"
  else
    log "WARNING: stash 列表在更新期间发生变化，保留自动备份 ${SITE_CONFIG_STASH_SHA}。"
  fi
}

clear_site_config_pending() {
  local pending_backup=""

  [[ -f "${SITE_CONFIG_PENDING_FILE}" ]] || return 0
  pending_backup="$(<"${SITE_CONFIG_PENDING_FILE}")"
  [[ "${pending_backup}" == "${SITE_CONFIG_BACKUP_DIR}" ]] || return 1
  rm -f -- "${SITE_CONFIG_PENDING_FILE}"
}

atomic_copy_file() {
  local source="$1"
  local target="$2"
  local temporary="${target}.livox-updater.$$"

  cp -p -- "${source}" "${temporary}" || return 1
  mv -f -- "${temporary}" "${target}" || return 1
}

restore_site_config() {
  local auxiliary_failure=0
  local invalid_target=0
  local path

  ((SITE_CONFIG_PREPARED)) || return 0
  if ! mkdir -p "${SITE_CONFIG_BACKUP_DIR}/upstream"; then
    auxiliary_failure=1
  fi
  for path in "${SITE_CONFIG_CHANGED_PATHS[@]}"; do
    if site_path_is_tracked_regular "${path}"; then
      if ! mkdir -p "${SITE_CONFIG_BACKUP_DIR}/upstream/$(dirname "${path}")" ||
          ! cp -p -- "${DRIVER_DIR}/${path}" \
            "${SITE_CONFIG_BACKUP_DIR}/upstream/${path}"; then
        auxiliary_failure=1
      fi
    else
      invalid_target=1
    fi
    if ! mkdir -p "${DRIVER_DIR}/$(dirname "${path}")" ||
        ! atomic_copy_file "${SITE_CONFIG_BACKUP_DIR}/original/${path}" \
          "${DRIVER_DIR}/${path}"; then
      log "ERROR: 无法原子恢复工位文件：${path}"
      return 1
    fi
  done
  for path in "${SITE_CONFIG_CHANGED_PATHS[@]}"; do
    if ! cmp -s -- "${SITE_CONFIG_BACKUP_DIR}/original/${path}" \
        "${DRIVER_DIR}/${path}"; then
      log "ERROR: 工位文件恢复后校验不一致：${path}"
      return 1
    fi
  done
  if ! git -C "${DRIVER_DIR}" restore --staged -- \
      "${SITE_CONFIG_CHANGED_PATHS[@]}" >/dev/null 2>&1; then
    auxiliary_failure=1
  fi
  if ! clear_site_config_pending; then
    log "ERROR: 工位文件已恢复，但无法清除 pending 事务。"
    return 1
  fi
  SITE_CONFIG_PREPARED=0
  if ((invalid_target)); then
    log "ERROR: 上游删除或改变了工位配置文件类型；已恢复原文件但不会重启服务。"
    log "ERROR: 请人工检查，备份目录：${SITE_CONFIG_BACKUP_DIR}"
    return 1
  fi
  if ((auxiliary_failure)); then
    log "ERROR: 工位原文件已恢复，但备份上游版本或 Git index 时发生错误；不会重启服务。"
    return 1
  fi
  drop_site_config_stash_if_top
  log "工位 JSON/launch 已按更新前原样恢复；持久备份：${SITE_CONFIG_BACKUP_DIR}"
  return 0
}

prepare_site_config() {
  local changed_output
  local head_sha
  local path
  local stash_message

  ((PRESERVE_SITE_CONFIG)) || return 0
  [[ -d "${DRIVER_DIR}/.git" ]] || return 0
  validate_site_config_scope

  changed_output="$(git -C "${DRIVER_DIR}" diff --name-only HEAD -- \
    "${SITE_CONFIG_PATHS[@]}")"
  SITE_CONFIG_CHANGED_PATHS=()
  while IFS= read -r path; do
    [[ -n "${path}" ]] && SITE_CONFIG_CHANGED_PATHS+=("${path}")
  done <<<"${changed_output}"

  if ((${#SITE_CONFIG_CHANGED_PATHS[@]} == 0)); then
    log "两份工位配置没有本地修改，无需暂存。"
    return 0
  fi

  head_sha="$(git -C "${DRIVER_DIR}" rev-parse HEAD)"
  SITE_CONFIG_BACKUP_DIR="${STATE_DIR}/site-config-backups/$(date '+%Y%m%d-%H%M%S')-$(short_sha "${head_sha}")-$$"
  SITE_CONFIG_PATCH="${SITE_CONFIG_BACKUP_DIR}/local-changes.patch"
  mkdir -p "${SITE_CONFIG_BACKUP_DIR}/base" "${SITE_CONFIG_BACKUP_DIR}/original"
  for path in "${SITE_CONFIG_CHANGED_PATHS[@]}"; do
    mkdir -p "${SITE_CONFIG_BACKUP_DIR}/base/$(dirname "${path}")"
    mkdir -p "${SITE_CONFIG_BACKUP_DIR}/original/$(dirname "${path}")"
    git -C "${DRIVER_DIR}" show "${head_sha}:${path}" >"${SITE_CONFIG_BACKUP_DIR}/base/${path}"
    cp -p -- "${DRIVER_DIR}/${path}" \
      "${SITE_CONFIG_BACKUP_DIR}/original/${path}"
  done
  atomic_write "${SITE_CONFIG_BACKUP_DIR}/base-sha" "${head_sha}"
  atomic_write "${SITE_CONFIG_BACKUP_DIR}/driver-root" "${DRIVER_DIR}"
  printf '%s\n' "${SITE_CONFIG_CHANGED_PATHS[@]}" >"${SITE_CONFIG_BACKUP_DIR}/changed-files.txt"
  git -C "${DRIVER_DIR}" diff --binary --full-index HEAD -- \
    "${SITE_CONFIG_CHANGED_PATHS[@]}" >"${SITE_CONFIG_PATCH}"
  [[ -s "${SITE_CONFIG_PATCH}" ]] || die "无法生成工位配置差异备份。"

  SITE_CONFIG_PREPARED=1
  atomic_write "${SITE_CONFIG_PENDING_FILE}" "${SITE_CONFIG_BACKUP_DIR}"
  stash_message="livox updater site config $(date '+%Y-%m-%d %H:%M:%S')"
  git -C "${DRIVER_DIR}" \
    -c user.name="Livox Stack Updater" \
    -c user.email="livox-updater@localhost" \
    stash push -q -m "${stash_message}" -- \
    "${SITE_CONFIG_CHANGED_PATHS[@]}" ||
    die "无法暂存工位配置；原文件备份位于 ${SITE_CONFIG_BACKUP_DIR}"
  SITE_CONFIG_STASH_SHA="$(git -C "${DRIVER_DIR}" rev-parse -q --verify refs/stash)" ||
    die "工位配置暂存后无法读取 stash。"
  atomic_write "${SITE_CONFIG_BACKUP_DIR}/stash-commit" "${SITE_CONFIG_STASH_SHA}"
  git -C "${DRIVER_DIR}" diff --quiet HEAD -- \
    "${SITE_CONFIG_CHANGED_PATHS[@]}" ||
    die "工位配置暂存后工作区仍不干净。"
  log "已安全暂存工位 JSON/launch；备份：${SITE_CONFIG_BACKUP_DIR}"
}

recover_pending_site_config() {
  local path
  local pending_backup

  [[ -f "${SITE_CONFIG_PENDING_FILE}" ]] || return 0
  pending_backup="$(<"${SITE_CONFIG_PENDING_FILE}")"
  [[ "${pending_backup}" == "${STATE_DIR}/site-config-backups/"* ]] ||
    die "工位配置 pending 路径异常：${pending_backup}"
  [[ -d "${pending_backup}/base" && -d "${pending_backup}/original" &&
     -s "${pending_backup}/local-changes.patch" &&
     -s "${pending_backup}/changed-files.txt" &&
     -s "${pending_backup}/driver-root" ]] ||
    die "工位配置 pending 备份不完整：${pending_backup}"
  [[ "$(<"${pending_backup}/driver-root")" == "${DRIVER_DIR}" ]] ||
    die "pending 备份属于另一个 Driver 目录：$(<"${pending_backup}/driver-root")"
  [[ -d "${DRIVER_DIR}/.git" ]] ||
    die "存在待恢复的工位配置，但 Driver 仓库不存在：${DRIVER_DIR}"

  SITE_CONFIG_BACKUP_DIR="${pending_backup}"
  SITE_CONFIG_PATCH="${pending_backup}/local-changes.patch"
  SITE_CONFIG_CHANGED_PATHS=()
  while IFS= read -r path; do
    case "${path}" in
      "${SITE_CONFIG_PATHS[0]}"|"${SITE_CONFIG_PATHS[1]}")
        SITE_CONFIG_CHANGED_PATHS+=("${path}")
        ;;
      *)
        die "pending 备份包含未授权路径：${path}"
        ;;
    esac
  done <"${pending_backup}/changed-files.txt"
  ((${#SITE_CONFIG_CHANGED_PATHS[@]} > 0)) ||
    die "pending 备份没有工位配置路径。"
  if [[ -s "${pending_backup}/stash-commit" ]]; then
    SITE_CONFIG_STASH_SHA="$(<"${pending_backup}/stash-commit")"
  fi
  SITE_CONFIG_PREPARED=1
  log "检测到上次中断留下的工位配置事务，正在先行恢复。"
  restore_site_config ||
    die "工位配置事务需要人工核对：${pending_backup}"
}

remote_branch_head() {
  local name="$1"
  local repository_url="$2"
  local branch="$3"
  local output
  local sha

  if ! output="$(timeout "${GIT_TIMEOUT_SEC}" "${GIT_PROXY[@]}" ls-remote --heads \
      "${repository_url}" "refs/heads/${branch}")"; then
    die "无法通过 Geph 检查 ${name}；请确认 Geph 已启动并监听 127.0.0.1:9909。"
  fi
  sha="${output%%[[:space:]]*}"
  [[ "${sha}" =~ ^[0-9a-f]{40}$ ]] ||
    die "GitHub 上找不到 ${name} 分支：${branch}"
  printf '%s\n' "${sha}"
}

sync_remote_branch() {
  local name="$1"
  local directory="$2"
  local repository_url="$3"
  local branch="$4"
  local allow_tracked_changes="${5:-0}"
  local parent

  if [[ -e "${directory}" && ! -d "${directory}/.git" ]]; then
    die "${name} 目录已存在但不是 Git 仓库：${directory}"
  fi

  if [[ ! -d "${directory}/.git" ]]; then
    parent="$(dirname "${directory}")"
    mkdir -p "${parent}"
    log "下载 ${name} 分支 ${branch} -> ${directory}"
    ACTIVE_CLONE_TEMP="$(mktemp -d "${parent}/.livox-updater-clone.XXXXXX")"
    timeout "${GIT_TIMEOUT_SEC}" "${GIT_PROXY[@]}" clone \
      --branch "${branch}" --single-branch --no-tags \
      "${repository_url}" "${ACTIVE_CLONE_TEMP}" ||
      die "${name} clone 失败或超时。"
    [[ ! -e "${directory}" ]] ||
      die "${name} 目标目录在 clone 期间被其他程序创建：${directory}"
    mv -T -- "${ACTIVE_CLONE_TEMP}" "${directory}"
    ACTIVE_CLONE_TEMP=""
  fi

  check_origin "${name}" "${directory}" "${repository_url}"
  if ((!allow_tracked_changes)); then
    check_clean_checkout "${name}" "${directory}"
  fi

  log "通过 Geph 检查 ${name} 最新版本"
  timeout "${GIT_TIMEOUT_SEC}" "${GIT_PROXY[@]}" -C "${directory}" fetch \
    --no-tags origin "+refs/heads/${branch}:refs/remotes/origin/${branch}" ||
    die "${name} fetch 失败或超时。"

  FETCHED_HEAD="$(git -C "${directory}" rev-parse "refs/remotes/origin/${branch}")"
  [[ "${FETCHED_HEAD}" =~ ^[0-9a-f]{40}$ ]] || die "无法解析 ${name} 远端版本。"
}

update_checkout() {
  local name="$1"
  local directory="$2"
  local branch="$3"
  local desired_sha="$4"
  local local_sha

  if git -C "${directory}" show-ref --verify --quiet "refs/heads/${branch}"; then
    git -C "${directory}" checkout "${branch}" ||
      die "无法切换到 ${name} 分支：${branch}"
  else
    local_sha="$(git -C "${directory}" rev-parse HEAD)"
    if ! git -C "${directory}" merge-base --is-ancestor \
        "${local_sha}" "${desired_sha}"; then
      die "${name} 当前提交不属于远端目标分支，拒绝放弃本地历史；请人工审阅。"
    fi
    git -C "${directory}" checkout -b "${branch}" "${local_sha}" ||
      die "无法创建 ${name} 本地分支：${branch}"
    git -C "${directory}" branch --set-upstream-to="origin/${branch}" "${branch}" ||
      die "无法设置 ${name} 上游分支。"
  fi

  check_clean_checkout "${name}" "${directory}"
  local_sha="$(git -C "${directory}" rev-parse HEAD)"
  if [[ "${local_sha}" == "${desired_sha}" ]]; then
    log "${name} 源码已是最新：$(short_sha "${desired_sha}")"
    return
  fi

  if git -C "${directory}" merge-base --is-ancestor \
      "${local_sha}" "${desired_sha}"; then
    log "${name} 需要更新：$(short_sha "${local_sha}") -> $(short_sha "${desired_sha}")"
    git -C "${directory}" merge --ff-only "${desired_sha}" ||
      die "${name} fast-forward 更新失败。"
  elif git -C "${directory}" merge-base --is-ancestor \
      "${desired_sha}" "${local_sha}"; then
    die "${name} 存在本地未推送提交，拒绝覆盖；请人工审阅。"
  else
    die "${name} 本地分支与远端已分叉，拒绝 reset；请人工审阅。"
  fi
}

install_sdk_if_needed() {
  local desired_sha="$1"
  local installed_sha=""
  local installed_library="${SDK_INSTALL_PREFIX}/lib/liblivox_sdk_static.a"
  local installed_def_header="${SDK_INSTALL_PREFIX}/include/livox_def.h"
  local installed_sdk_header="${SDK_INSTALL_PREFIX}/include/livox_sdk.h"

  if [[ -f "${SDK_STAMP}" ]]; then
    installed_sha="$(<"${SDK_STAMP}")"
  fi

  if ((FORCE_REBUILD)) || [[ "${installed_sha}" != "${desired_sha}" ]] ||
      [[ ! -f "${SDK_DIR}/build/sdk_core/liblivox_sdk_static.a" ]] ||
      [[ ! -f "${installed_library}" ]] ||
      [[ ! -f "${installed_def_header}" ]] ||
      [[ ! -f "${installed_sdk_header}" ]]; then
    log "配置并编译 SDK：$(short_sha "${desired_sha}")"
    cmake -S "${SDK_DIR}" -B "${SDK_DIR}/build" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${SDK_INSTALL_PREFIX}"
    cmake --build "${SDK_DIR}/build" --parallel "${JOBS}"
    log "安装 SDK 到系统目录（需要 sudo）"
    sudo -v
    sudo cmake --install "${SDK_DIR}/build"
    atomic_write "${SDK_STAMP}" "${desired_sha}"
    log "SDK 编译安装完成：$(short_sha "${desired_sha}")"
  else
    log "SDK 已安装且版本未变化，跳过重复编译。"
  fi
}

read_driver_sdk_pin() {
  local pin_file="${DRIVER_DIR}/livox_ros_driver/cmake/pinned_livox_sdk.cmake"
  local line
  local pinned_sha=""
  local in_commit_setting=0
  [[ -f "${pin_file}" ]] || die "Driver 缺少 SDK pin 文件：${pin_file}"
  while IFS= read -r line; do
    if [[ "${line}" == *"set(LIVOX_SDK_GIT_COMMIT"* ]]; then
      in_commit_setting=1
    fi
    if ((in_commit_setting)) && [[ "${line}" =~ ([0-9a-f]{40}) ]]; then
      pinned_sha="${BASH_REMATCH[1]}"
      break
    fi
  done <"${pin_file}"
  [[ "${pinned_sha}" =~ ^[0-9a-f]{40}$ ]] || die "无法读取 Driver 固定的 SDK SHA。"
  printf '%s\n' "${pinned_sha}"
}

build_driver_if_needed() {
  local driver_sha="$1"
  local sdk_sha="$2"
  local desired_stamp="${driver_sha} ${sdk_sha}"
  local built_stamp=""
  local driver_binary="${CATKIN_WS}/devel/lib/livox_ros_driver/livox_ros_driver_node"

  if [[ -f "${DRIVER_STAMP}" ]]; then
    built_stamp="$(<"${DRIVER_STAMP}")"
  fi

  if ((!FORCE_REBUILD)) && [[ "${built_stamp}" == "${desired_stamp}" ]] &&
      [[ -x "${driver_binary}" ]]; then
    log "Driver 已编译且版本未变化，跳过重复编译。"
    return
  fi

  log "加载 ROS Noetic 并编译 Driver：$(short_sha "${driver_sha}")"
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
  command -v catkin_make >/dev/null 2>&1 || die "加载 ROS 后仍找不到 catkin_make。"

  if [[ ! -e "${CATKIN_WS}/src/CMakeLists.txt" ]]; then
    command -v catkin_init_workspace >/dev/null 2>&1 ||
      die "加载 ROS 后仍找不到 catkin_init_workspace。"
    (cd "${CATKIN_WS}/src" && catkin_init_workspace)
  fi

  (cd "${CATKIN_WS}" && catkin_make --force-cmake -j"${JOBS}" -l"${JOBS}" \
    -DPYTHON_EXECUTABLE="${PYTHON_EXECUTABLE}" \
    -DCMAKE_POLICY_DEFAULT_CMP0079=NEW \
    -DLIVOX_SDK_SOURCE_DIR="${SDK_DIR}")

  [[ -x "${driver_binary}" ]] || die "catkin_make 结束但未找到 Driver 二进制：${driver_binary}"
  atomic_write "${DRIVER_STAMP}" "${desired_stamp}"
  log "Driver 编译完成：$(short_sha "${driver_sha}")"
}

[[ ${EUID} -ne 0 ]] || die "请使用普通用户运行；脚本只会在安装 SDK/重启服务时调用 sudo。"
[[ "${GIT_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] || die "LIVOX_GIT_TIMEOUT_SEC 必须是正整数。"
[[ "${SDK_INSTALL_PREFIX}" == /* ]] || die "LIVOX_SDK_INSTALL_PREFIX 必须是绝对路径。"
for command_name in git flock cmp; do
  command -v "${command_name}" >/dev/null 2>&1 || die "缺少命令：${command_name}"
done
mkdir -p "${STATE_DIR}" "${CATKIN_WS}/src"
exec 9>"${STATE_DIR}/update.lock"
flock -n 9 || die "已有另一个 Livox 更新任务正在运行。"

# Recover a prior interrupted configuration transaction before checking ROS,
# contacting GitHub or changing SDK/Driver state.
recover_pending_site_config

for command_name in cmake timeout nproc sudo; do
  command -v "${command_name}" >/dev/null 2>&1 || die "缺少命令：${command_name}"
done
if [[ -z "${JOBS}" ]]; then
  JOBS="$(nproc)"
fi
[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || die "LIVOX_JOBS 必须是正整数，当前值：${JOBS}"
[[ -f "${ROS_SETUP}" ]] || die "找不到 ROS 环境：${ROS_SETUP}"
[[ -x "${PYTHON_EXECUTABLE}" ]] ||
  die "找不到 ROS Noetic 所需的 Python：${PYTHON_EXECUTABLE}"
validate_site_config_scope

log "检查 Geph 代理与 GitHub 连通性：${PROXY_URL}"
SDK_ADVERTISED_HEAD="$(remote_branch_head \
  "Livox-SDK" "${SDK_URL}" "${SDK_BRANCH}")"
log "SDK 远端最新版本：$(short_sha "${SDK_ADVERTISED_HEAD}")"

# Dependency order is intentional: SDK is checked, updated, built and installed
# before the Driver working tree is updated or compiled.
sync_remote_branch "Livox-SDK" "${SDK_DIR}" "${SDK_URL}" "${SDK_BRANCH}"
SDK_REMOTE_HEAD="${FETCHED_HEAD}"
update_checkout "Livox-SDK" "${SDK_DIR}" "${SDK_BRANCH}" "${SDK_REMOTE_HEAD}"
install_sdk_if_needed "${SDK_REMOTE_HEAD}"

sync_remote_branch "livox_ros_driver" "${DRIVER_DIR}" "${DRIVER_URL}" \
  "${DRIVER_BRANCH}" "${PRESERVE_SITE_CONFIG}"
DRIVER_REMOTE_HEAD="${FETCHED_HEAD}"
# Network fetch is intentionally complete before the site files are stashed.
# This keeps the window in which a watchdog restart could see repository
# defaults limited to the local fast-forward operation below.
prepare_site_config
update_checkout "livox_ros_driver" "${DRIVER_DIR}" "${DRIVER_BRANCH}" "${DRIVER_REMOTE_HEAD}"
restore_site_config ||
  die "工位配置未能安全恢复，已禁止 Driver 编译与服务重启。"

# A paired SDK/Driver publish is not atomic on GitHub. Recheck both branch tips
# after updating so a mid-run push cannot produce a build that is already stale.
SDK_RECHECK_HEAD="$(remote_branch_head \
  "Livox-SDK" "${SDK_URL}" "${SDK_BRANCH}")"
DRIVER_RECHECK_HEAD="$(remote_branch_head \
  "livox_ros_driver" "${DRIVER_URL}" "${DRIVER_BRANCH}")"
[[ "${SDK_RECHECK_HEAD}" == "${SDK_REMOTE_HEAD}" ]] ||
  die "运行期间 SDK 远端版本发生变化，请重新运行脚本。"
[[ "${DRIVER_RECHECK_HEAD}" == "${DRIVER_REMOTE_HEAD}" ]] ||
  die "运行期间 Driver 远端版本发生变化，请重新运行脚本。"

PINNED_SDK_SHA="$(read_driver_sdk_pin)"
CURRENT_SDK_SHA="$(git -C "${SDK_DIR}" rev-parse HEAD)"
if [[ "${PINNED_SDK_SHA}" != "${CURRENT_SDK_SHA}" ]]; then
  die "最新版 Driver 固定 SDK $(short_sha "${PINNED_SDK_SHA}")，但 SDK 分支最新为 $(short_sha "${CURRENT_SDK_SHA}")；为避免错配已停止 Driver 编译，请稍后重试。"
fi

build_driver_if_needed "${DRIVER_REMOTE_HEAD}" "${CURRENT_SDK_SHA}"

if ((RESTART_SERVICE)); then
  POWER_MANAGER_STATE="$(power_manager_active_state)"
  case "${POWER_MANAGER_STATE}" in
    active|activating|reloading|deactivating)
      POWER_MANAGER_WAS_ACTIVE=1
      log "先安全停止 livox-power-cycle-manager（若正在 OFF，会先补回 ON）"
      POWER_MANAGER_STOPPED_BY_UPDATER=1
      sudo systemctl stop livox-power-cycle-manager.service
      POWER_MANAGER_STATE="$(power_manager_active_state)"
      [[ "${POWER_MANAGER_STATE}" == "inactive" || \
         "${POWER_MANAGER_STATE}" == "failed" ]] ||
        die "livox-power-cycle-manager 未能安全停止（state=${POWER_MANAGER_STATE}），拒绝重启 Driver。"
      ;;
  esac
  log "重启 livox-ros-driver 服务"
  sudo systemctl restart livox-ros-driver
  if ((POWER_MANAGER_WAS_ACTIVE)); then
    log "Driver 已恢复，重新启动 livox-power-cycle-manager 服务"
    sudo systemctl start livox-power-cycle-manager.service
    POWER_MANAGER_STOPPED_BY_UPDATER=0
  fi
fi

log "全部完成：SDK $(short_sha "${CURRENT_SDK_SHA}")，Driver $(short_sha "${DRIVER_REMOTE_HEAD}")"
if ((!RESTART_SERVICE)); then
  log "如需应用新二进制，请执行：sudo systemctl restart livox-ros-driver"
fi
