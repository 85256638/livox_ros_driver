#!/usr/bin/env bash

# Update the paired Livox-SDK and livox_ros_driver on Ubuntu/ROS Noetic.
# Every remote Git operation is routed through the local Geph SOCKS proxy.
# SDK is always built/installed before the Driver is updated and built.

set -Eeuo pipefail
IFS=$'\n\t'

PROXY_URL="${LIVOX_GEPH_PROXY:-socks5h://127.0.0.1:9909}"
SDK_URL="${LIVOX_SDK_URL:-https://github.com/85256638/Livox-SDK.git}"
SDK_BRANCH="${LIVOX_SDK_BRANCH:-mod_set&range_filter}"
DRIVER_URL="${LIVOX_DRIVER_URL:-https://github.com/85256638/livox_ros_driver.git}"
DRIVER_BRANCH="${LIVOX_DRIVER_BRANCH:-updated_workingmode&set_rangefilter}"
SDK_DIR="${LIVOX_SDK_DIR:-${HOME}/Livox-SDK}"
SDK_INSTALL_PREFIX="${LIVOX_SDK_INSTALL_PREFIX:-/usr/local}"
CATKIN_WS="${CATKIN_WS:-${HOME}/catkin_ws}"
DRIVER_DIR="${LIVOX_DRIVER_DIR:-${CATKIN_WS}/src/livox_ros_driver}"
STATE_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/livox-stack-updater"
SDK_STAMP="${STATE_DIR}/sdk-installed-commit"
DRIVER_STAMP="${STATE_DIR}/driver-built-revisions"
ROS_SETUP="${LIVOX_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
JOBS="${LIVOX_JOBS:-}"
GIT_TIMEOUT_SEC="${LIVOX_GIT_TIMEOUT_SEC:-300}"

FORCE_REBUILD=0
RESTART_SERVICE=0
FETCHED_HEAD=""
ACTIVE_CLONE_TEMP=""
ORIGINAL_ARGS=("$@")

usage() {
  cat <<'EOF'
Usage: update_livox_geph.sh [--force] [--restart-service]

Default environment:
  LIVOX_GEPH_PROXY=socks5h://127.0.0.1:9909
  LIVOX_SDK_DIR=$HOME/Livox-SDK
  LIVOX_SDK_INSTALL_PREFIX=/usr/local
  CATKIN_WS=$HOME/catkin_ws

Options:
  --force             Rebuild SDK and Driver even when revisions are unchanged.
  --restart-service   Restart livox-ros-driver after a successful build.
  -h, --help          Show this help.
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
  if [[ -n "${LIVOX_UPDATER_TEMP_COPY:-}" ]]; then
    rm -f -- "${LIVOX_UPDATER_TEMP_COPY}"
  fi
  if [[ -n "${ACTIVE_CLONE_TEMP}" && -d "${ACTIVE_CLONE_TEMP}" ]]; then
    rm -rf -- "${ACTIVE_CLONE_TEMP}"
  fi
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

[[ ${EUID} -ne 0 ]] || die "请使用普通用户运行；脚本只会在安装 SDK/重启服务时调用 sudo。"
[[ "${GIT_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] || die "LIVOX_GIT_TIMEOUT_SEC 必须是正整数。"
[[ "${SDK_INSTALL_PREFIX}" == /* ]] || die "LIVOX_SDK_INSTALL_PREFIX 必须是绝对路径。"

for command_name in git cmake timeout nproc sudo flock; do
  command -v "${command_name}" >/dev/null 2>&1 || die "缺少命令：${command_name}"
done
if [[ -z "${JOBS}" ]]; then
  JOBS="$(nproc)"
fi
[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || die "LIVOX_JOBS 必须是正整数，当前值：${JOBS}"
[[ -f "${ROS_SETUP}" ]] || die "找不到 ROS 环境：${ROS_SETUP}"
[[ -x /usr/bin/python3 ]] || die "找不到 ROS Noetic 所需的 /usr/bin/python3。"

mkdir -p "${STATE_DIR}" "${CATKIN_WS}/src"
exec 9>"${STATE_DIR}/update.lock"
flock -n 9 || die "已有另一个 Livox 更新任务正在运行。"

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
  check_clean_checkout "${name}" "${directory}"

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
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_POLICY_DEFAULT_CMP0079=NEW \
    -DLIVOX_SDK_SOURCE_DIR="${SDK_DIR}")

  [[ -x "${driver_binary}" ]] || die "catkin_make 结束但未找到 Driver 二进制：${driver_binary}"
  atomic_write "${DRIVER_STAMP}" "${desired_stamp}"
  log "Driver 编译完成：$(short_sha "${driver_sha}")"
}

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

sync_remote_branch "livox_ros_driver" "${DRIVER_DIR}" "${DRIVER_URL}" "${DRIVER_BRANCH}"
DRIVER_REMOTE_HEAD="${FETCHED_HEAD}"
update_checkout "livox_ros_driver" "${DRIVER_DIR}" "${DRIVER_BRANCH}" "${DRIVER_REMOTE_HEAD}"

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
  log "重启 livox-ros-driver 服务"
  sudo systemctl restart livox-ros-driver
fi

log "全部完成：SDK $(short_sha "${CURRENT_SDK_SHA}")，Driver $(short_sha "${DRIVER_REMOTE_HEAD}")"
if ((!RESTART_SERVICE)); then
  log "如需应用新二进制，请执行：sudo systemctl restart livox-ros-driver"
fi
