# The Driver's callback-context lifetime relies on the paired SDK's
# exactly-once completion/cancellation contract. Never silently link an
# arbitrary system library or fall back to the official default branch.
set(LIVOX_SDK_GIT_REPOSITORY
  "https://github.com/85256638/Livox-SDK.git")
set(LIVOX_SDK_GIT_BRANCH "mod_set&range_filter")
set(LIVOX_SDK_GIT_COMMIT
  "fe1a68cd54be70219821e4186e66329d375d224f")
set(LIVOX_SDK_SOURCE_DIR "" CACHE PATH
  "Optional local Livox-SDK checkout; it must be clean and at the pinned commit")

find_package(Git REQUIRED)

set(_livox_sdk_managed_checkout FALSE)
if(NOT LIVOX_SDK_SOURCE_DIR)
  set(LIVOX_SDK_SOURCE_DIR
    "${CMAKE_BINARY_DIR}/_deps/livox-sdk-src")
  set(_livox_sdk_managed_checkout TRUE)
endif()
get_filename_component(LIVOX_SDK_SOURCE_DIR
  "${LIVOX_SDK_SOURCE_DIR}" ABSOLUTE)

if(_livox_sdk_managed_checkout AND
   NOT EXISTS "${LIVOX_SDK_SOURCE_DIR}/.git")
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/_deps")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" clone
      --branch "${LIVOX_SDK_GIT_BRANCH}"
      --single-branch --no-tags
      "${LIVOX_SDK_GIT_REPOSITORY}"
      "${LIVOX_SDK_SOURCE_DIR}"
    RESULT_VARIABLE _livox_sdk_clone_result
    ERROR_VARIABLE _livox_sdk_clone_error)
  if(NOT _livox_sdk_clone_result EQUAL 0)
    message(FATAL_ERROR
      "Failed to clone the pinned Livox SDK: ${_livox_sdk_clone_error}")
  endif()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${LIVOX_SDK_SOURCE_DIR}"
      checkout --detach "${LIVOX_SDK_GIT_COMMIT}"
    RESULT_VARIABLE _livox_sdk_checkout_result
    ERROR_VARIABLE _livox_sdk_checkout_error)
  if(NOT _livox_sdk_checkout_result EQUAL 0)
    message(FATAL_ERROR
      "Failed to check out Livox SDK ${LIVOX_SDK_GIT_COMMIT}: "
      "${_livox_sdk_checkout_error}")
  endif()
endif()

if(NOT EXISTS "${LIVOX_SDK_SOURCE_DIR}/.git")
  message(FATAL_ERROR
    "LIVOX_SDK_SOURCE_DIR is not a Git checkout: ${LIVOX_SDK_SOURCE_DIR}")
endif()

execute_process(
  COMMAND "${GIT_EXECUTABLE}" -C "${LIVOX_SDK_SOURCE_DIR}"
    rev-parse HEAD
  RESULT_VARIABLE _livox_sdk_head_result
  OUTPUT_VARIABLE _livox_sdk_head
  ERROR_VARIABLE _livox_sdk_head_error
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _livox_sdk_head_result EQUAL 0 OR
   NOT "${_livox_sdk_head}" STREQUAL "${LIVOX_SDK_GIT_COMMIT}")
  message(FATAL_ERROR
    "Livox SDK revision mismatch. Expected ${LIVOX_SDK_GIT_COMMIT}, "
    "found '${_livox_sdk_head}' in ${LIVOX_SDK_SOURCE_DIR}. "
    "${_livox_sdk_head_error}")
endif()

execute_process(
  COMMAND "${GIT_EXECUTABLE}" -C "${LIVOX_SDK_SOURCE_DIR}"
    status --porcelain --untracked-files=no
  RESULT_VARIABLE _livox_sdk_status_result
  OUTPUT_VARIABLE _livox_sdk_status
  ERROR_VARIABLE _livox_sdk_status_error
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _livox_sdk_status_result EQUAL 0 OR
   NOT "${_livox_sdk_status}" STREQUAL "")
  message(FATAL_ERROR
    "Pinned Livox SDK checkout has tracked modifications: "
    "'${_livox_sdk_status}'. ${_livox_sdk_status_error}")
endif()

message(STATUS
  "Using pinned Livox SDK ${LIVOX_SDK_GIT_COMMIT} from ${LIVOX_SDK_SOURCE_DIR}")
set(_livox_driver_project_name "${PROJECT_NAME}")
set(PROJECT_NAME "livox_sdk")
add_subdirectory("${LIVOX_SDK_SOURCE_DIR}/sdk_core"
  "${CMAKE_BINARY_DIR}/_deps/livox-sdk-build" EXCLUDE_FROM_ALL)
set(PROJECT_NAME "${_livox_driver_project_name}")
find_package(Threads REQUIRED)
target_link_libraries(livox_sdk_static PUBLIC ${CMAKE_THREAD_LIBS_INIT})
