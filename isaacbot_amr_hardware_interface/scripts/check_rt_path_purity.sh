#!/usr/bin/env bash
#
# Copyright 2025 IsaacBot
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Fails if a known-blocking Modbus/Phidget-SDK symbol appears in code that runs on the RT control
# loop thread ticked by controller_manager (see .claude/rules/ros2_control_architecture.md §5):
#   - IsaacbotSystem::read()/write() themselves (src/isaacbot_amr_system/isaacbot_amr_system.cpp).
#   - Every per-variant *_system.cpp file's RT-reachable virtual overrides - updateHwStates(),
#     getSpeedCmd() and updateDriverStateMsg() (the latter from read()'s periodic driver-state
#     block) - since IsaacbotSystem::read()/write() call these via virtual dispatch (see the
#     RT contract comment on the IsaacbotSystem class in isaacbot_amr_system.hpp). This is scanned for every
#     file matching src/isaacbot_amr_system/*_system.cpp, not just ardennes_system.cpp, so a future isaacbot
#     variant (e.g. isaacbot_amr_b1_system.cpp) is covered automatically without editing this script.
#   - The RT-loop decision logic in src/application/isaacbot_amr_control_loop_use_case.cpp, since every
#     method on IsaacbotControlLoopUseCase is reachable from the RT path.
#   - The RT-reachable methods of the isaacbot_amr_driver/ and isaacbot_amr_safety_controller/ adapters behind
#     the domain ports (e.g. PhidgetMotorDriver::sendCmdVel()/readState(), IsaacbotSafetyController::
#     queryControlInterfaceIOStates()) - an explicit, hand-maintained list, see
#     RT_REACHABLE_ADAPTER_METHODS below.
#   - PhidgetImuSensor::read() and the updateExportedStateValues() it calls - the IMU is its own
#     SensorInterface, ticked by the same controller_manager loop. Its SDK callbacks and
#     calibration run on other threads and are not scanned.
#
# Deliberately NOT scanned: defineIsaacbotDriver()/defineIsaacbotController() and any *_system.cpp code
# reachable only from them, since those run during on_configure() (not on the RT path) and are
# expected to construct blocking backends (e.g. `IsaacbotSafetyController(...)`, a real Modbus TCP
# connection). Also NOT scanned: any adapter or sensor method not in
# RT_REACHABLE_ADAPTER_METHODS - a NEW RT-reachable adapter method, or a new driver/adapter
# backend, must be added to that list by hand or it will not be caught here. This is a substring
# scan of specific function bodies, NOT a real call-graph analysis: it catches a blocking call
# written directly in one of the scanned bodies, but not one added several call-levels deep, or
# hidden inside a new driver/adapter implementation. It complements, and does not replace, careful
# review of any change touching the RT path - see the file-level comment on this concern in
# isaacbot_amr_system.hpp. Invoked from CMakeLists.txt as a plain CTest add_test, mirroring
# check_domain_purity.sh.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <package_source_dir>" >&2
    exit 2
fi

PKG_DIR="$1"
RT_CHECK_SYSTEM_DIR="${PKG_DIR}/src/isaacbot_amr_system"
RT_CHECK_SYSTEM_FILE="${RT_CHECK_SYSTEM_DIR}/isaacbot_amr_system.cpp"
CONTROL_LOOP_USE_CASE_FILE="${PKG_DIR}/src/application/isaacbot_amr_control_loop_use_case.cpp"

if [[ ! -f "${RT_CHECK_SYSTEM_FILE}" ]]; then
    echo "check_rt_path_purity: ${RT_CHECK_SYSTEM_FILE} not found" >&2
    exit 1
fi

if [[ ! -f "${CONTROL_LOOP_USE_CASE_FILE}" ]]; then
    echo "check_rt_path_purity: ${CONTROL_LOOP_USE_CASE_FILE} not found" >&2
    exit 1
fi

# Every hardware_interface::SystemInterface implementation lives directly under
# src/isaacbot_amr_system/ as <variant>_system.cpp - isaacbot_amr_system.cpp itself (the generic base) plus one
# file per concrete isaacbot variant (e.g. ardennes_system.cpp).
mapfile -t SYSTEM_FILES < <(find "${RT_CHECK_SYSTEM_DIR}" -maxdepth 1 -type f -name '*_system.cpp' | sort)

if [[ ${#SYSTEM_FILES[@]} -eq 0 ]]; then
    echo "check_rt_path_purity: no *_system.cpp files found under ${RT_CHECK_SYSTEM_DIR}" >&2
    exit 1
fi

# Known-blocking symbols: raw Modbus TCP round-trips and synchronous Phidget SDK calls. Reads/
# writes to hardware on the RT path must instead go through the cached/try_lock ports
# (IsaacbotGpioPort, IsaacbotDriverInterface) and the motor driver's worker hand-off - see the RT
# contract comment on the IsaacbotSystem class in isaacbot_amr_system.hpp.
#
# IMPORTANT for whoever adds a new safety-IO or driver backend (e.g. CAN instead of Modbus, a
# different motor-controller SDK): the *_system.cpp file-glob above picks up a new isaacbot variant
# automatically, but THIS list does not. It is a fixed list of symbol names, not a pattern over
# "whatever the current backend's blocking calls are" - a new backend's blocking entry points must
# be added here by hand, or this script will silently pass while the new backend blocks on the RT
# thread. Grep the new backend's synchronous/network-round-trip calls and add them below.
FORBIDDEN_SYMBOLS=(
    awaitResponse
    sendRequest
    writeDiscreteCoil
    readDiscreteCoil
    readDiscreteContact
    "IsaacbotSafetyController("
    "ModbusDiscreteIoClient("
    "makeModbusTcpDiscreteIoClient("
    # Synchronous Phidget SDK calls. The DCC1020 velocity controller has no _async setter at all,
    # so PhidgetMotorDriver::sendCmdVel() only posts the target to a worker thread, which makes
    # the blocking setTargetVelocity call. The failsafe calls are confined by design to
    # IsaacbotSystem::on_activate() and resetEStopLatch() (see MotorDriverInterface::
    # armFailsafe()/resetFailsafe() in domain/driver.hpp), neither of which this script scans
    # today. Listed here anyway as a regression guard against a future accidental RT-path call.
    PhidgetMotorVelocityController_setTargetVelocity
    PhidgetMotorVelocityController_setEngaged
    PhidgetMotorVelocityController_resetFailsafe
    PhidgetMotorVelocityController_enableFailsafe
)

extract_body() {
    # Prints the lines from the first line matching the given function-signature pattern up to
    # (and including) the first column-0 closing brace after it. Empty output if no match.
    local file="$1"
    local pattern="$2"
    awk -v pat="${pattern}" '
        $0 ~ pat { found = 1 }
        found { print }
        found && /^}/ { exit }
    ' "${file}"
}

status=0

check_body() {
    # Scans $1 (a function body, or "" if the function wasn't found in this file) for every
    # forbidden symbol, attributing violations to the human-readable label $2.
    local body="$1"
    local label="$2"

    if [[ -z "${body}" ]]; then
        return
    fi

    for symbol in "${FORBIDDEN_SYMBOLS[@]}"; do
        if grep -qF -- "${symbol}" <<< "${body}"; then
            echo "RT PATH PURITY VIOLATION: ${label} calls blocking symbol '${symbol}'" >&2
            status=1
        fi
    done
}

read_body=$(extract_body "${RT_CHECK_SYSTEM_FILE}" 'return_type IsaacbotSystem::read')
write_body=$(extract_body "${RT_CHECK_SYSTEM_FILE}" 'return_type IsaacbotSystem::write')

if [[ -z "${read_body}" || -z "${write_body}" ]]; then
    echo "check_rt_path_purity: could not locate read()/write() in ${RT_CHECK_SYSTEM_FILE}" >&2
    exit 1
fi

check_body "${read_body}" "IsaacbotSystem::read()"
check_body "${write_body}" "IsaacbotSystem::write()"

# RT-reachable virtual overrides that a per-variant *_system.cpp defines - see the class-header
# comment above defineIsaacbotDriver()/updateHwStates()/getSpeedCmd() declarations in
# isaacbot_amr_system.hpp for why each is RT-reachable via read()/write()'s virtual dispatch.
# No trailing "(" in these patterns - awk's dynamic regexps need it backslash-escaped, and the
# method name alone is already unambiguous (mirrors the parenthesis-free patterns used for
# IsaacbotSystem::read/write above).
RT_REACHABLE_OVERRIDE_PATTERNS=(
    '::updateHwStates'
    '::getSpeedCmd'
    '::updateDriverStateMsg'
)

for system_file in "${SYSTEM_FILES[@]}"; do
    if [[ "${system_file}" == "${RT_CHECK_SYSTEM_FILE}" ]]; then
        continue  # read()/write() already scanned above.
    fi

    for pattern in "${RT_REACHABLE_OVERRIDE_PATTERNS[@]}"; do
        body=$(extract_body "${system_file}" "${pattern}")
        check_body "${body}" "${system_file} (${pattern})"
    done
done

check_body "$(cat "${CONTROL_LOOP_USE_CASE_FILE}")" "IsaacbotControlLoopUseCase"

# Blocking calls matched by pattern rather than by fixed name - applied to the adapter bodies
# below only. Any Phidget SDK setter (`Phidget*_set*(`) is a synchronous USB round-trip; an
# `_async` variant (`Phidget*_set*_async(`) would not match, because [A-Za-z]* can't span the `_`
# before `async`.
FORBIDDEN_ADAPTER_PATTERNS=(
    'Phidget[A-Za-z]*_set[A-Za-z]*[[:space:]]*\('
    'Phidget_open'
    'Phidget_close'
    'sleep_for'
    'operationWithAttempts'
    # A blocking mutex acquisition (`lock_guard<...> lck(mtx);` and the unique_lock/scoped_lock
    # equivalents) - the RT path must try_lock instead, since std::mutex has no priority
    # inheritance. The try_to_lock/defer_lock/adopt_lock forms carry a second argument (a comma),
    # so they don't match.
    '(lock_guard|unique_lock|scoped_lock)(<[^>]*>)?[[:space:]]+[A-Za-z_]+[(][A-Za-z_]+[)]'
    # Waiting or thread hand-off primitives that can block (the motor command path posts to a
    # LatestValueMailbox instead, whose post() is lock-free).
    'condition_variable'
    '[.]wait(_for|_until)?[[:space:]]*\('
    'std::thread'
)

# RT-reachable methods of the driver / safety-IO adapters behind the domain ports, reached from
# IsaacbotSystem::read()/write() and the *_system.cpp overrides via IsaacbotDriverInterface,
# MotorDriverInterface, IsaacbotGpioPort and EmergencyStopInterface. Format: "<file>|<pattern>".
# Patterns are anchored to a column-0 definition line ('^([A-Za-z].*[ &*])?' = an optional return
# type on the same line) so a comment or call site mentioning the method isn't mistaken for its
# definition, and end in '[(]' so e.g. isFailsafeTripped doesn't match isFailsafeTrippedReturnCode.
# Unlike the *_system.cpp overrides above, a missing definition is an error here: a rename must
# update this list, not silently drop the method from the scan.
#
# Deliberately NOT listed: ContactCoilHandler::queryControlInterfaceIOStates()/contactCoilHandler
# Thread() (the Modbus worker thread - blocking by design, handed off via getIoState()'s
# try_lock), and the on_configure()/on_activate()/service-callback-only methods (initialize(),
# activate(), armFailsafe(), resetFailsafe(), eStopUserBtnTrigger(), eStopLatchReset(), ...).
DRIVER_DIR="${PKG_DIR}/src/isaacbot_amr_driver"
SAFETY_DIR="${PKG_DIR}/src/isaacbot_amr_safety_controller"
SENSORS_DIR="${PKG_DIR}/src/isaacbot_amr_sensors"
RT_REACHABLE_ADAPTER_METHODS=(
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetMotorDriver::readState[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetMotorDriver::sendCmdVel[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetMotorDriver::isCommunicationError[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetMotorDriver::isFailsafeTripped[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetMotorDriver::motorTargetRpm[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetMotorDriver::getFaultFlags[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetMotorDriver::getCommandStats[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetDriver::readState[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_motor_driver.cpp|^([A-Za-z].*[ &*])?PhidgetDriver::isCommunicationError[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::updateCommunicationStatus[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::updateMotorsState[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::updateDriversState[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::attemptErrorFlagReset[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::isCommunicationError[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::isMotorStatesDataTimedOut[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::isDriverStateDataTimedOut[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::isFlagError[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::isFailsafeTripped[(]"
    "${DRIVER_DIR}/phidget_driver/phidget_isaacbot_amr_driver.cpp|^([A-Za-z].*[ &*])?PhidgetIsaacbotDriver::getData[(]"
    "${DRIVER_DIR}/ardennes_driver.cpp|^([A-Za-z].*[ &*])?ArdennesDriver::sendSpeedCmd[(]"
    "${DRIVER_DIR}/ardennes_driver.cpp|^([A-Za-z].*[ &*])?ArdennesDriver::attemptErrorFlagReset[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller.cpp|^([A-Za-z].*[ &*])?IsaacbotSafetyController::queryControlInterfaceIOStates[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller.cpp|^([A-Za-z].*[ &*])?IsaacbotSafetyController::isPinActive[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller.cpp|^([A-Za-z].*[ &*])?ContactCoilHandler::getIoState[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller.cpp|^([A-Za-z].*[ &*])?IsaacbotSafetyController::requestMotorDriverFault[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller.cpp|^([A-Za-z].*[ &*])?ContactCoilHandler::requestMotorDriverFault[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller_e_stop_io.cpp|^([A-Za-z].*[ &*])?IsaacbotSafetyControllerEStopIo::requestMotorDriverFault[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller_gpio_adapter.cpp|^([A-Za-z].*[ &*])?IsaacbotSafetyControllerGpioAdapter::queryControlInterfaceIOStates[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller_e_stop_io.cpp|^([A-Za-z].*[ &*])?IsaacbotSafetyControllerEStopIo::isUserButtonActive[(]"
    "${SAFETY_DIR}/isaacbot_amr_safety_controller_e_stop_io.cpp|^([A-Za-z].*[ &*])?IsaacbotSafetyControllerEStopIo::isLatchActive[(]"
    "${SENSORS_DIR}/phidget_imu_sensor.cpp|^([A-Za-z].*[ &*])?PhidgetImuSensor::read[(]"
    "${SENSORS_DIR}/phidget_imu_sensor.cpp|^([A-Za-z].*[ &*])?PhidgetImuSensor::updateExportedStateValues[(]"
)

for entry in "${RT_REACHABLE_ADAPTER_METHODS[@]}"; do
    file="${entry%%|*}"
    pattern="${entry#*|}"
    method="${pattern##*)?}"
    label="${file#"${PKG_DIR}/"} (${method%'[(]'}())"

    body=$(extract_body "${file}" "${pattern}")
    if [[ -z "${body}" ]]; then
        echo "check_rt_path_purity: could not locate ${label} - update RT_REACHABLE_ADAPTER_METHODS" >&2
        status=1
        continue
    fi

    check_body "${body}" "${label}"

    for regex in "${FORBIDDEN_ADAPTER_PATTERNS[@]}"; do
        if grep -qE -- "${regex}" <<< "${body}"; then
            echo "RT PATH PURITY VIOLATION: ${label} matches blocking pattern '${regex}'" >&2
            status=1
        fi
    done
done

if [[ ${status} -eq 0 ]]; then
    echo "check_rt_path_purity: OK (${#SYSTEM_FILES[@]} *_system.cpp file(s) + control loop use case + ${#RT_REACHABLE_ADAPTER_METHODS[@]} adapter method(s) checked)"
fi

exit "${status}"
