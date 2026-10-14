// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SENSORS_PHIDGET_SPATIAL_DEVICE_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SENSORS_PHIDGET_SPATIAL_DEVICE_HPP_

#include <phidget22.h>

#include <cstdint>
#include <functional>

#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/phidget_driver/phidget_utils.hpp"

namespace isaacbot_amr_hardware_interface
{

// Thin RAII wrapper over the libphidget22 PhidgetSpatial C API - just the calls PhidgetImuSensor
// needs. It replaces phidgets_api::Spatial: that ROS package is pinned to the
// ros-lyrical-libphidget22 vendor SDK (1.19), which predates the DCC1020 motor controllers, and
// both cannot be loaded into one process alongside the system libphidget22. Every method throws
// std::runtime_error on an SDK error.
class PhidgetSpatialDevice
{

public:

    using DataCallback = std::function<void(
        const double acceleration[3], const double angular_rate[3], const double magnetic_field[3],
        double timestamp)>;

    // Registers the handlers, then opens the channel and blocks until it attaches (or the SDK's
    // default timeout expires). Callbacks run on the Phidget SDK's own thread.
    PhidgetSpatialDevice(
        const std::int32_t serial_number,
        const int hub_port,
        const bool is_hub_port_device,
        DataCallback data_callback,
        std::function<void()> attach_callback,
        std::function<void()> detach_callback);

    ~PhidgetSpatialDevice();

    PHIDGET_UTILS_NO_COPY_NO_MOVE_NO_ASSIGN(PhidgetSpatialDevice)

    std::int32_t getSerialNumber() const;

    void setDataInterval(const std::uint32_t data_interval_ms);

    // Re-zeroes the gyroscope; the device must be stationary for ~2 s afterwards.
    void zero();

    void setCompassCorrectionParameters(
        const double magnetic_field, const double offset0, const double offset1, const double offset2,
        const double gain0, const double gain1, const double gain2, const double t0, const double t1,
        const double t2, const double t3, const double t4, const double t5);

    void setHeatingEnabled(const bool enabled);

private:

    static void CCONV spatialDataHandler(
        PhidgetSpatialHandle phid, void * ctx, const double acceleration[3],
        const double angular_rate[3], const double magnetic_field[3], double timestamp);

    static void CCONV attachHandler(PhidgetHandle phid, void * ctx);

    static void CCONV detachHandler(PhidgetHandle phid, void * ctx);

    PhidgetSpatialHandle handle_{nullptr};

    DataCallback data_callback_;
    std::function<void()> attach_callback_;
    std::function<void()> detach_callback_;
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SENSORS_PHIDGET_SPATIAL_DEVICE_HPP_
