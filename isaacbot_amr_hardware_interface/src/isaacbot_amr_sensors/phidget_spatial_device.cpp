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

#include "isaacbot_amr_hardware_interface/isaacbot_amr_sensors/phidget_spatial_device.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace isaacbot_amr_hardware_interface
{

namespace
{

void throwOnError(const PhidgetReturnCode ret, const std::string & what)
{
    if (ret != EPHIDGET_OK) {
        throw std::runtime_error(what + " failed: " + phidgetReturnCodeToString(ret));
    }
}

}  // namespace

PhidgetSpatialDevice::PhidgetSpatialDevice(
    const std::int32_t serial_number,
    const int hub_port,
    const bool is_hub_port_device,
    DataCallback data_callback,
    std::function<void()> attach_callback,
    std::function<void()> detach_callback)
: data_callback_(std::move(data_callback)),
  attach_callback_(std::move(attach_callback)),
  detach_callback_(std::move(detach_callback))
{
    throwOnError(PhidgetSpatial_create(&handle_), "PhidgetSpatial_create()");

    try {
        throwOnError(
            PhidgetSpatial_setOnSpatialDataHandler(handle_, spatialDataHandler, this),
            "PhidgetSpatial_setOnSpatialDataHandler()");
        throwOnError(
            Phidget_setOnAttachHandler(reinterpret_cast<PhidgetHandle>(handle_), attachHandler, this),
            "Phidget_setOnAttachHandler()");
        throwOnError(
            Phidget_setOnDetachHandler(reinterpret_cast<PhidgetHandle>(handle_), detachHandler, this),
            "Phidget_setOnDetachHandler()");

        openWaitForAttachment(
            reinterpret_cast<PhidgetHandle>(handle_), serial_number, hub_port, is_hub_port_device, 0);
    } catch (...) {
        auto handle = reinterpret_cast<PhidgetHandle>(handle_);
        closeAndDelete(&handle);
        throw;
    }
}

PhidgetSpatialDevice::~PhidgetSpatialDevice()
{
    auto handle = reinterpret_cast<PhidgetHandle>(handle_);
    closeAndDelete(&handle);
}

std::int32_t PhidgetSpatialDevice::getSerialNumber() const
{
    std::int32_t serial_number = 0;
    throwOnError(
        Phidget_getDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(handle_), &serial_number),
        "Phidget_getDeviceSerialNumber()");

    return serial_number;
}

void PhidgetSpatialDevice::setDataInterval(const std::uint32_t data_interval_ms)
{
    throwOnError(
        PhidgetSpatial_setDataInterval(handle_, data_interval_ms), "PhidgetSpatial_setDataInterval()");
}

void PhidgetSpatialDevice::zero()
{
    throwOnError(PhidgetSpatial_zeroGyro(handle_), "PhidgetSpatial_zeroGyro()");
}

void PhidgetSpatialDevice::setCompassCorrectionParameters(
    const double magnetic_field, const double offset0, const double offset1, const double offset2,
    const double gain0, const double gain1, const double gain2, const double t0, const double t1,
    const double t2, const double t3, const double t4, const double t5)
{
    throwOnError(
        PhidgetSpatial_setMagnetometerCorrectionParameters(
            handle_, magnetic_field, offset0, offset1, offset2, gain0, gain1, gain2, t0, t1, t2, t3, t4,
            t5),
        "PhidgetSpatial_setMagnetometerCorrectionParameters()");
}

void PhidgetSpatialDevice::setHeatingEnabled(const bool enabled)
{
    throwOnError(
        PhidgetSpatial_setHeatingEnabled(handle_, enabled ? 1 : 0), "PhidgetSpatial_setHeatingEnabled()");
}

void CCONV PhidgetSpatialDevice::spatialDataHandler(
    PhidgetSpatialHandle /*phid*/, void * ctx, const double acceleration[3],
    const double angular_rate[3], const double magnetic_field[3], double timestamp)
{
    auto * self = static_cast<PhidgetSpatialDevice *>(ctx);

    if (self->data_callback_) {
        self->data_callback_(acceleration, angular_rate, magnetic_field, timestamp);
    }
}

void CCONV PhidgetSpatialDevice::attachHandler(PhidgetHandle /*phid*/, void * ctx)
{
    auto * self = static_cast<PhidgetSpatialDevice *>(ctx);

    if (self->attach_callback_) {
        self->attach_callback_();
    }
}

void CCONV PhidgetSpatialDevice::detachHandler(PhidgetHandle /*phid*/, void * ctx)
{
    auto * self = static_cast<PhidgetSpatialDevice *>(ctx);

    if (self->detach_callback_) {
        self->detach_callback_();
    }
}

}  // namespace isaacbot_amr_hardware_interface
