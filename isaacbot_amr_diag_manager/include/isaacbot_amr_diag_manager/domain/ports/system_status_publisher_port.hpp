// Copyright 2025 Mechatronics Academy
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

#ifndef ISAACBOT_DIAG_MANAGER_DOMAIN_PORTS_SYSTEM_STATUS_PUBLISHER_PORT_HPP_
#define ISAACBOT_DIAG_MANAGER_DOMAIN_PORTS_SYSTEM_STATUS_PUBLISHER_PORT_HPP_

#include "isaacbot_amr_diag_manager/domain/health_report.hpp"
#include "isaacbot_amr_diag_manager/domain/system_sample.hpp"

namespace isaacbot_amr_diag_manager::domain
{

/** @brief Output port: makes a system sample and its health grade available to the system. */
class SystemStatusPublisherPort
{
public:
    virtual ~SystemStatusPublisherPort() = default;

    virtual void publish(const SystemSample & sample, const HealthReport & health) = 0;
};

}  // namespace isaacbot_amr_diag_manager::domain

#endif  // ISAACBOT_DIAG_MANAGER_DOMAIN_PORTS_SYSTEM_STATUS_PUBLISHER_PORT_HPP_
