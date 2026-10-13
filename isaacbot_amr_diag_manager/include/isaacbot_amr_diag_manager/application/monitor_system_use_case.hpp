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

#ifndef ISAACBOT_AMR_DIAG_MANAGER_APPLICATION_MONITOR_SYSTEM_USE_CASE_HPP_
#define ISAACBOT_AMR_DIAG_MANAGER_APPLICATION_MONITOR_SYSTEM_USE_CASE_HPP_

#include <memory>

#include "isaacbot_amr_diag_manager/domain/ports/system_metrics_source_port.hpp"
#include "isaacbot_amr_diag_manager/domain/ports/system_status_publisher_port.hpp"
#include "isaacbot_amr_diag_manager/domain/thresholds.hpp"

namespace isaacbot_amr_diag_manager::application
{

/**
 * @brief Samples the host once per tick, grades it and publishes both.
 * @details Stateless: thresholds are passed per tick so parameter changes apply immediately.
 */
class MonitorSystemUseCase
{
public:
    MonitorSystemUseCase(
        std::shared_ptr<domain::SystemMetricsSourcePort> metrics_source,
        std::shared_ptr<domain::SystemStatusPublisherPort> publisher);

    void tick(const domain::SystemHealthThresholds & thresholds);

private:
    std::shared_ptr<domain::SystemMetricsSourcePort> metrics_source_;
    std::shared_ptr<domain::SystemStatusPublisherPort> publisher_;
};

}  // namespace isaacbot_amr_diag_manager::application

#endif  // ISAACBOT_AMR_DIAG_MANAGER_APPLICATION_MONITOR_SYSTEM_USE_CASE_HPP_
