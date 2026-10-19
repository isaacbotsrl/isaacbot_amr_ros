// Copyright 2025 IsaacBot
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

#ifndef ISAACBOT_AMR_DIAG_MANAGER_DOMAIN_PORTS_SYSTEM_METRICS_SOURCE_PORT_HPP_
#define ISAACBOT_AMR_DIAG_MANAGER_DOMAIN_PORTS_SYSTEM_METRICS_SOURCE_PORT_HPP_

#include "isaacbot_amr_diag_manager/domain/system_sample.hpp"

namespace isaacbot_amr_diag_manager::domain
{

/** @brief Input port: measures the host's resource usage. */
class SystemMetricsSourcePort
{
public:
    virtual ~SystemMetricsSourcePort() = default;

    /** Samples every metric once. Never throws; an unreadable metric is left std::nullopt. */
    virtual SystemSample sample() = 0;
};

}  // namespace isaacbot_amr_diag_manager::domain

#endif  // ISAACBOT_AMR_DIAG_MANAGER_DOMAIN_PORTS_SYSTEM_METRICS_SOURCE_PORT_HPP_
