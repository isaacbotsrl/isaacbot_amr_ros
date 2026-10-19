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

#ifndef ISAACBOT_AMR_LED_INFRASTRUCTURE_PLUGINLIB_ANIMATION_FACTORY_HPP_
#define ISAACBOT_AMR_LED_INFRASTRUCTURE_PLUGINLIB_ANIMATION_FACTORY_HPP_

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"

#include "isaacbot_amr_led/domain/animation/animation.hpp"
#include "isaacbot_amr_led/domain/ports/animation_factory.hpp"

namespace isaacbot_amr_led
{

// Loads animation plugins exported against isaacbot_amr_led::Animation. Must outlive
// every animation it created.
class PluginlibAnimationFactory : public IAnimationFactory
{

public:

    PluginlibAnimationFactory();

    std::shared_ptr<Animation> create(const std::string & type) override;

private:

    pluginlib::ClassLoader<Animation> loader_;
};

}  // namespace isaacbot_amr_led

#endif  // ISAACBOT_AMR_LED_INFRASTRUCTURE_PLUGINLIB_ANIMATION_FACTORY_HPP_
