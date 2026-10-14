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

#ifndef ISAACBOT_AMR_LED_INFRASTRUCTURE_LED_CONTROLLER_NODE_HPP_
#define ISAACBOT_AMR_LED_INFRASTRUCTURE_LED_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "isaacbot_amr_msgs/msg/led_animation_catalog.hpp"
#include "isaacbot_amr_msgs/msg/led_state.hpp"
#include "isaacbot_amr_msgs/srv/set_led_animation.hpp"
#include "isaacbot_amr_msgs/srv/stop_led_animation.hpp"

#include "isaacbot_amr_led/application/get_led_state_use_case.hpp"
#include "isaacbot_amr_led/application/render_tick_use_case.hpp"
#include "isaacbot_amr_led/application/set_animation_use_case.hpp"
#include "isaacbot_amr_led/application/stop_animation_use_case.hpp"
#include "isaacbot_amr_led/infrastructure/led_controller_diagnostics.hpp"
#include "isaacbot_amr_led/infrastructure/pluginlib_animation_factory.hpp"
#include "isaacbot_amr_led/led_controller_parameters.hpp"
#include "isaacbot_amr_utils/shutdown_gate.hpp"

namespace isaacbot_amr_led
{

using ImageMsg = sensor_msgs::msg::Image;
using LedAnimationCatalogMsg = isaacbot_amr_msgs::msg::LedAnimationCatalog;
using LedStateMsg = isaacbot_amr_msgs::msg::LedState;
using SetLedAnimationSrv = isaacbot_amr_msgs::srv::SetLedAnimation;
using StopLedAnimationSrv = isaacbot_amr_msgs::srv::StopLedAnimation;

// ROS adapter of the animation side: loads the LED configuration, serves
// led/set_animation and led/stop_animation and publishes one RGBA8 frame per panel on
// led/channel_<n>_frame at controller_frequency (plus a copy on led/channel_<n>_preview at
// preview_publish_rate, for UIs). Reports the loaded
// animations once on led/animations and what every layer plays on led/state
// at state_publish_rate (both latched).
class LedControllerNode : public rclcpp::Node
{

public:

    LedControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~LedControllerNode() {}

private:

    void publishPanelFrame(
        const std::size_t channel, std::vector<std::uint8_t> frame, const std::size_t rows, bool preview_due);

    void setLedAnimationCallback(
        const SetLedAnimationSrv::Request::SharedPtr & request,
        SetLedAnimationSrv::Response::SharedPtr response);

    void stopLedAnimationCallback(
        const StopLedAnimationSrv::Request::SharedPtr & request,
        StopLedAnimationSrv::Response::SharedPtr response);

    void controllerTimerCallback();

    void stateTimerCallback();

    void publishAnimationCatalog(const std::vector<LedAnimationDescription> & animations);

    // Pre-shutdown: cancels the render and state timers.
    void stopTimers();

    // Diagnostics (hardware ID "Bumper Led"). Same default callback group as the render timer
    // and the service, so the segments and the recorded state are never read while they change.
    void diagnoseController(diagnostic_updater::DiagnosticStatusWrapper & status);

    // Declared first: animations created by the factory must be destroyed
    // before its class loader.
    std::shared_ptr<PluginlibAnimationFactory> animation_factory_;

    std::unordered_map<std::size_t, rclcpp::Publisher<ImageMsg>::SharedPtr> panel_publishers_;
    // Empty when preview_publish_rate is 0.
    std::unordered_map<std::size_t, rclcpp::Publisher<ImageMsg>::SharedPtr> preview_publishers_;
    std::chrono::steady_clock::duration preview_period_{};
    std::optional<std::chrono::steady_clock::time_point> last_preview_;

    std::unique_ptr<SetAnimationUseCase> set_animation_use_case_;
    std::unique_ptr<StopAnimationUseCase> stop_animation_use_case_;

    std::unique_ptr<RenderTickUseCase> render_tick_use_case_;

    std::unique_ptr<GetLedStateUseCase> get_led_state_use_case_;

    rclcpp::Publisher<LedAnimationCatalogMsg>::SharedPtr animation_catalog_publisher_;

    rclcpp::Publisher<LedStateMsg>::SharedPtr state_publisher_;

    std::shared_ptr<led_controller::ParamListener> param_listener_;

    led_controller::Params params_;

    rclcpp::Service<SetLedAnimationSrv>::SharedPtr set_led_animation_server_;
    rclcpp::Service<StopLedAnimationSrv>::SharedPtr stop_led_animation_server_;

    rclcpp::TimerBase::SharedPtr controller_timer_;

    rclcpp::TimerBase::SharedPtr state_timer_;

    // After the timers it cancels, so it is unregistered before they go away.
    isaacbot_amr_utils::ros::ShutdownGate shutdown_gate_;

    LedControllerDiagnostics diagnostics_;

    // FrequencyStatusParam holds pointers to these, so they must outlive render_rate_.
    double render_min_hz_ = 0.0;
    double render_max_hz_ = 0.0;
    std::unique_ptr<diagnostic_updater::FrequencyStatus> render_rate_;

    // Last member: destroyed first, so its timer never runs a task on a half-destroyed node.
    std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace isaacbot_amr_led

#endif  // ISAACBOT_AMR_LED_INFRASTRUCTURE_LED_CONTROLLER_NODE_HPP_
