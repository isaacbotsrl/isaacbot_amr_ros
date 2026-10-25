// Copyright 2026 IsaacBot
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

// ROS-level tests for CommandFreshnessNode: that fresh commands really reach the twist_mux input
// and late ones really do not. The filter's rules are covered by its unit tests; the topic names,
// QoS and the stamp/wall-clock plumbing live in the node.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "isaacbot_amr_twist_mux/infrastructure/command_freshness_node.hpp"

namespace isaacbot_amr_twist_mux
{
namespace
{

using namespace std::chrono_literals;

// The same QoS twist_mux subscribes with (SystemDefaultsQoS: reliable, volatile).
rclcpp::QoS commandQos() { return rclcpp::QoS(rclcpp::KeepLast(10)).reliable(); }

class CommandFreshnessNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        harness_ = std::make_shared<rclcpp::Node>("command_freshness_test_harness");

        command_pub_ = harness_->create_publisher<geometry_msgs::msg::TwistStamped>(
            "teleop_driver_interface_cmd_vel_stamped", commandQos());

        fresh_sub_ = harness_->create_subscription<geometry_msgs::msg::TwistStamped>(
            "teleop_driver_interface_cmd_vel_fresh_stamped", commandQos(),
            [this](const geometry_msgs::msg::TwistStamped & msg) {
                received_.push_back(msg.twist.linear.x);
                received_msgs_.push_back(msg);
            });

        node_ = std::make_shared<CommandFreshnessNode>("isaacbot_amr_command_freshness_node");

        executor_.add_node(harness_);
        executor_.add_node(node_);
    }

    void TearDown() override
    {
        executor_.remove_node(node_);
        executor_.remove_node(harness_);
        node_.reset();
        harness_.reset();
    }

    template <typename PredicateT>
    bool spinUntil(PredicateT predicate, const std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();

            if (predicate()) {
                return true;
            }

            std::this_thread::sleep_for(2ms);
        }

        executor_.spin_some();
        return predicate();
    }

    void spinFor(const std::chrono::milliseconds duration)
    {
        spinUntil([] { return false; }, duration);
    }

    // Publishes a command stamped `age` before now on the wall clock, tagged by `linear_x`.
    void publish(const double linear_x, const std::chrono::milliseconds age = 0ms)
    {
        const auto stamp = std::chrono::system_clock::now() - age;
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            stamp.time_since_epoch()).count();

        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp.sec = static_cast<int32_t>(ns / 1000000000);
        msg.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000);
        msg.header.frame_id = "base_link";
        msg.twist.linear.x = linear_x;
        command_pub_->publish(msg);
    }

    bool received(const double linear_x) const
    {
        for (const double value : received_) {
            if (value == linear_x) {
                return true;
            }
        }
        return false;
    }

    // Publishes fresh commands until the node passes one on, so discovery is done before the
    // test proper: a command lost to discovery would look like a dropped one. `clock_offset` puts
    // the sender's clock that far behind the isaacbot's.
    void establishBaseline(const std::chrono::milliseconds clock_offset = 0ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline && received_.empty()) {
            publish(0.01, clock_offset);
            spinFor(50ms);
        }
        ASSERT_FALSE(received_.empty()) << "no command made it through the node";
    }

    std::shared_ptr<rclcpp::Node> harness_;
    std::shared_ptr<CommandFreshnessNode> node_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr fresh_sub_;

    std::vector<double> received_;
    std::vector<geometry_msgs::msg::TwistStamped> received_msgs_;
};

TEST_F(CommandFreshnessNodeTest, FreshCommandsArePassedOn)
{
    establishBaseline();

    publish(0.5);
    EXPECT_TRUE(spinUntil([this] { return received(0.5); }, 2000ms));
}

// A browser whose clock runs 1.6 s behind (the Ardennes Driver UI, 2026-10-25): its commands are
// fresh by its own baseline and must come out stamped with the isaacbot's clock, or diff_drive
// drops every one as older than cmd_vel_timeout and the wheels never move.
TEST_F(CommandFreshnessNodeTest, CommandsFromALaggingClockAreRestampedWithTheIsaacbotsClock)
{
    establishBaseline(1600ms);

    publish(0.6, 1600ms);
    ASSERT_TRUE(spinUntil([this] { return received(0.6); }, 2000ms));

    const auto & msg = received_msgs_.back();
    const double now_s = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const double stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

    EXPECT_NEAR(stamp_s, now_s, 0.5) << "the output still carries the sender's clock";
    EXPECT_EQ(msg.header.frame_id, "base_link");
    EXPECT_DOUBLE_EQ(msg.twist.linear.x, 0.6);
}

// A command stamped 2 s before it arrives sat in a buffer: it must not reach twist_mux.
TEST_F(CommandFreshnessNodeTest, LateCommandIsDropped)
{
    establishBaseline();

    publish(0.7, 2000ms);
    spinFor(300ms);
    EXPECT_FALSE(received(0.7)) << "a command 2 s late was passed on";

    // And the node keeps passing fresh ones afterwards.
    publish(0.9);
    EXPECT_TRUE(spinUntil([this] { return received(0.9); }, 2000ms));
}

TEST_F(CommandFreshnessNodeTest, UnstampedCommandIsDropped)
{
    establishBaseline();

    geometry_msgs::msg::TwistStamped msg;
    msg.twist.linear.x = 0.3;
    command_pub_->publish(msg);
    spinFor(300ms);

    EXPECT_FALSE(received(0.3)) << "an unstamped command was passed on";
}

}  // namespace
}  // namespace isaacbot_amr_twist_mux
