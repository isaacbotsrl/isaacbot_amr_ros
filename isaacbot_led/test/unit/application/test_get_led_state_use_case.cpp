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

#include <memory>

#include "gtest/gtest.h"

#include "isaacbot_led/application/get_led_state_use_case.hpp"

#include "../test_helpers.hpp"

using isaacbot_led::GetLedStateUseCase;
using isaacbot_led::LedSegment;
using isaacbot_led::LedSegmentConfig;
using isaacbot_led::test::makeStub;

TEST(GetLedStateUseCase, ReportsEveryLayerOfEverySegmentSortedByName)
{
    isaacbot_led::SegmentMap segments;
    segments["rear"] = std::make_shared<LedSegment>(LedSegmentConfig{2, 0, 1});
    segments["front"] = std::make_shared<LedSegment>(LedSegmentConfig{1, 0, 1});

    auto animation = makeStub(2);
    animation->setInfo({1, "READY", ""});
    segments["front"]->setAnimation(animation, true, isaacbot_led::STATE);

    const auto snapshot = GetLedStateUseCase(segments).execute();

    ASSERT_EQ(snapshot.segments.size(), 2u);
    EXPECT_EQ(snapshot.segments[0].name, "front");
    EXPECT_EQ(snapshot.segments[0].channel, 1u);
    EXPECT_EQ(snapshot.segments[1].name, "rear");

    const auto & layers = snapshot.segments[0].layers;
    ASSERT_EQ(layers.size(), 4u);
    EXPECT_EQ(layers[0].priority, isaacbot_led::ERROR);
    EXPECT_EQ(layers[3].priority, isaacbot_led::STATE);
    EXPECT_FALSE(layers[0].status.has_value());
    ASSERT_TRUE(layers[3].status.has_value());
    EXPECT_EQ(layers[3].status->info.name, "READY");
    EXPECT_TRUE(layers[3].status->repeating);

    for (const auto & layer : snapshot.segments[1].layers) {
        EXPECT_FALSE(layer.status.has_value());
    }
}
