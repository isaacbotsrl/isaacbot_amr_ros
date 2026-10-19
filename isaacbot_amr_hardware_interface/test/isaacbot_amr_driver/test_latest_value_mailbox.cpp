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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <optional>
#include <thread>

#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/latest_value_mailbox.hpp"

namespace isaacbot_amr_hardware_interface
{

using std::chrono::milliseconds;
using std::chrono::seconds;

TEST(LatestValueMailboxTest, TakeTimesOutWhenNothingWasPosted)
{
    LatestValueMailbox<float> mailbox;

    EXPECT_FALSE(mailbox.take(milliseconds(1)).has_value());
}

TEST(LatestValueMailboxTest, DeliversAPostedValueOnce)
{
    LatestValueMailbox<float> mailbox;
    mailbox.post(12.5f);

    const auto taken = mailbox.take(milliseconds(1));
    ASSERT_TRUE(taken.has_value());
    EXPECT_FLOAT_EQ(taken->value, 12.5f);
    EXPECT_EQ(taken->coalesced, 0u);

    EXPECT_FALSE(mailbox.take(milliseconds(1)).has_value());
}

TEST(LatestValueMailboxTest, CoalescesToTheNewestValue)
{
    LatestValueMailbox<float> mailbox;

    for (int i = 1; i <= 5; ++i) {
        mailbox.post(static_cast<float>(i));
    }

    const auto taken = mailbox.take(milliseconds(1));
    ASSERT_TRUE(taken.has_value());
    EXPECT_FLOAT_EQ(taken->value, 5.0f);
    EXPECT_EQ(taken->coalesced, 4u);
    EXPECT_EQ(mailbox.postedCount(), 5u);
}

TEST(LatestValueMailboxTest, RepeatedIdenticalValuesAreEachDelivered)
{
    // The worker's send is what feeds the board's failsafe, so an unchanged target (e.g. 0 at
    // standstill) must still be delivered every time it is posted.
    LatestValueMailbox<float> mailbox;

    mailbox.post(0.0f);
    ASSERT_TRUE(mailbox.take(milliseconds(1)).has_value());

    mailbox.post(0.0f);
    ASSERT_TRUE(mailbox.take(milliseconds(1)).has_value());
}

TEST(LatestValueMailboxTest, WakesAWaitingConsumer)
{
    LatestValueMailbox<float> mailbox;
    std::promise<void> waiting;

    auto consumer = std::async(std::launch::async, [&] {
        waiting.set_value();
        return mailbox.take(seconds(10));
    });

    waiting.get_future().wait();
    mailbox.post(3.0f);

    ASSERT_EQ(consumer.wait_for(seconds(5)), std::future_status::ready);
    const auto taken = consumer.get();
    ASSERT_TRUE(taken.has_value());
    EXPECT_FLOAT_EQ(taken->value, 3.0f);
}

TEST(LatestValueMailboxTest, StopReleasesAWaitingConsumer)
{
    LatestValueMailbox<float> mailbox;
    std::promise<void> waiting;

    auto consumer = std::async(std::launch::async, [&] {
        waiting.set_value();
        return mailbox.take(seconds(10));
    });

    waiting.get_future().wait();
    mailbox.stop();

    ASSERT_EQ(consumer.wait_for(seconds(5)), std::future_status::ready);
    EXPECT_FALSE(consumer.get().has_value());
    EXPECT_TRUE(mailbox.isStopped());
}

TEST(LatestValueMailboxTest, RestartAfterStopDeliversAgain)
{
    LatestValueMailbox<float> mailbox;
    mailbox.stop();
    mailbox.post(1.0f);
    EXPECT_FALSE(mailbox.take(milliseconds(1)).has_value());

    mailbox.restart();
    mailbox.post(2.0f);

    const auto taken = mailbox.take(milliseconds(1));
    ASSERT_TRUE(taken.has_value());
    EXPECT_FLOAT_EQ(taken->value, 2.0f);
    EXPECT_EQ(taken->coalesced, 1u);
}

TEST(LatestValueMailboxTest, ConcurrentConsumerNeverSeesAValueGoBackwards)
{
    // Producer posts 1..N as fast as it can; the consumer must never see a value go backwards and
    // must end at N, with every post accounted for as delivered or coalesced. (A value can be
    // delivered twice: post() stores the value before bumping the count, so a take() can pair the
    // newest value with an older count - a harmless duplicate send.)
    constexpr std::uint64_t kCount = 200000;
    LatestValueMailbox<std::uint64_t> mailbox;

    auto consumer = std::async(std::launch::async, [&] {
        std::uint64_t last = 0;
        std::uint64_t delivered = 0;
        std::uint64_t coalesced = 0;

        while (last < kCount) {
            const auto taken = mailbox.take(seconds(5));
            if (!taken) {
                return false;
            }
            if (taken->value < last) {
                return false;
            }
            last = taken->value;
            ++delivered;
            coalesced += taken->coalesced;
        }

        return delivered + coalesced == kCount;
    });

    for (std::uint64_t i = 1; i <= kCount; ++i) {
        mailbox.post(i);
    }

    ASSERT_EQ(consumer.wait_for(seconds(30)), std::future_status::ready);
    EXPECT_TRUE(consumer.get());
}

}  // namespace isaacbot_amr_hardware_interface
