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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_LATEST_VALUE_MAILBOX_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_LATEST_VALUE_MAILBOX_HPP_

#include <semaphore.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <optional>
#include <stdexcept>

namespace isaacbot_amr_hardware_interface
{

// Single-producer / single-consumer hand-off of "the newest value" from the RT thread to a worker
// that performs a blocking call with it (e.g. a synchronous SDK setter that has no _async form).
// Values posted faster than the worker takes them are coalesced - only the latest is delivered,
// and the number skipped is reported - so the worker never acts on a stale command.
//
// post() takes no lock and never blocks: two atomic stores and a POSIX sem_post(), which is an
// atomic increment plus a futex wake only when the worker is waiting (async-signal-safe, no
// internal mutex - unlike pthread_cond_signal, whose internal lock a preempted worker could hold).
template <typename T>
class LatestValueMailbox
{

public:

    static_assert(std::atomic<T>::is_always_lock_free, "post() must stay lock-free on the RT path");

    struct Taken
    {
        T value;
        // Values posted since the previous take() that were superseded before being taken.
        std::uint64_t coalesced;
    };

    LatestValueMailbox()
    {
        if (sem_init(&wakeup_, 0, 0) != 0) {
            throw std::runtime_error("LatestValueMailbox: sem_init() failed");
        }
    }

    ~LatestValueMailbox()
    {
        sem_destroy(&wakeup_);
    }

    LatestValueMailbox(const LatestValueMailbox &) = delete;
    LatestValueMailbox & operator=(const LatestValueMailbox &) = delete;

    // RT side. Lock-free, never blocks.
    void post(const T value)
    {
        value_.store(value, std::memory_order_relaxed);
        posted_.fetch_add(1, std::memory_order_release);
        sem_post(&wakeup_);
    }

    // Worker side (one consumer thread only). Returns the newest value once one has been posted
    // since the previous take(), std::nullopt if `timeout` passes first or stop() was called.
    std::optional<Taken> take(const std::chrono::nanoseconds timeout)
    {
        timespec deadline{};
        clock_gettime(CLOCK_MONOTONIC, &deadline);
        const auto total_ns = static_cast<std::int64_t>(deadline.tv_nsec) + timeout.count();
        deadline.tv_sec += static_cast<time_t>(total_ns / 1000000000);
        deadline.tv_nsec = static_cast<long>(total_ns % 1000000000);  // NOLINT(runtime/int)

        while (!stopped_.load() && !hasNew()) {
            if (sem_clockwait(&wakeup_, CLOCK_MONOTONIC, &deadline) != 0 && errno == ETIMEDOUT) {
                break;
            }
        }

        // Wakeups for values already superseded would only cause empty spins later.
        while (sem_trywait(&wakeup_) == 0) {
        }

        if (stopped_.load() || !hasNew()) {
            return std::nullopt;
        }

        const std::uint64_t posted = posted_.load(std::memory_order_acquire);
        const T value = value_.load(std::memory_order_relaxed);
        const std::uint64_t coalesced = posted - taken_ - 1;
        taken_ = posted;

        return Taken{value, coalesced};
    }

    // Wakes a waiting take() and makes every later take() return std::nullopt.
    void stop()
    {
        stopped_.store(true);
        sem_post(&wakeup_);
    }

    // Re-arms after stop(), keeping the value; values posted while stopped count as coalesced.
    void restart()
    {
        stopped_.store(false);
    }

    bool isStopped() const
    {
        return stopped_.load();
    }

    std::uint64_t postedCount() const
    {
        return posted_.load(std::memory_order_relaxed);
    }

private:

    bool hasNew() const
    {
        return posted_.load(std::memory_order_acquire) != taken_;
    }

    std::atomic<T> value_{};
    std::atomic<std::uint64_t> posted_{0};
    // Only touched by the consumer thread.
    std::uint64_t taken_{0};
    std::atomic<bool> stopped_{false};

    sem_t wakeup_;
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_LATEST_VALUE_MAILBOX_HPP_
