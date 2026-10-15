// Copyright 2025 Enactic, Inc.
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

#pragma once
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>

struct PeriodicDeadlinePlan {
    bool overrun = false;
    int skipped_cycles = 0;
    timespec next{};
};

inline void add_timespec_microseconds(timespec* time, int period_us) {
    if (time == nullptr || period_us <= 0) {
        return;
    }
    const int64_t added_ns = static_cast<int64_t>(period_us) * 1000;
    int64_t nsec = static_cast<int64_t>(time->tv_nsec) + added_ns;
    time->tv_sec += static_cast<time_t>(nsec / 1000000000LL);
    time->tv_nsec = static_cast<long>(nsec % 1000000000LL);
}

inline int64_t timespec_delta_ns(const timespec& later, const timespec& earlier) {
    return (static_cast<int64_t>(later.tv_sec) - static_cast<int64_t>(earlier.tv_sec)) * 1000000000LL +
           (static_cast<int64_t>(later.tv_nsec) - static_cast<int64_t>(earlier.tv_nsec));
}

// `scheduled` is when the next callback was due. If `now` is already past it,
// do not replay the missed periods. The next wake is one period after `now`,
// and skipped_cycles is how many callbacks are not run. Callers keep the
// previous reference by simply not invoking on_timer for those cycles.
inline PeriodicDeadlinePlan plan_next_deadline(const timespec& scheduled, const timespec& now,
                                               int period_us) {
    PeriodicDeadlinePlan plan;
    plan.next = scheduled;
    if (period_us <= 0) {
        plan.next = now;
        return plan;
    }
    const int64_t behind_ns = timespec_delta_ns(now, scheduled);
    if (behind_ns <= 0) {
        return plan;
    }
    const int64_t period_ns = static_cast<int64_t>(period_us) * 1000;
    int skipped = static_cast<int>(behind_ns / period_ns);
    if (skipped < 1) {
        skipped = 1;
    }
    plan.overrun = true;
    plan.skipped_cycles = skipped;
    plan.next = now;
    add_timespec_microseconds(&plan.next, period_us);
    return plan;
}

class PeriodicTimerThread {
public:
    explicit PeriodicTimerThread(double hz = 1000.0) : is_running_(false) {
        if (hz <= 0.0) {
            throw std::invalid_argument("Hz must be positive");
        }
        period_us_.store(static_cast<int>(1e6 / hz));
    }

    virtual ~PeriodicTimerThread() { stop_thread(); }

    virtual void start_thread() { start_thread_base(); }

    virtual void stop_thread() { stop_thread_base(); }

    int64_t get_elapsed_time_us() const { return last_elapsed_us_.load(); }

    void set_period(double hz) {
        if (hz > 0.0) {
            period_us_.store(static_cast<int>(1e6 / hz));
        }
    }

protected:
    virtual void on_timer() = 0;

    // Records a realtime request. It is applied on the periodic thread, not on
    // the thread that called start_thread(). Subclasses that override
    // before_start and do not call this base do not request realtime priority.
    virtual void before_start() { set_thread_priority(50); }

    virtual void after_stop() {}

    // Records the priority. pthread_setschedparam runs once, inside the worker.
    void set_thread_priority(int priority) {
        if (priority > 0) {
            realtime_priority_ = priority;
        }
    }

private:
    void apply_realtime_priority_on_this_thread() {
        if (realtime_priority_ <= 0 || realtime_applied_) {
            return;
        }
        realtime_applied_ = true;
        sched_param param{};
        param.sched_priority = realtime_priority_;
        const int result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (result != 0) {
            static std::atomic<bool> logged{false};
            bool expected = false;
            if (logged.compare_exchange_strong(expected, true)) {
                std::cerr << "[WARN] Failed to set real-time priority (errno: " << result
                          << "). Try running with sudo or setcap." << std::endl;
            }
            return;
        }
        std::cout << "[INFO] Real-time priority set to " << realtime_priority_ << std::endl;
    }

    void warn_overrun_rate_limited(int skipped_cycles, int64_t elapsed_us, int period_us) {
        const auto now = std::chrono::steady_clock::now();
        if (last_overrun_log_.time_since_epoch().count() != 0 &&
            now - last_overrun_log_ < std::chrono::seconds(1)) {
            return;
        }
        last_overrun_log_ = now;
        std::cerr << "[WARN] periodic timer overrun: elapsed_us=" << elapsed_us
                  << " period_us=" << period_us << " skipped_cycles=" << skipped_cycles
                  << " holding previous cycle (no catch-up)" << std::endl;
    }

private:
    void start_thread_base() {
        before_start();
        // A restarted worker is a new pthread. Apply the recorded priority there.
        realtime_applied_ = false;
        is_running_ = true;
        pthread_create(&thread_, nullptr, &PeriodicTimerThread::thread_entry, this);
    }

    void stop_thread_base() {
        is_running_ = false;
        if (thread_) {
            pthread_join(thread_, nullptr);
            thread_ = 0;
        }
        after_stop();
    }

    static void* thread_entry(void* arg) {
        static_cast<PeriodicTimerThread*>(arg)->timer_thread();
        return nullptr;
    }

    void timer_thread() {
        apply_realtime_priority_on_this_thread();
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        while (is_running_) {
            auto start = std::chrono::steady_clock::now();

            try {
                on_timer();
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception in on_timer(): " << e.what() << std::endl;
            }

            auto end = std::chrono::steady_clock::now();
            const int64_t elapsed_us =
                std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            last_elapsed_us_.store(elapsed_us);

            const int period_us = period_us_.load();
            timespec scheduled = next_time;
            add_timespec_microseconds(&scheduled, period_us);
            timespec now_ts{};
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            const PeriodicDeadlinePlan plan = plan_next_deadline(scheduled, now_ts, period_us);
            if (plan.overrun) {
                warn_overrun_rate_limited(plan.skipped_cycles, elapsed_us, period_us);
            }
            next_time = plan.next;
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
        }
    }

    pthread_t thread_{};
    std::atomic<bool> is_running_;
    std::atomic<int> period_us_;
    std::atomic<int64_t> last_elapsed_us_{0};
    int realtime_priority_ = 0;
    bool realtime_applied_ = false;
    std::chrono::steady_clock::time_point last_overrun_log_{};
};
