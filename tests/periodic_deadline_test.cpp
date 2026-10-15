// plan_next_deadline must not schedule the missed callbacks.
// A late wake sleeps one period from now, not the next slot on the old grid.

#include <periodic_timer_thread.hpp>

#include <cstdint>
#include <iostream>
#include <string>

namespace {

int fail(const std::string& message) {
    std::cerr << message << "\n";
    return 1;
}

timespec from_us(int64_t microseconds) {
    timespec time{};
    time.tv_sec = static_cast<time_t>(microseconds / 1000000);
    time.tv_nsec = static_cast<long>((microseconds % 1000000) * 1000);
    return time;
}

}  // namespace

int main() {
    constexpr int kPeriodUs = 1000;
    const timespec scheduled = from_us(10000000);

    const PeriodicDeadlinePlan early = plan_next_deadline(scheduled, from_us(10000000 - 400), kPeriodUs);
    if (early.overrun || early.skipped_cycles != 0 || timespec_delta_ns(early.next, scheduled) != 0) {
        return fail("an early wake moved the deadline");
    }

    const PeriodicDeadlinePlan on_time = plan_next_deadline(scheduled, scheduled, kPeriodUs);
    if (on_time.overrun || on_time.skipped_cycles != 0 ||
        timespec_delta_ns(on_time.next, scheduled) != 0) {
        return fail("an on-time wake did not keep the scheduled deadline");
    }

    // 2.5 periods late. Catch-up would land on scheduled + 3 periods.
    // Holding the previous cycle lands on now + 1 period.
    const timespec late = from_us(10000000 + 2500);
    const PeriodicDeadlinePlan overrun = plan_next_deadline(scheduled, late, kPeriodUs);
    if (!overrun.overrun || overrun.skipped_cycles != 2) {
        return fail("a 2.5 period overrun did not skip two callbacks");
    }
    if (timespec_delta_ns(overrun.next, late) != static_cast<int64_t>(kPeriodUs) * 1000) {
        return fail("the next deadline was not one period after now");
    }
    const timespec catch_up_grid = from_us(10000000 + 3000);
    if (timespec_delta_ns(overrun.next, catch_up_grid) == 0) {
        return fail("the next deadline caught up to the old grid");
    }
    return 0;
}
