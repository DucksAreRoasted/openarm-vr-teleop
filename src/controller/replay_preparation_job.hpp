#pragma once

#include <controller/replay_preparation.hpp>
#include <future>

// File parsing, retiming and transition generation must never run in the servo.
// Declare this before hardware owners so unwinding disables motors before any
// worker join. Normal stop also disables motors before destroying this object.
class ReplayPreparationJob {
public:
    template <typename Builder>
    void start(Builder builder) {
        if (busy()) throw std::runtime_error("return preparation already pending");
        result_ = std::async(std::launch::async, std::move(builder));
    }
    bool busy() const { return result_.valid(); }
    bool take(ReplayPreparation* preparation) {
        if (!busy() || result_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return false;
        *preparation = result_.get();
        return true;
    }
private:
    std::future<ReplayPreparation> result_;
};
