#pragma once

// Joint-impedance profiles for real_arm. P0 matches the historical compiled gains.
// P-stop uses a small kp (not zero): the draft table's kp=0 would not hold the arm,
// and the finalized choice is J1–J4 kp≈5 so a person can still push it.

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

enum class IntegralMode { Legacy, Gated, Frozen, Off };

struct ImpedanceProfile {
    std::string name;
    std::array<double, 7> kp{};
    std::array<double, 7> kd{};
    std::array<double, 7> restore_tau_max{};
    std::array<double, 7> restore_kd{};
    double restore_width = 0.025;
    IntegralMode integral = IntegralMode::Legacy;
    double stall_error = 0.18;
    double slow_error = 0.10;
    double settle_error = 0.05;
    double duration_scale = 3.0;
    double duration_margin = 2.0;
    double gain_ramp_s = 0.5;
};

inline const std::array<double, 7>& motor_tmax_nm() {
    // DM8009, DM8009, DM4340, DM4340, DM4310, DM4310, DM4310 (real_arm motor list).
    static const std::array<double, 7> kTmax{54.0, 54.0, 28.0, 28.0, 10.0, 10.0, 10.0};
    return kTmax;
}

inline std::unordered_map<std::string, ImpedanceProfile> builtin_impedance_profiles() {
    auto make = [](const char* name, std::array<double, 7> kp, std::array<double, 7> kd,
                   IntegralMode mode, double stall, double slow, double settle, double scale,
                   double margin) {
        ImpedanceProfile profile;
        profile.name = name;
        profile.kp = kp;
        profile.kd = kd;
        profile.integral = mode;
        profile.stall_error = stall;
        profile.slow_error = slow;
        profile.settle_error = settle;
        profile.duration_scale = scale;
        profile.duration_margin = margin;
        return profile;
    };
    std::unordered_map<std::string, ImpedanceProfile> profiles;
    profiles.emplace(
        "P0", make("P0", {55, 55, 55, 55, 16, 14, 14}, {1.3, 1.3, 1.3, 1.3, 0.35, 0.28, 0.35},
                   IntegralMode::Legacy, 0.18, 0.10, 0.05, 3.0, 2.0));
    profiles.emplace(
        "P1", make("P1", {40, 40, 40, 40, 12, 10, 10}, {1.5, 1.5, 1.5, 1.5, 0.30, 0.25, 0.30},
                   IntegralMode::Gated, 0.20, 0.12, 0.06, 3.0, 2.0));
    profiles.emplace(
        "P2", make("P2", {25, 25, 25, 25, 8, 7, 7}, {1.2, 1.2, 1.2, 1.2, 0.25, 0.20, 0.25},
                   IntegralMode::Frozen, 0.25, 0.15, 0.08, 4.0, 3.0));
    profiles.emplace(
        "P3", make("P3", {12, 12, 12, 12, 5, 4, 4}, {0.9, 0.9, 0.9, 0.9, 0.20, 0.15, 0.20},
                   IntegralMode::Off, 0.30, 0.20, 0.12, 4.0, 3.0));
    // J6 only. Setting restore_tau_max to all zeros disables the extra spring.
    profiles["P2"].restore_tau_max[5] = 0.12;
    profiles["P2"].restore_kd[5] = 0.10;
    profiles["P3"].restore_tau_max[5] = 0.20;
    profiles["P3"].restore_kd[5] = 0.15;
    // Soft hold, not the draft "kp = 0" row. Wrist kp is inside 1–2.
    profiles.emplace(
        "P-stop", make("P-stop", {5, 5, 5, 5, 2, 1.5, 1.5}, {2, 2, 2, 2, 0.3, 0.3, 0.3},
                       IntegralMode::Off, 0.30, 0.20, 0.12, 4.0, 3.0));
    profiles["P-stop"].gain_ramp_s = 0.2;
    // Hand-drag. kp = 0 is damping only; kd stays above 0 (Damiao forbids kd = 0 in position mode).
    profiles.emplace(
        "drag", make("drag", {0, 0, 0, 0, 0, 0, 0}, {2, 2, 2, 2, 0.3, 0.3, 0.3}, IntegralMode::Off,
                     0.30, 0.20, 0.12, 4.0, 3.0));
    return profiles;
}

inline void validate_impedance_profile(const ImpedanceProfile& profile) {
    if (!(profile.stall_error > profile.slow_error && profile.slow_error > 0.0 &&
          profile.settle_error > 0.0 && profile.duration_scale >= 1.0 &&
          profile.duration_margin >= 0.0 && profile.gain_ramp_s > 0.0 &&
          profile.gain_ramp_s <= 2.0 && std::isfinite(profile.restore_width) &&
          profile.restore_width >= 0.01 && profile.restore_width <= 0.2)) {
        throw std::runtime_error("阻抗档位 " + profile.name + " 的阈值或斜坡时间无效");
    }
    for (size_t index = 0; index < 7; ++index) {
        const double kp = profile.kp[index];
        const double kd = profile.kd[index];
        const double cap = profile.restore_tau_max[index];
        const double extra_kd = profile.restore_kd[index];
        if (!(kp >= 0.0 && kp <= 500.0) || !(kd > 0.0 && kd <= 5.0) || !std::isfinite(kp) ||
            !std::isfinite(kd)) {
            throw std::runtime_error(
                "阻抗档位 " + profile.name + " 关节 " + std::to_string(index + 1) +
                " 增益越界（要求 0≤kp≤500 且 0<kd≤5），拒绝启动，不截断");
        }
        if (!std::isfinite(cap) || cap < 0.0 || cap > 0.5 ||
            !std::isfinite(extra_kd) || extra_kd < 0.0 || kd + extra_kd > 5.0 ||
            kp + cap / profile.restore_width > 500.0 ||
            ((profile.name == "P-stop" || profile.name == "drag") &&
             (cap != 0.0 || extra_kd != 0.0))) {
            throw std::runtime_error("阻抗档位 " + profile.name + " 非线性弹簧参数越界");
        }
    }
}

inline IntegralMode parse_integral_mode(const std::string& text) {
    if (text == "legacy") {
        return IntegralMode::Legacy;
    }
    if (text == "gated") {
        return IntegralMode::Gated;
    }
    if (text == "frozen") {
        return IntegralMode::Frozen;
    }
    if (text == "off") {
        return IntegralMode::Off;
    }
    throw std::runtime_error("未知积分模式: " + text);
}

inline std::unordered_map<std::string, ImpedanceProfile> load_impedance_profiles(
    const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("读不了阻抗档位文件: " + path);
    }
    auto profiles = builtin_impedance_profiles();
    ImpedanceProfile* current = nullptr;
    std::string line;
    while (std::getline(input, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) {
            line = line.substr(0, hash);
        }
        if (line.find_first_not_of(" \t\r") == std::string::npos) {
            continue;
        }
        const auto indent = line.find_first_not_of(' ');
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("阻抗档位文件无法解析: " + line);
        }
        std::string key = line.substr(0, colon);
        std::string value = line.substr(colon + 1);
        auto trim = [](std::string* text) {
            const auto begin = text->find_first_not_of(" \t\r");
            const auto end = text->find_last_not_of(" \t\r");
            *text = begin == std::string::npos ? std::string() : text->substr(begin, end - begin + 1);
        };
        trim(&key);
        trim(&value);
        if (indent == 0) {
            continue;  // "profiles:"
        }
        if (indent <= 2 && value.empty()) {
            if (!profiles.count(key)) {
                profiles.emplace(key, profiles.at("P0"));
                profiles.at(key).name = key;
            }
            current = &profiles.at(key);
            current->name = key;
            continue;
        }
        if (current == nullptr) {
            throw std::runtime_error("阻抗档位文件在档位名之前出现字段: " + key);
        }
        auto numbers = [](const std::string& text) {
            std::string cleaned = text;
            for (char& ch : cleaned) {
                if (ch == '[' || ch == ']' || ch == ',') {
                    ch = ' ';
                }
            }
            std::stringstream stream(cleaned);
            std::vector<double> out;
            double value = 0.0;
            while (stream >> value) {
                out.push_back(value);
            }
            return out;
        };
        if (key == "kp" || key == "kd" || key == "restore_tau_max" || key == "restore_kd") {
            const auto parsed = numbers(value);
            if (parsed.size() != 7) {
                throw std::runtime_error(current->name + "." + key + " 需要 7 个数");
            }
            std::array<double, 7> array{};
            std::copy(parsed.begin(), parsed.end(), array.begin());
            if (key == "kp") {
                current->kp = array;
            } else if (key == "kd") {
                current->kd = array;
            } else if (key == "restore_tau_max") {
                current->restore_tau_max = array;
            } else {
                current->restore_kd = array;
            }
        } else if (key == "integral") {
            current->integral = parse_integral_mode(value);
        } else if (key == "stall") {
            current->stall_error = std::stod(value);
        } else if (key == "slow") {
            current->slow_error = std::stod(value);
        } else if (key == "settle") {
            current->settle_error = std::stod(value);
        } else if (key == "duration_scale") {
            current->duration_scale = std::stod(value);
        } else if (key == "duration_margin") {
            current->duration_margin = std::stod(value);
        } else if (key == "gain_ramp_s") {
            current->gain_ramp_s = std::stod(value);
        } else if (key == "restore_width") {
            current->restore_width = std::stod(value);
        } else {
            throw std::runtime_error("未知阻抗字段: " + key);
        }
    }
    for (const auto& [_, profile] : profiles) {
        validate_impedance_profile(profile);
    }
    return profiles;
}
