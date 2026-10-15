// Export Damiao motor EEPROM/runtime parameters over CAN (RID query).
// Does not enable motors or move the arms.

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

struct RidInfo {
    int rid;
    const char* name;
};

// Subset that matters for force / mode diagnosis; names match Damiao RID docs.
constexpr RidInfo kRids[] = {
    {0, "UV_Value"},  {1, "KT_Value"}, {2, "OT_Value"},  {3, "OC_Value"},
    {4, "ACC"},       {5, "DEC"},      {6, "MAX_SPD"},   {7, "MST_ID"},
    {8, "ESC_ID"},    {9, "TIMEOUT"},  {10, "CTRL_MODE"},{11, "Damp"},
    {12, "Inertia"},  {13, "hw_ver"},  {14, "sw_ver"},   {15, "SN"},
    {16, "NPP"},      {17, "Rs"},      {18, "LS"},       {19, "Flux"},
    {20, "Gr"},       {21, "PMAX"},    {22, "VMAX"},     {23, "TMAX"},
    {24, "I_BW"},     {25, "KP_ASR"},  {26, "KI_ASR"},   {27, "KP_APR"},
    {28, "KI_APR"},   {29, "OV_Value"},{30, "GREF"},     {35, "can_br"},
    {36, "sub_ver"},  {55, "dir"},
};

const char* motor_type_name(openarm::damiao_motor::MotorType type) {
    using MT = openarm::damiao_motor::MotorType;
    switch (type) {
        case MT::DM3507:
            return "DM3507";
        case MT::DM4310:
            return "DM4310";
        case MT::DM4310_48V:
            return "DM4310_48V";
        case MT::DM4340:
            return "DM4340";
        case MT::DM4340_48V:
            return "DM4340_48V";
        case MT::DM6006:
            return "DM6006";
        case MT::DM8006:
            return "DM8006";
        case MT::DM8009:
            return "DM8009";
        case MT::DM10010L:
            return "DM10010L";
        case MT::DM10010:
            return "DM10010";
        case MT::DMH3510:
            return "DMH3510";
        case MT::DMH6215:
            return "DMH6215";
        case MT::DMG6220:
            return "DMG6220";
        default:
            return "?";
    }
}

const char* ctrl_mode_name(double value) {
    const int mode = static_cast<int>(std::lround(value));
    switch (mode) {
        case 1:
            return "MIT";
        case 2:
            return "POS_VEL";
        case 3:
            return "VEL";
        case 4:
            return "POS_FORCE";
        default:
            return "?";
    }
}

std::string env_or(const char* key, const std::string& fallback) {
    const char* value = std::getenv(key);
    return (value && *value) ? std::string(value) : fallback;
}

void init_arm(openarm::can::socket::OpenArm* arm) {
    using openarm::damiao_motor::MotorType;
    arm->init_arm_motors(
        {MotorType::DM8009, MotorType::DM8009, MotorType::DM4340, MotorType::DM4340,
         MotorType::DM4310, MotorType::DM4310, MotorType::DM4310},
        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}, {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17});
    arm->init_gripper_motor(MotorType::DM4310, 0x08, 0x18);
}

using ParamTable = std::vector<std::vector<double>>;  // [motor][rid_index]

ParamTable read_params(openarm::can::socket::OpenArm* bus, const char* label) {
    auto& arm = bus->get_arm();
    const size_t n = arm.get_motors().size();
    ParamTable table(n, std::vector<double>(std::size(kRids), -1.0));

    arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
    bus->get_gripper().set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

    std::cout << "[" << label << "] 读取 " << n << " 个关节电机参数…" << std::endl;
    for (size_t rid_i = 0; rid_i < std::size(kRids); ++rid_i) {
        const int rid = kRids[rid_i].rid;
        arm.query_param_all(rid);
        bus->recv_all(3000);
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        // Second pass for dropouts
        arm.query_param_all(rid);
        bus->recv_all(3000);
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        for (size_t m = 0; m < n; ++m) {
            table[m][rid_i] = arm.get_motor(static_cast<int>(m)).get_param(rid);
        }
        std::cout << "  RID " << std::setw(2) << rid << " " << kRids[rid_i].name << " ok"
                  << std::endl;
    }

    // Also grab a state snapshot.
    arm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
    for (int cycle = 0; cycle < 10; ++cycle) {
        bus->refresh_all();
        bus->recv_all(2000);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return table;
}

void print_side(std::ostream& out, const char* side, openarm::can::socket::OpenArm* bus,
                const ParamTable& table) {
    const auto motors = bus->get_arm().get_motors();
    out << "\n========== " << side << " ==========\n";
    for (size_t m = 0; m < motors.size(); ++m) {
        const auto& motor = motors[m];
        const auto limits = openarm::damiao_motor::Motor::get_limit_param(motor.get_motor_type());
        out << "\n--- " << side << " j" << (m + 1) << "  type=" << motor_type_name(motor.get_motor_type())
            << "  send=0x" << std::hex << motor.get_send_can_id() << " recv=0x"
            << motor.get_recv_can_id() << std::dec << " ---\n";
        out << "  state: q=" << motor.get_position() << " dq=" << motor.get_velocity()
            << " tau=" << motor.get_torque() << " err=" << motor.get_error_code()
            << " tmos=" << motor.get_state_tmos() << " trotor=" << motor.get_state_trotor() << "\n";
        out << "  lib_limits: pMax=" << limits.pMax << " vMax=" << limits.vMax
            << " tMax=" << limits.tMax << "\n";
        for (size_t rid_i = 0; rid_i < std::size(kRids); ++rid_i) {
            const double value = table[m][rid_i];
            out << "  " << std::setw(10) << std::left << kRids[rid_i].name << std::right
                << " RID=" << std::setw(2) << kRids[rid_i].rid << "  value=";
            if (value < -0.5 && value > -1.5 && value != 0.0) {
                // get_param default is -1 when missing
                out << "(未读到)";
            } else {
                out << value;
                if (kRids[rid_i].rid == 10) {
                    out << " (" << ctrl_mode_name(value) << ")";
                }
            }
            out << "\n";
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    const std::string root = env_or("OPENARM_ROOT", "");
    if (root.empty()) {
        std::cerr << "[motor-params] 需要 OPENARM_ROOT" << std::endl;
        return 1;
    }
    const std::string left_can = env_or("OPENARM_LEFT_FOLLOWER_CAN", "can1");
    const std::string right_can = env_or("OPENARM_RIGHT_FOLLOWER_CAN", "can0");
    const std::string out_dir = env_or("OPENARM_MOTOR_PARAM_DIR", root + "/recordings/motor_params");
    std::filesystem::create_directories(out_dir);

    std::cout << "[motor-params] left=" << left_can << " right=" << right_can << std::endl;
    std::cout << "[motor-params] 不使能电机，只读参数；请先停止遥操/零重力/MoveIt。" << std::endl;

    std::unique_ptr<openarm::can::socket::OpenArm> left;
    std::unique_ptr<openarm::can::socket::OpenArm> right;
    try {
        left = std::make_unique<openarm::can::socket::OpenArm>(left_can, true);
        right = std::make_unique<openarm::can::socket::OpenArm>(right_can, true);
        init_arm(left.get());
        init_arm(right.get());
    } catch (const std::exception& error) {
        std::cerr << "[motor-params] 打开 CAN 失败：" << error.what() << std::endl;
        return 1;
    }

    ParamTable left_table;
    ParamTable right_table;
    try {
        left_table = read_params(left.get(), "left");
        right_table = read_params(right.get(), "right");
    } catch (const std::exception& error) {
        std::cerr << "[motor-params] 读取失败：" << error.what() << std::endl;
        return 1;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream path;
    path << out_dir << "/motor_params_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".txt";

    std::ofstream file(path.str());
    if (!file) {
        std::cerr << "[motor-params] 写不了 " << path.str() << std::endl;
        return 1;
    }

    print_side(std::cout, "LEFT", left.get(), left_table);
    print_side(std::cout, "RIGHT", right.get(), right_table);

    print_side(file, "LEFT", left.get(), left_table);
    print_side(file, "RIGHT", right.get(), right_table);

    // CSV for spreadsheet
    std::ostringstream csv_path;
    csv_path << out_dir << "/motor_params_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    std::ofstream csv(csv_path.str());
    csv << "side,joint,type,send_id,recv_id";
    for (const auto& rid : kRids) {
        csv << "," << rid.name;
    }
    csv << ",q,dq,tau,error\n";
    auto dump_csv = [&](const char* side, openarm::can::socket::OpenArm* bus,
                        const ParamTable& table) {
        const auto motors = bus->get_arm().get_motors();
        for (size_t m = 0; m < motors.size(); ++m) {
            const auto& motor = motors[m];
            csv << side << "," << (m + 1) << "," << motor_type_name(motor.get_motor_type()) << ",0x"
                << std::hex << motor.get_send_can_id() << ",0x" << motor.get_recv_can_id()
                << std::dec;
            for (size_t rid_i = 0; rid_i < std::size(kRids); ++rid_i) {
                csv << "," << table[m][rid_i];
            }
            csv << "," << motor.get_position() << "," << motor.get_velocity() << ","
                << motor.get_torque() << "," << motor.get_error_code() << "\n";
        }
    };
    dump_csv("left", left.get(), left_table);
    dump_csv("right", right.get(), right_table);

    std::cout << "\nREPORT=" << path.str() << std::endl;
    std::cout << "CSV=" << csv_path.str() << std::endl;
    return 0;
}
