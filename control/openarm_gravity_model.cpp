// Offline evaluator: uses the same Dynamics and YAML interpretation as zero-g.
#include <controller/dynamics.hpp>
#include <controller/gravity_comp.hpp>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: openarm_gravity_model URDF YAML; stdin: side,q1,...,q7\n";
        return 2;
    }
    try {
        GravityTune left, right;
        GravityModel model;
        if (!load_gravity_yaml(argv[2], &left, &right, &model))
            throw std::runtime_error("Cannot load gravity YAML");
        std::array<std::unique_ptr<Dynamics>, 2> arms;
        // Dynamics::Init prints status; reserve stdout for the numeric protocol.
        auto* output = std::cout.rdbuf(std::cerr.rdbuf());
        for (int index = 0; index < 2; ++index) {
            const std::string side = index == 0 ? "left" : "right";
            auto& dynamics = arms[index];
            dynamics = std::make_unique<Dynamics>(argv[1], "openarm_body_link0",
                                                  "openarm_" + side + "_link7");
            dynamics->SetLinkMassScales(link_mass_scales_for(model, side));
            dynamics->SetLinkComDeltas(link_com_deltas_for(model, side));
            if (!dynamics->Init()) throw std::runtime_error("Cannot initialize dynamics");
            dynamics->SetUrdfGravityScale(model.urdf_scale);
            const double motor = model.gripper_motor_kg > 0.0 ? model.gripper_motor_kg : 0.12;
            dynamics->SetEndEffectorPointMass(2.0 * 0.03602545343277134 + motor +
                                              model.ee_payload_kg, 0.0, 0.0, 0.1475);
            const auto& offset = index == 0 ? model.wrist_camera_offset_left :
                                             model.wrist_camera_offset_right;
            dynamics->SetWristCameraMass(model.wrist_camera_kg, index == 0,
                                         offset[0], offset[1], offset[2]);
            dynamics->SetDistalMassGains(model.distal_ee_gain, model.distal_cam_gain);
        }
        std::cout.rdbuf(output);
        std::cout << std::setprecision(17);
        std::string line;
        while (std::getline(std::cin, line)) {
            std::stringstream row(line);
            std::string token;
            std::getline(row, token, ',');
            if (token != "left" && token != "right") throw std::runtime_error("Invalid side");
            const int index = token == "left" ? 0 : 1;
            std::array<double, 7> q{}, g{};
            for (double& value : q) {
                if (!std::getline(row, token, ',')) throw std::runtime_error("Missing joint angle");
                size_t used = 0;
                value = std::stod(token, &used);
                if (used != token.size() || !std::isfinite(value))
                    throw std::runtime_error("Invalid joint angle");
            }
            if (std::getline(row, token, ',')) throw std::runtime_error("Extra joint angle");
            arms[index]->GetGravity(q.data(), g.data());
            for (int j = 0; j < 7; ++j) std::cout << (j ? "," : "") << g[j];
            std::cout << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
