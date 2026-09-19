#include <controller/control.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, const char* message) {
    check(std::abs(actual - expected) < 1e-9, message);
}

int main(int argc, char** argv) {
    try {
        const std::string mode = argc > 1 ? argv[1] : "follower";
        openarm::can::socket::OpenArm arm;
        Dynamics leader, follower;
        follower.bias = 3.0;
        const int role = mode.find("leader") == 0 ? ROLE_LEADER : ROLE_FOLLOWER;
        auto state = std::make_shared<RobotSystemState>(7, 1);
        Control control(&arm, &leader, &follower, state, 0.002, role, 7, 1);
        std::vector<double> zeros(8, 0.0);
        control.SetParameter(std::vector<double>(8, 4.0), std::vector<double>(8, 0.2),
                             zeros, zeros, zeros, zeros);
        control.SetGravityCompensationScale(0.4);
        std::vector<JointState> refs(7, JointState{0.8, 0.15, 99.0});
        state->arm_state().set_all_references(refs);
        const double gravity = role == ROLE_LEADER ? 1.25 : 3.25;
        auto step = [&] {
            if (mode.find("bilateral") != std::string::npos) control.bilateral_step();
            else control.unilateral_step();
        };
        if (mode == "parameters") {
            bool rejected = false;
            try { control.SetParameter(std::vector<double>(7), zeros, zeros, zeros, zeros, zeros); }
            catch (const std::invalid_argument&) { rejected = true; }
            check(rejected, "missing gripper gain must be rejected");
        } else if (mode.find("startup") != std::string::npos) {
            control.AdjustPosition();
            near(arm.arm.history.front()[0].tau, 0.01 / 0.5 * 0.4 * gravity,
                 "startup must ramp gravity while holding position");
            near(arm.arm.commands[0].tau, 0.4 * gravity, "startup must establish gravity");
            const double previous = arm.arm.commands[0].tau;
            step();
            near(arm.arm.commands[0].tau, previous, "handover must preserve gravity");
        } else if (mode.find("invalid") != std::string::npos) {
            (role == ROLE_LEADER ? leader : follower).count = 8;
            step();
            near(arm.arm.commands[0].tau, 0.0, "mismatched model must be skipped");
        } else if (mode.find("disabled") != std::string::npos) {
            (role == ROLE_LEADER ? leader : follower).valid = false;
            step();
            near(arm.arm.commands[0].tau, 0.0, "invalid solver must be skipped");
        } else if (mode == "leader_friction") {
            arm.gripper.motors[0].velocity = 0.5;
            auto viscous = zeros;
            viscous[7] = 2.0;
            control.SetParameter(std::vector<double>(8, 4.0), std::vector<double>(8, 0.2),
                                 zeros, zeros, viscous, zeros);
            step();
            near(arm.gripper.commands[0].tau, 0.3, "gripper friction must use its own velocity");
        } else {
            step();
            near(arm.arm.commands[0].tau, 0.004 * 0.4 * gravity,
                 "first torque must use own measured pose, model, scale and ramp");
            for (int i = 1; i < 250; ++i) step();
            near(arm.arm.commands[0].tau, 0.4 * gravity, "full ramp torque");
            if (role == ROLE_FOLLOWER) {
                near(arm.arm.commands[0].q, 0.8, "preserve leader position reference");
                near(arm.arm.commands[0].dq, 0.15, "preserve leader velocity reference");
                near(arm.arm.commands[0].kp, 4.0, "preserve tracking stiffness");
            }
            control.SetGravityCompensationScale(0.0);
            step();
            near(arm.arm.commands[0].tau, 0.0, "zero scale disables gravity");
            near(arm.gripper.commands[0].tau, 0.0, "no arm gravity applied to gripper");
        }
        std::cout << "PASS " << mode << '\n';
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
