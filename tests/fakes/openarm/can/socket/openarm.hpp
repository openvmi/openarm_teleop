#pragma once
#include <openarm/oy_motor/oy_motor_types.hpp>
#include <vector>
#include <chrono>
#include <functional>

namespace openarm::can::socket {
struct TestMotor {
    double position = 0.25;
    double velocity = 0.0;
    std::chrono::steady_clock::time_point feedback = std::chrono::steady_clock::now();
    auto last_feedback() const { return feedback; }
    double get_position() const { return position; }
    double get_velocity() const { return velocity; }
};
struct TestComponent {
    std::vector<TestMotor> motors;
    std::vector<oy_motor::MITParam> commands;
    std::vector<std::vector<oy_motor::MITParam>> history;
    const auto& get_motors() const { return motors; }
    void oy_mit_control_all(const std::vector<oy_motor::MITParam>& values) {
        commands = values;
        history.push_back(values);
    }
};
class OpenArm {
public:
    TestComponent arm{std::vector<TestMotor>(7), {}, {}};
    TestComponent gripper{std::vector<TestMotor>(1), {}, {}};
    std::function<void(OpenArm&)> receive;
    auto& get_arm() { return arm; }
    auto& get_gripper() { return gripper; }
    void recv_all(int = 500) {
        if (receive) { receive(*this); return; }
        for (auto& motor : arm.motors) motor.feedback = std::chrono::steady_clock::now();
        for (auto& motor : gripper.motors) motor.feedback = std::chrono::steady_clock::now();
    }
    void disable_all() {}
};
}  // namespace openarm::can::socket
