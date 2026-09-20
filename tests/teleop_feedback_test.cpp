#include <controller/control.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    try {
        openarm::can::socket::OpenArm arm;
        Dynamics leader, follower;
        auto state = std::make_shared<RobotSystemState>(7, 1);
        const std::string mode = argc > 1 ? argv[1] : "late";
        const int role = mode == "direct" ? ROLE_FOLLOWER : ROLE_LEADER;
        Control control(&arm, &leader, &follower, state, 0.01, role, 7, 1);
        std::vector<double> zeros(8, 0.0);
        control.SetParameter(zeros, zeros, zeros, zeros, zeros, zeros);
        if (mode == "direct") {
            auto source = std::make_shared<RobotSystemState>(7, 1);
            control.SetReferenceSource(source);
            state->arm_state().set_all_references(std::vector<JointState>(7, {0.25, 0.0, 0.0}));
            // Uninitialized sources must not overwrite the held position with zeros.
            control.unilateral_step();
            if (arm.arm.commands[3].q != 0.25) throw std::runtime_error("uninitialized target");
            auto now = std::chrono::steady_clock::now();
            source->arm_state().set_all_responses(std::vector<JointState>(7, {0.6, 0.3, 0.0, now}));
            control.unilateral_step();
            if (arm.arm.commands[3].q != 0.6 || arm.arm.commands[3].dq != 0.3)
                throw std::runtime_error("fresh leader sample must be used without AdminThread");
            auto old = now - std::chrono::seconds(1);
            source->arm_state().set_all_responses(std::vector<JointState>(7, {0.9, 0.8, 0.0, old}));
            control.unilateral_step();
            if (arm.arm.commands[3].q != 0.6 || arm.arm.commands[3].dq != 0.0)
                throw std::runtime_error("stale leader sample must hold last target without velocity");
            std::cout << "PASS direct reference and stale feedback handling\n";
            return 0;
        }
        if (mode == "timeout") {
            arm.receive = [](auto& bus) {
                for (size_t i = 0; i < 3; ++i)
                    bus.arm.motors[i].feedback = std::chrono::steady_clock::now();
            };
            const auto start = std::chrono::steady_clock::now();
            control.unilateral_step();
            const auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed < std::chrono::milliseconds(8) || elapsed > std::chrono::milliseconds(100))
                throw std::runtime_error("receive must have a bounded total wait");
            const auto stats = control.GetFeedbackStats();
            if (stats.timeouts != 1 || stats.updates[2] != 1 || stats.missing[2] != 0 ||
                stats.updates[3] != 0 || stats.missing[3] != 1 || stats.missing[7] != 1)
                throw std::runtime_error("diagnostics must identify missing J4-J7 and gripper");
            std::cout << "PASS bounded wait and per-motor diagnostics\n";
            return 0;
        }
        int batches = 0;
        arm.receive = [&](auto& bus) {
            ++batches;
            // USB CAN delivers J1-J3 first; J4-J7 and the gripper arrive later.
            const size_t begin = batches == 1 ? 0 : 3;
            const size_t end = batches == 1 ? 3 : 7;
            for (size_t i = begin; i < end; ++i) {
                bus.arm.motors[i].position = 0.5 + i * 0.1;
                bus.arm.motors[i].feedback = std::chrono::steady_clock::now();
            }
            if (batches > 1) bus.gripper.motors[0].feedback = std::chrono::steady_clock::now();
        };
        control.unilateral_step();
        for (size_t i = 0; i < 7; ++i) {
            if (std::abs(state->arm_state().get_response(i).position - (0.5 + i * 0.1)) > 1e-9)
                throw std::runtime_error("current cycle must publish late J4-J7 feedback too");
        }
        const auto stats = control.GetFeedbackStats();
        if (stats.cycles != 1 || stats.timeouts != 0 || stats.updates[6] != 1)
            throw std::runtime_error("late feedback must count as received in this cycle");
        std::cout << "PASS all joints published after receive\n";
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
