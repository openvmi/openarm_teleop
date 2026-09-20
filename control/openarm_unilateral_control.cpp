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

#include <atomic>
#include <chrono>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/oy_motor/oy_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <thread>
#include <yamlloader.hpp>

// Match the proven ROS 2 CAN worker cadence (8 request/reply pairs per bus).
constexpr double UNILATERAL_FREQUENCY = 100.0;
std::atomic<bool> keep_running(true);

// Called by main, not by the CAN threads. Counters distinguish cached reads
// from decoded frames; positions that remain still still count as updates.
void print_feedback_stats(const char* side, Control* control,
                          const RobotSystemState& state, Control::FeedbackStats& previous) {
    const auto stats = control->GetFeedbackStats();
    const auto samples = state.get_all_responses();
    const auto now = std::chrono::steady_clock::now();
    std::cout << "[CAN " << side << "] cycles=" << stats.cycles - previous.cycles
              << " timeout=" << stats.timeouts - previous.timeouts << " updates=";
    for (size_t i = 0; i < stats.updates.size(); ++i) {
        const auto last = i < previous.updates.size() ? previous.updates[i] : 0;
        std::cout << (i ? "," : "") << stats.updates[i] - last;
    }
    std::cout << " missing=";
    for (size_t i = 0; i < stats.missing.size(); ++i) {
        const auto last = i < previous.missing.size() ? previous.missing[i] : 0;
        std::cout << (i ? "," : "") << stats.missing[i] - last;
    }
    std::cout << " age_ms=";
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto age = samples[i].feedback_time.time_since_epoch().count() == 0
            ? -1
            : std::chrono::duration_cast<std::chrono::milliseconds>(
                  now - samples[i].feedback_time).count();
        std::cout << (i ? "," : "") << age;
    }
    std::cout << std::endl;
    previous = stats;
}

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

class LeaderArmThread : public PeriodicTimerThread {
public:
    LeaderArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_l,
                    double hz = UNILATERAL_FREQUENCY)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_l_(control_l) {}

protected:
    void before_start() override { std::cout << "leader start thread " << std::endl; }

    void after_stop() override { std::cout << "leader stop thread " << std::endl; }

    void on_timer() override {
        control_l_->unilateral_step();
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_l_;
};

class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_f,
                      double hz = UNILATERAL_FREQUENCY)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f) {}

protected:
    void before_start() override { std::cout << "follower start thread " << std::endl; }

    void after_stop() override { std::cout << "follower stop thread " << std::endl; }

    void on_timer() override {
        control_f_->unilateral_step();
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_f_;
};

int main(int argc, char **argv) {
    try {
        std::signal(SIGINT, signal_handler);

        // default configuration
        std::string arm_side = "right_arm";
        std::string leader_urdf_path;
        std::string follower_urdf_path;

        // Defaults follow the topology in leaderfollowerteleop.md section 2
        // (right arm: leader=can2, follower=can0; the launch scripts pass these explicitly).
        std::string leader_can_interface = "can2";
        std::string follower_can_interface = "can0";

        if (argc < 3) {
            std::cerr << "Usage: " << argv[0]
                      << " <leader_urdf_path> <follower_urdf_path> [arm_side] [leader_can] "
                         "[follower_can] [leader_gravity_scale] [follower_gravity_scale]"
                      << std::endl;
            std::cerr << "Note: gravity scales are optional CLI overrides of the yaml "
                         "gravity_compensation_scale (leader.yaml / follower.yaml)."
                      << std::endl;
            return 1;
        }

        // Required: URDF paths
        leader_urdf_path = argv[1];
        follower_urdf_path = argv[2];

        // Optional: arm_side
        if (argc >= 4) {
            arm_side = argv[3];
            if (arm_side != "left_arm" && arm_side != "right_arm") {
                std::cerr << "[ERROR] Invalid arm_side: " << arm_side
                          << ". Must be 'left_arm' or 'right_arm'." << std::endl;
                return 1;
            }
        }

        // Optional: CAN interfaces
        if (argc >= 6) {
            leader_can_interface = argv[4];
            follower_can_interface = argv[5];
        }

        // URDF file existence check
        if (!std::filesystem::exists(leader_urdf_path)) {
            std::cerr << "[ERROR] Leader URDF not found: " << leader_urdf_path << std::endl;
            return 1;
        }
        if (!std::filesystem::exists(follower_urdf_path)) {
            std::cerr << "[ERROR] Follower URDF not found: " << follower_urdf_path << std::endl;
            return 1;
        }

        // Setup dynamics
        std::string arm_prefix = (arm_side == "left_arm") ? "left_" : "right_";
        std::string root_link = "openarm_" + arm_prefix + "link0";
        std::string leaf_link = "openarm_" + arm_prefix + "hand";

        // Output confirmation
        std::cout << "=== OpenArm Unilateral Control ===" << std::endl;
        std::cout << "Arm side         : " << arm_side << std::endl;
        std::cout << "Leader CAN       : " << leader_can_interface << std::endl;
        std::cout << "Follower CAN     : " << follower_can_interface << std::endl;
        std::cout << "Leader URDF path : " << leader_urdf_path << std::endl;
        std::cout << "Follower URDF path: " << follower_urdf_path << std::endl;
        std::cout << "Root link         : " << root_link << std::endl;
        std::cout << "Leaf link         : " << leaf_link << std::endl;

        YamlLoader leader_loader("config/leader.yaml");
        YamlLoader follower_loader("config/follower.yaml");

        // Leader parameters
        std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> leader_k = leader_loader.get_vector("LeaderArmParam", "k");
        std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");
        double leader_gravity_scale =
            leader_loader.has("LeaderArmParam", "gravity_compensation_scale")
                ? leader_loader.get_double("LeaderArmParam", "gravity_compensation_scale")
                : 0.1;

        // Follower parameters
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");
        double follower_gravity_scale =
            follower_loader.has("FollowerArmParam", "gravity_compensation_scale")
                ? follower_loader.get_double("FollowerArmParam", "gravity_compensation_scale")
                : 0.1;

        // Optional: command-line gravity scale overrides (argv[6]/argv[7]),
        // applied after the yaml values so the CLI wins when provided.
        if (argc >= 7) leader_gravity_scale = std::stod(argv[6]);
        if (argc >= 8) follower_gravity_scale = std::stod(argv[7]);

        std::cout << "Gravity comp scale - leader: " << leader_gravity_scale
                  << ", follower: " << follower_gravity_scale << std::endl;

        Dynamics *leader_arm_dynamics = new Dynamics(leader_urdf_path, root_link, leaf_link);
        if (!leader_arm_dynamics->Init()) {
            // Mirror openarm_hardware's graceful degradation: a failed KDL
            // build disables gravity compensation instead of crashing later.
            std::cerr << "[WARN] Leader dynamics init failed — "
                         "gravity compensation DISABLED for leader"
                      << std::endl;
        }

        Dynamics *follower_arm_dynamics = new Dynamics(follower_urdf_path, root_link, leaf_link);
        if (!follower_arm_dynamics->Init()) {
            std::cerr << "[WARN] Follower dynamics init failed — "
                         "gravity compensation DISABLED for follower"
                      << std::endl;
        }

        std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *leader_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(leader_can_interface, false);

        std::cout << "=== Initializing Follower OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *follower_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(follower_can_interface, false);

        size_t leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
        size_t follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
        size_t leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
        size_t follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();

        std::cout << "leader arm motor num : " << leader_arm_motor_num << std::endl;
        std::cout << "follower arm motor num : " << follower_arm_motor_num << std::endl;
        std::cout << "leader hand motor num : " << leader_hand_motor_num << std::endl;
        std::cout << "follower hand motor num : " << follower_hand_motor_num << std::endl;

        // Joint-count validation, mirroring openarm_hardware's
        // "KDL chain has N joints, expected ARM_DOF — disabled" check.
        if (leader_arm_dynamics->IsValid() &&
            leader_arm_dynamics->GetJointCount() != leader_arm_motor_num) {
            std::cerr << "[WARN] Leader KDL chain has "
                      << leader_arm_dynamics->GetJointCount()
                      << " joints, expected " << leader_arm_motor_num
                      << " — gravity compensation DISABLED for leader" << std::endl;
            leader_gravity_scale = 0.0;
        } else if (!leader_arm_dynamics->IsValid()) {
            leader_gravity_scale = 0.0;
        }
        if (follower_arm_dynamics->IsValid() &&
            follower_arm_dynamics->GetJointCount() != follower_arm_motor_num) {
            std::cerr << "[WARN] Follower KDL chain has "
                      << follower_arm_dynamics->GetJointCount()
                      << " joints, expected " << follower_arm_motor_num
                      << " — gravity compensation DISABLED for follower" << std::endl;
            follower_gravity_scale = 0.0;
        } else if (!follower_arm_dynamics->IsValid()) {
            follower_gravity_scale = 0.0;
        }

        // Declare robot_state (Joint and motor counts are assumed to be equal)
        std::shared_ptr<RobotSystemState> leader_state =
            std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);

        std::shared_ptr<RobotSystemState> follower_state =
            std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);

        Control *control_leader = new Control(
            leader_openarm, leader_arm_dynamics, follower_arm_dynamics, leader_state,
            1.0 / UNILATERAL_FREQUENCY, ROLE_LEADER, arm_side,
            leader_arm_motor_num, leader_hand_motor_num);
        Control *control_follower =
            new Control(follower_openarm, leader_arm_dynamics, follower_arm_dynamics,
                        follower_state, 1.0 / UNILATERAL_FREQUENCY, ROLE_FOLLOWER, arm_side,
                        follower_arm_motor_num, follower_hand_motor_num);

        control_leader->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv,
                                     leader_Fo);

        control_follower->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k,
                                       follower_Fv, follower_Fo);

        control_leader->SetGravityCompensationScale(leader_gravity_scale);
        control_follower->SetGravityCompensationScale(follower_gravity_scale);

        // set home postion
        std::thread thread_l(&Control::AdjustPosition, control_leader);
        std::thread thread_f(&Control::AdjustPosition, control_follower);
        thread_l.join();
        thread_f.join();

        // Start control process
        LeaderArmThread leader_thread(leader_state, control_leader, UNILATERAL_FREQUENCY);
        FollowerArmThread follower_thread(follower_state, control_follower, UNILATERAL_FREQUENCY);
        control_follower->SetReferenceSource(leader_state);
        std::cout << "CAN control: " << UNILATERAL_FREQUENCY
                  << " Hz, per-motor feedback check, direct leader -> follower" << std::endl;

        leader_thread.start_thread();
        follower_thread.start_thread();

        const char* diagnostics_env = std::getenv("OPENARM_TELEOP_DIAGNOSTICS");
        const bool diagnostics = diagnostics_env && std::string(diagnostics_env) == "1";
        Control::FeedbackStats previous_leader, previous_follower;
        auto next_diagnostics = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (diagnostics && std::chrono::steady_clock::now() >= next_diagnostics) {
                print_feedback_stats("leader", control_leader, *leader_state, previous_leader);
                print_feedback_stats("follower", control_follower, *follower_state, previous_follower);
                next_diagnostics = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            }
        }

        leader_thread.stop_thread();
        follower_thread.stop_thread();

        leader_openarm->disable_all();
        follower_openarm->disable_all();

    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
