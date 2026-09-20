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
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/oy_motor/oy_motor_constants.hpp>
#include <openarm_constants.hpp>
#include <thread>

int main(int argc, char** argv) {
    try {
        std::cout << "=== OpenArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality" << std::endl;

        std::string can_interface = "can0";
        if (argc > 1) {
            can_interface = argv[1];
        }

        std::cout << "[INFO] Using CAN interface: " << can_interface << std::endl;

        // Initialize OpenArm with CAN interface (classic CAN, matches init_can.sh 1 Mbps setup)
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm(can_interface, false);  // Classic CAN frames

        // Initialize arm motors (OY motor configuration, same as DEFAULT_MOTOR_CONFIG)
        const MotorConfig& config = DEFAULT_MOTOR_CONFIG;
        openarm.init_arm_motors(config.arm_motor_types, config.arm_send_can_ids,
                                config.arm_recv_can_ids);

        // Initialize gripper
        std::cout << "Initializing gripper..." << std::endl;
        openarm.init_gripper_motor(config.gripper_motor_type, config.gripper_send_can_id,
                                   config.gripper_recv_can_id);

        // Set callback mode to ignore and refresh all motors
        openarm.set_callback_mode_all(openarm::oy_motor::CallbackMode::IGNORE);
        openarm.refresh_all();
        openarm.recv_all();

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        openarm.enable_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Switch to MIT state mode and print motor states
        std::cout << "\n=== Querying Motor States (OY MIT) ===" << std::endl;
        openarm.set_callback_mode_all(openarm::oy_motor::CallbackMode::STATE);
        openarm.read_mit_state_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        openarm.recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Access motors through components
        for (const auto& motor : openarm.get_arm().get_motors()) {
            std::cout << "Arm Motor: 0x" << std::hex << motor.get_send_can_id() << std::dec
                      << " position: " << motor.get_position()
                      << " velocity: " << motor.get_velocity()
                      << " torque: " << motor.get_torque() << std::endl;
        }
        for (const auto& motor : openarm.get_gripper().get_motors()) {
            std::cout << "Gripper Motor: 0x" << std::hex << motor.get_send_can_id() << std::dec
                      << " position: " << motor.get_position()
                      << " velocity: " << motor.get_velocity()
                      << " torque: " << motor.get_torque() << std::endl;
        }

        // Control arm motors (zero-torque MIT command, NOTE: OY field order {q, dq, kp, kd, tau})
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        openarm.get_arm().oy_mit_control_all({openarm::oy_motor::MITParam{0, 0, 0, 0, 0},
                                              openarm::oy_motor::MITParam{0, 0, 0, 0, 0},
                                              openarm::oy_motor::MITParam{0, 0, 0, 0, 0},
                                              openarm::oy_motor::MITParam{0, 0, 0, 0, 0},
                                              openarm::oy_motor::MITParam{0, 0, 0, 0, 0},
                                              openarm::oy_motor::MITParam{0, 0, 0, 0, 0},
                                              openarm::oy_motor::MITParam{0, 0, 0, 0, 0}});

        openarm.get_gripper().oy_mit_control_all({openarm::oy_motor::MITParam{0, 0, 0, 0, 0}});

        openarm.recv_all();

        // Control gripper
        std::cout << "Opening gripper..." << std::endl;
        // openarm.get_gripper().open();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (int i = 0; i < 100; i++) {
            openarm.refresh_all();
            openarm.recv_all();

            // Display arm motor states
            for (const auto& motor : openarm.get_arm().get_motors()) {
                std::cout << "Arm Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
            // Display gripper state
            for (const auto& motor : openarm.get_gripper().get_motors()) {
                std::cout << "Gripper Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Test gripper close
        std::cout << "Closing gripper..." << std::endl;
        // openarm.get_gripper().close();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        openarm.disable_all();
        openarm.recv_all();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
