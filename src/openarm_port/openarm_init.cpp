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

#include "openarm_init.hpp"

#include "../openarm_constants.hpp"

namespace openarm_init {

openarm::can::socket::OpenArm *OpenArmInitializer::initialize_openarm(const std::string &can_device,
                                                                      bool can_fd) {
    MotorConfig config = DEFAULT_MOTOR_CONFIG;
    return initialize_openarm(can_device, config, can_fd);
}

openarm::can::socket::OpenArm *OpenArmInitializer::initialize_openarm(const std::string &can_device,
                                                                      const MotorConfig &config,
                                                                      bool can_fd) {
    // Create OpenArm instance (OY motors, CAN-FD capable)
    openarm::can::socket::OpenArm *openarm =
        new openarm::can::socket::OpenArm(can_device, can_fd);

    // Perform common initialization
    initialize_(openarm, config);

    return openarm;
}

void OpenArmInitializer::initialize_(openarm::can::socket::OpenArm *openarm,
                                     const MotorConfig &config) {
    std::cout << "Initializing arm motors (OY) on " << openarm->can_interface()
              << (openarm->can_fd_enabled() ? " [CAN-FD]" : " [classic CAN]") << "..." << std::endl;

    // Initialize arm motors
    openarm->init_arm_motors(config.arm_motor_types, config.arm_send_can_ids,
                             config.arm_recv_can_ids);

    std::cout << "Initializing gripper motor (OY)..." << std::endl;

    // Initialize gripper motor
    openarm->init_gripper_motor(config.gripper_motor_type, config.gripper_send_can_id,
                                config.gripper_recv_can_id);

    // Set callback mode for all motors
    openarm->set_callback_mode_all(openarm::oy_motor::CallbackMode::STATE);

    std::cout << "Enabling motors..." << std::endl;

    // Enable all motors with appropriate delays
    openarm->enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm->recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Print motor counts for verification
    size_t arm_motor_num = openarm->get_arm().get_motors().size();
    size_t gripper_motor_num = openarm->get_gripper().get_motors().size();

    std::cout << "Arm motor count: " << arm_motor_num << std::endl;
    std::cout << "Gripper motor count: " << gripper_motor_num << std::endl;
}

}  // namespace openarm_init
