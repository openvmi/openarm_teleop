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

#pragma once

#include <chrono>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <string>
#include <thread>

#include "../openarm_constants.hpp"

namespace openarm_init {

class OpenArmInitializer {
public:
    /**
     * @brief Initialize OpenArm with default configuration
     * @param can_device CAN device name (e.g., "can0", "can1")
     * @param can_fd Enable CAN-FD frames on the socket (default false: the OY
     *               arms run classic CAN at 1 Mbps, see init_can.sh and the
     *               can_fd:=false default in oy.urdf.xacro)
     * @return Initialized OpenArm pointer (caller owns memory)
     */
    static openarm::can::socket::OpenArm *initialize_openarm(const std::string &can_device,
                                                             bool can_fd = false);

    /**
     * @brief Initialize OpenArm with custom motor configuration
     * @param can_device CAN device name
     * @param config Custom motor configuration
     * @param can_fd Enable CAN-FD frames on the socket (default false, classic CAN)
     * @return Initialized OpenArm pointer (caller owns memory)
     */
    static openarm::can::socket::OpenArm *initialize_openarm(const std::string &can_device,
                                                             const MotorConfig &config,
                                                             bool can_fd = false);

private:
    /**
     * @brief Common initialization steps for OpenArm
     */
    static void initialize_(openarm::can::socket::OpenArm *openarm, const MotorConfig &config);
};

}  // namespace openarm_init
