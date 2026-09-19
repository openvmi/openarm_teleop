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

#include <controller/dynamics.hpp>

Dynamics::Dynamics(std::string urdf_path, std::string start_link, std::string end_link) {
    this->urdf_path = urdf_path;
    this->start_link = start_link;
    this->end_link = end_link;
}

Dynamics::~Dynamics() {}

bool Dynamics::Init() {
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        fprintf(stderr, "Failed to open URDF file: %s\n", urdf_path.c_str());
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    urdf_model_interface = urdf::parseURDF(buffer.str());
    if (!urdf_model_interface) {
        fprintf(stderr, "Failed to parse URDF: %s\n", urdf_path.c_str());
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(*urdf_model_interface, kdl_tree)) {
        fprintf(stderr, "Failed to extract KDL tree: %s\n", urdf_path.c_str());
        return false;
    }

    if (!kdl_tree.getChain(start_link, end_link, kdl_chain)) {
        fprintf(stderr, "Failed to get KDL chain\n");
        return false;
    }

    std::cout << "[GetGravity] kdl_chain.getNrOfJoints() = " << kdl_chain.getNrOfJoints()
              << std::endl;

    coriolis_forces.resize(kdl_chain.getNrOfJoints());
    gravity_forces.resize(kdl_chain.getNrOfJoints());
    inertia_matrix.resize(kdl_chain.getNrOfJoints());

    coriolis_forces.data.setZero();
    gravity_forces.data.setZero();
    inertia_matrix.data.setZero();

    // Gravity vector expressed in the chain (arm base) frame, following the
    // openarm_hardware/oy_hardware.cpp implementation: in the bimanual URDF
    // the arm roots openarm_{left|right}_link0 are mounted with +/-90 deg
    // rotations relative to the tree root, so the world gravity (0, 0, -9.81)
    // must be rotated into the chain base frame before handing it to
    // ChainDynParam. Using the world vector directly made the compensation
    // torque push in wrong directions and destabilized the arms.
    KDL::Vector world_gravity(0.0, 0.0, -9.81);
    KDL::Vector chain_gravity = world_gravity;
    const std::string tree_root = kdl_tree.getRootSegment()->first;
    if (start_link != tree_root) {
        KDL::Chain chain_to_arm;
        if (kdl_tree.getChain(tree_root, start_link, chain_to_arm)) {
            KDL::Frame T = KDL::Frame::Identity();
            for (unsigned i = 0; i < chain_to_arm.getNrOfSegments(); ++i)
                T = T * chain_to_arm.getSegment(i).pose(0.0);
            chain_gravity = T.M.Inverse() * world_gravity;
        }
    }
    std::cout << "[Dynamics] gravity vector in chain base frame: (" << chain_gravity.x()
              << ", " << chain_gravity.y() << ", " << chain_gravity.z() << ")" << std::endl;

    solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, chain_gravity);

    return true;
}

void Dynamics::GetGravity(const double *motor_position, double *gravity) {
    const auto njoints = kdl_chain.getNrOfJoints();

    KDL::JntArray q_(kdl_chain.getNrOfJoints());

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
    }

    solver->JntToGravity(q_, gravity_forces);
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        gravity[i] = gravity_forces(i);
    }
}

void Dynamics::GetCoriolis(const double *motor_position, const double *motor_velocity,
                           double *coriolis) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    KDL::JntArray q_dot(kdl_chain.getNrOfJoints());

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
        q_dot(i) = motor_velocity[i];
    }

    solver->JntToCoriolis(q_, q_dot, coriolis_forces);

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        coriolis[i] = coriolis_forces(i);
    }
}

void Dynamics::GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    KDL::JntSpaceInertiaMatrix inertia_matrix(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
    }

    solver->JntToMass(q_, inertia_matrix);

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        inertia_diag[i] = inertia_matrix(i, i);
    }
}

void Dynamics::GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::Jacobian kdl_jac(kdl_chain.getNrOfJoints());
    KDL::ChainJntToJacSolver jac_solver(kdl_chain);
    jac_solver.JntToJac(q_, kdl_jac);

    jacobian = Eigen::MatrixXd(6, kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < kdl_chain.getNrOfJoints(); ++j) {
            jacobian(i, j) = kdl_jac(i, j);
        }
    }
}

void Dynamics::GetNullSpace(const double *motor_position, Eigen::MatrixXd &nullspace) {
    const size_t dof = kdl_chain.getNrOfJoints();

    bool use_stable_svd = false;

    Eigen::MatrixXd J;
    GetJacobian(motor_position, J);

    Eigen::MatrixXd J_pinv;

    if (use_stable_svd) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol =
            1e-6 * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff();
        Eigen::VectorXd singularValuesInv = svd.singularValues();
        for (int i = 0; i < singularValuesInv.size(); ++i) {
            singularValuesInv(i) = (singularValuesInv(i) > tol) ? 1.0 / singularValuesInv(i) : 0.0;
        }
        J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    } else {
        J_pinv = J.transpose() * (J * J.transpose()).inverse();
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
    nullspace = I - J_pinv * J;

    //        std::cout << "[INFO] Null space projector computed.\n";
}

void Dynamics::GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T) {
    const size_t dof = kdl_chain.getNrOfJoints();
    bool use_stable_svd = false;

    Eigen::MatrixXd J;
    GetJacobian(motor_position, J);

    Eigen::MatrixXd J_pinv;

    if (use_stable_svd) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol =
            1e-6 * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff();
        Eigen::VectorXd singularValuesInv = svd.singularValues();
        for (int i = 0; i < singularValuesInv.size(); ++i) {
            singularValuesInv(i) = (singularValuesInv(i) > tol) ? 1.0 / singularValuesInv(i) : 0.0;
        }
        J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    } else {
        J_pinv = J.transpose() * (J * J.transpose()).inverse();
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
    Eigen::MatrixXd N = I - J_pinv * J;

    nullspace_T = N.transpose();
}

void Dynamics::GetEECordinate(const double *motor_position, Eigen::Matrix3d &R,
                              Eigen::Vector3d &p) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;

    if (fk_solver.JntToCart(q_, kdl_frame) < 0) {
        //  std::cerr << "[KDL] FK failed in GetEECordinate!" << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);

    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
}

void Dynamics::GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R,
                                 Eigen::Vector3d &p) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;

    if (fk_solver.JntToCart(q_, kdl_frame, kdl_chain.getNrOfSegments() - 1) < 0) {
        //        std::cerr << "[KDL] FK failed in GetPreEECordinate!" << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);

    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
}
