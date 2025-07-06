#ifndef TOOLS_H
#define TOOLS_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>
#include <unordered_map>
#include "HYYRobotInterface.h"
#include "BusCommunication.h"
#include "CommonTypes.h"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace SIAT {

std::optional<CommandType> ParseCommand(const std::string& str);
std::string CommandToString(CommandType cmd);
std::string GetResourcePath(const std::string& relative_path);

State convertToState(const State02& state);

double* calculationZMP(const ForceTorque& left_data, const ForceTorque& right_data,
                       const Eigen::Vector3d& left_pos, const Eigen::Vector3d& right_pos);

State joint_pr(const State& Base, const State& Link);

Eigen::VectorXd inverse_kinematics(pinocchio::Model model, pinocchio::Data data,
                                   State Base, State R_foot, State L_foot,
                                   Eigen::VectorXd joint_init);

int readSensors(ForceTorque& sensor_data, int start_byte, bool* rca_flag = nullptr);

}  // namespace SIAT

#endif  // TOOLS_H
