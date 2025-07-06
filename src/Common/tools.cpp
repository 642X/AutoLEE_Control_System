#ifndef TOOLS_H
#define TOOLS_H

#include <Eigen/Dense>
#include <vector>
#include "HYYRobotInterface.h"
#include "BusCommunication.h"
#include "CommonTypes.h"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace SIAT {

std::optional<CommandType> ParseCommand(const std::string& str) {
    static const std::unordered_map<std::string, CommandType> kCommandMap = {
        {"start", CommandType::kStart},
        {"stop", CommandType::kStop},
        {"pause", CommandType::kPause},
        {"reset", CommandType::kReset}
    };

    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    auto it = kCommandMap.find(lower);
    return (it != kCommandMap.end()) ? std::optional<CommandType>(it->second) : std::nullopt;
}

std::string CommandToString(CommandType cmd) {
    switch (cmd) {
        case CommandType::kStart: return "start";
        case CommandType::kStop:  return "stop";
        case CommandType::kPause: return "pause";
        case CommandType::kReset: return "reset";
        default:                  return "unknown";
    }
}

std::string GetResourcePath(const std::string& relative_path) {
    const char* root = std::getenv("RESOURCE_PATH");
    if (root)
        return std::string(root) + "/" + relative_path;
    else
        return "../share/RobotSystem/system/" + relative_path;  // fallback 路径
}

State convertToState(const State02& state) {
    State result;
    result.pos = state.pos;
    result.rot = state.rot.toRotationMatrix();
    return result;
}

double* calculationZMP(const ForceTorque& left_data, const ForceTorque& right_data, 
    const Eigen::Vector3d& left_pos, const Eigen::Vector3d& right_pos) {
    // ForceTorque 中的数据转换为 double
    double ft1_data[6] = {left_data.force.x, left_data.force.y, left_data.force.z, 
        left_data.torque.x, left_data.torque.y, left_data.torque.z};
    double ft2_data[6] = {right_data.force.x, right_data.force.y, right_data.force.z, 
        right_data.torque.x, right_data.torque.y, right_data.torque.z};

    // 这里继续处理 ZMP 计算逻辑
    ft1_data[2] = -ft1_data[2];
    ft1_data[5] = -ft1_data[5];
    ft2_data[2] = -ft2_data[2];
    ft2_data[5] = -ft2_data[5];

    double tmp_1_0 = ft1_data[0];
    double tmp_1_3 = ft1_data[3];
    ft1_data[0] = ft1_data[1];
    ft1_data[3] = ft1_data[4];
    ft1_data[1] = -tmp_1_0;
    ft1_data[4] = -tmp_1_3;

    double tmp_2_0 = ft2_data[0];
    double tmp_2_3 = ft2_data[3];
    ft2_data[0] = ft2_data[1];
    ft2_data[3] = ft2_data[4];
    ft2_data[1] = -tmp_2_0;
    ft2_data[4] = -tmp_2_3;

    double P_L_x;
    double P_L_y;
    const double L_x = 0.044 + left_pos.x();
    const double L_y = -0.0975 + left_pos.y();

    double P_R_x;
    double P_R_y;
    const double R_x = 0.044 + right_pos.x();
    const double R_y = 0.0975 + right_pos.y();

    double temple = 10.0;
    double d = 0.0195;

    // 计算左侧 ZMP 位置
    double P_L_x_0 = (-ft1_data[4] - ft1_data[0] * d) / fmax(ft1_data[2], temple);
    double P_L_y_0 = (ft1_data[3] - ft1_data[1] * d) / fmax(ft1_data[2], temple);
    P_L_x = P_L_x_0 + L_x;
    P_L_y = P_L_y_0 + L_y;

    // 计算右侧 ZMP 位置
    double P_R_x_0 = (-ft2_data[4] - ft2_data[0] * d) / fmax(ft2_data[2], temple);
    double P_R_y_0 = (ft2_data[3] - ft2_data[1] * d) / fmax(ft2_data[2], temple);
    P_R_x = P_R_x_0 + R_x;
    P_R_y = P_R_y_0 + R_y;

    // 计算双足 ZMP 位置
    double P_x = (P_L_x * ft1_data[2] + P_R_x * ft2_data[2]) / 
    (fmax(ft1_data[2], temple) + fmax(ft2_data[2], temple));
    double P_y = (P_L_y * ft1_data[2] + P_R_y * ft2_data[2]) / 
    (fmax(ft1_data[2], temple) + fmax(ft2_data[2], temple));

    static double P[2] = {P_x, P_y};
    return P;
}


State joint_pr(const State& Base, const State& Link)
{
    State state;

    state.pos = Base.rot.transpose() * (Link.pos - Base.pos);
    state.rot = Base.rot.transpose() * Link.rot;
    return state;
}

Eigen::VectorXd inverse_kinematics(pinocchio::Model model, pinocchio::Data data, State Base, State R_foot, State L_foot, Eigen::VectorXd joint_init)
{
    State state_R = joint_pr(Base, R_foot);
    State state_L = joint_pr(Base, L_foot);
    //std::cout << "base:" << std::endl << Base.pos << std::endl;
    //std::cout << "base:" << std::endl << R_foot.pos << std::endl;
    const int JOINT_ID_L = 6;
    const int JOINT_ID_R = 12;

    pinocchio::SE3 oMdes_R(state_R.rot, state_R.pos);
    pinocchio::SE3 oMdes_L(state_L.rot, state_L.pos);

    Eigen::VectorXd q = joint_init;
    const double eps = 1e-6;
    const int IT_MAX = 100;
    const double DT = 1;
    const double damp = 1e-6;

    pinocchio::Data::Matrix6x J_R(6,model.nv);
    J_R.setZero();
    pinocchio::Data::Matrix6x J_L(6,model.nv);
    J_L.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err_R;
    Vector6d err_L;

    Eigen::VectorXd v_R(model.nv);
    Eigen::VectorXd v_L(model.nv);

    for (int i=0; ;i++)
    {
        pinocchio::forwardKinematics(model, data, q);
        const pinocchio::SE3 iMd_R = data.oMi[JOINT_ID_R].actInv(oMdes_R);
        const pinocchio::SE3 iMd_L = data.oMi[JOINT_ID_L].actInv(oMdes_L);

        err_R = pinocchio::log6(iMd_R).toVector();
        err_L = pinocchio::log6(iMd_L).toVector();

        if (err_R.norm() < eps && err_L.norm() < eps)
        {
            success = true;
            break;
        }

        if (i >= IT_MAX)
        {
            // std::cout << "unsuccess" << std::endl;
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model, data, q, JOINT_ID_R, J_R);
        pinocchio::Data::Matrix6 Jlog_R;
        pinocchio::Jlog6(iMd_R.inverse(), Jlog_R);
        J_R = -Jlog_R * J_R;
        pinocchio::Data::Matrix6 JJt_R;
        JJt_R.noalias() = J_R * J_R.transpose();
        JJt_R.diagonal().array() += damp;
        v_R.noalias() = - J_R.transpose() * JJt_R.ldlt().solve(err_R);
        q = pinocchio::integrate(model,q,v_R*DT);

        pinocchio::computeJointJacobian(model, data, q, JOINT_ID_L, J_L);
        pinocchio::Data::Matrix6 Jlog_L;
        pinocchio::Jlog6(iMd_L.inverse(), Jlog_L);
        J_L = -Jlog_L * J_L;
        pinocchio::Data::Matrix6 JJt_L;
        JJt_L.noalias() = J_L * J_L.transpose();
        JJt_L.diagonal().array() += damp;
        v_L.noalias() = - J_L.transpose() * JJt_L.ldlt().solve(err_L);
        q = pinocchio::integrate(model,q,v_L*DT);
    }

    if (success)
    {
        return q;
    }
    else
    {
        return joint_init;
    }
}

/**
 * @brief 读取传感器数据并填充到 `ForceTorque` 类型的结构中。
 *
 * 该函数会从传感器获取力和力矩数据，并将其存储到传入的 `result` 中。
 *
 * @param[out] result 用于存储读取到的力和力矩数据。
 * @param[in] start_byte 起始字节位置，用于指定传感器数据的偏移量。
 * @param[out] rca_flag 可选参数，指示是否有 RCA（如果传入 NULL，则不处理）。
 * @return 成功返回 0。
 */
int readSensors(ForceTorque& sensor_data, int start_byte, bool* rca_flag = nullptr) {
    // 初始化传感器，只有第一次调用时进行
    static bool init_sensor_flag = true;
    if (init_sensor_flag) {
        InitMaster(298, 162, 0);  // 初始化设备，输入输出长度
        init_sensor_flag = false;   // 确保只初始化一次
    }

    // 临时变量
    float fx, fy, fz, mx, my, mz;

    // 获取力传感器数据
    GetMasterPdo(0, start_byte, 32, &fx);    // 获取力数据
    GetMasterPdo(0, start_byte + 32, 32, &fy);
    GetMasterPdo(0, start_byte + 64, 32, &fz);

    // 获取力矩传感器数据
    GetMasterPdo(0, start_byte + 96, 32, &mx);  // 获取力矩数据
    GetMasterPdo(0, start_byte + 128, 32, &my);
    GetMasterPdo(0, start_byte + 160, 32, &mz);

    // 填充到 `sensor_data` 结构中
    sensor_data.force = Vec3d(fx, fy, fz);
    sensor_data.torque = Vec3d(mx, my, mz);

    // 获取 RCA 数据
    if (rca_flag != nullptr) {
        GetMasterPdo(0, 0, 1, rca_flag);
    }

    return 0;
}



}  // namespace SIAT

#endif  // TOOLS_H
