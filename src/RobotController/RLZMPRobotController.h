#ifndef RL_ZMP_ROBOT_CONTROLLER_H
#define RL_ZMP_ROBOT_CONTROLLER_H

#include "RobotController.h"
#include "CommonTypes.h" 
#include "RLModel.h"
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
namespace SIAT {


class RLZMPRobotController : public RobotController {
    public:
        RLZMPRobotController();
        ~RLZMPRobotController();

        bool initialize() override;
        void shutdown() override;
        void setTrajectory(const std::vector<std::string>& traj) override;
        void executeCommand(CommandType cmd) override;
        void run() override; 


    private:
        // std::vector<std::vector<double>> readTrajectory();
        // std::vector<std::vector<double>> parseTrajectory(const std::vector<std::string>& trajectory);
        Eigen::MatrixXd readTrajectory();
        Eigen::MatrixXd parseTrajectory(const std::vector<std::string>& lines);

        void controlRobot(const Eigen::Vector3d& l_pos,
            const Eigen::Vector3d& r_pos,
            int flagIndex,
            double l_zmp,
            double r_zmp,
            const Eigen::Vector3d& com);
        void runTrajectory();
        at::Tensor calculateObservation(at::Tensor obs, double* position, double* zmp);
        
        void performGracefulStop();

    private:   
        static constexpr int kJointCount = 6;
        static constexpr int kRobotCount = 2;
        static constexpr int kMotorCount = kJointCount * kRobotCount;

        // ================== 控制状态标志 ==================
        bool is_model_loaded_ = false;
        bool is_connected_ = false;
        bool is_active_ = false;
        bool stop_requested_ = false;

        // ================== 模型与运动学相关 ==================
        RLModel model_rl_;
        pinocchio::Model pinocchio_model_;
        pinocchio::Data pinocchio_data_;
        Eigen::VectorXd p_r_;
        double* pos_ptr_ = nullptr;

        // ================== 轨迹与感知数据 ==================
        Eigen::MatrixXd active_trajectory_;  // 当前轨迹
        Eigen::MatrixXd pending_trajectory_; // 待切换轨迹
        bool trajectory_switch_requested_ = false;           // 是否请求切换轨迹
        std::mutex trajectory_mutex_;                         // 轨迹更新时的锁        

        Eigen::MatrixXd trajectory_matrix_, l_pos_, r_pos_, com_, zmp_;
        at::Tensor obs_ = torch::zeros(38);

        // ================== 硬件接口 ==================
        int axis_ids_[kJointCount];
        const char* robot_names_[kRobotCount];
        double pos_target_[kMotorCount] = {0.0};
        int torque_[kMotorCount] = {0};
        double position_[kMotorCount] = {0.0};

        // ================== 状态缓存 ==================
        State02 base_, r_foot_, l_foot_;
        State base_state_, r_foot_state_, l_foot_state_;
        ForceTorque initial_left_foot_sensor_data_;  // 左脚力传感器的初始数据
        ForceTorque initial_right_foot_sensor_data_; // 右脚力传感器的初始数据
        // ================== ZMP 相关 ==================
        double zmp_ref_[2];
        double delta_zmp_[2];
        double delta_com_[2];
        double obs_record_[38];

        // ================== 轨迹控制索引 ==================
        int trajectory_size_ = 0;

        // ================== 保存数据 ==================
        bool save_to_file_ = true;  // 控制是否保存数据到文件

        // 保存数据到文件的函数
        void saveDataToFile(
            const Eigen::Vector3d& l_pos,
            const Eigen::Vector3d& r_pos,
            int flagIndex,
            double l_zmp,
            double r_zmp,
            const Eigen::Vector3d& com,
            const double* temp,
            const int* torque_,
            const double* position_);
    
};


}  // namespace SIAT

#endif  // RL_ZMP_ROBOT_CONTROLLER_H
