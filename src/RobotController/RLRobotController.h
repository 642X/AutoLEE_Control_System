// #ifndef RLROBOTCONTROLLER_H
// #define RLROBOTCONTROLLER_H

// #include "RobotController.h"
// #include "RLModel.h"
// #include <vector>
// #include <string>
// #include <Eigen/Dense>

// namespace SIAT {

// class RLRobotController : public RobotController {
// public:
//     RLRobotController();
//     virtual ~RLRobotController();

//     bool initialize() override;
//     void shutdown() override;

//     std::vector<std::vector<double>> readTrajectory() override;
//     at::Tensor calculate_obs(at::Tensor obs, double* position, double* L_position, double* R_position);
//     void controlRobot(const std::vector<double>& frame) override;
//     void runTrajectory(const std::vector<std::vector<double>>& trajectory) override;
//     std::string readRobotState() override;
//     void communicateWithLayer(const std::string& message) override;

// private:
//     RLModel model;
//     bool isModelLoaded;
//     bool isConnected;
//     std::string currentState;
//     std::string robotID;

//     // 机器人和电机相关参数
//     const int jointNumbers = 6;  // 每个机器人有6个关节
//     const int robotNumbers = 2;  // 机器人数量（2个机器人）
//     int axisIDs[6];              // 电机轴ID
//     const char* robotNames[2];   // 常量指针数组，用于存储机器人名称

//     // 轨迹和机器人状态
//     Eigen::MatrixXd trajectoryMatrix;
//     Eigen::MatrixXd L_pos, R_pos;
//     double posTarget[12];
//     int torque[12];
//     double position[12];

//     // 观察值
//     at::Tensor obs;

//     // 内部函数
//     at::Tensor preprocessTrajectory(const std::vector<std::string>& trajectory);
// };

// } // namespace SIAT

// #endif // RLROBOTCONTROLLER_H
