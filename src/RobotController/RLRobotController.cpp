// #include "RLRobotController.h"
// #include "HYYRobotInterface.h"
// #include "torch/script.h"
// #include "torch/torch.h"
// #include "Logger.h"  // 添加日志模块
// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>

// namespace SIAT {

// RLRobotController::RLRobotController() : isModelLoaded(false), isConnected(false) {
//     LOGI("RLController", "构造函数调用");
// }

// RLRobotController::~RLRobotController() {
//     shutdown();  // 保证资源释放
//     LOGI("RLController", "析构函数调用");
// }

// bool RLRobotController::initialize() {
//     // 加载强化学习模型、初始化状态等
//     LOGI("RLController", "正在加载模型...");
//     isModelLoaded = model.loadModel("/home/robot/Work/system/policy_net/policy.pt");

//     if (!isModelLoaded) {
//         LOGE("RLController", "模型加载失败，初始化中止");
//         return false;
//     }

//     // 获取机器人名称并初始化
//     robotNames[0] = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 0);
//     robotNames[1] = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 1);

//     // 初始化机器人
//     sleep(1);
//     for (int i = 0; i < robotNumbers; i++) {
//         for (int j = 0; j < jointNumbers; j++) {
//             axisIDs[j] = j + 1;
//             HYYRobotBase::set_axis_control(robotNames[i], 128, axisIDs[j]);
//             HYYRobotBase::set_axis_mode(robotNames[i], 8, axisIDs[j]);  // 同步位置模式
//             sleep(1);
//         }
//     }

//     // 上电启动机器人
//     HYYRobotBase::move_start();

//     // 读取轨迹并存入Eigen::MatrixXd
//     std::vector<std::vector<double>> trajectory = readTrajectory();
//     trajectoryMatrix = Eigen::MatrixXd(trajectory.size(), trajectory[0].size());

//     for (int i = 0; i < trajectory.size(); ++i) {
//         for (int j = 0; j < trajectory[0].size(); ++j) {
//             trajectoryMatrix(i, j) = trajectory[i][j];  // 填充轨迹矩阵
//         }
//     }

//     // 提取左右脚的位姿
//     L_pos = trajectoryMatrix.block(0, 0, trajectoryMatrix.rows(), 3);  // 左脚位置
//     R_pos = trajectoryMatrix.block(0, 3, trajectoryMatrix.rows(), 3);  // 右脚位置

//     isModelLoaded = true;
//     isConnected = true;
//     robotID = "RLBot01";

//     LOGI("RLController", "初始化完成，ID: " + robotID);
//     return true;
// }

// void RLRobotController::shutdown() {
//     isModelLoaded = false;
//     isConnected = false;
//     LOGI("RLController", "控制器已关闭，资源已释放");
// }

// std::vector<std::vector<double>> RLRobotController::readTrajectory() {
//     std::vector<std::vector<double>> trajectory;
//     std::string trajectory_filename = "trajectory/trajectory.txt";
//     std::ifstream ifs(trajectory_filename);
//     std::string line;

//     if (!ifs) {
//         LOGE("RLController", "无法打开轨迹文件：" + trajectory_filename);
//         return trajectory;  // 返回空轨迹
//     }

//     while (getline(ifs, line)) {
//         std::stringstream ss(line);
//         std::string token;
//         std::vector<double> singleInput;

//         while (getline(ss, token, ',')) {
//             singleInput.push_back(stod(token));
//         }
//         trajectory.push_back(singleInput);
//     }
//     ifs.close();

//     LOGD("RLController", "读取轨迹，共 " + std::to_string(trajectory.size()) + " 个点");
//     return trajectory;
// }

// at::Tensor RLRobotController::calculate_obs(at::Tensor obs, double* position, double* L_position, double* R_position) {
//     TORCH_CHECK(obs.dim() == 1 && obs.size(0) >= 38, "obs 必须是一维长度至少为 38 的 Tensor");

//     // 平移历史观测
//     obs.index_put_({torch::indexing::Slice(24, 36)}, obs.index({torch::indexing::Slice(12, 24)}));
//     obs.index_put_({torch::indexing::Slice(12, 24)}, obs.index({torch::indexing::Slice(0, 12)}));

//     // 写入新观测（位置）
//     for (int i = 0; i < 6; ++i) {
//         obs.index_put_({i * 2}, position[6 + i] / 10.0);     // 速度
//         obs.index_put_({i * 2 + 1}, position[i] / 10.0);     // 位置
//     }

//     // 写入左右手位置
//     obs.index_put_({36}, *L_position);
//     obs.index_put_({37}, *R_position);

//     return obs;
// }


// void RLRobotController::controlRobot(const std::vector<double>& frame) {
//     if (!model.isLoaded()) {
//         LOGE("RLController", "控制失败：模型未初始化！");
//         return;
//     }

//     // 计算当前观测量（obs）
//     obs = calculate_obs(obs, position, R_footState.pos, L_footState.pos);

//     // 推理得到控制指令
//     std::vector<double> output = model.infer(obs);
//     double temp[16];
//     // 控制电机
//     for (int i = 0; i < robotNumbers; ++i) {
//         for (int j = 0; j < jointNumbers; ++j) {
//             posTarget[i * 6 + j] = output[i * 6 + j] * 10.0;

//             HYYRobotBase::SetAxisPosition(robotNames[i], posTarget[i * 6 + j], axisIDs[j]);
//             temp[i * 6 + j]     = HYYRobotBase::GetAxisTargetPosition(robotNames[i], axisIDs[j]);
//             torque[i * 6 + j]   = HYYRobotBase::get_axis_torque(robotNames[i], axisIDs[j]);
//             position[i * 6 + j] = HYYRobotBase::GetAxisPosition(robotNames[i], axisIDs[j]);
//         }
//     }

//     LOGD("RLController", "控制帧完成");
// }

// void RLRobotController::runTrajectory(const std::vector<std::vector<double>>& trajectory) {
//     LOGI("RLController", "开始运行轨迹，共 " + std::to_string(trajectory.size()) + " 帧");

//     for (const auto& frame : trajectory) {
//         controlRobot(frame);
//         std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 控制频率：100Hz
//     }

//     LOGI("RLController", "轨迹运行结束");
// }


// std::string RLRobotController::readRobotState() {
//     std::string state = currentState.empty() ? "IDLE" : currentState;
//     LOGD("RLController", "读取机器人状态: " + state);
//     return state;
// }

// void RLRobotController::communicateWithLayer(const std::string& message) {
//     LOGD("RLController", "与通信层交互信息: " + message);
// }

// } // namespace SIAT
