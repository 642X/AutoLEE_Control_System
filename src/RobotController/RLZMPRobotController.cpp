#include "RLZMPRobotController.h"
#include "HYYRobotInterface.h"
#include "tools.h"
#include "torch/script.h"
#include "torch/torch.h"
#include "Logger.h"  // 添加日志模块
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace SIAT {

RLZMPRobotController::RLZMPRobotController() : 
    is_model_loaded_(false), 
    is_connected_(false),
    pinocchio_model_(),
    pinocchio_data_(pinocchio_model_) {
    LOGI("RLController", "构造函数调用");
}

RLZMPRobotController::~RLZMPRobotController() {
    shutdown();  // 保证资源释放
    LOGI("RLController", "析构函数调用");
}

bool RLZMPRobotController::initialize() {
    // 加载强化学习模型、初始化状态等
    LOGI("RLController", "正在加载模型...");
    is_model_loaded_ = model_rl_.loadModel(GetResourcePath("policy_net/policy.pt"));

    if (!is_model_loaded_) {
        LOGE("RLController", "模型加载失败，初始化中止");
        return false;
    }

    // 获取机器人名称并初始化
    robot_names_[0] = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 0);
    robot_names_[1] = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 1);

    // 初始化机器人
    sleep(1);
    for (int i = 0; i < kRobotCount; i++) {
        for (int j = 0; j < kJointCount; j++) {
            axis_ids_[j] = j + 1;
            HYYRobotBase::set_axis_control(robot_names_[i], 128, axis_ids_[j]);
            HYYRobotBase::set_axis_mode(robot_names_[i], 8, axis_ids_[j]);  // 同步位置模式
            sleep(1);
        }
    }

    // 上电启动机器人
    HYYRobotBase::move_start();

    //pinocchio模型建立和缓存区申请
    std::string urdf_filename = GetResourcePath("urdf/SIAT01/urdf/SIAT01.urdf");
    pinocchio::urdf::buildModel(urdf_filename, pinocchio_model_);    
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
        
    // 读取轨迹并存入Eigen::MatrixXd
    active_trajectory_ = readTrajectory();
    trajectory_size_ = active_trajectory_.rows();  // 获取轨迹大小

    // 初始化状态

    base_.pos = Eigen::Vector3d(0, 0, 0); // 基础位置
    base_.rot = Eigen::Quaterniond(1, 0, 0, 0); // 基础旋转

    r_foot_.pos = Eigen::Vector3d(0, 0, 0); // 右脚位置
    r_foot_.rot = Eigen::Quaterniond(0, -0.7071, 0, -0.7071); // 右脚旋转

    l_foot_.pos = Eigen::Vector3d(0, 0, 0); // 左脚位置
    l_foot_.rot = Eigen::Quaterniond(0, -0.7071, 0, -0.7071); // 左脚旋转

    base_state_ = convertToState(base_); // 转换为State类型
    r_foot_state_ = convertToState(r_foot_);
    l_foot_state_ = convertToState(l_foot_);

    p_r_ = Eigen::VectorXd::Zero(kMotorCount);

    SIAT::readSensors(initial_left_foot_sensor_data_, 1504);//FT1
    SIAT::readSensors(initial_right_foot_sensor_data_, 2192);//FT2

    is_model_loaded_ = true;
    is_connected_ = true;

    LOGI("RLController", "初始化完成 ");
    return true;
}

void RLZMPRobotController::shutdown() {
    is_model_loaded_ = false;
    is_connected_ = false;
    LOGI("RLController", "控制器已关闭，资源已释放");
}

void RLZMPRobotController::setTrajectory(const std::vector<std::string>& traj) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    // 解析新轨迹（保持原来的解析逻辑）
    pending_trajectory_ = parseTrajectory(traj);

    // 请求切换
    trajectory_switch_requested_ = true;

    LOGI("RLController", "新轨迹已设置，等待切换");
}


Eigen::MatrixXd RLZMPRobotController::readTrajectory() {
    std::string trajectory_filename = GetResourcePath("trajectory/trajectory.txt");
    std::ifstream ifs(trajectory_filename);
    std::string line;

    if (!ifs) {
        LOGE("RLController", "无法打开轨迹文件：" + trajectory_filename);
        return Eigen::MatrixXd();  // 返回空矩阵
    }

    std::vector<std::vector<double>> temp_trajectory;
    size_t cols = 0;

    while (std::getline(ifs, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ',')) {
            values.push_back(std::stod(token));
        }

        if (cols == 0) {
            cols = values.size();  // 确定列数
        } else if (values.size() != cols) {
            LOGE("RLController", "轨迹数据列数不一致");
            return Eigen::MatrixXd();  // 错误处理
        }

        temp_trajectory.push_back(std::move(values));
    }
    ifs.close();

    size_t rows = temp_trajectory.size();
    Eigen::MatrixXd trajectory_matrix(rows, cols);

    for (size_t i = 0; i < rows; ++i) {
        trajectory_matrix.row(i) = Eigen::Map<Eigen::RowVectorXd>(temp_trajectory[i].data(), cols);
    }

    LOGD("RLController", "读取轨迹，共 " + std::to_string(rows) + " 个点");
    return trajectory_matrix;
}

Eigen::MatrixXd RLZMPRobotController::parseTrajectory(const std::vector<std::string>& lines) {
    if (lines.empty()) {
        return Eigen::MatrixXd();  // 返回空矩阵
    }

    std::vector<std::vector<double>> temp_data;
    size_t cols = 0;

    for (const std::string& line : lines) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> row;

        while (std::getline(ss, token, ',')) {
            row.push_back(std::stod(token));
        }

        if (cols == 0) {
            cols = row.size();  // 记录列数
        } else if (row.size() != cols) {
            LOGE("RLController", "轨迹行数据长度不一致");
            return Eigen::MatrixXd();  // 错误处理
        }

        temp_data.push_back(std::move(row));
    }

    size_t rows = temp_data.size();
    Eigen::MatrixXd result(rows, cols);

    for (size_t i = 0; i < rows; ++i) {
        result.row(i) = Eigen::Map<Eigen::RowVectorXd>(temp_data[i].data(), cols);
    }

    return result;
}


void RLZMPRobotController::controlRobot(
    const Eigen::Vector3d& l_pos,
    const Eigen::Vector3d& r_pos,
    int flagIndex,
    double l_zmp,
    double r_zmp,
    const Eigen::Vector3d& com){

    if (!model_rl_.isLoaded()) {
        LOGE("RLController", "控制失败：模型未初始化！");
        return;
    }

    // 更新传感器数据
    ForceTorque left_foot_sensor_data;  // 左脚力传感器
    ForceTorque right_foot_sensor_data; // 右脚力传感器
    SIAT::readSensors(left_foot_sensor_data, 1504);//FT1
    SIAT::readSensors(right_foot_sensor_data, 2192);//FT2

    left_foot_sensor_data.force -= initial_left_foot_sensor_data_.force;
    left_foot_sensor_data.torque -= initial_left_foot_sensor_data_.torque;
    right_foot_sensor_data.force -= initial_right_foot_sensor_data_.force;
    right_foot_sensor_data.torque -= initial_right_foot_sensor_data_.torque;

    double* p_zmp = calculationZMP(left_foot_sensor_data, right_foot_sensor_data, l_pos, r_pos);
    double zmp_real[2];
    zmp_real[0] = *p_zmp;
    zmp_real[1] = *(p_zmp + 1);


    State base_state_;
    State r_foot_state_;
    State l_foot_state_;
    //----------------------czq---------------------
    //前4s和后4s都不加补偿量
    if(flagIndex<1125||flagIndex>(trajectory_size_-1000)){
        base_state_.pos = com;
        r_foot_state_.pos = r_pos;
        l_foot_state_.pos = l_pos;
    }else{
        //取参考ZMP和参考足底力
        delta_zmp_[0] = zmp_real[0] - l_zmp- 0.205; // 计算ZMP差值
        delta_zmp_[1] = zmp_real[1] - 0.8 * r_zmp - 0.00; // 计算ZMP差值

        obs_ = calculateObservation(obs_, position_, zmp_real);

        // 推理得到控制指令
        std::vector<double> output = model_rl_.infer(obs_);

        Eigen::Vector3d delta_com{output[0] * 0.01, output[1] * 0.01, 0.0};
        base_state_.pos = com + delta_com;// 更新基础位置
        // base_state_.pos = COM.row(flagIndex) + deta_COM;
        // r_foot_state_.pos = R_pos.row(flagIndex);
        // l_foot_state_.pos = L_pos.row(flagIndex);
    }

    p_r_= inverse_kinematics(pinocchio_model_, pinocchio_data_, base_state_, r_foot_state_, l_foot_state_, p_r_);
    double* pos_Ptr = p_r_.data();
    double target_position[kMotorCount];

    // 控制电机
    for (int i = 0; i < kRobotCount; ++i) {
        for (int j = 0; j < kJointCount; ++j) {
            pos_target_[i * 6 + j] = pos_Ptr[i * 6 + j] * 10.0;

            HYYRobotBase::SetAxisPosition(robot_names_[i], pos_target_[i * 6 + j], axis_ids_[j]);
            target_position[i * 6 + j]     = HYYRobotBase::GetAxisTargetPosition(robot_names_[i], axis_ids_[j]);
            torque_[i * 6 + j]   = HYYRobotBase::get_axis_torque(robot_names_[i], axis_ids_[j]);
            position_[i * 6 + j] = HYYRobotBase::GetAxisPosition(robot_names_[i], axis_ids_[j]);
        }
    }
    // 只在开关为 true 时才保存数据
    if (save_to_file_) {
        saveDataToFile(l_pos, r_pos, flagIndex, l_zmp, r_zmp, com, target_position, torque_, position_);
    }
    LOGD("RLController", "控制帧完成");
}

void RLZMPRobotController::runTrajectory() {
    LOGI("RLController", "开始运行轨迹，共 " + std::to_string(trajectory_size_) + " 帧");

    int flag_index = 0;

    while (is_active_) {
        // 优先检查是否有停机请求
        if (stop_requested_) {
            LOGW("RLController", "检测到停机请求，进入缓停模式");
            performGracefulStop();  // 停止当前轨迹并进入缓停
            stop_requested_ = false;
            is_active_ = false;
            break;
        }

        // 检查是否请求切换轨迹
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (trajectory_switch_requested_) {
                active_trajectory_ = pending_trajectory_;  // 切换轨迹
                trajectory_switch_requested_ = false;
                flag_index = 0;  // 从新轨迹的开始处执行
                trajectory_size_ = active_trajectory_.rows();  // 更新轨迹大小
                LOGI("RLController", "轨迹已切换");
            }
        }

        if (flag_index < trajectory_size_) {
            // 直接从 active_trajectory_ 提取当前帧数据
            const Eigen::Vector3d l_pos = active_trajectory_.row(flag_index).segment<3>(0);  // 左脚位置
            const Eigen::Vector3d r_pos = active_trajectory_.row(flag_index).segment<3>(3);  // 右脚位置
            const Eigen::Vector3d com   = active_trajectory_.row(flag_index).segment<3>(6);  // 质心位置
            double l_zmp = active_trajectory_(flag_index, 9);  // 左脚 ZMP
            double r_zmp = active_trajectory_(flag_index, 10); // 右脚 ZMP
        
            // 调用控制函数
            controlRobot(l_pos, r_pos, flag_index, l_zmp, r_zmp, com);
        
            flag_index++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(4));  // 控制频率
    }

    LOGI("RLController", "轨迹运行结束");
}

at::Tensor RLZMPRobotController::calculateObservation(at::Tensor obs, double * position, double * zmp){

    obs.index_put_({torch::indexing::Slice(24, 36)}, obs.index({torch::indexing::Slice(12, 24)}));
    obs.index_put_({torch::indexing::Slice(12, 24)}, obs.index({torch::indexing::Slice(0, 12)}));

    for (int i = 0; i < 6; ++i) {
        obs.index_put_({i*2}, position[6+i]/10);
        obs.index_put_({i*2+1}, position[i]/10);
    }

    for (int i = 0; i < 2; ++i) {
        obs.index_put_({36 + i}, zmp[i]);
    }

    return obs;
}



void RLZMPRobotController::performGracefulStop() {
    LOGI("RLController", "执行平滑停机...");

    const int stop_steps = 100;  // 停机过程的插值步数
    Eigen::VectorXd current_pose = p_r_;
    Eigen::VectorXd zero_pose = Eigen::VectorXd::Zero(kMotorCount);

    for (int step = 0; step < stop_steps; ++step) {
        double ratio = 1.0 - static_cast<double>(step) / stop_steps;
        Eigen::VectorXd target_pose = ratio * current_pose + (1 - ratio) * zero_pose;

        for (int i = 0; i < kRobotCount; ++i) {
            for (int j = 0; j < kJointCount; ++j) {
                int idx = i * 6 + j;
                pos_target_[idx] = target_pose[idx] * 10.0;
                HYYRobotBase::SetAxisPosition(robot_names_[i], pos_target_[idx], axis_ids_[j]);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(4)); // 匹配原始控制频率
    }

    LOGI("RLController", "平滑停机完成");
}

void RLZMPRobotController::executeCommand(CommandType cmd) {
    switch (cmd) {
        case CommandType::kPause:
            if (is_active_) {
                LOGI("RLController", "收到 pause 命令，暂停控制器");
                is_active_ = false;  // 暂停控制（停止轨迹执行，但不中断系统）
            } else {
                LOGW("RLController", "控制器已暂停，无需重复执行 pause");
            }
            break;

        case CommandType::kReset:
            LOGI("RLController", "收到 reset 命令，重置控制器状态");
            shutdown();       // 清除现有状态
            initialize();     // 重新初始化
            break;

        case CommandType::kStop:
            stop_requested_ = true;  // 设置停机请求标志
            LOGW("RLController", "stop 命令");
            break;

        case CommandType::kStart:
            is_active_ = true;  // 设置活动状态
            LOGW("RLController", "start 命令");
            break;

        default:
            LOGW("RLController", "executeCommand 收到未知命令，忽略处理");
            break;
    }
}
void RLZMPRobotController::saveDataToFile(
    const Eigen::Vector3d& l_pos,
    const Eigen::Vector3d& r_pos,
    int flagIndex,
    double l_zmp,
    double r_zmp,
    const Eigen::Vector3d& com,
    const double* target_position,
    const int* torque,
    const double* position) {

    // 使用 GetResourcePath 获取路径
    std::string path = GetResourcePath("data/");

    // 创建文件流
    std::ofstream file_l_pos(path + "l_pos.txt", std::ios::app);
    std::ofstream file_r_pos(path + "r_pos.txt", std::ios::app);
    std::ofstream file_flagIndex(path + "flagIndex.txt", std::ios::app);
    std::ofstream file_l_zmp(path + "l_zmp.txt", std::ios::app);
    std::ofstream file_r_zmp(path + "r_zmp.txt", std::ios::app);
    std::ofstream file_com(path + "com.txt", std::ios::app);
    std::ofstream file_temp_target_position(path + "target_position.txt", std::ios::app);
    std::ofstream file_torque(path + "torque.txt", std::ios::app);
    std::ofstream file_position(path + "position.txt", std::ios::app);

    // 保存数据到文件
    if (file_l_pos.is_open()) {
        file_l_pos << l_pos.transpose() << "\n";
    }
    if (file_r_pos.is_open()) {
        file_r_pos << r_pos.transpose() << "\n";
    }
    if (file_flagIndex.is_open()) {
        file_flagIndex << flagIndex << "\n";
    }
    if (file_l_zmp.is_open()) {
        file_l_zmp << l_zmp << "\n";
    }
    if (file_r_zmp.is_open()) {
        file_r_zmp << r_zmp << "\n";
    }
    if (file_com.is_open()) {
        file_com << com.transpose() << "\n";
    }

    // 控制过程中的临时数据保存
    if (file_temp_target_position.is_open()) {
        for (int i = 0; i < kRobotCount; ++i) {
            for (int j = 0; j < kJointCount; ++j) {
                file_temp_target_position << target_position[i * 6 + j] << " ";
            }
            file_temp_target_position << "\n";
        }
    }

    if (file_torque.is_open()) {
        for (int i = 0; i < kRobotCount; ++i) {
            for (int j = 0; j < kJointCount; ++j) {
                file_torque << torque[i * 6 + j] << " ";
            }
            file_torque << "\n";
        }
    }

    if (file_position.is_open()) {
        for (int i = 0; i < kRobotCount; ++i) {
            for (int j = 0; j < kJointCount; ++j) {
                file_position << position[i * 6 + j] << " ";
            }
            file_position << "\n";
        }
    }

    // 关闭文件流
    file_l_pos.close();
    file_r_pos.close();
    file_flagIndex.close();
    file_l_zmp.close();
    file_r_zmp.close();
    file_com.close();
    file_temp_target_position.close();
    file_torque.close();
    file_position.close();

    LOGD("RLController", "控制帧完成并保存数据到文件");
}

void RLZMPRobotController::run() {
    if (!is_model_loaded_ || !is_connected_) {
        LOGE("RLController", "运行失败：模型或连接尚未初始化");
        return;
    }

    LOGI("RLController", "控制器开始运行轨迹");
    runTrajectory();
    is_active_ = false;
}




} // namespace SIAT
