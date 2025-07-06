#include "RobotSystem.h"
#include "Logger.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace SIAT {

RobotSystem::RobotSystem() : is_running_(false) {
    LOGI("RobotSystem", "构造函数调用");
}

RobotSystem::~RobotSystem() {
    stopSystem();
    LOGI("RobotSystem", "析构函数调用");
}

bool RobotSystem::initializeSystem(const std::string& ip, int port) {
    // //------------------------initialize----------------------------------
    // HYYRobotBase::command_arg arg;
    // int err = HYYRobotBase::commandLineParser(0, nullptr, &arg);
    // if (err != 0) {
    //     LOGE("RobotSystem", "命令行参数解析失败");
    //     return false;
    // }
	// err=HYYRobotBase::system_initialize(&arg);
	// if (0!=err)
	// {
	// 	return err;
	// }
	//-----------------------user designation codes---------------
    sever_name_ = "Server1";
    if (!comm_.init(ip, port, sever_name_)) {
        LOGE("RobotSystem", "通信初始化失败");
        return false;
    }

    if (!controller_.initialize()) {
        LOGE("RobotSystem", "控制器初始化失败");
        return false;
    }

    LOGI("RobotSystem", "系统初始化成功");
    return true;
}

void RobotSystem::startSystem() {
    if (is_running_) {
        LOGW("RobotSystem", "系统已在运行状态");
        return;
    }

    is_running_ = true;
    comm_thread_ = std::thread(&RobotSystem::runCommunicationThread, this);
    controller_thread_ = std::thread(&RobotSystem::runControllerThread, this);

    LOGI("RobotSystem", "系统已启动");
}

void RobotSystem::stopSystem() {
    if (!is_running_) return;

    is_running_ = false;

    if (comm_thread_.joinable()) comm_thread_.join();
    if (controller_thread_.joinable()) controller_thread_.join();

    comm_.closeConnection(sever_name_);
    controller_.shutdown();

    LOGI("RobotSystem", "系统已停止");
}

bool RobotSystem::isRunning() const {
    return is_running_;
}

void RobotSystem::runCommunicationThread() {
    LOGI("RobotSystem", "通信线程启动");

    while (is_running_) {
        if (!comm_.isConnected()) {
            LOGE("RobotSystem", "通信未连接，尝试重新连接");
            comm_.createConnection();  // 尝试重新建立连接
            if (!comm_.isConnected()) {
                LOGE("RobotSystem", "通信连接失败，等待重试");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;  // 继续下一次循环
            }else{
                LOGI("RobotSystem", "通信连接成功，等待命令或轨迹数据");
            }
        }else{
            comm_.receiveAndParse();  // 接收并解析命令或轨迹
            if (comm_.hasCommand()) {
                CommandType cmd = comm_.popCommand();
                controller_.executeCommand(cmd);// 所有命令都交由控制器处理
                switch (cmd) {
                    case CommandType::kStop:
                        LOGI("RobotSystem", "收到 stop 命令，系统将停止");
                        stopSystem();  // 主动关闭系统
                        return;         // 直接退出线程

                    case CommandType::kStart:
                        LOGI("RobotSystem", "收到 start 命令，控制器启动");
                        break;

                    case CommandType::kPause:
                    case CommandType::kReset:
                        
                        break;

                    default:
                        LOGW("RobotSystem", "收到未知命令，忽略");
                        break;
                }
            }
            if (comm_.hasTrajectory()) {
                std::vector<std::string> traj = comm_.popTrajectory();
                controller_.setTrajectory(traj);  // 设置轨迹数据
            }            
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 控制频率约50Hz
    }

    LOGI("RobotSystem", "通信线程结束");
}

void RobotSystem::runControllerThread() {
    LOGI("RobotSystem", "控制线程启动");

    while (is_running_) {
        controller_.run();
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

    LOGI("RobotSystem", "控制线程结束");
}



} // namespace SIAT
