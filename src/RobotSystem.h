#ifndef ROBOTSYSTEM_H
#define ROBOTSYSTEM_H

#include <thread>
#include "RobotCommunication/RobotCommunication.h"
#include "RobotController/RLRobotController.h"
#include "RobotController/RLZMPRobotController.h"


namespace SIAT {

class RobotSystem {
public:
    RobotSystem();
    ~RobotSystem();

    // 初始化系统（包括通信和控制系统的初始化）
    bool initializeSystem(const std::string& ip, int port);

    // 启动系统（包括通信和控制线程）
    void startSystem();

    // 停止系统（停止线程）
    void stopSystem();
    
    // 返回系统是否正在运行
    bool isRunning() const;  

private:

    // 处理通信线程的函数
    void runCommunicationThread();

    // 处理控制线程的函数
    void runControllerThread();    
    
    // 通信和控制类实例
    RobotCommunication comm_;
    std::string sever_name_;
    // RLRobotController controller_;
    RLZMPRobotController controller_;
    // 用于运行通信和控制的线程
    std::thread comm_thread_;
    std::thread controller_thread_;

    // 系统运行标志
    bool is_running_;

};

} // namespace SIAT

#endif // ROBOTSYSTEM_H
