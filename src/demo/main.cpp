#include "RobotSystem.h"
#include "HYYRobotInterface.h"
#include <iostream>

int main(int argc, char** argv) {
    // HYYRobotBase::command_arg arg;
    // if (HYYRobotBase::commandLineParser(argc, argv, &arg) != 0) {
    //     return -1;
    // }

    // if (HYYRobotBase::system_initialize(&arg) != 0) {
    //     return -1;
    // }

    SIAT::RobotSystem robotSystem;

    // 初始化系统，连接到 IP 和端口
    if (!robotSystem.initializeSystem("127.0.0.1", 8888)) {
        std::cerr << "Failed to initialize system." << std::endl;
        return -1;
    }

    // 启动系统
    robotSystem.startSystem();

    // 等待直到系统运行
    while (robotSystem.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 主线程每100ms检查一次
    }

    // 停止系统
    robotSystem.stopSystem();

    return 0;
}
