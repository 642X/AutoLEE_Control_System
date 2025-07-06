#include "src/RobotSystem.h"
#include <vector>
#include <iostream>

int main() {
    SIAT::RobotSystem robotSystem;

    // 初始化系统，连接到 IP 和端口
    if (!robotSystem.initializeSystem("192.168.1.2", 8888)) {
        std::cerr << "Failed to initialize system." << std::endl;
        return -1;
    }

    // 启动系统
    robotSystem.startSystem();

    // 启动机器人
    robotSystem.startRobot();

    // 发送轨迹数据
    std::vector<std::string> trajectory = {"point1", "point2", "point3"};
    robotSystem.sendTrajectoryData(trajectory);

    // 停止机器人
    robotSystem.stopRobot();

    // 停止系统
    robotSystem.stopSystem();

    return 0;
}
