#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <string>
#include <vector>
#include "CommonTypes.h"

namespace SIAT {

/**
 * @brief 抽象基类：定义通用的机器人控制器接口。
 *
 * 所有具体的机器人控制器（例如 RLZMPRobotController）应继承该类并实现其纯虚方法。
 * 提供了控制器初始化、轨迹设置、运行控制循环、命令响应等标准接口。
 */
class RobotController {
public:
    /// @brief 默认构造函数
    RobotController() = default;

    /// @brief 虚析构函数，确保资源释放时多态行为正确
    virtual ~RobotController() = default;

    /**
     * @brief 初始化控制器（如模型加载、硬件连接等）
     * @return 初始化是否成功
     */
    virtual bool initialize() = 0;

    /**
     * @brief 安全关闭控制器，释放资源并断开连接。
     */
    virtual void shutdown() = 0;

    /**
     * @brief 设置轨迹数据
     * @param traj 接收到的轨迹数据，通常由通信层传入
     */
    virtual void setTrajectory(const std::vector<std::string>& traj) = 0;

    /**
     * @brief 启动控制主循环（例如开始执行轨迹）
     *
     * 子类应实现该方法以持续处理当前轨迹，直到结束或收到外部命令打断。
     */
    virtual void run() = 0;

    /**
     * @brief 响应来自上层系统的命令，例如 start、stop、pause 等。
     * @param cmd 枚举类型的控制命令
     */
    virtual void executeCommand(CommandType cmd) = 0;

protected:
    /// 控制器是否处于激活状态（可执行控制逻辑）
    bool is_active_ = false;

    /// 控制器是否与下层机器人硬件连接成功
    bool is_connected_ = false;

    /// 当前控制器接收到的轨迹数据（原始字符串格式，待解析）
    std::vector<std::string> trajectory_data_;
};

} // namespace SIAT

#endif // ROBOTCONTROLLER_H
