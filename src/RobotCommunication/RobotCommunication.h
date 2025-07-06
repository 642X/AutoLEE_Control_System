#ifndef ROBOT_COMMUNICATION_H
#define ROBOT_COMMUNICATION_H

#include <string>
#include <queue>
#include <mutex>
#include <vector>
#include "CommonTypes.h" 

namespace SIAT {
class RobotCommunication {
public:
    RobotCommunication();
    ~RobotCommunication();

    bool init(const std::string& ip, int port, const std::string& name, bool is_client = false);

    void createConnection();  
    bool isConnected() const;
    
    void receiveAndParse();  // 主动调用接收 + 解析

    bool hasCommand() const;
    CommandType popCommand();

    bool hasTrajectory() const;
    std::vector<std::string> popTrajectory();

    int closeConnection(const std::string& name);

private:
    bool initAsClient(const std::string& ip, int port, const std::string& client_name);
    bool initAsServer(const std::string& ip, int port, const std::string& server_name);
    void parseReceived(const std::string& raw);

    std::string socket_name_;
    mutable std::mutex mtx_;
    std::queue<CommandType> command_queue_;
    std::queue<std::vector<std::string>> trajectory_queue_;

    // 保存连接参数
    std::string ip_;
    int port_;
    std::string name_;
    bool is_client_ = false;  // 是否为客户端
    bool is_connected_ = false;  // 是否已连接
};

}  // namespace SIAT

#endif  // ROBOT_COMMUNICATION_H
