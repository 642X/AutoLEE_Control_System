#include "RobotCommunication.h"
#include "HYYRobotInterface.h"
#include "Log/Logger.h"
#include "tools.h"
#include <thread>
#include <sstream>

namespace SIAT {

RobotCommunication::RobotCommunication() {
    LOGI("RobotComm", "构造函数调用");
}

RobotCommunication::~RobotCommunication() {
    LOGI("RobotComm", "析构函数调用，清理资源");
}

bool RobotCommunication::init(const std::string& ip, int port, const std::string& name, bool is_client) {
    ip_ = ip;
    port_ = port;
    name_ = name;
    is_client_ = is_client;
    return true;
}

void RobotCommunication::createConnection() {
    if (is_client_) {
        is_connected_ = initAsClient(ip_, port_, name_);
    } else {
        is_connected_ = initAsServer(ip_, port_, name_);
    }
}

bool RobotCommunication::isConnected() const { return is_connected_; } 

bool RobotCommunication::initAsClient(const std::string& ip, int port, const std::string& client_name) {
    int result = HYYRobotBase::ClientCreate(ip.c_str(), port, client_name.c_str());
    if (result == 0) {
        socket_name_ = client_name;
        LOGI("RobotComm", "Client 创建成功: " + client_name);
        return true;
    } else {
        LOGE("RobotComm", "Client 创建失败: " + client_name);
        return false;
    }
}

bool RobotCommunication::initAsServer(const std::string& ip, int port, const std::string& server_name) {
    int result = HYYRobotBase::SocketCreate(ip.c_str(), port, server_name.c_str());
    if (result == 0) {
        socket_name_ = server_name;
        LOGI("RobotComm", "Server 创建成功: " + server_name);
        return true;
    } else {
        LOGE("RobotComm", "Server 创建失败: " + server_name);
        return false;
    }
}

void RobotCommunication::receiveAndParse() {
    char buffer[1024] = {0};
    int result = HYYRobotBase::SocketRecvString(buffer, socket_name_.c_str());
    if (result > 0) {
        std::string raw(buffer);
        parseReceived(raw);
    }
}

void RobotCommunication::parseReceived(const std::string& raw) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (raw.rfind("CMD:", 0) == 0) {
        std::string cmd_str = raw.substr(4);
        auto cmd_opt = ParseCommand(cmd_str);
        if (cmd_opt.has_value()) {
            command_queue_.push(cmd_opt.value());
            LOGI("RobotComm", "接收到命令: " + CommandToString(cmd_opt.value()));
        } else {
            LOGW("RobotComm", "无法识别的命令: " + cmd_str);
        }
    }
    else if (raw.rfind("TRAJ:", 0) == 0) {
        std::vector<std::string> traj;
        std::stringstream ss(raw.substr(5));
        std::string token;
        while (std::getline(ss, token, ',')) {
            traj.push_back(token);
        }
        trajectory_queue_.push(traj);
        LOGI("RobotComm", "接收到轨迹，共 " + std::to_string(traj.size()) + " 点");
    }
    else {
        LOGW("RobotComm", "未知数据类型: " + raw);
    }
}


bool RobotCommunication::hasCommand() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return !command_queue_.empty();
}

CommandType RobotCommunication::popCommand() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (command_queue_.empty()) return CommandType::kUnknown;

    // 获取最新命令
    CommandType latest = command_queue_.back();

    // 清空队列
    std::queue<CommandType> empty;
    std::swap(command_queue_, empty);

    return latest;
}



bool RobotCommunication::hasTrajectory() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return !trajectory_queue_.empty();
}

std::vector<std::string> RobotCommunication::popTrajectory() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (trajectory_queue_.empty()) return {};

    // 获取最新一条轨迹
    std::vector<std::string> latest = std::move(trajectory_queue_.back());

    // 清空队列
    std::queue<std::vector<std::string>> empty;
    std::swap(trajectory_queue_, empty);

    return latest;
}


int RobotCommunication::closeConnection(const std::string& name) {
    int result = HYYRobotBase::SocketClose(name.c_str());
    if (result == 0) {
        LOGI("RobotComm", "连接关闭成功: " + name);
    } else {
        LOGE("RobotComm", "关闭连接失败: " + name);
    }
    return result;
}

}  // namespace SIAT
