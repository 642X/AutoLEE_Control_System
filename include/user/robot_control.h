#ifndef ROBOT_CONTORL_H_
#define ROBOT_CONTORL_H_

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <vector>
#include <thread>
#include <queue>
#include <iostream>
#include <mutex>
#include "HYYRobotInterface.h"
namespace piano_server
{

#define LEFT_ARM_INDEX 1
#define RIGHT_ARM_INDEX 0
#define HEAD_INDEX 0
#define WAIST_INDEX 1
#define FOOT_INDEX 2

enum{
    _arm_left_l,
    _arm_right_l,
    _arm_both_l,
    _arm_left_j,
    _arm_right_j,
    _arm_both_j,
    _foot_j,
    _head_j,
    _waist_j,
    _robot_l,
    _robot_j
};

enum{
    _start=1,
    _stop=2,
    _pause=3,
    _continue=4
};

typedef struct robot_server_data{
    uint8_t type;
    uint64_t time_ns;
    double left_cartesian[10][6];
    uint32_t left_cartesian_num;
    double right_cartesian[10][6];
    uint32_t right_cartesian_num;
    double left_joint[7];
    double right_joint[7];
    double head_joint[2];
    double waist_joint[2];
    double foot_joint[1];
    double run_time;
}robot_server_data;

class MessageServer
{
public:
	MessageServer(){}
	~MessageServer() {}
    bool StartMessageServer(double feedback_rate,HYYRobotBase::tool* tool_left,HYYRobotBase::tool* tool_right,HYYRobotBase::wobj* wobj_left,HYYRobotBase::wobj* wobj_right);
    void CloseMessageServer();
    void SetMessageData(robot_server_data &data);
    bool GetMessageData(std::vector<robot_server_data> &data);
    bool HasMessageData();
    bool ClearMessageData();
    uint8_t GetControlMode();
    void SetControlMode(uint8_t mode);
private:

    double feedback_rate_;
    std::thread feedbace_thread_;
    std::queue<robot_server_data> server_data_;
    std::recursive_mutex data_lock_; 
    uint8_t control_mode_;
    HYYRobotBase::tool tool_left_;
    HYYRobotBase::tool tool_right_;
    HYYRobotBase::wobj wobj_left_;
    HYYRobotBase::wobj wobj_right_;
    void feedbace_server();

};

class RobotControl
{
public:
    RobotControl(std::string config_path){config_path_=config_path;}
	RobotControl(){config_path_="/home/robot/Work/system/roobt_config";}
	~RobotControl() {}
    bool StartControl(double feedback_rate=1);

    MessageServer* GetMessageServer();

    void CloseControl();

    void LoopControl();

private:
    std::string config_path_;
    uint64_t time_;
    HYYRobotBase::RTimer timer_;
    HYYRobotBase::tool ltool;
    HYYRobotBase::wobj lwobj;
    HYYRobotBase::tool rtool;
    HYYRobotBase::wobj rwobj;
    uint64_t dt;//ns
    uint64_t robot_timer();
    void sync_time(uint64_t time);
    bool move(std::vector<robot_server_data> &data);
    bool is_run(std::vector<robot_server_data> &data);
    MessageServer message_server_;
};


}
#endif /* ROBOT_CONTORL_H_ */