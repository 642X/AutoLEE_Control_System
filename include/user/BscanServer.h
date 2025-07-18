#ifndef BSCANSERVER_H_
#define BSCANSERVER_H_

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <vector>
#include <thread>
#include <queue>
#include <iostream>
#include <mutex>
#include "HYYRobotInterface.h"
namespace bscan_server
{

enum{
    _start=1,
    _stop=2,
    _pause=3,
    _continue=4
};
#define MAXFD 5
class BscanServer
{
public:
	BscanServer(){input_feedback=0;server_state_=0;feedbacd_fd_=-1;}
	~BscanServer() {}
    void StartBscanServer(int cycle_times,HYYRobotBase::tool* tool,HYYRobotBase::wobj* wobj);
private:
    std::thread thread_;
    std::vector<std::vector<double>> server_data_;
    std::recursive_mutex data_lock_; 
    HYYRobotBase::tool tool_;
    HYYRobotBase::wobj wobj_;
    uint8_t server_state_;
    int feedbacd_fd_;
    int input_feedback;
    int cycle_times_;
    int cmd_server(int fd);
    void feedback_server(void);
    void state_server();
    void stop_server();
    void pause_server();
    void continue_server();
    void rpy2tr(double* rpy, double R[3][3], int flag);
    void tr2rpy(double R[3][3], double* rpy, int flag);
    int tcp_concurrent_server(const char* ip, int port);
};




}

#endif /* BSCANSERVER_H_ */