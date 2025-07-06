/**
 * @file egm_interface.h
 *
 * @brief  机器人控制系统外部设备引导接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-08
 *
 */

#ifndef EGM_INTERFACE_H_
#define EGM_INTERFACE_H_
#include "Base/robotStruct.h"


#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 创建外部修正数据区(用于控制器内使用)
 *
 * @param input_type 1:启用外部关节输入 2:启用外部工件坐标系笛卡尔输入 3:启用外部工具坐标系笛卡尔输入
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMCreate(int input_type,tool* rtool, wobj* rwobj,int robot_index);

/**
 * @brief 开始修正(用于控制器内使用)
 *
 * @param guide_type 修正方式,0:相对; 1:绝对
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMStart(int guide_type, int robot_index);

/**
 * @brief 启用外部引导(用于控制器内使用),将禁止执行运动指令
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMGuideMove(int robot_index);

/**
 * @brief 停止修正(用于控制器内使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMStop(int robot_index);

/**
 * @brief 释放外部修正数据区(用于控制器内使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMDelete(int robot_index);

/**
 * @brief 设置修正数据(用于控制器内使用),需要按照总线周期进行调用
 *
 * @param position 修正数据
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int SetEGMInput(double* position, int robot_index);

/**
 * @brief 获取引导基准(用于控制器内使用),调用EGMGuideMove()时机器人所处的关节位置或笛卡尔位置
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern double* GetEGMGuideBase(int robot_index);

/**
 * @brief 创建外部修正数据区(用于控制器外通讯使用)
 *
 * @param input_type 1:启用外部关节输入 2:启用外部工件坐标系笛卡尔输入 3:启用外部工具坐标系笛卡尔输入
 * @param cycle_times 修正周期倍数（为总线周期的倍数）
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmCreate(int input_type,int cycle_times,tool* rtool, wobj* rwobj,int robot_index);

/**
 * @brief 设置修正数据限制(用于控制器外通讯使用)
 *
 * @param velocity_max_per 最大速度百分比(0~1)
 * @param acceleration_max_per 最大加速度百分比(0~1)
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmSetLimit(double velocity_max_per, double acceleration_max_per,int robot_index);

/**
 * @brief 设置使用的传感器名称(用于控制器外通讯使用)
 *
 * @param sensor_name 传感器名称
 * @param is_open_sensor 是否创建传感器，如果传感器已经创建直接设置为0，否则设置为1
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmSetForceSensor(const char* sensor_name, int is_open_sensor,int robot_index);

/**
 * @brief 开始修正(用于控制器外通讯使用)
 *
 * @param ip 引导程序udp服务器ip
 * @param port 引导程序udp服务器端口号
 * @param guide_type 修正方式,0:相对; 1:绝对
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmStart(const char* ip, int port,int guide_type, int robot_index);

/**
 * @brief 启用外部引导(用于控制器外通讯使用),将禁止执行运动指令
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmGuideMove(int robot_index);

/**
 * @brief 停止修正(用于控制器外通讯使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmStop(int robot_index);

/**
 * @brief 释放外部修正数据区(用于控制器外通讯使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmDelete(int robot_index);






/**
 * @brief 创建外部引导数据区
 *
 * @param name 数据区名字
 * @param robot_index 机器人索引
 * @param guide_type 引导方式，0:关节空间引导；1:笛卡尔空间引导
 * @param input_type 输入数据类型，0:关节输入；1:笛卡尔输入
 * @param cycle_times 引导周期相对基础控制周期的倍数
 * @param tool 使用的工具
 * @param wobj 使用的工件
 * @return 0:创建成功；否则创建失败
 *
  注：引导通信方式为UDP,  egm服务ip:192.168.0.99, port:6680+robot_index
        引导数据格式：数据之间用逗号分割,如：“xyzrpy[0],xyzrpy[1],xyzrpy[2],xyzrpy[3],xyzrpy[4],xyzrpy[5]"or “joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]"
                                   UDP 发送”100,100,100,100,100,100“启动引导，UDP 发送”101,101,101,101,101,101“关闭引导
        反馈数据格式：数据之间用逗号分割,如：“xyzrpy[0],xyzrpy[1],xyzrpy[2],xyzrpy[3],xyzrpy[4],xyzrpy[5],sensor_force[0],sensor_force[1],sensor_force[2],sensor_force[3],sensor_force[4],sensor_force[5]”


        引导按键通信方式为：UDP, 服务ip:192.168.0.99, port:6690+robot_index
        按键数据格式：”按键索引,按键状态“，如：”0,1“
 */
extern int EGMCreate_c(const char* name, int robot_index, int guide_type, int input_type, int cycle_times,  tool* tool, wobj* wobj);

/**
 * @brief 释放外部引导数据区
 *
 * @param name 数据区名字
  */
extern void EGMRelease_c(const char* name);

/**
 * @brief 启动关节空间引导
 *
 * @param name 数据区名字
 * @param block 是否阻塞运行，0:非阻塞运行，1:阻塞运行，-1:开线程运行
 * @return 0:停止引导（非阻塞或阻塞）；1:非阻塞方式返回，其他：执行错误
 */
extern int EGMRunJoint_c(const char* name, int block);


/**
 * @brief 创建外部引导数据区
 *
 * @param robot_index 机器人索引
 * @param guide_type 引导方式，0:关节空间引导；1:笛卡尔空间引导
 * @param input_type 输入数据类型，0:关节输入；1:笛卡尔输入
 * @param cycle_times 引导周期相对基础控制周期的倍数
 * @param tool 使用的工具名称
 * @param wobj 使用的工件名称
 * @return 0:创建成功；否则创建失败
 *
  注：引导通信方式为UDP,  egm服务ip:192.168.0.99, port:6680+robot_index
        引导数据格式：数据之间用逗号分割,如：“xyzrpy[0],xyzrpy[1],xyzrpy[2],xyzrpy[3],xyzrpy[4],xyzrpy[5]"or “joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]"
                                   UDP 发送”100,100,100,100,100,100“启动引导，UDP 发送”101,101,101,101,101,101“关闭引导
        反馈数据格式：数据之间用逗号分割,如：“xyzrpy[0],xyzrpy[1],xyzrpy[2],xyzrpy[3],xyzrpy[4],xyzrpy[5],sensor_force[0],sensor_force[1],sensor_force[2],sensor_force[3],sensor_force[4],sensor_force[5]”


        引导按键通信方式为：UDP, 服务ip:192.168.0.99, port:6690+robot_index
        按键数据格式：”按键索引,按键状态“，如：”0,1“
 */
extern int EGMCreateForMATLAB(int robot_index, int guide_type, int input_type, int cycle_times, const char* tool, const char* wobj);

/**
 * @brief 获取外部设备的引导数据
 *
 * @param robot_index 机器人索引
 * @param data 返回外部设备的引导数据
 */
extern void EGMInputForMATLAB(int robot_index, double* data);

/**
 * @brief 获取外部设备的引导关节数据
 *
 * @param robot_index 机器人索引
 * @param data 返回外部设备的引导关节数据
 */
extern void EGMInputJointForMATLAB(int robot_index, double* data);

/**
 * @brief 为外部设备反馈机器人位姿和接触力
 *
 * @param robot_index 机器人索引
 * @param xyzrpy 设置机器人位姿
 * @param sensor_force 设置机器人接触力
 * @return 0:成功；其他：失败
 */
extern int EGMOutputForMATLAB(int robot_index, double* xyzrpy, double* sensor_force);

/**
 * @brief 获取外部设备的开关状态
 *
 * @param robot_index 机器人索引
 * @param key_index 外部设备开关索引
 * @return 开关状态
 */
extern int EGMInputKeyForMATLAB(int robot_index, int key_index);

/**
 * @brief 释放外部引导数据区
 *
 * @param robot_index 机器人索引
  */
extern void EGMReleaseForMATLAB(int robot_index);

#ifdef __cplusplus
}
}
#endif


#endif /* EGM_INTERFACE_H_ */
