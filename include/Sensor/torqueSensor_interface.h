/**
 * @file torqueSensor_interface.h
 *
 * @brief  六维力传感器接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-9
 *
 */
#ifndef TORQUESENSOR_INTERFACE_H_
#define TORQUESENSOR_INTERFACE_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/robotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 创建六维力传感器数据
 *
 * @param torqueSensorName 创建时使用的名字
 * @return int 0:成功，其他：失败
 */
extern int CreateTorqueSensor(const char* torqueSensorName);

/**
 * @brief 创建六维力传感器数据
 *
 * @param torqueSensorName 创建时使用的名字
 * @param reName 重命名
 * @return int 0:成功，其他：失败
 */
extern int CreateTorqueSensor2(const char* torqueSensorName, const char* reName);

/**
 * @brief 释放六维力传感器数据
 *
 * @param torqueSensorName 创建时使用的名字
 * @return int 0:成功，其他：失败
 */
extern int DestroyTorqueSensor(const char* torqueSensorName);

/**
 * @brief 获取六维力传感器数目
 *
 * @return int 传感器数目
 */
extern int GetTorqueSensorNum();

/**
 * @brief 获取六维力传感器名字
 *
 * @param index 传感器索引
 * @param torqueSensorName 保存传感器名字数据空间，可以为空(NULL)
 * @return char 传感器名字数据区首地址
 */
extern const char* GetTorqueSensorName(int index,char* torqueSensorName);

/**
 * @brief 获取六维力传感器力数据
 *
 * @param torqueSensorName 创建时使用的名字
 * @param tor 返回传感器数据(单位：N, Nm)
 * @return int 0:成功，其他：失败
 */
extern int GetSensorTorque(const char* torqueSensorName, double* tor);

/**
 * @brief 获取转换到工件下的六维力传感器力数据
 *
 * @param torqueSensorName 创建时使用的名字
 * @param angle 机器人当前关节位置（rad）
 * @param torque 返回工具或工件坐标系下作用力(单位：N, Nm)
 * @return int 0:成功，其他：失败
 */
extern int GetSensorTorqueTransformToTool(const char* torqueSensorName, double* angle, double* torque);

/**
 * @brief 获取六维力传感器标定
 *
 * @param torqueSensorName 创建时使用的名字
 * @return int 0:成功，其他：失败
 */
extern int TorqueSensorCalibration(const char* torqueSensorName);

/**
 * @brief 开启六维力传感器数据清零
 *
 * @param torqueSensorName 创建时使用的名字
 * @return int 0:成功，其他：失败
 */
extern int TorqueSensorOpenBias(const char* torqueSensorName);

/**
 * @brief 关闭六维力传感器数据清零
 *
 * @param torqueSensorName 创建时使用的名字
 * @return int 0:成功，其他：失败
 */
extern int TorqueSensorCloseBias(const char* torqueSensorName);

/**
 * @brief 设置传感器转换数据(SensorTorqueTransform)
 *
 * @param torqueSensorName 创建时使用的名字
 * @param serialLinkName 机器人名字
 * @param rtool 工具
 * @param rwobj 工件
 * @return int 0:成功，其他：失败
 */
extern int SetSensorTorqueTransformData(const char* torqueSensorName, const char* serialLinkName, tool* rtool, wobj* rwobj);


/**
 * @brief 获取工具或（工件）坐标系下作用力
 *
 * @param torqueSensorName 传感器名称
 * @param angle 机器人当前关节位置（rad）
 * @param tor 输入传感器力矩数据，返回工具或工件坐标系下作用力(单位：N, Nm)
 * @return int 0:成功，其他：失败
 */
extern int SensorTorqueTransformToTool(const char* torqueSensorName, double* angle, double* tor);

/**
 * @brief 获取法兰坐标系下作用力(作用力产生于工具上)
 *
 * @param torqueSensorName 传感器名称
 * @param angle 机器人当前关节位置（rad）
 * @param tor 输入传感器力矩数据，返回法兰坐标系下作用力(单位：N, Nm)
 *
 * @return int 0:成功，其他：失败
 */
extern int SensorTorqueTransformToFlange(const char* torqueSensorName, double* angle, double* tor);

/**
 * @brief 根据工具或（工件）坐标系下作用力，获取传感器数据
 *
 * @param torqueSensorName 传感器名称
 * @param angle 机器人当前关节位置（rad）
 * @param tor 输入工具或工件坐标系下的作用力，返回传感器数据(含重力)(单位：N, Nm)
 *
 * @return int 0:成功，其他：失败
 */
extern int ToolTransformToSensorTorque(const char* torqueSensorName, double* angle, double* tor);

/**
 * @brief 获取仿真环境中六维力传感器力数据
 *
 * @param torqueSensorName 传感器名称
 * @param angle 机器人当前关节位置（rad）
 * @param env_pose 三维位置和旋转空间环境约束的上下限制
 * @param env_stiffness 环境刚度系数,分别为x,y,z和旋转的刚度系数
 * @param tor 返回传感器检测力(单位：N, Nm)s
 *
 * @return int 0:成功，其他：失败
 */
extern int GetSensorTorque_sim(const char* torqueSensorName, double* angle,  double env_pose[4][2], double env_stiffness[4], double* tor);



#ifdef __cplusplus
}
}
#endif


#endif /* TORQUESENSOR_INTERFACE_H_ */
