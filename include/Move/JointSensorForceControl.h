/**
 * @file JointSensorForceControl.h
 * 
 * @brief  基于关节力矩传感器的力控制
 * @author HanBing
 * @version 11.0.0
 * @date 2022-05-19
 * 
 */

#ifndef JOINTSENSORFORCECONTROL_H_
#define JOINTSENSORFORCECONTROL_H_
#include "Base/robotStruct.h"

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
* @brief 创建传感器力控制数据
*
* @param name 指定创建名称
* @param control_type 控制方法
* @param rtool 工具
* @param rwobj 工件
* @param robot_index 机器人索引
* @return int 成功返回0，错误返回其他
*/
int JSFCInit(const char* name, int control_type, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 创建传感器力控制数据
*
* @param name 创建时指定的名称
* @param Ia 关节惯性
* @param B 阻尼系数
* @param K 刚度系数
* @return int 成功返回0，错误返回其他
*/
int JSFCSetJointImpedanceCtrlParam(const char* name, double* Ia, double* B, double* K);

/**
* @brief 设置力控限制
*
* @param name 创建时指定的名称
* @param protect_limit 限制力矩/力矩
* @return int 成功返回0，错误返回其他
*/
int JSFCSetSensorForceLimit(const char* name, double protect_limit);

/**
* @brief 启动传感器力控制
*
* @param name 创建时指定的名称
* @return int 成功返回0，失败返回其他
*/
int JSFCStart(const char* name);

/**
* @brief 停止传感器力控制
*
* @param name 创建时指定的名称
* @return int 成功返回0，失败返回其他
*/
int JSFCEnd(const char* name);

#ifdef __cplusplus
}
}
#endif

#endif /* JOINTSENSORFORCECONTROL_H_ */