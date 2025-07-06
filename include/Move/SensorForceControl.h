/**
 * @file SensorForceControl.h
 * 
 * @brief  基于末端六维力传感器的力控制
 * @author HanBing
 * @version 11.0.0
 * @date 2020-09-11
 * 
 */

#ifndef SENSORFORCECONTROL_H_
#define SENSORFORCECONTROL_H_

/*---------------------------- Includes ------------------------------------*/
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
 * @param control_type 力控制方式；0:导纳控制；1:力位置混合控制
 * @param is_tool_ctrl 力控制参考坐标系；0:工件坐标系；1:工具坐标系
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引
 * @return int 成功返回0，失败返回其他
 */
extern int SFCInit(const char* name, int control_type, int is_tool_ctrl, tool* rtool, wobj* rwobj, int robot_index);

/**
 * @brief 设置力位混合控制参数
 *
 * @param name 创建时指定的名称
 * @param M 惯性系数
 * @param B 阻尼系数
 * @param coeff 预留
 * @return int 成功返回0，失败返回其他
 */
extern int SFCSetHybridForceMotionCtrlParam(const char* name, double* M, double* B, double* coeff);

/**
 * @brief 设置控制目标力矩(导纳控制将刚度系数设置为0，将跟踪期望力矩)
 *
 * @param name 创建时指定的名称
 * @param target_force 作用在机器人上的目标力矩[fx,fy,fz,tx,ty,tz](N,Nm), 对于力位混合控制为0的方向上采用位置控制，非零方向采用力控制；参考坐标系由初始化决定，为工具标系或工件坐标系。
 * @return int 成功返回0，失败返回其他
 */
int SFCSetTargetForce(const char* name, double* target_force);

/**
 * @brief 设置导纳控制参数
 *
 * @param name 创建时指定的名称
 * @param M 惯性系数
 * @param B 阻尼系数
 * @param K 刚度系数
 * @return int 成功返回0，失败返回其他
 */
extern int SFCSetAdmittanceCtrlParam(const char* name, double* M, double* B, double* K);

/**
 * @brief 设置传感器接触力限制
 *
 * @param name 创建时指定的名称
 * @param max_force 传感器最大理解限制[fx,fy,fz,tx,ty,tz](N,Nm)
 * @return int 成功返回0，失败返回其他
 */
extern int SFCSetSensorForceLimit(const char* name, double* max_force);

/**
 * @brief 设置笛卡尔移动速度
 *
 * @param name 创建时指定的名称
 * @param tcp 平动速度限制(mm/s)
 * @param orl 转动速度限制(rad/s)
 * @return int 成功返回0，失败返回其他
 */
extern int SFCSetMoveSpeedLimit(const char* name, double tcp, double orl);

/**
 * @brief 设置传感器响应死区
 *
 * @param name 创建时指定的名称
 * @param force 传感器死区大小[fx,fy,fz,tx,ty,tz](N,Nm)
 * @return int 成功返回0，失败返回其他
 */
extern int SFCSetSensorDeadZone(const char* name, double* force);

/**
 * @brief 设置传感器名称
 *
 * @param name 创建时指定的名称
 * @param sensor_name 传感器名称
 * @return int 成功返回0，失败返回其他
 */
extern int SFCSetForceSensorName(const char* name, const char* sensor_name);

/**
 * @brief 启动传感器力控制（期望的运动目标由Move运动提供）
 *
 * @param name 创建时指定的名称
 * @return int 成功返回0，失败返回其他
 */
extern int SFCStart(const char* name);

/**
 * @brief 判读是否达到力和力矩条件
 *
 * @param name 创建时指定的名称
 * @param torque_condition 力和力矩条件(N, Nm)
 * @param is_valid 某个维度上的力或力矩是否作为判别条件，1：进行判别，0：忽略不判别
 * @param threshold_f 力阈值(N)
 * @param threshold_t 力矩阈值(Nm)
 * @return int 0：不满足条件，1：满足条件
 */
extern int SFCIsSatisfyForceCondition(const char* name, double* torque_condition,double* is_valid,double threshold_f,double threshold_t);

/**
 * @brief 判读是否达到位置和姿态矩条件
 *
 * @param name 创建时指定的名称
 * @param robpose_condition 位置和姿态条件(m,rad)
 * @param is_valid 某个维度上的位置和姿态是否作为判别条件，1：进行判别，0：忽略不判别
 * @param threshold_p 位置阈值(m)
 * @param threshold_r 姿态阈值（rad）
 * @return int 0：不满足条件，1：满足条件
 */
extern int SFCIsSatisfyCartesianCondition(const char* name, robpose* robpose_condition,double* is_valid,double threshold_p,double threshold_r);


/**
 * @brief 停止传感器力控制
 *
 * @param name 创建时指定的名称
 * @return int 成功返回0，失败返回其他
 */
extern int SFCEnd(const char* name);

#ifdef __cplusplus
}
}
#endif

#endif /* SENSORFORCECONTROL_H_ */
