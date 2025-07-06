/**
 * @file LimitDetection.h
 *
 * @brief  约束检测
 * @author hanbing
 * @version 11.4.1
 * @date 2022-10-2
 *
 */
#ifndef LIMITDETECTION_H_
#define LIMITDETECTION_H_

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 检测机器人关节位置限制
 *
 * @param serialLinkName 机器人名字
 * @param position 关节位置
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotPositionLimit(const char* serialLinkName, double* position);

/**
 * @brief 检测机器人关节位置限制
 *
 * @param serialLinkName 机器人名字
 * @param position 关节位置
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotPositionLimitMargin(const char* serialLinkName, double* position, double margin);

/**
 * @brief 检测机器人关节速度限制
 *
 * @param serialLinkName 机器人名字
 * @param velocity 关节速度
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotVelocityLimit(const char* serialLinkName, double* velocity);

/**
 * @brief 检测机器人关节速度限制
 *
 * @param serialLinkName 机器人名字
 * @param velocity 关节速度
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotVelocityLimitMargin(const char* serialLinkName, double* velocity,double margin);

/**
 * @brief 检测机器人关节加速度限制
 *
 * @param serialLinkName 机器人名字
 * @param acceleration 关节加速度
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotAccelerationLimit(const char* serialLinkName, double* acceleration);

/**
 * @brief 检测机器人关节加速度限制
 *
 * @param serialLinkName 机器人名字
 * @param acceleration 关节加速度
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotAccelerationLimitMargin(const char* serialLinkName, double* acceleration,double margin);

/**
 * @brief 检测机器人关节力矩限制
 *
 * @param serialLinkName 机器人名字
 * @param torque 关节力矩
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotTorqueLimit(const char* serialLinkName, double* torque);

/**
 * @brief 检测机器人关节力矩限制
 *
 * @param serialLinkName 机器人名字
 * @param torque 关节力矩
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsRobotTorqueLimitMargin(const char* serialLinkName, double* torque,double margin);

/**
 * @brief 检测附加轴组关节位置限制
 *
 * @param addLinkName 附加轴组名字
 * @param position 关节位置
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddPositionLimit(const char* addLinkName, double* position);

/**
 * @brief 检测附加轴组关节位置限制
 *
 * @param addLinkName 附加轴组名字
 * @param position 关节位置
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddPositionLimitMargin(const char* addLinkName, double* position, double margin);

/**
 * @brief 检测附加轴组关节速度限制
 *
 * @param addLinkName 附加轴组名字
 * @param velocity 关节速度
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddVelocityLimit(const char* addLinkName, double* velocity);

/**
 * @brief 检测附加轴组关节速度限制
 *
 * @param addLinkName 附加轴组名字
 * @param velocity 关节速度
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddVelocityLimitMargin(const char* addLinkName, double* velocity,double margin);

/**
 * @brief 检测附加轴组关节加速度限制
 *
 * @param addLinkName 附加轴组名字
 * @param acceleration 关节加速度
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddAccelerationLimit(const char* addLinkName, double* acceleration);

/**
 * @brief 检测附加轴组关节加速度限制
 *
 * @param addLinkName 附加轴组名字
 * @param acceleration 关节加速度
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddAccelerationLimitMargin(const char* addLinkName, double* acceleration,double margin);

/**
 * @brief 检测附加轴组关节力矩限制
 *
 * @param addLinkName 附加轴组名字
 * @param torque 关节力矩
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddTorqueLimit(const char* addLinkName, double* torque);

/**
 * @brief 检测附加轴组关节力矩限制
 *
 * @param addLinkName 附加轴组名字
 * @param torque 关节力矩
 * @param margin 边界放缩系数
 * @return int 0:未超限; >0:轴超限，值为超限轴ID(1,2,……)
 */
extern int IsAddTorqueLimitMargin(const char* addLinkName, double* torque,double margin);

#ifdef __cplusplus
}
}
#endif

#endif /*LIMITDETECTION_H_*/