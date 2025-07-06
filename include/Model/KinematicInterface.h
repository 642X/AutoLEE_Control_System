/**
 * @file KinematicInterface.h
 *
 * @brief  机器人运动学接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-9
 *
 */
#ifndef KINEMATICINTERFACE_H_
#define KINEMATICINTERFACE_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/robotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif


/*-------------------------------------------------------------------------*/
/**
  @brief	运动学数据

	描述机器人笛卡尔位姿数据和关节数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	double		R[3][3];/**< 姿态矩阵*/
	double		X[3];/**< 位置*/
	double		joint[10];/**< 关节位置*/
	double		kps[3];/**< 姿态*/
	int       		dof;/**< 机器人自由度*/
	double      redundancy;/**< 机器人冗余角*/

	TOOL tool;/**< 工具描述*/
	WOBJ wobj;/**< 工件描述*/
}R7_KINE;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动速度数据

	描述机器人笛卡尔位姿数据和关节速度数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	double joint[ROBOT_MAX_DOF];/**< 关节位置*/
	double joint_vel[ROBOT_MAX_DOF];/**< 关节速度*/
	double pose_vel[ROBOT_MAX_DOF];/**< 笛卡尔速度*/
	int dof;/**< 机器人自由度*/
	TOOL tool;/**< 工具描述*/
	WOBJ wobj;/**< 工件描述*/
}R_KINE_VEL;

/**
 * @brief 初始化运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 */
extern void init_R7_KINE(R7_KINE* rkine);

/**
 * @brief 初始化运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param angle 机器人关节,单位: rad
 * @param dof 机器人自由度
 */
extern void init_R7_KINE1(R7_KINE* rkine, double* angle, int* dof);

/**
 * @brief 初始化运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param angle 机器人关节,单位: rad
 * @param dof 机器人自由度
 * @param tool 工具描述
 * @param wobj 工件描述
 */
extern void init_R7_KINE2(R7_KINE* rkine, double* angle, int* dof, TOOL* tool, WOBJ* wobj);

/**
 * @brief 设置运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param joint 机器人关节,单位：rad
 */
extern void set_R7_KINE_joint(R7_KINE* rkine, double* joint);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param X 位置，单位：mm
 * @param RPY 姿态，单位：rad
 */
extern void set_R7_KINE_pose(R7_KINE* rkine, double* X, double* RPY);

/**
 * @brief 获取运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param joint 返回机器人关节,单位：rad
 */
extern void get_R7_KINE_joint(R7_KINE* rkine, double* joint);

/**
 * @brief 获取运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param X 返回位置，单位：mm
 * @param RPY 返回姿态，单位：rad
 */
extern void get_R7_KINE_pose(R7_KINE* rkine, double* X, double* RPY);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param tool 工具描述
 * @param wobj 工件描述
 */
extern void setToolWobjToR7_KINE(R7_KINE* rkine, TOOL* tool, WOBJ* wobj);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param tool 工具描述
 */
extern void setToolToR7_KINE(R7_KINE* rkine, TOOL* tool);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param wobj 工件描述
 */
extern void setWobjToR7_KINE(R7_KINE* rkine, WOBJ* wobj);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param dof 机器人自由度
 */
extern void init_R_KINE_VEL(R_KINE_VEL* rkine_vel, int dof);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param joint_vel 关节运动速度，单位：rad/s
 * @param dof 机器人自由度
 */
extern void init_R_KINE_VEL1(R_KINE_VEL* rkine_vel, double* joint,double* joint_vel, int dof);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param pose_vel 笛卡尔位姿速度，单位：mm/s，rad/s
 * @param dof 机器人自由度
 */
extern void init_R_KINE_VEL2(R_KINE_VEL* rkine_vel, double* joint,double* pose_vel, int dof);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param joint_vel 关节运动速度，单位：rad/s
 * @param pose_vel 笛卡尔位姿速度，单位：mm/s，rad/s
 * @param dof 机器人自由度
 * @param tool 工具描述
 * @param wobj 工具描述
 */
extern void init_R_KINE_VEL3(R_KINE_VEL* rkine_vel, double* joint, double* joint_vel, double* pose_vel, int dof, TOOL* tool, WOBJ* wobj);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param joint_vel 关节运动速度，单位：rad/s
 */
extern void set_R_KINE_VEL_joint(R_KINE_VEL* rkine_vel, double* joint, double* joint_vel);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param pose_vel 笛卡尔位姿速度，单位：mm/s，rad/s
 */
extern void set_R7_KINE_VEL_pose(R_KINE_VEL* rkine_vel, double* joint, double* pose_vel);

/**
 * @brief 获取运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint_vel 关节运动速度，单位：rad/s
 */
extern void get_R_KINE_VEL_joint(R_KINE_VEL* rkine_vel, double* joint_vel);

/**
 * @brief 获取运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param pose_vel 笛卡尔位姿速度，单位：mm/s，rad/s
 */
extern void get_R7_KINE_VEL_pose(R_KINE_VEL* rkine_vel, double* pose_vel);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param tool 工具描述
 * @param wobj 工件描述
 */
extern void setToolWobjToR_KINE_VEL(R_KINE_VEL* rkine_vel, TOOL* tool, WOBJ* wobj);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param tool 工具描述
 */
extern void setToolToR_KINE_VEL(R_KINE_VEL* rkine_vel, TOOL* tool);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param wobj 工件描述
 */
extern void setWobjToR_KINE_VEL(R_KINE_VEL* rkine_vel, WOBJ* wobj);

/**
 * @brief 正运动学
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param r7kine 运动学数据
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_Forward(const char* serialLinkName, R7_KINE* r7kine);

/**
 * @brief 逆运动学
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param r7kine 运动学数据
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_Inverse(const char* serialLinkName, R7_KINE* r7kine);

/**
 * @brief 给定笛卡尔速度求关节速度
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rkine_vel 运动速度数据
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_EndToJointVelocity(const char* serialLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 给定关节速度求笛卡尔速度
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rkine_vel 运动速度数据
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_JointToEndVelocity(const char* serialLinkName, R_KINE_VEL* rkine_vel);

#ifdef __cplusplus
}
}
#endif


#endif /* KINEMATICINTERFACE_H_ */
