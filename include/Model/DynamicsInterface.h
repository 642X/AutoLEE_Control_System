/**
 * @file DynamicsInterface.h
 *
 * @brief  机器人运动学接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-9
 *
 */
#ifndef DYNAMICSINTERFACE_H_
#define DYNAMICSINTERFACE_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/robotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif


/*-------------------------------------------------------------------------*/
/**
  @brief	惯性矩阵

	描述动力学中的惯性项
 */
/*-------------------------------------------------------------------------*/
typedef struct Dyn_inertia{
	double M[ROBOT_MAX_DOF][ROBOT_MAX_DOF];/**< 矩阵*/
	int order_num;/**< 矩阵维度*/
}Dyn_inertia;

/*-------------------------------------------------------------------------*/
/**
  @brief	科氏力及向心力

	描述动力学中的科氏力及向心力项
 */
/*-------------------------------------------------------------------------*/
typedef struct Dyn_coriolis{
	double C[ROBOT_MAX_DOF][ROBOT_MAX_DOF];/**< 矩阵*/
	int order_num;/**< 矩阵维度*/
}Dyn_coriolis;

/*-------------------------------------------------------------------------*/
/**
  @brief	重力

	描述动力学中的重力项
 */
/*-------------------------------------------------------------------------*/
typedef struct Dyn_gravity{
	double D[ROBOT_MAX_DOF];/**< 矩阵*/
	int order_num;/**< 矩阵维度*/
}Dyn_gravity;

/*-------------------------------------------------------------------------*/
/**
  @brief	摩擦力

	描述动力学中的摩擦力项
 */
/*-------------------------------------------------------------------------*/
typedef struct Dyn_friction{
	double Fcv[ROBOT_MAX_DOF];/**< 矩阵*/
	int order_num;/**< 矩阵维度*/
}Dyn_friction;

/*-------------------------------------------------------------------------*/
/**
  @brief	外部施加力

	描述动力学中的外力项
 */
/*-------------------------------------------------------------------------*/
typedef struct Dyn_externalTorque{
	double Fn[ROBOT_MAX_DOF];/**< 矩阵*/
	int order_num;/**< 矩阵维度*/
}Dyn_externalTorque;

/*-------------------------------------------------------------------------*/
/**
  @brief	动力学数据

	描述机器人动力学数据
 */
/*-------------------------------------------------------------------------*/
typedef struct {
	double q[ROBOT_MAX_DOF];/**< 关节位置(rad 或 mm)*/
	double qd[ROBOT_MAX_DOF];/**< 关节速度(rad/s 或 mm/s)*/
	double qdd[ROBOT_MAX_DOF];/**< 关节加速度(rad/s^2 或 mm/s^2)*/
	double fext[6];/**< 外部力*/
	int fcv_flag;/**< 是否使用摩擦力标识，0:不使用，1:使用*/
	int g_flag;/**< 是否使用重力标识，0:不使用，1:使用*/
	double torque[ROBOT_MAX_DOF];/**< 关节力矩(Nmm 或 N)*/
	double torque_1[ROBOT_MAX_DOF];/**< 前一时刻关节力矩(Nmm 或 N)*/
	int dof;/**< 机器人自由度*/
	PayLoad payload;/**< 负载*/


	//
	Dyn_inertia inertia;/**< 惯性项*/
	Dyn_coriolis coriolis;/**< 科氏力及向心力项*/
	Dyn_gravity gravity;/**< 重力项*/
	Dyn_friction friction;/**< 摩擦力项*/
	Dyn_externalTorque externalTorque;/**< 外力项*/

}R_DYNAMICS;

/**
 * @brief 初始化动力学数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 关节位置(rad 或 mm)
 * @param qd 关节速度(rad/s 或 mm/s)
 * @param qdd 关节加速度(rad/s^2 或 mm/s^2)
 * @param torque 关节力矩(Nmm 或 N)
 * @param fext 外部力
 * @param fcv_flag 是否使用摩擦力标识，0:不使用，1:使用
 * @param g_flag 是否使用重力标识，0:不使用，1:使用
 * @param _dof 机器人自由度
 * @param payload 负载
 */
extern void init_R_DYNAMICS(R_DYNAMICS* _rdyn, double* q, double* qd, double* qdd, double* torque, double* fext, int fcv_flag, int g_flag, int _dof, PayLoad* payload);

/**
 * @brief 初始化动力学数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 关节位置(rad 或 mm)
 * @param qd 关节速度(rad/s 或 mm/s)
 * @param qdd 关节加速度(rad/s^2 或 mm/s^2)
 * @param _dof 机器人自由度
 */
extern void init_R_DYNAMICS2(R_DYNAMICS* _rdyn, double* q, double* qd, double* qdd, int _dof);

/**
 * @brief 设置动力学负载数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param payload 负载
 */
extern void set_R_DYNAMICS_payload(R_DYNAMICS* _rdyn, PayLoad* payload);

/**
 * @brief 设置动力学负载数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param payloadname 负载名字
 */
extern void set_R_DYNAMICS_payload1(R_DYNAMICS* _rdyn, char* payloadname);

/**
 * @brief 设置动力学关节状态数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 关节位置(rad 或 mm)
 * @param qd 关节速度(rad/s 或 mm/s)
 * @param qdd 关节加速度(rad/s^2 或 mm/s^2)
 */
extern void set_joint_state_to_R_DYNAMICS(R_DYNAMICS* _rdyn, double* q, double* qd, double* qdd);

/**
 * @brief 获取动力学关节状态数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 返回关节位置(rad 或 mm)
 * @param qd 返回关节速度(rad/s 或 mm/s)
 * @param qdd 返回关节加速度(rad/s^2 或 mm/s^2)
 */
extern void get_joint_state_from_R_DYNAMICS(R_DYNAMICS* _rdyn, double* q,double* qd,double* qdd);

/**
 * @brief 设置动力学关节力矩数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param torque 关节力矩阵(Nmm 或 N)
 */
extern void set_torque_to_R_DYNAMICS(R_DYNAMICS* _rdyn, double* torque);

/**
 * @brief 获取动力学关节力矩数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param torque 返回关节力矩阵(Nmm 或 N)
 */
extern void get_torque_from_R_DYNAMICS(R_DYNAMICS* _rdyn, double* torque);

/**
 * @brief 设置动力学关节外力数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param fext 外力(N, Nmm)
 */
extern void set_fext_to_R_DYNAMICS(R_DYNAMICS* _rdyn, double* fext);

/**
 * @brief 设置动力学惯性项所需数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 关节位置(rad 或 mm)
 * @param _dof 机器人自由度
 */
extern void set_R_DYNAMICS_INERTIA(R_DYNAMICS* _rdyn, double* q, int _dof);

/**
 * @brief 设置动力学科氏力及向心力所需数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 关节位置(rad 或 mm)
 * @param qd 关节速度(rad/s 或 mm/s)
 * @param _dof 机器人自由度
 */
extern void set_R_DYNAMICS_CORIOLIS(R_DYNAMICS* _rdyn, double* q, double* qd, int _dof);

/**
 * @brief 设置动力学重力项所需数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 关节位置(rad 或 mm)
 * @param _dof 机器人自由度
 */
extern void set_R_DYNAMICS_GRAVITY(R_DYNAMICS* _rdyn, double* q, int _dof);

/**
 * @brief 设置动力学摩擦项所需数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param qd 关节速度(rad/s 或 mm/s)
 * @param _dof 机器人自由度
 */
extern void set_R_DYNAMICS_FRICTION(R_DYNAMICS* _rdyn, double* qd, int _dof);

/**
 * @brief 设置动力学外力项所需数据(R_DYNAMICS)
 *
 * @param _rdyn 动力学数据
 * @param q 关节位置(rad 或 mm)
 * @param fext 外力(N, Nmm)
 * @param _dof 机器人自由度
 */
extern void set_R_DYNAMICS_EXTERNALTORQUE(R_DYNAMICS* _rdyn, double* q,  double* fext, int _dof);

/**
 * @brief 逆动力学(使用关节力矩传感器时，与Dyn_InverseAxis配合使用)
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据
 * @return int 0:成功，其他：失败
 */
extern int Dyn_Inverse(const char* serialLinkName, R_DYNAMICS* rdyn);

/**
 * @brief 关节动力学，使用关节力矩传感器时有效，与Dyn_Inverse一起组成完成的机器人动力学
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据(需要传入关节测量力矩)
 * @return int 0:成功，其他：失败
 */
extern int Dyn_InverseAxis(const char* serialLinkName, R_DYNAMICS* rdyn);

/**
 * @brief 正动力学
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据
 * @return int 0:成功，其他：失败
 */
extern int Dyn_Forward(const char* serialLinkName, R_DYNAMICS* rdyn);

/**
 * @brief 惯性力
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据，利用set_R_DYNAMICS_INERTIA设置参数
 * @return int 0:成功，其他：失败
 */
extern int Dyn_Inertia(const char* serialLinkName, R_DYNAMICS* rdyn);

/**
 * @brief 科氏力及向心力
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据，利用set_R_DYNAMICS_CORIOLIS设置参数
 * @return int 0:成功，其他：失败
 */
extern int Dyn_Coriolis(const char* serialLinkName, R_DYNAMICS* rdyn);

/**
 * @brief 重力
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据，利用set_R_DYNAMICS_GRAVITY设置参数
 * @return int 0:成功，其他：失败
 */
extern int Dyn_Gravity(const char* serialLinkName, R_DYNAMICS* rdyn);

/**
 * @brief 摩擦力
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据，利用set_R_DYNAMICS_FRICTION设置参数
 * @return int 0:成功，其他：失败
 */
extern int Dyn_Friction(const char* serialLinkName, R_DYNAMICS* rdyn);

/**
 * @brief 外部力
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rdyn 动力学数据，利用set_R_DYNAMICS_EXTERNALTORQUE设置参数
 * @return int 0:成功，其他：失败
 */
extern int Dyn_ExternalTorque(const char* serialLinkName, R_DYNAMICS* rdyn);

#ifdef __cplusplus
}
}
#endif


#endif /* DYNAMICSINTERFACE_H_ */
