/**
 * @file SafeAreas.h
 * 
 * @brief  安全区
 * @author hanbing
 * @version 11.0.0
 * @date 2020-11-18
 * 
 */


#ifndef SAFEAREAS_H_
#define SAFEAREAS_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/robotStruct.h"

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 添加球区域（最先设置检测优先级最低，最后设置检测优先级最高）
 *
 * @param centre 球心（单位：mm）
 * @param radius 球半径（单位：mm）
 * @param property 区域性质，_constraint_notallow:完全不允许区域; _constraint_slowallow:不允许区域，但允许缓慢移动;_constraint_allow:完全允许区域，其他区域完全不允许;_constraint_allowslow:完全允许区域，其他区域允许缓慢移动;
 * @param rtool 末端工具（球体描述坐标系，工具不在机器人上有效）
 * @param rwobj 工件坐标系（球体描述坐标系，工件不在机器人上有效）
 * @param name 指定几何体名字
 * @return int 0:添加成功：其他：添加失败
 */
int AddSpatialConstraintSphere(double* centre, double radius, int  property, tool* rtool, wobj* rwobj, const char* name);

/**
 * @brief 添加立方体（最先设置检测优先级最低，最后设置检测优先级最高）
 *
 * @param centre 立方体角点（单位：mm）
 * @param length centre所在长边的另一个角点（单位：mm）
 * @param width centre所在宽边的另一个角点（单位：mm）
 * @param high centre所在高边的另一个角点（单位：mm）
 * @param property 区域性质，_constraint_notallow:完全不允许区域; _constraint_slowallow:不允许区域，但允许缓慢移动;_constraint_allow:完全允许区域，其他区域完全不允许;_constraint_allowslow:完全允许区域，其他区域允许缓慢移动;
 * @param rtool 末端工具（球体描述坐标系，工具不在机器人上有效）
 * @param rwobj 工件坐标系（球体描述坐标系，工件不在机器人上有效）
 * @param name 指定几何体名字
 * @return int 0:添加成功：其他：添加失败
 */
int AddSpatialConstraintCuboid(double* centre, double* length, double* width, double* high, int  property, tool* rtool, wobj* rwobj, const char* name);

/**
 * @brief 添加圆柱体或空心圆柱（最先设置检测优先级最低，最后设置检测优先级最高）
 *
 * @param centreb 下底面圆形点（单位：mm）
 * @param centreh  上底面圆形点（单位：mm）
 * @param inner_radius 内半径（单位：mm）
 * @param external_radius 外半径（单位：mm）
 * @param property 区域性质，_constraint_notallow:完全不允许区域; _constraint_slowallow:不允许区域，但允许缓慢移动;_constraint_allow:完全允许区域，其他区域完全不允许;_constraint_allowslow:完全允许区域，其他区域允许缓慢移动;
 * @param rtool 末端工具（球体描述坐标系，工具不在机器人上有效）
 * @param rwobj 工件坐标系（球描述坐标系，工件不在机器人上有效）
 * @param name 指定几何体名字
 * @return int 0:添加成功：其他：添加失败
 */
int AddSpatialConstraintCylinder(double* centreb, double* centreh, double inner_radius, double external_radius, int  property, tool* rtool, wobj* rwobj, const char* name);

/**
 * @brief 删除添加的几何体

 * @param name 几何体名字
 * @return int 0:删除成功：其他：删除失败
 */
int DeleteSpatialConstraint(const char* name);

/**
 * @brief 删除添加的所有几何体

 * @return int 0:删除成功：其他：删除失败
 */
int DeleteAllSpatialConstraint();

/**
 * @brief 设置缓慢允许区系数

 * @param slow_coeff 缓慢允许系数（0~1.0）
 * @param name 几何体名字
 * @return int 0:删除成功：其他：删除失败
 */
int SetSpatialConstraintSlowCoeff(double slow_coeff, const char* name);

/**
 * @brief 开启检测（默认开启）
 */
void OnSpatialConstraint();

/**
 * @brief 关闭检测
 */
void OffSpatialConstraint();

/**
 * @brief 检测机器人所处区域
 *
 * @param pos 机器人但前位置
 * @param pose 机器人当前姿态（允许为NULL）
 * @param rtool 末端工具（机器人位姿数据描述坐标系，工具不在机器人上有效）
 * @param rwobj 工件坐标系（机器人位姿数据描述坐标系，工件不在机器人上有效）
 * @param result 返回区域表面法向，大小与进入区域的深度成正比。注：会作为下次检测的输入。（允许为：NULL）
 * @param slow_coeff 返回缓慢区域系数（0~1.0） 
 * @return int _constraint_allowstate:处于允许区域 ;_constraint_slowstate: 处于不允许区域，但允许缓慢移动;_constraint_noallowstate:处于完全不允许区域
 */
int IsSpatialConstraint(double* pos, double* pose, tool* rtool, wobj* rwobj, double* result, double* slow_coeff);


#ifdef __cplusplus
}
}
#endif

#endif /* SAFEAREAS_H_ */
