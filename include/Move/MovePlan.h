/**
 * @file MovePlan.h
 * 
 * @brief  机器人运动控制接口函数
 * @author hanbing
 * @version 11.3.0
 * @date 2020-03-31
 * 
 */

#ifndef MOVEPLAN_H_
#define MOVEPLAN_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/robotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 设置指令线程运行方式，设置后对所有指令起作用
 *
 * @param isthread 1:线程运行，0:阻塞运行
 */
extern void setMoveThread(int isthread);

/**
 * @brief 设置指令线程运行方式，设置后对所有指令起作用
 *
 * @param isthread 1:线程运行，0:阻塞运行
 * @param wait_time_ns 线程同步等待时间(ns)
 */
extern void setMoveThread1(int isthread,uint64_t wait_time_ns);

/**
 * @brief 以线程方式运行指令时，用改指令启动执行，可用于多运动指令同步执行
 *
 * @return 0成功,其他失败
 */
extern int MoveSyncStart();

/**
 * @brief 等待运动结束（开线程运行时有效）
 *
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int wait_move_finish(int robot_index);

/**
 * @brief 为机器人启用附加轴服务（机器人的运动将受附加轴影响）
 *
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引号
 * @return 0: 成功; 其他: 失败
 */
extern int StartAdditionServer(tool* rtool, wobj* rwobj,int robot_index);

/**
 * @brief 为机器人关闭附加轴服务
 *
 * @param robot_index 机器人索引号
 * @return 0: 成功; 其他: 失败
 */
extern int StopAdditionServer(int robot_index);

/**
 * @brief 获取机器人运动状态
 *
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int get_robot_move_state(int robot_index);

/**
 * @brief 获取附加轴运动状态
 *
 * @param addition_index 附加轴组索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int get_addition_move_state(int addition_index);

/**
 * @brief 清楚机器人运动错误
 *
 * @param robot_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_robot_move_error(int robot_index);

/**
 * @brief 清楚机器人驱动错误
 *
 * @param robot_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_robot_driver_error(int robot_index);

/**
 * @brief 清楚附加轴组运动错误
 *
 * @param addition_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_addition_move_error(int addition_index);

/**
 * @brief 清楚附加轴组驱动错误
 *
 * @param addition_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_addition_driver_error(int addition_index);

/**
 * @brief 休眠
 *
 * @param millisecond 休眠时间（毫秒）
 */
extern void Rsleep(int millisecond);

/**
 * @brief 设置机器人运动速度的百分比（加速度和加加速度）
 *
 * @param acc 加速度（0~1）
 * @param jerk 加加速度（0~1）
 * @param robot_index 机器人索引号
 */
extern void AccSet(double acc, double jerk, int robot_index);

/**
 * @brief 上电
 *
 */
extern void move_start(void);

/**
 * @brief 停止当前运动并下电(后续运动命令无法执行)
 *
 */
extern void move_stop(void);

/**
 * @brief 停止当前运动(不下电，后续运动命令可以执行)
 *
 */
extern void move_stop_run(void);

/**
 * @brief 绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad
 * @param rspeed 运动速度   单位：rad/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 绝对位置运动指令
*
* @param rjoint 期望位置（关节） 单位：rad
* @param rspeed 运动速度   单位：rad/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveA(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);


/**
 * @brief 双臂绝对位置运动指令
 * 
 * @param rjoint1 机器人1的期望位置（关节）  单位：rad
 * @param rjoint2 机器人2的期望位置（关节）   单位：rad
 * @param rspeed1 机器人1的运动速度   单位：rad/s
 * @param rspeed2 机器人2的运动速度   单位：rad/s
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveA(robjoint *rjoint1, robjoint *rjoint2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂绝对位置运动指令
*
* @param rjoint1 机器人1的期望位置（关节）  单位：rad
* @param rjoint2 机器人2的期望位置（关节）   单位：rad
* @param rspeed1 机器人1的运动速度   单位：rad/s
* @param rspeed2 机器人2的运动速度   单位：rad/s
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveA(robjoint* rjoint1,robjoint* rjoint2, speed* rspeed1,speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);;

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad
 * @param rspeed 运动速度   单位：rad/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做绝对位置运动指令
*
* @param rjoint 期望位置（关节） 单位：rad
* @param rspeed 运动速度   单位：rad/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveA(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 关节运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行 
 */
extern int moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 关节运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveJ(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂关节运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveJ(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂关节运动指令
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveJ(robpose* rpose1, robpose* rpose2, speed* rspeed1,speed* rspeed2, zone* rzone1,zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);;

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做绝对位置运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveJ(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 线性运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 线性运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveL(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂线性运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveL(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂线性运动指令
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveL(robpose* rpose1, robpose* rpose2, speed* rspeed1,speed* rspeed2, zone* rzone1,zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);;

/**
 * @brief 选取多机械臂中的一个做线性运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做线性运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveL(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 圆弧运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rpose_mid 途经点期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 圆弧运动指令
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 途经点期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveC(robpose* rpose, robpose* rpose_mid, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂圆弧运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rpose_mid1 机器人1的途经点期望位置（位姿）
 * @param rpose_mid2 机器人2的途经点期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveC(robpose *rpose1, robpose *rpose2, robpose *rpose_mid1, robpose *rpose_mid2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂圆弧运动指令
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rpose_mid1 机器人1的途经点期望位置（位姿）
* @param rpose_mid2 机器人2的途经点期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveC(robpose* rpose1, robpose* rpose2, robpose* rpose_mid1, robpose* rpose_mid2, speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做圆弧运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rpose_mid 途经点期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做圆弧运动指令
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 途经点期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveC(robpose* rpose, robpose* rpose_mid, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 螺旋线运动指令
 * 
 * @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line 方向位置（位姿）(决定最终运动姿态)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系 
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 螺旋线运动指令
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(mm)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 平面螺旋线运动指令
 *
 * @param pose_line 方向位置（位姿）
 * @param radius 半径(mm)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 平面螺旋线运动指令
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(mm)
* @param screw 螺距(mm)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂螺旋线运动指令
 * 
 * @param rpose1 机器人1的圆弧位置（位姿）（不影响姿态运动）
 * @param rpose2 机器人2的圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid1 机器人1的圆弧中间位置（位姿）（不影响姿态运动）
 * @param rpose_mid2 机器人2的圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line1 机器人1的方向位置（位姿）(决定最终运动姿态)
 * @param pose_line2 机器人2的方向位置（位姿）(决定最终运动姿态)
 * @param screw1 机器人1螺距(mm)
 * @param screw2 机器人2螺距(mm)
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveH(robpose* rpose1, robpose* rpose2, robpose* rpose_mid1, robpose* rpose_mid2, robpose* pose_line1,robpose* pose_line2, double screw1,double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
* @brief 双臂螺旋线运动指令
*
* @param rpose1 机器人1的圆弧位置（位姿）（不影响姿态运动）
* @param rpose2 机器人2的圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid1 机器人1的圆弧中间位置（位姿）（不影响姿态运动）
* @param rpose_mid2 机器人2的圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line1 机器人1的方向位置（位姿）(决定最终运动姿态)
* @param pose_line2 机器人2的方向位置（位姿）(决定最终运动姿态)
* @param screw1 机器人1螺距(mm)
* @param screw2 机器人2螺距(mm)
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveH(robpose* rpose1, robpose* rpose2, robpose* rpose_mid1, robpose* rpose_mid2, robpose* pose_line1,robpose* pose_line2, double screw1,double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 双臂平面螺旋线运动指令
 *
 * @param pose_line1 机器人1的方向位置（位姿）
 * @param pose_line2 机器人2的方向位置（位姿）
 * @param radius1 机器人1半径（mm）
 * @param radius2 机器人2半径（mm）
 * @param screw1 机器人1螺距(mm)
 * @param screw2 机器人2螺距(mm)
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveHP(robpose* pose_line1, robpose* pose_line2, double radius1, double radius2, double screw1, double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
* @brief 双臂平面螺旋线运动指令
*
* @param pose_line1 机器人1的方向位置（位姿）
* @param pose_line2 机器人2的方向位置（位姿）
* @param radius1 机器人1半径（mm）
* @param radius2 机器人2半径（mm）
* @param screw1 机器人1螺距(mm)
* @param screw2 机器人2螺距(mm)
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveHP(robpose* pose_line1, robpose* pose_line2, double radius1, double radius2, double screw1, double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做螺旋线运动指令
 * 
 * @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line 方向位置（位姿）(决定最终运动姿态)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系  
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做螺旋线运动指令
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(mm)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 选取多机械臂中的一个做平面螺旋线运动指令
 *
 * @param pose_line 方向位置（位姿）
 * @param radius 半径(mm)
 * @param screw 螺距(mm)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做平面螺旋线运动指令
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(mm)
* @param screw 螺距(mm)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 关节空间的B样条运动
 * 
 * @param filename 轨迹点存放文件名
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj);

/**
* @brief 关节空间的B样条运动
*
* @param filename 轨迹点存放文件名
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveS1(const char* filename, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 关节空间的B样条运动
*
* @param rpose 样条必经点
* @param num 样条必经点数目
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveS(robpose* rpose, int num, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂关节空间的B样条运动
 * 
 * @param filename1 机器人1的轨迹点存放文件名
 * @param filename2 机器人2的轨迹点存放文件名
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveS(char *filename1, char *filename2, speed *rspeed1, speed *rspeed2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂关节空间的B样条运动
*
* @param filename1 机器人1的轨迹点存放文件名
* @param filename2 机器人2的轨迹点存放文件名
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveS(const char* filename1, const char* filename2, speed* rspeed1, speed* rspeed2,zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做关节空间的B样条运动
 * 
 * @param filename 轨迹点存放文件名
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做关节空间的B样条运动
*
* @param filename 轨迹点存放文件名
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveS1(const char* filename, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做关节空间的B样条运动
*
* @param rpose 样条必经点
* @param num 样条必经点数目
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveS(robpose* rpose, int num, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 动态目标笛卡尔空间运动
 * 
 * @param rpose 期望位置（位姿）
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveD(robpose* rpose, speed* rspeed, tool* rtool, wobj* rwobj,int stop_cond);

/**
* @brief 动态目标笛卡尔空间运动
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rtool 末端工具
* @param rwobj 坐标系
* @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveD(robpose* rpose, speed* rspeed, tool* rtool, wobj* rwobj, int stop_cond);

/**
 * @brief 双臂动态目标笛卡尔空间运动
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveD(robpose* rpose1,robpose* rpose2, speed* rspeed1,speed* rspeed2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2,int stop_cond);

/**
* @brief 双臂动态目标笛卡尔空间运动
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveD(robpose* rpose1, robpose* rpose2, speed* rspeed1,speed* rspeed2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2, int stop_cond);


/**
 * @brief 选取多机械臂中的一个做动态目标笛卡尔空间运动
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveD(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int stop_cond,int _index);

/**
* @brief 选取多机械臂中的一个做动态目标笛卡尔空间运动
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rtool 末端工具
* @param rwobj 坐标系
* @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveD(robpose* rpose, speed* rspeed, tool* rtool, wobj* rwobj, int stop_cond,int _index);

/**
 * @brief 选取多机械臂中的一个设置动态目标笛卡尔空间运动的目标
 * 
 * @param rpose 期望位置（位姿） 
 * @param robot_index 机器人索引号
 */
extern void set_multi_moveD_target(robpose* rpose, int robot_index);

/**
* @brief 选取多机械臂中的一个设置动态目标笛卡尔空间运动的目标
*
* @param rpose 期望位置（位姿）
* @param robot_index 机器人索引号
*/
extern void SetMultiMoveDTarget(robpose* rpose, int robot_index);

/**
 * @brief 设置动态目标笛卡尔空间运动的目标
 * 
 * @param rpose 期望位置（位姿） 
 */
extern void set_moveD_target(robpose* rpose);

/**
* @brief 设置动态目标笛卡尔空间运动的目标
*
* @param rpose 期望位置（位姿）
*/
extern void SetMoveDTarget(robpose* rpose);

/**
 * @brief 关节空间多项式规划
 * 
 * @param rtool 末端工具
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveAP(tool* rtool);

/**
* @brief 关节空间多项式规划
*
* @param rtool 末端工具
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveAP(tool* rtool);

/**
 * @brief 双臂关节空间多项式规划
 * 
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具 
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveAP(tool* rtool1, tool* rtool2);

/**
* @brief 双臂关节空间多项式规划
*
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveAP(tool* rtool1, tool* rtool2);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划
 * 
 * @param _index 机器人索引号
 * @param rtool 末端工具
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveAP(int _index,tool* rtool);

/**
* @brief 选取多机械臂中的一个做关节空间多项式规划
*
* @param robot_index 机器人索引号
* @param rtool 末端工具
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveAP(int robot_index,tool* rtool);

/**
 * @brief 关节空间多项式规划插入路点
 * 
 * @param time 时间（s）
 * @param joint 关节位置 (rad 或 m)
 * @param velocity 关节速度 (rad/s 或 m/s)
 * @param acceleration 关节加速度（rad/s^2  或 m/s^2）
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int moveAP_push_way_point(double time, double *joint,double* velocity, double* acceleration);

/**
* @brief 关节空间多项式规划插入路点
*
* @param time 时间（s）
* @param joint 关节位置 (rad 或 m)
* @param velocity 关节速度 (rad/s 或 m/s)
* @param acceleration 关节加速度（rad/s^2  或 m/s^2）
* @return int >=0:路点数目; <0:插入路点失败
*/
extern int PushMoveAPWayPoint(double time, double *joint,double* velocity, double* acceleration);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划插入路点
 * @param time 时间（s）
 * @param joint 关节位置 (rad 或 m)
 * @param velocity 关节速度 (rad/s 或 m/s)
 * @param acceleration 关节加速度（rad/s^2  或 m/s^2） 
 * @param _index 机器人索引号
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int multi_moveAP_push_way_point(double time, double *joint,double* velocity, double* acceleration, int _index);

/**
* @brief 选取多机械臂中的一个做关节空间多项式规划插入路点
* @param time 时间（s）
* @param joint 关节位置 (rad 或 m)
* @param velocity 关节速度 (rad/s 或 m/s)
* @param acceleration 关节加速度（rad/s^2  或 m/s^2）
* @param robot_index 机器人索引号
* @return int >=0:路点数目; <0:插入路点失败
*/
extern int MultiPushMoveAPWayPoint(double time, double *joint,double* velocity, double* acceleration, int robot_index);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划删除路点
 * @param robot_index 机器人索引号
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int MultiPopMoveAPWayPoint(int robot_index);

/**
 * @brief 关节空间多项式规划删除路点
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int PopMoveAPWayPoint(void);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划删除所有路点
 * @param robot_index 机器人索引号
 */
extern void MultiEmptyMoveAPWayPoint(int robot_index);

/**
 * @brief 关节空间多项式规划删除所有路点
 */
extern void EmptyMoveAPWayPoint(void);

/**
 * @brief 附加轴绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad or m
 * @param rspeed 运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveAdd(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 选取多附加轴中的一个做绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad or m
 * @param rspeed 运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveAdd(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令(含附加轴组)
 * 
 * @param rjoint 期望位置（关节） 单位：rad or m
 * @param ajoint 附加轴组期望位置（关节） 单位：rad or m
 * @param rspeed 运动速度   单位：rad/s or m/s
 * @param aspeed 附加轴组运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveJoint(robjoint* rjoint,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿） 
 * @param ajoint 附加轴组期望位置（关节） 单位：rad or m 
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveJoint1(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
 * @brief 选取多机械臂中的一个做线性运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿）
 * @param ajoint 附加轴组期望位置（关节） 单位：rad or m 
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveLine(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做圆弧运动指令(含附加轴组)
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 中间位置（位姿）
* @param ajoint 附加轴组期望位置（关节） 单位：rad or m
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度   单位：rad/s or m/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveCircle(robpose* rpose,robpose* rpose_mid,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做螺旋线运动指令(含附加轴组)
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节） 单位：rad or m
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度   单位：rad/s or m/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveHelical(robpose* rpose,robpose* rpose_mid,robpose* pose_line,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做平面螺旋线运动指令(含附加轴组)
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(m)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节） 单位：rad or m
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度   单位：rad/s or m/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveHelicalPlane(robpose* pose_line,double radius,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做关节空间的B样条运动(含附加轴组)
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param num 数据数目
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveBSpline(robpose* rpose,int num,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
 * @brief 绝对位置运动指令(含附加轴组)
 * 
 * @param rjoint 期望位置（关节）
 * @param ajoint 附加轴组期望位置（关节）
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveJoint(robjoint* rjoint,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 绝对位置运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿） 
 * @param ajoint 附加轴组期望位置（关节）
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveJoint1(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 线性运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿）
 * @param ajoint 附加轴组期望位置（关节）
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveLine(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 圆弧运动指令(含附加轴组)
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 途经点期望位置（位姿）
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveCircle(robpose* rpose,robpose* rpose_mid,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 螺旋线运动指令(含附加轴组)
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveHelical(robpose* rpose,robpose* rpose_mid,robpose* pose_line,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 平面螺旋线运动指令(含附加轴组)
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(m)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveHelicalPlane(robpose* pose_line,double radius,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 关节空间的B样条运动
*
* @param rpose 样条必经点
* @param num 样条必经点数目
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveBSpline(robpose* rpose,int num,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);


/**
 * @brief 机器人位姿绝对工件偏移补偿
 *
 * @param rpose 机器人初始位姿
 * @param x x方向补偿值(mm)
 * @param y y方向补偿值(mm)
 * @param z z方向补偿值(mm)
 * @param k 姿态变量1补偿值(rad)
 * @param p 姿态变量2补偿值(rad)
 * @param s 姿态变量3补偿值(rad)
 * @return robpose 机器人补偿后位姿
 */
extern robpose Offs(const robpose* rpose, double x, double y, double z, double k, double p, double s);

/**
 * @brief 机器人位姿相对工具偏移补偿
 *
 * @param rpose 机器人初始位姿
 * @param x x方向补偿值(mm)
 * @param y y方向补偿值(mm)
 * @param z z方向补偿值(mm)
 * @param k 姿态变量1补偿值(rad)
 * @param p 姿态变量2补偿值(rad)
 * @param s 姿态变量3补偿值(rad)
 * @return robpose 机器人补偿后位姿
 */
extern robpose OffsRel(const robpose* rpose, double x, double y, double z, double k, double p, double s);

/**
 * @brief 获取机器人当前关节角
 * 
 * @param joint 当前关节角数据(rad/mm)
 * @param _index 机器人索引号
 */
extern void robot_getJoint(double* joint, int _index);

/**
* @brief 获取机器人当前关节角
*
* @param joint 当前关节角数据(rad/mm)
* @param _index 机器人索引号
*/
extern void GetCurrentJoint(double* joint, int _index);

/**
 * @brief 获取机器人当前目标关节角
 * 
 * @param joint 当前关节角数据(rad/mm)
 * @param _index 机器人索引号
 */
extern void GetCurrentTargetJoint(double* joint, int _index);

/**
 * @brief 获取附加轴组当前关节角
 * 
 * @param joint 当前关节角数据(rad/mm)
 * @param _index 机器人索引号
 */
extern void GetCurrentAdditionJoint(double* joint, int _index);

/**
 * @brief 获取附加轴组当前目标关节角
 * 
 * @param joint 当前关节角数据(rad/mm)
 * @param _index 机器人索引号
 */
extern void GetCurrentTargetAdditionJoint(double* joint, int _index);

/**
 * @brief 获取机器人当前位姿
 * 
 * @param rtool 工具
 * @param rwobj 工件
 * @param pospose 当前位姿数据(mm,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int robot_getCartesian(tool* rtool,wobj* rwobj, double* pospose, int _index);

/**
* @brief 获取机器人当前位姿
*
* @param rtool 工具
* @param rwobj 工件
* @param pospose 当前位姿数据(mm,rad)
* @param _index 机器人索引号
* @return int 0:成功；其他失败
*/
extern int GetCurrentCartesian(tool* rtool,wobj* rwobj, robpose* pospose, int _index);

/**
 * @brief 获取机器人当前目标位姿
 * 
 * @param rtool 工具
 * @param rwobj 工件
 * @param pospose 当前位姿数据(mm,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int GetCurrentTargetCartesian(tool* rtool,wobj* rwobj, robpose* pospose, int _index);

/**
* @brief 获取附加轴组当前位姿
*
* @param pospose 当前位姿数据(mm,rad)
* @param _index 机器人索引号
* @return int 0:成功；其他失败
*/
extern int GetCurrentAdditionCartesian(robpose* pospose, int _index);

/**
 * @brief 获取附加轴组当前目标位姿
 * 
 * @param pospose 当前目标位姿数据(mm,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int GetCurrentAdditionTargetCartesian(robpose* pospose, int _index);

/**
 * @brief 获取机器人移动速度系数（0~1）
 * 
 * @param robot_index 机器人索引
 * @return double 速度系数（0~1），如果小于1，机器人运动速度将低于设定速度运行
 */
double GetRobotMoveSpeedCoeff(int robot_index);

/**
 * @brief 设置机器人移动速度系数（0~1），动态生效
 * 
 * @param robot_index 机器人索引
 * @param value 速度系数（0~1）
 * @return int 0:正确，其他错误
 */
int SetRobotMoveSpeedCoeff(double value, int robot_index);


/**
 * @brief 设置数字量输出
 * 
 * @param id 数字量索引号
 * @param flag 0或1
 */
extern void SetDo(int id, int flag);

/**
 * @brief 获取数字量输入
 * 
 * @param id 数字量输出索引号
 * @return int 返回值0或1
 */
extern int GetDi(int id);

/**
 * @brief 等待数字量输入赋值
 * 
 * @param id 数字量输入索引号
 * @param value 触发信号0或1
 */
extern void WaitDi(int id, int value);

/**
 * @brief 设置模拟量输出
 * 
 * @param id 模拟量输出索引号
 * @param flag 模拟量
 */
extern void SetAo(int id, double flag);

/**
 * @brief 获取模拟量输入
 * 
 * @param id 模拟量输入索引号
 * @return double 模拟量
 */
extern double GetAi(int id);



/**
 * @brief 获取机器人自由度
 * 
 * @param _index 机器人索引号
 * @return int 返回机器人自由度
 */
extern int robot_getDOF(int _index);

/**
 * @brief 获取机器人附加轴数量
 * 
 * @param _index 机器人附加轴分组索引
 * @return int 返回附加轴数量 
 */
extern int additionaxis_getDOF(int _index);

/**
 * @brief 获取机器人数量
 * 
 * @return int 返回机器人数量
 */
extern int robot_getNUM();

/**
 * @brief 获取附加轴组数量
 *
 * @return int 返回附加轴组数量
 */
extern int additionaxis_getNUM();


//----------------------------------------------------------------------------------------thread interface--------------------------------------------------------------------------
/**
 * @brief 创建线程
 * 
 * @param fun 线程回调执行函数
 * @param arg 执行参数
 * @param name 线程名
 * @param detached_flag 线程属性标识thread attribute,0:PTHREAD_CREATE_JOINABLE  other:PTHREAD_CREATE_DETACHED
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadCreat(void *(*fun)(void *), void *arg, const char *name, int detached_flag);

/**
 * @brief 线程数据释放
 * 
 * @param name 线程名
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadDataFree(const char *name);


/**
 * @brief thread join (当线程属性为PTHREAD_CREATE_JOINABLE时有效)
 * 
 * @param name 线程名
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadWait(const char *name);


#ifdef __cplusplus
}
}
#endif


#endif /* MOVEPLAN_H_ */
