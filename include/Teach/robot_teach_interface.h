/**
 * @file robot_teach_interface.h
 *
 * @brief  机器人示教接口
 * @author hanbing
 * @version 11.1.0
 * @date 2020-05-23
 *
 */

#ifndef ROBOT_TEACH_INTERFACE_H_
#define ROBOT_TEACH_INTERFACE_H_

#include "Base/robotStruct.h"

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

//----------------------------数据操作接口见robotStruct.h--------------------------


//-------------------------------------------------------------------------------
/**
 * @brief 设置要操作机器人的索引
 *
 * @param robot_index 机器人索引
 * @return int 0: 成功, 其他: 失败
 */
extern int set_robot_index(int robot_index);

/**
 * @brief 设置要操作附加轴组的索引
 *
 * @param addition_index 附加轴组索引
 * @return int 0: 成功, 其他: 失败
 */
extern int set_addition_index(int addition_index);

/**
 * @brief 获取当前操作机器人索引
 *
 * @return int 机器人索引
 */
extern int get_robot_index();

/**
 * @brief 获取当前操作附加轴组索引
 *
 * @return int 附加轴组索引
 */
extern int get_addition_index();

/**
 * @brief 设置当前操作机器人工具
 *
 * @param tool_name 工具名称
 * @return int 0: 成功, 其他: 失败
 */
extern int set_robot_tool(const char* tool_name);

/**
 * @brief 获取当前操作机器人工具
 *
 * @return char* 工具名称
 */
extern const char* get_robot_tool();

/**
 * @brief 设置当前操作机器人工具
 *
 * @param wobj_name 工件名称
 * @return int 0: 成功, 其他: 失败
 */
extern int set_robot_wobj(const char* wobj_name);

/**
 * @brief 获取当前操作机器人工件
 *
 * @return char* 工件名称
 */
extern const char* get_robot_wobj();

/**
 * @brief 设置当前操作机器人的坐标系
 *
 * @param frame 坐标系编号, 0:关节;1:机器人基坐标系;2:机器人工具坐标系;3:机器人工件坐标系
 * @return int 0: 成功, 其他: 失败
 */
extern int set_robot_teach_coordinate(int frame);

/**
 * @brief 获取当前操作机器人的坐标系
 *
 * @return int 坐标系编号, 0:关节;1:机器人基坐标系;2:机器人工具坐标系;3:机器人工件坐标系
 */
extern int get_robot_teach_coordinate();

/**
 * @brief 获取机器人数目
 *
 * @return int 机器人数目
 */
extern int get_robot_num();

/**
 * @brief 获取附加轴组数目
 *
 * @return int 附加轴组数目
 */
extern int get_addition_num();

/**
 * @brief 获取当前操作机器人的自由度
 *
 * @return int 机器人自由度
 */
extern int get_robot_dof();

/**
 * @brief 获取当前操作附加轴组的自由度
 *
 * @return int 附加轴组自由度
 */
extern int get_addition_dof();

/**
 * @brief 获取当前操作机器人的关节位置
 *
 * @param joint 机器人关节位置(rad)
 * @return int 0: 成功, 其他: 失败
 */
extern int get_robot_joint(double* joint);

/**
 * @brief 获取当前操作附加轴组的关节位置
 *
 * @param joint 附加轴组关节位置(rad)
 * @return int 0: 成功, 其他: 失败
 */
extern int get_addition_joint(double* joint);

/**
 * @brief 获取当前操作机器人的笛卡尔位置
 *
 * @param cartesin 机器人笛卡尔位置(mm, rad), 数据相对设置的坐标系描述
 * @return int 0: 成功, 其他: 失败
 */
extern int get_robot_cartesian(double* cartesin);

/**
 * @brief 设置当前操作机器人的最大示教速度
 *
 * @param vel_percent 示教速度百分比(0~1)
 */
extern void set_robot_teach_velocity(double vel_percent);

/**
 * @brief 获取当前操作机器人的最大示教速度
 *
 * @return double 示教速度百分比(0~1)
 */
extern double get_robot_teach_velocity();

/**
 * @brief 设置当前操作机器人的动态速度系数（影响机器人示教速度和项目中机器人移动速度）
 *
 * @param vel_percent 速度百分比系数(0~1)
 */
extern void set_robot_dynamic_velocity(double vel_percent);

/**
 * @brief 获取当前操作机器人的动态速度系数
 *
 * @return double 速度百分比系数(0~1)
 */
extern double get_robot_dynamic_velocity();

/**
 * @brief 设置当前操作机器人的运行方式
 *
 * @param isSimulation 0:非仿真运行；1:仿真运行
 */
extern void set_robot_run_type(int isSimulation);

/**
 * @brief 获取当前操作机器人的运行方式
 *
 * @return int 0:非仿真运行；1:仿真运行
 */
extern int get_robot_run_type();

/**
 * @brief 获取当前操作机器人的运行状态
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int get_robot_run_state();

/**
 * @brief 获取当前操作附加轴组的运行状态
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int get_addition_run_state();

/**
 * @brief 创建末端六维力传感器
 *
 * @param torqueSensorName 创建时指定的名字，其他操作用到该名字。该名字与设备描述配置文件名一致(不包含后缀)
 * @return int 0: 成功, -1:传感器已创建，-2:设备初始化失败
 */
extern int create_robot_torque_sensor(const char* torqueSensorName);

/**
 * @brief 末端六维力传感器无负载简易标定（仅需一次，永久生效）
 *
 * @param torqueSensorName 创建时指定的名字
 * @return int 0: 成功, 其他：失败
 */
extern int calibration_robot_torque_sensor(const char* torqueSensorName);

/**
 * @brief 采集传感器精确标定数据(两次采集且传感器受力情况对称)
 *
 * @param torqueSensorName 创建时指定的名字
 * @param data_index 数据索引（为0 和 1）
 * @return int 0: 成功, 其他：失败
 */
extern int calibration_robot_torque_sensor_accurate_data(const char* torqueSensorName, int data_index);

/**
 * @brief 末端六维力传感器精确标定,需要采集数据（仅需一次，永久生效）
 *
 * @param torqueSensorName 创建时指定的名字
 * @return int 0: 成功, 其他：失败
 */
extern int calibration_robot_torque_sensor_accurate(const char* torqueSensorName);

/**
 * @brief 获取末端六维力传感器数据
 *
 * @param torqueSensorName 创建时指定的名字
 * @param torque 六维传感器数据，[fx,fy,fz,tx,ty,tz](N, Nm)
 * @return int 0: 成功, 其他：失败
 */
extern int get_robot_torque_sensor(const char* torqueSensorName, double* torque);

/**
 * @brief 获取末端六维力传感器转换到工具数据
 *
 * @param torqueSensorName 创建时指定的名字
 * @param torque 六维传感器数据，[fx,fy,fz,tx,ty,tz](N, Nm)
 * @return int 0: 成功, 其他：失败
 */
extern int get_robot_torque_sensor_transform(const char* torqueSensorName, double* torque);

/**
 * @brief 获取关节力传感器数据
 *
 * @param torque 关节力传感器数据(N or Nm)
 * @return int 0: 成功, 其他：失败
 */
extern int get_robot_joint_torque_sensor(double* torque);

/**
 * @brief 创建夹爪
 *
 * @param gripName 创建时指定的名字，其他操作用到该名字。该名字与设备描述配置文件名一致(不包含后缀)
 * @return int 0: 成功, -1:传感器已创建，-2:设备初始化失败
 */
extern int create_robot_grip(const char* gripName);

/**
 * @brief 夹爪开合控制
 *
 * @param gripName 创建时指定的名字
 * @param close_percent 夹爪闭合程度，0.0~1.0之间取值，0.0完全伸展开，1.0完全夹紧；
 * @return int 0: 成功, 其他：失败
 */
extern int control_robot_grip(const char* gripName, double close_percent);

/**
 * @brief 清除当前操作机器人的错误运动状态
 *
 */
extern void robot_move_error_clear();

/**
 * @brief 清除当前操作附加轴组的错误运动状态
 *
 */
extern void addition_move_error_clear();

/**
 * @brief 清除当前操作机器人的错误驱动状态
 *
 */
extern void robot_driver_error_clear();

/**
 * @brief 清除当前操作附加轴组的错误驱动状态
 *
 */
extern void addition_driver_error_clear();

/**
 * @brief 启动机器人项目
 *
 * @param project_name 机器人项目名
 * @return int 0:项目成功执行；-1:输入项目名过长；-2:系统处于繁忙状态；-3:机器人运动状态不正确；-4: 创建启动项目失败；-5:项目编译失败；-6:项目加载失败；-7:项目主函数定位失败
 */
extern int start_robot_project(const char* project_name);

/**
 * @brief 关闭机器人项目
 *
 *return int 0: 成功, 其他：失败
 */
extern int close_robot_project();

/**
 * @brief 启动机器人C项目
 *
 * @param project_name 机器人项目名
 * @return int 0:项目成功执行；-1:输入项目名过长；-2:系统处于繁忙状态；-3:机器人运动状态不正确；-4: 项目编译失败；-5:创建启动项目失败；
 */
extern int start_robot_c_project(const char* project_name);

/**
 * @brief 关闭机器人C项目
 *
 * return int 0: 成功, 其他：失败
 */
extern int close_robot_c_project();

/**
 * @brief 启动机器人lua项目
 *
 * @param project_name 机器人项目名
 * @return int 0:项目成功执行；-1:输入项目名过长；-2:系统处于繁忙状态；-3:机器人运动状态不正确；-4: 创建启动项目失败；
 */
extern int start_robot_lua_project(const char* project_name);

/**
 * @brief 关闭机器人lua项目
 *
 * return int 0: 成功, 其他：失败
 */
extern int close_robot_lua_project();

/**
 * @brief 保存当期操作机器人的当前关节数据
 *
 * @param data_name 数据名字，建议数据命名规则：”j1","j2",...,"jn"，重名数据被覆盖
 * @return int 0: 成功, 其他：失败(-1:指定名字无效)
 */
extern int save_robot_current_joint_data(const char* data_name);

/**
 * @brief 保存当期操作机器人的当前位置及姿态数据
 *
 * @param data_name 数据名字，建议数据命名规则：”p1","p2",...,"pn"，重名数据被覆盖
 * @return int 0: 成功, 其他：失败(-1:指定名字无效)
 */
extern int save_robot_current_cartesian_data(const char* data_name);

/**
 * @brief 设置机器人系统使用的数据所在路径
 *
 * @param path 路径
 * @return int 0: 成功, 其他：失败(-1:路径设置失败,-2默认使用路径设置失败)
 */
extern int set_robot_data_path(const char* path);

/**
 * @brief 设置机器人系统当前使用的数据所在路径为默认路径
 *
 * @return int 0: 成功, 其他：失败
 */
extern int set_robot_data_default_path();

/**
 * @brief 获取机器人系统使用的数据所在路径
 *
 * @return const char* 路径数据指针
 */
extern const char* get_robot_data_path();

/**
 * @brief 启动基于末端力矩传感器的拖动
 *
 * @param inertia 惯性系数
 * @param damping 阻尼系数
 * @param teach_axis 拖动轴向量[x,y,z,Rx,Ry,Rz],为1表示拖动轴，0表示非拖动轴。
 * @param is_tool_ctrl 拖动轴的参考坐标系，为1表示参考坐标系为工具坐标系，0表示工件坐标系
 * @return int 0:执行成功；其他：错误
 */
extern int robot_direct_teach_start(double inertia, double damping, int* teach_axis, int is_tool_ctrl);

/**
 * @brief 关闭基于末端力矩传感器的拖动
 *
 * @return int 0:执行成功；其他：错误
 */
extern int robot_direct_teach_end();

/**
 * @brief 启动遥控手柄示教
 *   手柄工作于X模式
		按键名               ：用途                                                                   :  备注
		A                     打开夹爪
		B
		X                     停止c/lua项目
		Y                     停止c/lua项目
		lb                    使能
		rb,X                  启动c项目                                                               项目名称“joystick_program”
		rb,Y                  启动lua项目                                                             项目名称“joystick_program”
		rb,A                  闭合夹爪
		rb,B                  回零运动                                                                需要lb使能
		back
		start
		logitech
		ljb                   退出
		rjb                   清楚错误

		lt
		rt
		left,rx,ry            示教移动                                                                配合rx,ry使用，对应y轴平移旋转或3,4关节，需要lb使能
		right 
		up     ,rx,ry         示教移动                                                                配合rx,ry使用，对应x轴平移旋转或1,2关节，需要lb使能
		down,rx,ry            示教移动                                                                配合rx,ry使用，对应z轴平移旋转或5,6关节，需要lb使能
 *
 * @return int 0:执行成功；其他：错误
 */
extern int start_robot_teach_joystick();

/**
 * @brief 遥控手柄示教
 *
 * @return int 0:执行成功；其他：错误
 */
extern int close_robot_teach_joystick();

//----------------------------------------------------运动相关接口---------------------------

/**
 * @brief 使能当前操作机器人
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_teach_enable();

/**
 * @brief 使能当前操作附加轴组
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int addition_teach_enable();

/**
 * @brief 去使能当前操作机器人
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_teach_disenable();

/**
 * @brief 非使能当前操作附加轴组
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int addition_teach_disenable();

/**
 * @brief 获取当前操作机器人使能状态
 *
 * @return 0:非使能；1:使能
 */
extern int robot_teach_enable_state();

/**
 * @brief 获取当前操作附加轴组使能状态
 *
 * @return 0:非使能；1:使能
 */
extern int addition_teach_enable_state();

/**
 * @brief 停止当前操作机器人的示教移动
 *
 * return int 0: 成功, 其他：失败
 */
extern int robot_teach_stop();

/**
 * @brief 停止当前操作附加轴组的示教移动
 *
 * return int 0: 成功, 其他：失败
 */
extern int addition_teach_stop();

/**
 * @brief 当前操作机器人回零运动
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_home();

/**
 * @brief 当前操作附加轴组回零运动
 *
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int addition_home();

/**
 * @brief 当前操作机器人运动到指定目标关节位置
 *
 * @param J 机器人目标关节位置(rad)
 * @param dof 机器人机器人自由度
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_goto_joint(double* J, int dof);

/**
 * @brief 当前操作附加轴组运动到指定目标关节位置
 *
 * @param J 附加轴组目标关节位置(rad or m)
 * @param dof 附加轴组机器人自由度
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int addition_goto_joint(double* J, int dof);

/**
 * @brief 当前操作机器人运动到指定目标位姿
 *
 * @param position 机器人目标位姿(m,rad)
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_goto_cartesian(double* position);

/**
 * @brief 设置示教运动类型
 *
 * @param move_type 0:continuous; 1:步进0.01m or pi/18 rad; 2:步进0.001m or pi/180 rad; 3:步进0.0001m or pi/1800 rad
*/
extern void set_teach_move_type(int move_type);

/**
 * @brief 获取示教运动类型
 *
 * @return int 返回示教运动类型, 0:continuous; 1:步进0.01m or pi/18 rad; 2:步进0.001m or pi/180 rad; 3:步进0.0001m or pi/1800 rad
*/
extern int get_teach_move_type();

/**
 * @brief 在关节空间示教当前操作机器人
 *
 * @param axis_index 机器人关节索引(0,1,2,...)
 * @param dir 运动方向，>0:正方向运动，<0:负方向运动，=0:不运动
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_teach_joint(int axis_index, int dir);

/**
 * @brief 在关节空间示教当前操作附加轴组
 *
 * @param axis_index 附加轴组关节索引(0,1,2,...)
 * @param dir 运动方向，>0:正方向运动，<0:负方向运动，=0:不运动
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int addition_teach_joint(int axis_index,int dir);

/**
 * @brief 在笛卡尔空间示教当前操作机器人，运动相对坐标系包括：基座坐标系，工件坐标系，工具坐标系，由当前设置的坐标系决定。
 *
 * @param axis_index 机器人笛卡尔轴索引(0:x,1:y,2:z,3:rx,4:ry,5:rz)
 * @param dir 运动方向，>0:正方向运动，<0:负方向运动，=0:不运动
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_teach_cartesian(int axis_index,int dir);

/**
 * @brief 在关节空间示教当前操作附加轴组,在工件坐标系下与之关联的机器人末端相对工件静止
 *
 * @param axis_index 附加轴组关节索引(0,1,2,...)
 * @param dir 运动方向，>0:正方向运动，<0:负方向运动，=0:不运动
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int addition_teach_cartesian(int axis_index,int dir);

/**
 * @brief 示教当前操作机器人，运动相对坐标系包括：关节坐标系，基座坐标系，工件坐标系，工具坐标系，由当前设置的坐标系决定。
 *
 * @param axis_index 机器人关节索引(0,1,2,...)或机器人笛卡尔轴索引(0:x,1:y,2:z,3:rx,4:ry,5:rz)
 * @param dir 运动方向，>0:正方向运动，<0:负方向运动，=0:不运动
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int robot_teach_move(int axis_index,int dir);

/**
 * @brief 示教当前操作附加轴组，运动相对坐标系包括：关节坐标系，基座坐标系，工件坐标系，工具坐标系，由当前设置的坐标系决定。
 *
 * @param axis_index 附加轴组关节索引(0,1,2,...)
 * @param dir 运动方向，>0:正方向运动，<0:负方向运动，=0:不运动
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int addition_teach_move(int axis_index,int dir);

//-----------------------------------------------------------------------------------------------------------
/**
 * @brief 标定机器人轴的零点位置
 *
 * @param axis_index 机器人关节索引(0,1,2,...)
 * @return int 0: 成功, 其他：失败
 */
extern int calibrate_robot_zero_axis_position(int axis_index);

/**
 * @brief 标定附加轴组轴的零点位置
 *
 * @param axis_index 附加轴组关节索引(0,1,2,...)
 * @return int 0: 成功, 其他：失败
 */
extern int calibrate_addition_zero_axis_position(int axis_index);

/**
 * @brief 标定机器人轴力矩传感器标定
 *
 * @param axis_index 机器人关节索引(0,1,2,...)
 * @return int 0: 成功, 其他：失败
 */
extern int calibrate_robot_zero_axis_sensortorque(int axis_index);

/**
 * @brief 标定机器人零点位置
 *
 * @return int 0: 成功, 其他：失败
 */
extern int calibrate_robot_zero_position();

/**
 * @brief 标定附加轴组零点位置
 *
 * @return int 0: 成功, 其他：失败
 */
extern int calibrate_addition_zero_position();

/**
 * @brief 标定机器人关节力矩传感器标定
 *
 * @return int 0: 成功, 其他：失败
 */
extern int calibrate_robot_zero_sensortorque();

/**
 * @brief 采集工具标定数据
 *
 * @param data_index 标定数据索引
 * @return int 0: 成功, 其他：失败
 */
extern int calibration_tool_data(int data_index);

/**
 * @brief 工具标定
 *
 * @param robhold 是否手持工具，0:非手持；1:手持
 * @param data_num 标定点数目; 3:工具姿态标定（需进行位置标定）；4:工具位置标定；6:工具位置姿态标定
 * @param payload_name 工具所使用的负载名称
 * @param tool_name 标定工具名称
 * @param restul_tool 返回标定的工具
 * @return int >=0:标定误差*1000, <0：失败
 */
extern int calibration_tool(int robhold, int data_num, const char* payload_name, const char* tool_name, tool* restul_tool);

/**
 * @brief 采集用户坐标系标定数据
 *
 * @param add_name 关联的附加轴坐标系
 * @param data_index 标定数据索引
 * @return int 0: 成功, 其他：失败
 */
int calibration_user_data(const char* add_name, int data_index);

/**
 * @brief 用户坐标系标定
 *
 * @param user_name 标定用户坐标系名称
 * @param restul_user 返回标定的用户坐标系
 * @return int 0: 成功, 其他：失败
 */
extern int calibration_user( const char* user_name, robpose* restul_user);

/**
 * @brief 采集工件标定数据
 *
 * @param add_name 关联的附加轴坐标系
 * @param data_index 标定数据索引
 * @return int 0: 成功, 其他：失败
 */
extern int calibration_wobj_data(const char* add_name, int data_index);

/**
 * @brief 工件坐标系标定
 *
 * @param robhold 是否手持工件，0:非手持；1:手持
 * @param addhold 1：关联附加轴；0：不关联附加轴
 * @param ufmec 预留，未使用，设置为0
 * @param user_name 标定工件坐标系所需的用户坐标系名称
 * @param add_name 标定附加轴坐标系名称
 * @param wobj_name 标定工件坐标系名称
 * @param restul_wobj 返回标定的用户坐标系
 * @return int 0: 成功, 其他：失败
 */
extern int calibration_wobj(int robhold, int addhold,int ufmec, const char* user_name, const char* add_name, const char* wobj_name,wobj* restul_wobj);

/**
 * @brief 采集动力学辨识数据（耗时操作，需要用get_dynamics_identification_state()获取辨识状态，完成后方可操作机器人）
 *
 * @return int 0: 成功, 其他：失败
 */
extern int identification_dynamics_data();

/**
 * @brief 动力学辨识（耗时操作，需要用get_dynamics_identification_state()获取辨识状态，完成后方可操作机器人）
 *
 * @return int 0: 成功, 其他：失败
 */
extern int identification_dynamics();

/**
 * @brief 负载辨识，采集安装负载的运行数据（耗时操作，需要用get_dynamics_identification_state()获取辨识状态，完成后方可操作机器人）
 *
 * @return int 0: 成功, 其他：失败
 */
extern int identification_payload_dynamics_data();

/**
 * @brief 负载辨识，采集无负载的运行数据（耗时操作，需要用get_dynamics_identification_state()获取辨识状态，完成后方可操作机器人）
 *
 * @return int 0: 成功, 其他：失败
 */
extern int identification_nopayload_dynamics_data();

/**
 * @brief 负载辨识（耗时操作，需要用get_dynamics_identification_state()获取辨识状态，完成后方可操作机器人）
 *
 * @param payload_name 标定工件坐标系所需的用户坐标系名称
 * @return int 0: 成功, 其他：失败
 */
extern int identification_payload_dynamics(const char* payload_name);

/**
 * @brief 获取本体动力学或负载动力学辨识状态
 *
 * @return int 0:未运行辨识；1:采集动力学辨识数据；2:动力学辨识；3:采集负载辨识带负载数据；4:采集负载辨识无负载数据；5:负载辨识
 */
extern int get_dynamics_identification_state();

/**
 * @brief 负载辨识数据采集(需要开启传感器),移动机器人到3个不同的姿态，分别采集负载数
 *
 * @param torqueSensorName 传感器名称
 * @param data_index 数据索引（为0，1，2）
 * @return int 0: 成功, 其他：失败
 */
extern int identification_payload_dynamics_use_sensor_data(const char* torqueSensorName, int data_index);

/**
 * @brief 使用六维力传感器辨识负载
 *
 * @param torqueSensorName 传感器名称
 * @param payload_name 标定工件坐标系所需的用户坐标系名称
 * @return int 0: 成功, 其他：失败
 */
extern int identification_payload_dynamics_use_sensor(const char* torqueSensorName, const char* payload_name);

#ifdef __cplusplus
}
}
#endif


#endif /* ROBOT_TEACH_INTERFACE_H_ */
