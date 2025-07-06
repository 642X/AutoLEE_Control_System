/**
 * @file device_interface.h
 *
 * @brief 设备通信接口
 * @author hanbing
 * @version 11.1.0
 * @date 2020-04-08
 */
#ifndef DEVICE_INTERFACE_H_
#define DEVICE_INTERFACE_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 获取驱动设备名（创建时指定的名称）
 *
 * @param index 设备索引(0,1,2,...)
 * @param name 返回设备名，非NULL时，为提供的存储空间；可以为NULL,名称数据空间由内部提供
 * @return char* 创建时指定的设备名称地址
 */
extern const char* get_deviceName(int index,char* name);

/**
 * @brief 打印设备信息
 *
 * @param device_name 设备名称
 * @return int 0:成功; <0:错误
 */
extern int device_describe_printf(const char* device_name);

/**
 * @brief 获取设备控制周期
 *
 * @param device_name 设备名称
 * @return double 设备控制周期（s）
 */
extern double get_control_cycle(const char* device_name);

/**
 * @brief 设备中数字IO子设备数量
 *
 * @param device_name 设备名称
 * @return int >=0: 数字IO子设备数量; <0:错误
 */
extern int hasNumber_dio_device(const char* device_name);

/**
 * @brief 设备中模拟IO子设备数量
 *
 * @param device_name 设备名称
 * @return int >=0: 模拟IO子设备数量; <0:错误
 */
extern int hasNumber_aio_device(const char* device_name);

/**
 * @brief 设备中附加轴组子设备数量
 *
 * @param device_name 设备名称
 * @return int >=0: 附加轴组子设备数量; <0:错误
 */
extern int hasNumber_additionaxis_device(const char* device_name);

/**
 * @brief 设备中机器人组子设备数量
 *
 * @param device_name 设备名称
 * @return int >=0: 机器人组子设备数量; <0:错误
 */
extern int hasNumber_robot_device(const char* device_name);

/**
 * @brief 获取数字IO子设备名称
 *
 * @param device_name 设备名称
 * @return char* 子设备名称地址
 */
extern const char* get_name_dio_device(const char* device_name);

/**
 * @brief 获取模拟IO子设备名称
 *
 * @param device_name 设备名称
 * @return char* 子设备名称地址
 */
extern const char* get_name_aio_device(const char* device_name);

/**
 * @brief 获取逻辑数字IO子设备名称，用于电气逻辑控制
 *
 * @param device_name 设备名称
 * @return char* 子设备名称地址
 */
extern const char* get_name_lio_device(const char* device_name);

/**
 * @brief 获取附加轴子设备名称
 *
 * @param device_name 设备名称
 * @param index 子设备索引(0,1,2,...)
 * @return char* 子设备名称地址
 */
extern const char* get_name_additionaxis_device(const char* device_name, int index);

/**
 * @brief 获取附加轴子设备索引
 *
 * @param subdevice_name 子设备名称
 * @return int 子设备索引(0,1,2,...)
 */
extern int get_index_additionaxis_device(const char* subdevice_name);

/**
 * @brief 获取机器人子设备名称
 *
 * @param device_name 设备名称
 * @param index 子设备索引(0,1,2,...)
 * @return char* 子设备名称地址
 */
extern const char* get_name_robot_device(const char* device_name, int index);

/**
 * @brief 获取机器人子设备索引
 *
 * @param subdevice_name 子设备名称
 * @return int 子设备索引(0,1,2,...)
 */
extern int get_index_robot_device(const char* subdevice_name);

/**
* @brief 获取设备数据
*
* @param device_name 设备名称
* @param data_name 数据名称
* @param data  返回获取的数据
* @param size   获取数据字节数
*
* @return int 0:成功；其他失败
*/
extern int get_custom_device_data(const char* device_name, const char* data_name, void* data, int size);

/**
* @brief 设置设备数据
*
* @param device_name 设备名称
* @param data_name 数据名称
* @param data  设置数据
* @param size   设置数据字节数
*
* @return int 0:成功；其他失败
*/
extern int set_custom_device_data(const char* device_name, const char* data_name, void* data, int size);

/**
 * @brief 判断是否为数字IO设备
 *
 * @param subdevice_name 子设备名称
 * @return int 1:是；0:否
 */
extern int is_dio_device(const char* subdevice_name);

/**
 * @brief 判断是否为模拟IO设备
 *
 * @param subdevice_name 子设备名称
 * @return int 1:是；0:否
 */
extern int is_aio_device(const char* subdevice_name);


/**
 * @brief 判断是否为逻辑IO设备
 *
 * @param subdevice_name 子设备名称
 * @return int 1:是；0:否
 */
extern int is_lio_device(const char* subdevice_name);

/**
 * @brief 判断是否为附加轴
 *
 * @param subdevice_name 子设备名称
 * @return int 1:是；0:否
 */
extern int is_additionaxis_device(const char* subdevice_name);

/**
 * @brief 判断是否为机器人
 *
 * @param subdevice_name 子设备名称
 * @return int 1:是；0:否
 */
extern int is_robot_device(const char* subdevice_name);

/**
 * @brief 轴使能
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int axis_power_on(const char* subdevice_name,int axis_ID);

/**
 * @brief 轴去使能
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int axis_power_off(const char* subdevice_name,int axis_ID);

/**
 * @brief 设置轴模式
 *
 * @param subdevice_name 子设备名称
 * @param mode 模式（8:同步位置模式；9:同步速度模式；10:同步力矩模式）
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_mode(const char* subdevice_name,signed char mode,int axis_ID);

/**
 * @brief 设置轴控制命令
 *
 * @param subdevice_name 子设备名称
 * @param control 控制命令
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_control(const char* subdevice_name,unsigned short control,int axis_ID);

/**
 * @brief 获取轴状态
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return short >=0:状态; <0:错误
 */
extern unsigned short get_axis_status(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴编码器位置
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 编码器位置
 */
extern int get_axis_position(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴目标编码器位置
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 编码器位置
 */
extern int get_axis_target_position(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴编码器速度
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 编码器速度（count）
 */
extern int get_axis_velocity(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴目标编码器速度
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 编码器速度（count/s）
 */
extern int get_axis_target_velocity(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴力矩
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return short 力矩(一般为额定电流千分比)
 */
extern short get_axis_torque(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴第二路编码器位置
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 编码器位置
 */
extern int get_axis_position2(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴第二路编码器速度
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 编码器速度（count）
 */
extern int get_axis_velocity2(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴关节力矩传感器数据
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return short 力矩
 */
extern short get_axis_sensor_torque(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴模式
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return signed char 模式（8:同步位置模式；9:同步速度模式；10:同步力矩模式）
 */
extern signed char get_axis_mode(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴错误代码
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return short 设备状态错误代码
 */
extern short get_axis_ErrorCode(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴跟踪误差
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 跟踪误差
 */
extern int get_axis_FollowingErrorActualValue(const char* subdevice_name,int axis_ID);

/**
 * @brief 设置轴位置
 *
 * @param subdevice_name 子设备名称
 * @param pos 位置(count)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_position(const char* subdevice_name,int pos,int axis_ID);

/**
 * @brief 设置轴速度
 *
 * @param subdevice_name 子设备名称
 * @param vel 速度(count/s)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_velocity(const char* subdevice_name,int vel,int axis_ID);

/**
 * @brief 设置轴力矩
 *
 * @param subdevice_name 子设备名称
 * @param tor 力矩(一般为额定电流千分比)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_torque(const char* subdevice_name,short tor,int axis_ID);

/**
 * @brief 设置轴速度补偿
 *
 * @param subdevice_name 子设备名称
 * @param veloffset 补偿速度
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_velocityOffset(const char* subdevice_name,int veloffset,int axis_ID);

/**
 * @brief 设置轴力矩补偿
 *
 * @param subdevice_name 子设备名称
 * @param toroffset 补偿力矩
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_torqueOffset(const char* subdevice_name,short toroffset,int axis_ID);

/**
 * @brief 设置轴最大力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torlimit 力矩限制
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_torqueMaxLimit(const char* subdevice_name,short torlimit,int axis_ID);

/**
 * @brief 设置轴最小力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torlimit 力矩限制
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int set_axis_torqueMinLimit(const char* subdevice_name,short torlimit,int axis_ID);

/**
 * @brief 设置数字输出
 *
 * @param subdevice_name 子设备名称
 * @param id_index 数字输出索引(0,1,2,3,...)
 * @param flag 输出值: 0,1
 * @return int 0:成功; <0:错误
 */
extern int set_do(const char* subdevice_name, int id_index,int flag);

/**
 * @brief 设置逻辑数字输出
 *
 * @param subdevice_name 子设备名称
 * @param id_index 数字输出索引(0,1,2,3,...)
 * @param flag 输出值: 0,1
 * @return int 0:成功; <0:错误
 */
extern int set_lo(const char* subdevice_name, int id_index,int flag);

/**
 * @brief 获取数字输入
 *
 * @param subdevice_name 子设备名称
 * @param id_index 数字输入索引(0,1,2,3,...)
 * @return int 数字输入值，0,1
 */
extern int get_di(const char* subdevice_name,int id_index);

/**
 * @brief 获取逻辑数字输入
 *
 * @param subdevice_name 子设备名称
 * @param id_index 数字输入索引(0,1,2,3,...)
 * @return int 数字输入值，0,1
 */
extern int get_li(const char* subdevice_name,int id_index);

/**
 * @brief 设置模拟输出
 *
 * @param subdevice_name 子设备名称
 * @param id_index 模拟输出索引(0,1,2,3,...)
 * @param ao 输出值
 * @return int 0:成功; <0:错误
 */
extern int set_ao(const char* subdevice_name, int id_index, short ao);

/**
 * @brief 获取模拟输入
 *
 * @param subdevice_name 子设备名称
 * @param id_index 模拟输入索引(0,1,2,3,...)
 * @return int 模拟输入值
 */
extern short get_ai(const char* subdevice_name,int id_index);

/**
 * @brief 获取模拟输出数量
 *
 * @param subdevice_name 子设备名称
 * @return int 模拟输出数量
 */
extern int get_ao_num(const char* subdevice_name);

/**
 * @brief 获取模拟输入数量
 *
 * @param subdevice_name 子设备名称
 * @return int 模拟输入数量
 */
extern int get_ai_num(const char* subdevice_name);

/**
 * @brief 获取轴组中轴的数量
 *
 * @param subdevice_name 子设备名称
 * @return int 轴的数量
 */
extern int get_group_dof(const char* subdevice_name);

/**
 * @brief 轴组使能
 *
 * @param subdevice_name 子设备名称
 * @return 0:成功; <0:错误
 */
extern int group_power_on(const char* subdevice_name);

/**
 * @brief 轴组去使能
 *
 * @param subdevice_name 子设备名称
 * @return 0:成功; <0:错误
 */
extern int group_power_off(const char* subdevice_name);

/**
 * @brief 设置轴组控制字
 *
 * @param subdevice_name 子设备名称
 * @param control 控制字
 * @return int 0:成功; <0:错误
 */
extern int set_group_control(const char* subdevice_name, unsigned short* control);

/**
 * @brief 设置轴组模式
 *
 * @param subdevice_name 子设备名称
 * @param mode 模式（8:同步位置模式；9:同步速度模式；10:同步力矩模式）
 * @return 0:成功; <0:错误
 */
extern int set_group_mode(const char* subdevice_name, signed char* mode);

/**
 * @brief 获取轴组模式
 *
 * @param subdevice_name 子设备名称
 * @param mode 返回轴组模式（8:同步位置模式；9:同步速度模式；10:同步力矩模式）
 * @return 0:成功; <0:错误
 */
extern int get_group_mode(const char* subdevice_name,signed char* mode);

/**
 * @brief 获取轴组编码器位置
 *
 * @param subdevice_name 子设备名称
 * @param pos 返回编码器位置(count)
 * @return 0:成功; <0:错误
 */
extern int get_group_position(const char* subdevice_name,int* pos);

/**
 * @brief 获取轴组目标编码器位置
 *
 * @param subdevice_name 子设备名称
 * @param pos 返回编码器位置(count)
 * @return 0:成功; <0:错误
 */
extern int get_group_target_position(const char* subdevice_name,int* pos);

/**
 * @brief 获取轴组编码器速度
 *
 * @param subdevice_name 子设备名称
 * @param vel 返回编码器速度(count/s)
 * @return 0:成功; <0:错误
 */
extern int get_group_velocity(const char* subdevice_name,int* vel);

/**
 * @brief 获取轴组目标编码器速度
 *
 * @param subdevice_name 子设备名称
 * @param vel 返回编码器速度(count/s)
 * @return 0:成功; <0:错误
 */
extern int get_group_target_velocity(const char* subdevice_name,int* vel);

/**
 * @brief 获取轴组力矩
 *
 * @param subdevice_name 子设备名称
 * @param tor 返回轴组力矩(一般为额定电流千分比)
 * @return 0:成功; <0:错误
 */
extern int get_group_torque(const char* subdevice_name,short* tor);

/**
 * @brief 获取轴组第二路编码器位置
 *
 * @param subdevice_name 子设备名称
 * @param pos 返回编码器位置(count)
 * @return 0:成功; <0:错误
 */
extern int get_group_position2(const char* subdevice_name,int* pos);

/**
 * @brief 获取轴组第二路编码器速度
 *
 * @param subdevice_name 子设备名称
 * @param vel 返回编码器速度(count/s)
 * @return 0:成功; <0:错误
 */
extern int get_group_velocity2(const char* subdevice_name,int* vel);

/**
 * @brief 获取轴组关节力矩传感器数据
 *
 * @param subdevice_name 子设备名称
 * @param tor 返回传感器力矩
 * @return 0:成功; <0:错误
 */
extern int get_group_sensor_torque(const char* subdevice_name,short* tor);

/**
 * @brief 获取轴组错误代码
 *
 * @param subdevice_name 子设备名称
 * @param err 返回错误代码
 * @return 0:成功; <0:错误
 */
extern int get_group_ErrorCode(const char* subdevice_name, short* err);

/**
 * @brief 获取轴组跟踪误差
 *
 * @param subdevice_name 子设备名称
 * @param err 返回跟踪误差
 * @return 0:成功; <0:错误
 */
extern int get_group_FollowingErrorActualValue(const char* subdevice_name, int* err);

/**
 * @brief 设置轴组编码器位置
 *
 * @param subdevice_name 子设备名称
 * @param pos 编码器位置(count)
 * @return 0:成功; <0:错误
 */
extern int set_group_position(const char* subdevice_name,int* pos);

/**
 * @brief 设置轴组编码器速度
 *
 * @param subdevice_name 子设备名称
 * @param vel 编码器速度(count)
 * @return 0:成功; <0:错误
 */
extern int set_group_velocity(const char* subdevice_name,int* vel);

/**
 * @brief 设置轴组力矩
 *
 * @param subdevice_name 子设备名称
 * @param tor 力矩
 * @return 0:成功; <0:错误
 */
extern int set_group_torque(const char* subdevice_name,short* tor);

/**
 * @brief 设置轴组速度补偿
 *
 * @param subdevice_name 子设备名称
 * @param veloffset 补偿速度
 * @return int 0:成功; <0:错误
 */
extern int set_group_velocityOffset(const char* subdevice_name,int* veloffset);

/**
 * @brief 设置轴组力矩补偿
 *
 * @param subdevice_name 子设备名称
 * @param toroffset 补偿力矩
 * @return int 0:成功; <0:错误
 */
extern int set_group_torqueOffset(const char* subdevice_name,short* toroffset);

/**
 * @brief 设置轴组最大力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torlimit 力矩限制
 * @return int 0:成功; <0:错误
 */
extern int set_group_torqueMaxLimit(const char* subdevice_name,short* torlimit);

/**
 * @brief 设置轴组最小力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torlimit 力矩限制
 * @return int 0:成功; <0:错误
 */
extern int set_group_torqueMinLimit(const char* subdevice_name,short* torlimit);

/**
 * @brief 获取轴关节位置
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return double 关节位置(rad 或 mm)
 */
extern double GetAxisPosition(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴关节目标位置
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return double 关节目标位置(rad 或 mm)
 */
extern double GetAxisTargetPosition(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴关节速度
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return double 关节速度(rad/s 或 mm/s)
 */
extern double GetAxisVelocity(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴关节目标速度
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return double 关节目标速度(rad/s 或 mm/s)
 */
extern double GetAxisTargetVelocity(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴关节力矩
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return double 关节关节力矩(Nmm 或mN)
 */
extern double GetAxisTorque(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴第二路编码器获取的关节位置
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return double 关节位置(rad 或 mm)
 */
extern double GetAxisPosition2(const char* subdevice_name,int axis_ID);

/**
 * @brief 获取轴关节力矩传感器
 *
 * @param subdevice_name 子设备名称
 * @param axis_ID 轴地址(1,2,3,...)
 * @return double 关节传感器力矩(Nmm 或mN)
 */
extern double GetAxisSensorTorque(const char *subdevice_name, int axis_ID);

/**
 * @brief 设置轴关节位置
 *
 * @param subdevice_name 子设备名称
 * @param position 目标关节位置(rad 或 mm)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAxisPosition(const char* subdevice_name, double position, int axis_ID);

/**
 * @brief 设置轴关节速度
 *
 * @param subdevice_name 子设备名称
 * @param velocity 目标关节速度(rad/s 或 mm/s)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAxisVelocity(const char* subdevice_name, double velocity, int axis_ID);

/**
 * @brief 设置轴速度补偿
 *
 * @param subdevice_name 子设备名称
 * @param velocity_offset 补偿速度(rad/s或mm/s)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAxisVelocityOffset(const char* subdevice_name, double velocity_offset, int axis_ID);

/**
 * @brief 设置轴关节力矩
 *
 * @param subdevice_name 子设备名称
 * @param torque 目标关节力(Nmm 或 mN)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAxisTorque(const char* subdevice_name, double torque, int axis_ID);

/**
 * @brief 设置轴力矩补偿
 *
 * @param subdevice_name 子设备名称
 * @param torque_offset 补偿速度(Nmm 或 mN)
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAxisTorqueOffset(const char* subdevice_name, double torque_offset, int axis_ID);

/**
 * @brief 设置轴最大力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torque_limit 力矩限制
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAxisTorqueMaxLimit(const char* subdevice_name, double torque_limit, int axis_ID);

/**
 * @brief 设置轴最小力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torque_limit 力矩限制
 * @param axis_ID 轴地址(1,2,3,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAxisTorqueMinLimit(const char* subdevice_name, double torque_limit, int axis_ID);

/**
 * @brief 获取模拟输入
 *
 * @param subdevice_name 子设备名称
 * @param id_index 索引(0,1,2,...)
 * @return double 模拟输入(物理量纲)
 */
extern double GetAI(const char* subdevice_name,int id_index);

/**
 * @brief 设置模拟输出
 *
 * @param subdevice_name 子设备名称
 * @param ao 模拟输出(物理量纲)
 * @param id_index 索引(0,1,2,...)
 * @return int 0:成功; <0:错误
 */
extern int SetAO(const char* subdevice_name, double ao, int id_index);

/**
 * @brief 获取轴组关节位置
 *
 * @param subdevice_name 子设备名称
 * @param position 关节位置(rad 或 mm)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupPosition(const char* subdevice_name, double* position);

/**
 * @brief 获取轴组目标关节位置
 *
 * @param subdevice_name 子设备名称
 * @param position 目标关节位置(rad 或 mm)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupTargetPosition(const char* subdevice_name, double* position);

/**
 * @brief 获取轴组关节速度
 *
 * @param subdevice_name 子设备名称
 * @param velocity 关节速度(rad /s或 mm/s)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupVelocity(const char* subdevice_name, double* velocity);

/**
 * @brief 获取轴组目标关节速度
 *
 * @param subdevice_name 子设备名称
 * @param velocity 目标关节速度(rad /s或 mm/s)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupTargetVelocity(const char* subdevice_name, double* velocity);

/**
 * @brief 获取轴组关节力矩
 *
 * @param subdevice_name 子设备名称
 * @param torque 关节力矩(Nmm或 mN)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupTorque(const char* subdevice_name, double* torque);

/**
 * @brief 获取轴组第二路采集关节位置
 *
 * @param subdevice_name 子设备名称
 * @param position 关节位置(rad 或 mm)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupPosition2(const char* subdevice_name, double* position);

/**
 * @brief 获取轴组关节力矩传感器
 *
 * @param subdevice_name 子设备名称
 * @param torque 关节传感器力矩(Nm或 N)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupSensorTorque(const char *subdevice_name, double *torque);

/**
 * @brief 设置轴组关节位置
 *
 * @param subdevice_name 子设备名称
 * @param position 关节位置(rad 或 mm)
 * @return int 0:成功; <0:错误
 */
extern int SetGroupPosition(const char* subdevice_name, double* position);

/**
 * @brief 设置轴组关节速度
 *
 * @param subdevice_name 子设备名称
 * @param velocity 关节速度(rad/s 或 mm/s)
 * @return int 0:成功; <0:错误
 */
extern int SetGroupVelocity(const char* subdevice_name, double* velocity);

/**
 * @brief 设置轴组速度补偿
 *
 * @param subdevice_name 子设备名称
 * @param velocity_offset 补偿速度(rad/s 或 m/s)
 * @return int 0:成功; <0:错误
 */
extern int SetGroupVelocityOffset(const char* subdevice_name, double* velocity_offset);

/**
 * @brief 设置轴组关节力矩
 *
 * @param subdevice_name 子设备名称
 * @param torque 关节力矩(Nm或N)
 * @return int 0:成功; <0:错误
 */
extern int SetGroupTorque(const char* subdevice_name, double* torque);

/**
 * @brief 设置轴组力矩补偿
 *
 * @param subdevice_name 子设备名称
 * @param torque_offset 补偿速度(Nm或N)
 * @return int 0:成功; <0:错误
 */
extern int SetGroupTorqueOffset(const char* subdevice_name, double* torque_offset);

/**
 * @brief 设置轴组最大力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torque_limit 力矩限制
 * @return int 0:成功; <0:错误
 */
extern int SetGroupTorqueMaxLimit(const char* subdevice_name, double* torque_limit);

/**
 * @brief 设置轴组最小力矩限制
 *
 * @param subdevice_name 子设备名称
 * @param torque_limit 力矩限制
 * @return int 0:成功; <0:错误
 */
extern int SetGroupTorqueMinLimit(const char* subdevice_name, double* torque_limit);

/**
 * @brief 开启轴组控制目标输入
 * 开启后通过SetGroupCtrlTarget()设置的目标不会直接发送给设备，而是缓存到中间数据区，
 * 可以使用GetGroupCtrlTarget()和GetGroupCtrlTargetOnce()获取该目标数据，
 *
 * @param subdevice_name 子设备名称
 * @return int 0:成功; <0:错误
 */
extern int OpenCtrlInput(const char* subdevice_name);

/**
 * @brief 关闭轴组控制目标输入
 *	关闭后通过SetGroupCtrlTarget()设置的目标会直接发送给设备，GetGroupCtrlTarget()和GetGroupCtrlTargetOnce()获取的数据为由SetGroupCtrlTarget()设置的目标数据。
 *
 * @param subdevice_name 子设备名称
 * @return int 0:成功; <0:错误
 */
extern int CloseCtrlInput(const char* subdevice_name);

/**
 * @brief 是否开启控制输入

 * @param subdevice_name 子设备名称
 * @return int 1:是; 0:否
 */
extern int IsOpenCtrlInput(const char *subdevice_name);

/**
 * @brief 设置轴组控制目标
 * 当使用OpenCtrlInput()开启控制输入后，数据发送到中间缓冲区，否则这接发送给设备
 *
 * @param subdevice_name 子设备名称
 * @param position 期望关节位置(rad 或 mm)
 * @param velocity 期望关节速度(rad/s 或 mm/s)
 * @param acceleration 期望关节加速度(rad/s^2 或 mm/s^2)
 * @return int 0:成功; <0:错误
 */
extern int SetGroupCtrlTarget(const char* subdevice_name, double* position, double* velocity, double* acceleration);

/**
 * @brief 获取轴组控制目标
 *
 * @param subdevice_name 子设备名称
 * @param position 期望关节位置(rad 或 mm)
 * @param velocity 期望关节速度(rad/s 或 mm/s)
 * @param acceleration 期望关节加速度(rad/s^2 或 mm/s^2)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupCtrlTarget(const char* subdevice_name, double* position, double* velocity, double* acceleration);

/**
 * @brief 设置轴组位置控制目标
 * 当使用OpenCtrlInput()开启控制输入后，数据发送到中间缓冲区，否则这接发送给设备
 *
 * @param subdevice_name 子设备名称
 * @param position 期望关节位置(rad 或 mm)
 * @return int 0:成功; <0:错误
 */
extern int SetGroupCtrlTargetPosition(const char* subdevice_name, double* position);

/**
 * @brief 获取轴组位置控制目标
 *
 * @param subdevice_name 子设备名称
 * @param position 期望关节位置(rad 或 mm)
 * @return int 0:成功; <0:错误
 */
extern int GetGroupCtrlTargetPosition(const char* subdevice_name, double* position);

#ifdef __cplusplus
}
}
#endif


#endif /* DEVICE_INTERFACE_H_ */
