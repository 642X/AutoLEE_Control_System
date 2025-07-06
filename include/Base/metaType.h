/**
 * @file metaType.h
 *
 * @brief  数据类型
 * @author hanbing
 * @version 11.4.0
 * @date 2020-9-17
 *
 */
#ifndef METATYPE_H_
#define METATYPE_H_

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/*-------------------------------------------------------------------------*/
/**
 * @brief 机器人运动状态
 */
/*-------------------------------------------------------------------------*/

typedef enum MoveState{
	_move_error=-1,///<错误状态
	_move_finish=0,///<允许开始新的运动
	_move_run_joint,///<正在进行关节空间运动
	_move_run_line,///<正在进行直线运动
	_move_run_circle,///<正在进行圆弧运动
	_move_run_helical,///<正在进行螺旋线运动
	_move_run_bspline,///<正在进行样条曲线运动
	_move_run_zone,///<正在进行转弯区运动
	_move_stop,///<驱动被强制停止，需要恢复到_move_finish才可重新运动
	_move_run_zone_finish,///<转弯区完成状态
	_move_egm,///<正在进行外部引导运动
	_move_direct_teach,///<正在进行拖动运动
	_move_run_dynamic,///<正在进行动态轨迹运动
	_move_run_polynomial_segment,///<正在进行多项式轨迹运动
	_move_dynamic_test///<正在进行动力学测试运动
}MoveState;

/*-------------------------------------------------------------------------*/
/**
 * @brief 返回状态
 */
/*-------------------------------------------------------------------------*/
#define SUCCESS 0//!< 返回成功
#define ERR_ROBOTMODE -101 //!< 机器人模式错误
#define ERR_FUNNCTIONBUSY -102 //!< 系统忙，正在运行功能模块
#define ERR_UNINITIALIZEDDATA -103 //!< 使用的数据异常
#define ERR_THREADCREATEFAILURE -104 //!< 线程创建失败
#define ERR_TARGETDATAHELD -105 //!< 目标数据区被占用
#define ERR_FORCE_SENSOR -106 //!< 力矩传感器数据获取或转换失败
#define ERR_INVERSEKINEMATICS -107 //!< 逆运动学求解错误
#define ERR_TARGETJOINTJUMP -108 //!< 目标关节位置跳跃
#define ERR_MOVESTATE -109 //!< 运动状态错误
#define ERR_NUPOWER -110 //!< 未使能
#define ERR_MOVING -111 //!< 机器人正在移动
#define ERR_ROBOTINDEX -112 //!< 机器人索引错误
#define ERR_ROBOTJOINTLIMIT -113 //!< 关节限位
#define ERR_ROBOTJOINTVELOCITYLIMIT -114//!< 关节速度限制
#define ERR_ROBOTSINGULAR -115 //!< 机器人奇异
#define ERR_KINEMATICS -116 //!< 运动学求解错误
#define ERR_FRAMETRANSFORM -117 //!< 坐标变换错误
#define ERR_POWERON -118 //!< 使能失败
#define ERR_POWEROFF -119 //!< 下使能失败
#define ERR_ROBOTPROJECT -120 //!< 机器人项目失败
#define ERR_TEACHMOVE -121 //!< 示教运动失败
#define ERR_GRIPCONTROL -122 //!< 夹抓控制失败
#define ERR_MOVESPEEDLIMIT -123 //!< 笛卡尔移动速度超限
#define ERR_FORCESENSORLIMIT -124 //!< 六维力传感器超限
#define ERR_UNDEFINEDFUNCTION -125 //!< 未定义功能实现
#define ERR_DEVICEINEXISTENCE -126 //!< 操作设备不存在
#define ERR_SAVEDATA -127 //!< 保存数据失败
#define ERR_INDEXLIMIT -128 //!< 索引超出限制
#define ERR_TOOLWOBJ -129 //!< 工具或工件使用错误
#define ERR_SPATIALCONSTRAINT  -130 //!< 笛卡尔空间限制
#define ERR_GRIPINDEX -131 //!< 夹爪索引错误
#define ERR_CHARTOOLONG -132 //!< 操作字符过长
#define ERR_FORCESENSORINDEX -133 //!< 力传感器索引错误
#define ERR_INPUTPARAMETER -134 //!< 输入参数错误
#define ERR_VELOCITYPLANNING -135 //!< 速度规划错误
#define ERR_VELOCITYRENEWPLANNING -136 //!< 更新速度规划错误
#define ERR_DATANAMEEXIST -137 //!< 数据名称已存在
#define ERR_PATHPLANNING -138 //!< 路径规划失败
#define ERR_ZONECONDITION -139 //!< 转弯区规划条件错误
#define ERR_READDATA -140 //!< 读数据失败
#define ERR_IMPORTLICENSE -141 //!< 导入数据失败
#define ERR_HARDWAREINFORMATION -142 //!< 获取硬件信息失败
#define ERR_BUS_CONFIGURE -143 //!< BusConfig.yaml加载失败
#define ERR_GLOBAL_CONFIGURE -144 //!< GlobalConfig.yaml加载失败
#define ERR_EXTERN_DEVICE_CONFIGURE -145 //!< ExternDeviceConfig.yaml加载失败
#define ERR_DEVICE_CONFIGURE -146 //!< DeviceConfig.yaml加载失败
#define ERR_CONFIGURE_DATA -147 //!< 配置文件版本不匹配或控制对象不一致
#define ERR_INITBUSDATA -148 //!< 总线数据区错误
#define ERR_INITBUSCONFIG -149 //!< 总线数据配置错误
#define ERR_INITROBOT -150 //!< 机器人初始化失败
#define ERR_MALLOCAL -151 //!< malloc 失败
#define ERR_DYNMAICS_IDENTIFY -152 //!< 动力学辨识错误
#define ERR_PAYLOAD_DYNMAICS_IDENTIFY -153 //!< 动力学辨识错误
#define ERR_DATAFILEPATH -154 //!< 数据路径错误
#define ERR_EXTERNALDEVICE -155 //!< 外部设备操作失败
#define ERR_STACKTECHNOLOGYDATA -156 //!< 码垛数据异常
#define ERR_STACKTECHNOLOGYCONFIGDATA -157 //!< 码垛配置数据异常
#define ERR_DATADIMENSION -158 //!< 数据维度不符
#define ERR_DYNAMICS -159 //!< 数据维度不符
#define ERR_PROTOCOL -160 //!< 协议错误
#define ERR_SOCKET -161 //!< socket 失败
#define ERR_ADDITIONSERER -162 //!< 附加轴服务启动失败
#define ERR_DYNMAICS -163 //!< 动力学求解错误
#define ERR_ROBOTJOINTTORQUELIMIT -164 //!< 关节力限制


/*-------------------------------------------------------------------------*/
/**
 * @brief 约束属性
 */
/*-------------------------------------------------------------------------*/
typedef enum{
	_constraint_noallow=0,///<完全不允许区域
	_constraint_slowallow,///<不允许区域，但允许缓慢移动
	_constraint_allow,///<完全允许区域，其他区域完全不允许
	_constraint_allowslow///<完全允许区域，其他区域允许缓慢移动
}constraint_property;

/*-------------------------------------------------------------------------*/
/**
 * @brief 约束状态
 */
/*-------------------------------------------------------------------------*/
typedef enum{
	_constraint_allowstate=0,///<处于允许区域
	_constraint_slowstate,///<处于不允许区域，但允许缓慢移动
	_constraint_noallowstate,///<处于完全不允许区域
}constraint_state;

#ifdef __cplusplus
}
}
#endif

#endif /* METATYPE_H_ */
