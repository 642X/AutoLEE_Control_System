/**
 * @file robotStruct.h
 *
 * @brief  机器人运动指令数据类型
 * @author hanbing
 * @version 11.3.0
 * @date 2020-04-08
 *
 */

#ifndef ROBOTSTRUCT_H_
#define ROBOTSTRUCT_H_

/*---------------------------- Includes ------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif


/*---------------------------- Defines -------------------------------------*/
#define ROBOT_MAX_NUM 10 //!<机器人最大数量
#define ROBOT_MAX_DOF 10 //!<最大自由度
#define MAX_CHAR_NUM 300 //!<内部字符最大长度
#define MAX_BSPLINE_NUM 200 //!<最大样条点数目
#define ROBOT_DATA_NAME_LEN_MAX 50 //!<数据名称字符最大长度
#define NULL3 NULL,NULL,NULL //!<NULL,NULL,NULL
#define NULL2 NULL,NULL //!<NULL,NULL
typedef unsigned char byte; //!<byte


/*-------------------------------------------------------------------------*/
/**
  @brief	控制器机器人类型

	选择所要控制的机器人，默认为_other，从配置文件读取机器人类型
 */
/*-------------------------------------------------------------------------*/
typedef enum{
	_ethercat=0,///< EtherCAT直接驱动驱动器的控制方式
	_canopen,///< CanOpen直接驱动驱动器的控制方式
	_general///< 商用机器人通用模式
}controller_mode;

/*-------------------------------------------------------------------------*/
/**
 * @brief	命令行数据结构

	保存命令行数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	char* path; /**< 配置文件所在路径 */
	controller_mode mode; /**< 机器人类型 */
	int dete_background;/**< 是否开启后台关节状态异常检测 */
	int EtherCATonly;/**< 是否仅启动EtherCAT通信，关闭其他运动控制功能 */
	int server_modbus;/**< 是否启动modbus服务 */
	int port;/**< 示教服务端口号 */
	int iscopy;/**< 运行程序是否为副本 */
	char* data_namespace; /**< 共享数据名字空间 */
}command_arg;

/*-------------------------------------------------------------------------*/
/**
  @brief	负载数据

	描述负载动力学
 */
/*-------------------------------------------------------------------------*/
typedef struct PayLoad{
	double m;/**< 连杆质量(t=1000kg)*/
	double cm[3];/**< 相对末端连杆坐标系的质心(mm)*/
	double I[3][3];/**< 相对质心坐标系的惯性张量(t*mm^2=0.001kg*m^2)*/
	double I2[3][3];/**< 相对末端连杆坐标系的惯性张量(t*mm^2=0.001kg*m^2)*/
}PayLoad;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，关节数据

	存储机器人关节数据
 */
/*-------------------------------------------------------------------------*/
typedef struct robjoint{
	double angle[ROBOT_MAX_DOF];/**< 机器人关节角度，单位rad */
	int dof;/**< 机器人自由度*/
}robjoint;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，笛卡尔位置及姿态数据

	存储机器人位姿数据
 */
/*-------------------------------------------------------------------------*/
typedef struct robpose{
	double xyz[3];/**< 位置，单位mm*/
	double kps[3];/**< 姿态，单位rad*/
}robpose;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，速度数据

	存储机器人关节速度和笛卡尔速度数据
 */
/*-------------------------------------------------------------------------*/
typedef struct speed{
	double per[ROBOT_MAX_DOF];/**< 关节速度，per_flag=0: 最大速度的百分比0~1; per_flag=1: 速度，单位rad/s; per_flag=2: 时间，单位s;*/
	int per_flag;/**< per 含义，0:百分比；1:速度；2:时间; -1:不是用*/
	double tcp;/**< 笛卡尔移动速度，tcp_flag=0: 最大速度的百分比0~1; tcp_flag=1: 速度，单位mm/s; tcp_flag=2: 时间，单位s;*/
	double orl;/**< 笛卡尔转动速度，tcp_flag=0: 最大速度的百分比0~1; tcp_flag=1: 速度，单位rad/s; tcp_flag=2: 时间，单位s;*/
	int tcp_flag;/**< tcp 含义，0:百分比；1:速度；2:时间; -1:不是用*/

	int dof;/**< 机器人自由度*/
}speed;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，转弯区数据

	应用机器人连续两条指令之间的过度曲线，存储机器人转弯区数据
 */
/*-------------------------------------------------------------------------*/
typedef struct zone{
	int zone_flag;/**< 转弯区大小，zone_size=0:不使用转弯区；zone_size=1:运动总长度的百分比0~1；zone_size=2:转弯大小，移动m,转动rad*/
	double zone_size;/**< zone_flag含义，0:不使用；1:百分比；2:具体大小*/
}zone;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，工具数据

	描述机器人所使用的工具
 */
/*-------------------------------------------------------------------------*/
typedef struct tool{
	int robhold;/**< 是否手持工具，0:非手持；1:手持*/
	robpose tframe;/**< 工具坐标系相对法兰坐标系的位置及姿态描述*/
	PayLoad payload;/**< 负载*/
}tool;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，工件标系数据

	描述机器人加工工件的坐标系，与工具配对使用
 */
/*-------------------------------------------------------------------------*/
typedef struct wobj{
	int robhold;///< 工件是否在机器人上，0:不在机器人上；1:在机器人上
	int addhold;///< 是否使用附加轴组
	int ufmec;///< 预留，未使用
	robpose uframe;///< 用户坐标在基坐标系下的描述
	robpose oframe;///< 工件坐标在用户坐标系下的描述
	robpose aframe;///< 附加轴组坐标系下的描述
}wobj;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，关节数据(私有数据)

	存储机器人关节数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	double angle[ROBOT_MAX_DOF];///< 机器人关节角度，单位rad
	int dof;///< 机器人自由度
}ROBJOINT;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，笛卡尔位置及姿态数据(私有数据)

	存储机器人位姿数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	double xyzkps[6];///< 位置(0,1,2)及姿态(3,4,5)，单位mm,rad
}ROBPOSE;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，速度数据(私有数据)

	存储机器人关节速度和笛卡尔速度数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	double per[ROBOT_MAX_DOF];///< 关节速度，per_flag=0: 最大速度的百分比0~1; per_flag=1: 速度，单位rad/s; per_flag=2: 时间，单位s;
	int per_flag;///< per 含义，0:百分比；1:速度；2:时间; -1:不是用
	double tcp;///< 笛卡尔移动速度，tcp_flag=0: 最大速度的百分比0~1; tcp_flag=1: 速度，单位mm/s; tcp_flag=2: 时间，单位s;
	double orl;///< 笛卡尔转动速度，tcp_flag=0: 最大速度的百分比0~1; tcp_flag=1: 速度，单位rad/s; tcp_flag=2: 时间，单位s;
	int tcp_flag;///< tcp 含义，0:百分比；1:速度；2:时间; -1:不是用

	int dof;///< 机器人自由度
}SPEED;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，转弯区数据(私有数据)

	应用机器人连续两条指令之间的过度曲线，存储机器人转弯区数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	int zone_flag;///< 转弯区大小，zone_size=0:不使用转弯区；zone_size=1:运动总长度的百分比0~1；zone_size=2:转弯大小，移动mm,转动rad
	double zone_size;///< zone_flag含义，0:不使用；1:百分比；2:具体大小
}ZONE;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，工具数据(私有数据)

	描述机器人所使用的工具
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	int robhold;///< 是否手持工具，0:非手持；1:手持
	double toolframe[6];///< 工具坐标系相对法兰坐标系的位置及姿态描述
	PayLoad payload;///< 负载
}TOOL;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入，工件标系数据(私有数据)

	描述机器人加工工件的坐标系，与工具配对使用
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	int robhold;///< 工件是否在机器人上，0:不在机器人上；1:在机器人上
	int addhold;///< 是否使用附加轴组
	int ufmec;///< 预留，未使用
	double userframe[6];///< 用户坐标在基坐标系下的描述
	double workframe[6];///< 工件坐标在用户坐标系下的描述
	double addframe[6];///< 附加轴组坐标系下的描述
}WOBJ;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动指令输入数据类型

	描述输入数据的类型
 */
/*-------------------------------------------------------------------------*/
typedef enum robdatatype{
	_robjoint,///< 关节数据
	_robpose,///< 笛卡尔数据
	_speed,///< 速度数据
	_zone,///< 转弯区数据
	_tool,///< 工具数据
	_wobj,///< 工件数据
	_user,///< 用户坐标系数据
	_add,///< 附加轴坐标系数据
	_payload///< 负载数据
}robdatatype;

/*-------------------------------------------------------------------------*/
/**
  @brief	版本号

	描述版本号：主版本号.子版本号.内部版本号
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	int major;///< 主版本号
	int minor;///< 子版本号
	int build;///< 内部版本号
}SYSTEMVERSION;

/**
 * @brief 获取版本号
 *
 * @return SYSTEMVERSION 版本号
 */
SYSTEMVERSION getSystemVerison();

/**
 * @brief 获取主版本号
 *
 * @return int 主版本号
 */
int getSystemVerisonMajor();

/**
 * @brief 获取子版本号
 *
 * @return int 子版本号
 */
int getSystemVerisonMinor();

/**
 * @brief 获取内部版本号
 *
 * @return int 内部版本号
 */
int getSystemVerisonBuild();

/**
 * @brief 打印版本号
 *
 */
void printfSystemVerison();

/**
 * @brief 从系统获取数据文件内数据的数目
 *
 * @param rdt 数据文件类型, _robjoint, _robpose, _speed, _zone, _tool, _wobj，_user
 * @return int 正确数据的数目，错误返回其他
 */
extern int getDataNum(robdatatype rdt);

/**
 * @brief 从系统获取数据文件内索引为的数据名字
 *
 * @param rdt 数据文件类型, _robjoint, _robpose, _speed, _zone, _tool, _wobj，_user
 * @param n 数据索引,0,1,2,...
 * @param dataname 返回数据名字,需要分配内存,为保证可靠的数据大小区,建议大于等于ROBOT_DATA_NAME_LEN_MAX
 * @return int 正确返回0，错误返回其他
 */
extern int getDataName(robdatatype rdt, int n, char* dataname);

/**
 * @brief 从系统获取关节位置
 *
 * @param J 数据名字
 * @param rjoint 返回关节位置 单位：rad
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getrobjoint(const char* J, robjoint* rjoint);

/**
 * @brief 从系统获取位姿
 *
 * @param P 数据名字
 * @param rpose 返回位姿(mm, rad),姿态为固定角描述
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getrobpose(const char* P, robpose* rpose);

/**
 * @brief 从系统获取运动速度
 *
 * @param S  数据名字
 * @param sp 返回运动速度
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getspeed(const char* S, speed* sp);

/**
 * @brief 从系统获取转弯区大小
 *
 * @param Z 数据名字
 * @param zo 返回转弯区数据
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getzone(const char* Z, zone* zo);

/**
 * @brief 从系统获取末端工具
 *
 * @param T 数据名字
 * @param to 返回末端工具数据
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int gettool(const char* T, tool* to);

/**
 * @brief 从系统获取工件坐标系
 *
 * @param W 数据名字
 * @param wo 坐标系数据
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getwobj(const char* W, wobj* wo);

/**
 * @brief 从系统获取用户坐标系
 *
 * @param U 数据名字
 * @param us 返回位姿(mm, rad),姿态为固定角描述
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getuser(const char* U,robpose* us);

/**
 * @brief 从系统获取附加轴坐标系
 *
 * @param A 数据名字
 * @param ad 返回位姿(mm, rad),姿态为固定角描述
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getadd(const char* A,robpose* ad);

/**
 * @brief 从系统获取负载
 *
 * @param payload 返回负载描述
 * @param payloadname 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int getPayLoad(PayLoad* payload, const char* payloadname);

/**
 * @brief 写关节位置至系统
 *
 * @param J 数据名字
 * @param rjoint 关节位置 单位：rad
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writerobjoint(const char* J,robjoint* rjoint);

/**
 * @brief 写位姿至系统
 *
 * @param P 数据名字
 * @param rpose 位姿(mm, rad),姿态为固定角描述
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writerobpose(const char* P, robpose* rpose);

/**
 * @brief写运动速度至系统
 *
 * @param S  数据名字
 * @param sp 运动速度
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writespeed(const char* S, speed* sp);

/**
 * @brief 写转弯区数据至系统
 *
 * @param Z 数据名字
 * @param zo 转弯区数据
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writezone(const char* Z, zone* zo);

/**
 * @brief 写工件坐标系数据至系统
 *
 * @param T 数据名字
 * @param to 末端工具数据
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writetool(const char* T, tool* to);

/**
 * @brief 写工件坐标系数据至系统
 *
 * @param W 数据名字
 * @param wo 坐标系数据
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writewobj(const char* W, wobj* wo);

/**
 * @brief 写用户坐标系至系统
 *
 * @param U 数据名字
 * @param us 位姿(mm, rad),姿态为固定角描述
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writeuser(const char* U, robpose* us);

/**
 * @brief 写附加轴坐标系至系统
 *
 * @param A 数据名字
 * @param ad 位姿(mm, rad),姿态为固定角描述
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writeadd(const char* A, robpose* ad);

/**
 * @brief 写负载至系统
 *
 * @param payload 负载描述
 * @param payloadname 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
 */
extern int writePayLoad(PayLoad* payload, const char* payloadname);

/**
 * @brief 删除系统中的关节数据
 *
 * @param J 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deleterobjoint(const char* J);

/**
 * @brief 删除系统中的笛卡尔数据
 *
 * @param P 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deleterobpose(const char* P);

/**
 * @brief 删除系统中的速度数据
 *
 * @param S 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deletespeed(const char* S);

/**
 * @brief 删除系统中的转弯区数据
 *
 * @param Z 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deletezone(const char* Z);

/**
 * @brief 删除系统中的工具数据
 *
 * @param T 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deletetool(const char* T);

/**
 * @brief 删除系统中的工件数据
 *
 * @param W 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deletewobj(const char* W);

/**
 * @brief 删除系统中的用户坐标系数据
 *
 * @param U 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deleteuser(const char* U);

/**
 * @brief 删除系统中的附加轴坐标系数据
 *
 * @param A 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
int deleteadd(const char* A);

/**
 * @brief 删除系统中的负载
 *
 * @param payloadname 数据名字
 * @return int 正确返回0，错误返回其他(-1:指定名字无效)
*/
extern int deletePayLoad(const char* payloadname);

/**
 * @brief 初始化关节目标
 *
 * @param rjoint 返回关节位姿数据
 * @param data 机器人关节目标数据，单位: rad
 * @param dof 数据维度，即机器人自由度
 */
extern void init_robjoint(robjoint* rjoint, double* data, int dof);

/**
 * @brief 初始化笛卡尔位姿目标
 *
 * @param rpose 返回笛卡尔位姿数据
 * @param xyz 机器人位置目标数据，单位: mm
 * @param rpy 机器人姿态目标数据（xyz固定角），单位: rad
 */
extern void init_robpose(robpose* rpose, double* xyz, double* rpy);

/**
 * @brief 初始化关节速度
 *
 * @param sp 返回关节速度数据
 * @param data 机器人关节速度数据，含义由flag决定
 * @param dof 数据维度，即机器人自由度
 *  @param flag  0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 */
extern void init_speed_joint(speed* sp, double* data, int dof, int flag);

/**
 * @brief 初始化笛卡尔速度
 *
 * @param sp 返回笛卡尔速度数据
 * @param tcp 平动速度，含义由flag决定
 * @param orl 转动速度，含义由flag决定
 * @param flag  0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 */
extern void init_speed_cartesian(speed* sp, double tcp, double orl, int flag);

/**
 * @brief 初始速度
 *
 * @param sp 返回速度数据
 * @param data 机器人关节速度数据，含义由per_flag决定
 * @param dof 数据维度，即机器人自由度
 *  @param per_flag  关节速度含义0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 * @param tcp 平动速度，含义由tcp_flag决定
 * @param orl 转动速度，含义由tcp_flag决定
 * @param tcp_flag 笛卡尔速度含义 0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 */
extern void init_speed(speed* sp, double* data, int dof, int per_flag, double tcp, double orl, int tcp_flag);

/**
 * @brief 初始化机器人转弯区
 *
 * @param ze 返回转弯区数据
 * @param size 转弯区大小，含义由flag决定
 * @param flag  0:不采用转弯区平滑过度，1：百分比(0~1)，2：距离(mm)或圆周角(rad)
 */
extern void init_zone(zone* ze, double size,  int flag);

/**
 * @brief 初始化机器人工具
 *
 * @param tl 返回工具数据
 * @param frame 工具坐标系(mm, rad)
 * @param robhold 1：安装在机器人上，0：未安装在机器人上
 * @param payload 1：工具负载
 */
extern void init_tool(tool* tl, double* frame, int robhold, PayLoad* payload);

/**
 * @brief 初始化机器人工件
 *
 * @param wj 返回工件数据
 * @param workframe 工件坐标系(mm, rad)
 * @param userframe 用户坐标系(mm, rad)
 * @param addframe 附加轴坐标系(mm, rad)
 * @param robhold 1：安装在机器人上，0：未安装在机器人上
 * @param addhold 1：关联附加轴；0：不关联附加轴
 * @param ufmec 预留，未使用
 */
extern void init_wobj(wobj* wj, double* userframe, double* workframe, double* addframe, int robhold, int addhold, int ufmec);

/**
 * @brief 初始化机器人负载
 *
 * @param pl 返回负载数据
 * @param m 连杆质量(t=1000kg)
 * @param cm 相对末端连杆坐标系的质心(mm)
 * @param I 相对质心坐标系的惯性张量(t*mm^2=0.001kg*m^2)
 */
extern void init_PayLoad(PayLoad* pl,double m, double* cm,double (*I)[3]);

/**
 * @brief 解析关节目标
 *
 * @param rjoint 关节位姿数据
 * @param data 返回机器人关节目标数据，单位: rad
 * @param dof 返回数据维度，即机器人自由度
 */
extern void parse_robjoint(robjoint* rjoint, double* data, int* dof);

/**
 * @brief 解析笛卡尔位姿目标
 *
 * @param rpose 笛卡尔位姿数据
 * @param xyz 返回机器人位置目标数据，单位: mm
 * @param rpy 返回机器人姿态目标数据（xyz固定角），单位: rad
 */
extern void parse_robpose(robpose* rpose, double* xyz, double* rpy);

/**
 * @brief 解析化关节速度
 *
 * @param sp 关节速度数据
 * @param data 返回 机器人关节速度数据，含义由flag决定
 * @param dof 返回数据维度，即机器人自由度
 *  @param flag  返回数据含义0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 */
extern void parse_speed_joint(speed* sp, double* data, int*dof, int* flag);

/**
 * @brief 解析笛卡尔速度
 *
 * @param sp 笛卡尔速度数据
 * @param tcp 返回平动速度，含义由flag决定
 * @param orl 返回转动速度，含义由flag决定
 * @param flag  返回数据含义0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 */
extern void parse_speed_cartesian(speed* sp, double* tcp, double* orl, int* flag);

/**
 * @brief 解析速度
 *
 * @param sp 速度数据
 * @param data 返回机器人关节速度数据，含义由per_flag决定
 * @param dof 返回数据维度，即机器人自由度
 *  @param per_flag 返回关节数据含义per_flag  0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 * @param tcp 返回平动速度，含义由tcp_flag决定
 * @param orl 返回转动速度，含义由tcp_flag决定
 * @param tcp_flag  返回笛卡尔速度含义0:百分比(0~1)；1:速度（mm/s or rad/s）;2:时间（s）
 */
extern void parse_speed(speed* sp, double* data, int*dof, int* per_flag, double* tcp, double* orl, int* tcp_flag);


/**
 * @brief 解析机器人转弯区
 *
 * @param ze 转弯区数据
 * @param size 返回转弯区大小，含义由flag决定
 * @param flag  返回0:不采用转弯区平滑过度，1：百分比(0~1)，2：距离(mm)或圆周角(rad)
 */
extern void parse_zone(zone* ze, double* size,  int* flag);

/**
 * @brief 解析机器人工具
 *
 * @param tl 工具数据
 * @param frame 返回工具坐标系(mm, rad)
 * @param robhold 返回1：安装在机器人上，0：未安装在机器人上
 * @param payload 1：工具负载
 */
extern void parse_tool(tool* tl, double* frame, int* robhold, PayLoad* payload);

/**
 * @brief 解析机器人工件
 *
 * @param wj 工件数据
 * @param workframe 返回工件坐标系(mm, rad)
 * @param userframe 返回用户坐标系(mm, rad)
 * @param addframe 返回附加轴坐标系(mm, rad)
 * @param robhold 返回 1：安装在机器人上，0：未安装在机器人上
 * @param addhold 1：关联附加轴；0：不关联附加轴
 * @param ufmec 返回预留，未使用
 */
extern void parse_wobj(wobj* wj, double* userframe, double* workframe, double* addframe, int* robhold, int* addhold, int* ufmec);

/**
 * @brief 解析机器人负载数据
 *
 * @param pl 负载数据
 * @param m 返回连杆质量(t=1000kg)
 * @param cm 返回相对末端连杆坐标系的质心(mm)
 * @param I 返回相对质心坐标系的惯性张量(t*mm^2=0.001kg*m^2)
 */
extern void parse_PayLoad(PayLoad* pl,double* m, double* cm,double (*I)[3]);


////////////////////////////////////////////////////数据批量操作,用于数据两操作较大的情况////////////////////////////////////////////////
/*说明:数据批量操作接口,用于操作数据量较大时使用,如批量读取和写入.数据量较小时可用get***(),write***()等接口操作.数据量大时建议使用批量操作接口.
 *          使用时利用CreatDataBatch()创建批量操作数据,创建时需要指定操作数据类型和读(0)写(1)方式,再利用数据类型对应的get***_batch() 或write***_batch()进行读或写操作,
 *          操作完成,必须使用FreeDataDatch()释放操作数据.
 *
 * 使用代码例子:
 *             void robot_read_data_batch_test()
 *             {
 *              	int err=0;
 *              	char name[ROBOT_DATA_NAME_LEN_MAX];
 *             	robjoint rjoint;
 *             	double angle[ROBOT_MAX_DOF];
 *             	int dof=0;
 *             	int num=CreatDataBatch(_robjoint, 0);//创建批操作数据
 *             	if (num<0)
 *             	{
 *             		printf("CreatDataBatch failure\n");
 *             		return;
 *             	}
 *             	int i=0;
 *             	int j=0;
 *             	for (i=0;i<num;i++)
 *             	{
 *             		err=getrobjoint_batch(name,&rjoint, i);//获取索引为i的数据内容及名字
 *             		if (0!=err)
 *             		{
 *             			printf("getrobjoint_batch failure\n");
 *             			break;
 *             		}
 *             		//------------------------------数据使用------------------
 *             		parse_robjoint(&rjoint, angle, &dof);//解析数据
 *             		printf("读取数据名字:%s,数据维度%d, 数据内容: ",name,dof);
 *             		for (j=0;j<dof;j++)
 *             		{
 *             			printf("%f,",angle[j]);
 *             		}
 *             		printf("\n");
 *             	}
 *             	err=FreeDataDatch();//释放批操作数据
 *             		if (0!=err)
 *             		{
 *             			printf("FreeDataDatch failure\n");
 *             		}
 *             }
 *
 *             void robot_write_data_batch_test()
 *             {
 *             	char name[ROBOT_DATA_NAME_LEN_MAX];
 *             	robjoint rjoint;
 *             	double angle[ROBOT_MAX_DOF];
 *             	int dof=0;
 *             	int err=CreatDataBatch(_robjoint, 1);//创建批操作数据
 *             	if (err<0)
 *             	{
 *             		printf("CreatDataBatch failure\n");
 *             		return;
 *             	}
 *             	int i=0;
 *             	int num=10;//写入十个数据
 *             	for (i=0;i<num;i++)
 *             	{
 *             		//------------------------设置写入数据----------------------------
 *             		dof=6;//设置数据维度要与机器人自由度一致,可以通过相应接口获取
 *             		angle[0]=0;angle[1]=0;angle[2]=0;angle[3]=0;angle[4]=1;angle[5]=0;
 *             		sprintf(name,"%s,%d","J",i);//设置数据名称
 *             		init_robjoint(&rjoint, angle, dof);//设置机器人数据
 *             		//------------------写入数据----------------------
 *             		err=writerobjoint_batch(name,&rjoint);
 *             		if (0!=err)
 *             		{
 *             			printf("getrobjoint_batch failure\n");
 *             			break;
 *             		}
 *             	}
 *             	err=FreeDataDatch();//释放批操作数据
 *             	if (0!=err)
 *             	{
 *             		printf("FreeDataDatch failure\n");
 *             	}
 *             }
 *
 * */

/**
 * @brief 创建批数据区,操作完成必须使用FreeDataDatch()释放批数据
 *
 * @param rdt 数据类型_robjoint, _robpose, _speed, _zone, _tool, _wobj
 * @param RW_flag 转弯区数据,0:读;1:写
 * @return int 正确返回已有数据数目，错误返回其他
 */
extern int CreatDataBatch(robdatatype rdt, int RW_flag);

/**
 * @brief 释放批数据区
 *
 * @return int 正确返回0，错误返回其他
 */
extern int FreeDataDatch();

/**
 * @brief 获取指定索引的关节数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param J 返回数据名字
 * @param rjoint 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int getrobjoint_batch(char* J,robjoint* rjoint, int index);

/**
 * @brief 获取指定索引的位姿数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param P 返回数据名字
 * @param rpose 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int getrobpose_batch(char* P,robpose* rpose, int index);

/**
 * @brief 获取指定索引的速度数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param S 返回数据名字
 * @param sp 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int getspeed_batch(char* S, speed* sp, int index);

/**
 * @brief 获取指定索引的转弯区数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param Z 返回数据名字
 * @param zo 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int getzone_batch(char* Z, zone* zo, int index);

/**
 * @brief 获取指定索引的工具数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param T 返回数据名字
 * @param to 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int gettool_batch(char* T, tool* to, int index);

/**
 * @brief 获取指定索引的工件数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param W 返回数据名字
 * @param wo 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int getwobj_batch(char* W, wobj* wo, int index);

/**
 * @brief 获取指定索引的用户坐标系数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param U 返回数据名字
 * @param us 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int getuser_batch(char* U,robpose* us, int index);

/**
 * @brief 获取指定索引的附加轴坐标系数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param A 返回数据名字
 * @param ad 返回数据
 * @param index 获取数据的索引,(0,1,2,...),最大值小于CreatDataBatch()的返回值.
 * @return int 正确返回0，错误返回其他
 */
extern int getadd_batch(char* A,robpose* ad, int index);

/**
 * @brief 写入关节数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param J 写入数据名字
 * @param rjoint 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writerobjoint_batch(const char* J,robjoint* rjoint);

/**
 * @brief 写入位姿数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param P 写入数据名字
 * @param rpose 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writerobpose_batch(const char* P, robpose* rpose);

/**
 * @brief 写入速度数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param S 写入数据名字
 * @param sp 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writespeed_batch(const char* S, speed* sp);

/**
 * @brief 写入转弯区数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param Z 写入数据名字
 * @param zo 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writezone_batch(const char* Z, zone* zo);

/**
 * @brief 写入工具数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param T 写入数据名字
 * @param to 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writetool_batch(const char* T, tool* to);

/**
 * @brief 写入工件数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param W 写入数据名字
 * @param wo 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writewobj_batch(const char* W, wobj* wo);

/**
 * @brief 写入用户坐标系数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param U 写入数据名字
 * @param us 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writeuser_batch(const char* U, robpose* us);

/**
 * @brief 写入附加轴坐标系数据,必须使用CreatDataBatch()创建批数据区
 *
 * @param A 写入数据名字
 * @param ad 写入数据内容
 * @return int 正确返回0，错误返回其他
 */
extern int writeadd_batch(const char* A, robpose* ad);

//数据变量宏定义
typedef  robjoint* robjointptr; //!<robjoint指针
typedef  robpose* robposeptr; //!<robpose指针
typedef  speed* speedptr; //!<speed指针
typedef  zone* zoneptr; //!<zone指针
typedef  tool* toolptr; //!<tool指针
typedef  wobj* wobjptr; //!<wobj指针
typedef  PayLoad* PayLoadptr; //!<PayLoad指针

#ifdef __cplusplus

//数据操作定义
/**
 * @brief 设置7轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 * @param v5 5关节位置（rad）
 * @param v6 6关节位置（rad）
 * @param v7 7关节位置（rad）
 */
#define SETJOINT7(name,v1,v2,v3,v4,v5,v6,v7)  \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->angle[4]=v5;name->angle[5]=v6;name->angle[6]=v7;name->dof=7;

/**
 * @brief 设置6轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 * @param v5 5关节位置（rad）
 * @param v6 6关节位置（rad）
 */
#define SETJOINT6(name,v1,v2,v3,v4,v5,v6)  \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->angle[4]=v5;name->angle[5]=v6;name->dof=6;

/**
 * @brief 设置5轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 * @param v5 5关节位置（rad）
 */
#define SETJOINT5(name,v1,v2,v3,v4,v5)  \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->angle[4]=v5;name->dof=5;

/**
 * @brief 设置4轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 */
#define SETJOINT4(name,v1,v2,v3,v4)  \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->dof=4;

/**
 * @brief 设置3轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 */
#define SETJOINT3(name,v1,v2,v3)  \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->dof=3;

/**
 * @brief 设置2轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 */
#define SETJOINT2(name,v1,v2)  \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->dof=2;

/**
 * @brief 设置1轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 */
#define SETJOINT1(name,v1)  \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->dof=1;

/**
 * @brief 导入关节数据
 * @param name 关节数据指针名（与数据同名）
 */
#define IMPORTJOINT(name) \
		HYYRobotBase::robjoint _____##name##__; \
		HYYRobotBase::robjointptr name=&(_____##name##__);\
		if(0!=HYYRobotBase::getrobjoint(#name,name))\
		{name=NULL;}

/**
 * @brief 设置笛卡尔位置
 * @param name 笛卡尔数据指针名
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SETPOSE(name,x,y,z,k,p,s)  \
		HYYRobotBase::robpose _____##name##__; \
		HYYRobotBase::robposeptr name=&(_____##name##__);\
		name->xyz[0]=x;name->xyz[1]=y;name->xyz[2]=z;name->kps[0]=k;name->kps[1]=p;name->kps[2]=s;

/**
 * @brief 导入笛卡尔数据
 * @param name 笛卡尔数据指针名（与数据同名）
 */
#define IMPORTPOSE(name) \
		HYYRobotBase::robpose _____##name##__; \
		HYYRobotBase::robposeptr name=&(_____##name##__);\
		if (0!=HYYRobotBase::getrobpose(#name,name))\
		{name=NULL;}

/**
 * @brief 设置速度百分比
 * @param name 速度数据指针名
 * @param v 速度百分比（0~1）
 */
#define SETSPEEDPER(name,v)  \
		HYYRobotBase::speed _____##name##__; \
		HYYRobotBase::speedptr name=&(_____##name##__);\
		name->per[0]=v;name->per[1]=v;name->per[2]=v;name->per[3]=v;name->per[4]=v;name->per[5]=v;name->per[6]=v;name->per[7]=v;name->per[8]=v;name->per[9]=v;name->per_flag=0;name->dof=ROBOT_MAX_DOF;\
		name->tcp=v;name->orl=v;name->tcp_flag=0;

/**
 * @brief 设置速度(rad/s 或 mm/s）
 * @param name 速度数据指针名
 * @param rot 关节或笛卡尔旋转速度（rad/s）
 * @param tra 笛卡尔平动速度（mm/s）
 */
#define SETSPEED(name,rot,tra)  \
		HYYRobotBase::speed _____##name##__; \
		HYYRobotBase::speedptr name=&(_____##name##__);\
		name->per[0]=rot;name->per[1]=rot;name->per[2]=rot;name->per[3]=rot;name->per[4]=rot;name->per[5]=rot;name->per[6]=rot;name->per[7]=rot;name->per[8]=rot;name->per[9]=rot;name->per_flag=1;name->dof=ROBOT_MAX_DOF;\
		name->tcp=tra;name->orl=rot;name->tcp_flag=1;

/**
 * @brief 设置速度时间
 * @param name 速度数据指针名
 * @param t 时间（s）
 */
#define SETSPEEDTIME(name,t)  \
		HYYRobotBase::speed _____##name##__; \
		HYYRobotBase::speedptr name=&(_____##name##__);\
		name->per[0]=t;name->per[1]=t;name->per[2]=t;name->per[3]=t;name->per[4]=t;name->per[5]=t;name->per[6]=t;name->per[7]=t;name->per[8]=t;name->per[9]=t;name->per_flag=3;name->dof=ROBOT_MAX_DOF;\
		name->tcp=t;name->orl=t;name->tcp_flag=3;

/**
 * @brief 导入速度
 * @param name 速度数据指针名（与数据同名）
 */
#define IMPORTSPEED(name) \
		HYYRobotBase::speed _____##name##__; \
		HYYRobotBase::speedptr name=&(_____##name##__);\
		if(0!=HYYRobotBase::getspeed(#name,name))\
		{name=NULL;}

/**
 * @brief 设置转弯区百分比
 * @param name 转弯区数据指针名
 * @param v 转弯区百分比（0~1）
 */
#define SETZONE(name,v) \
		HYYRobotBase::zone _____##name##__; \
		HYYRobotBase::zoneptr name=&(_____##name##__);\
		name->zone_flag=1;\
		name->zone_size=v;

/**
 * @brief 设置转弯区大小（物理量）
 * @param name 转弯区数据指针名
 * @param v 转弯区大小（物理量）
 */
#define SETZONESIZE(name,v) \
		HYYRobotBase::zone _____##name##__; \
		HYYRobotBase::zoneptr name=&(_____##name##__);\
		name->zone_flag=2;\
		name->zone_size=v;

/**
 * @brief 导入转弯区
 * @param name 转弯区数据指针名（与数据同名）
 */
#define IMPORTZONE(name) \
		HYYRobotBase::zone _____##name##__; \
		HYYRobotBase::zoneptr name=&(_____##name##__);\
		if (0!=HYYRobotBase::getzone(#name,name))\
		{name=NULL;}

/**
 * @brief 设置工具
 * @param name 工具数据指针名
 * @param hold 工具是否手持（0:非手持；1:手持）
 * @param pose 工具坐标系描述（robposeptr类型）
 * @param load 工具负载描述（PayLoadptr类型）
 */
#define SETTOOL(name,hold,pose,load) \
		HYYRobotBase::tool _____##name##__; \
		HYYRobotBase::toolptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->tframe=*pose; \
		name->payload=*load;

/**
 * @brief 设置工具
 * @param name 工具数据指针名
 * @param hold 工具是否手持（0:非手持；1:手持）
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SETTOOL_FRAME(name,hold,x,y,z,k,p,s) \
		HYYRobotBase::tool _____##name##__; \
		HYYRobotBase::toolptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->tframe.xyz[0]=x;name->tframe.xyz[1]=y;name->tframe.xyz[2]=z;\
		name->tframe.kps[0]=k;name->tframe.kps[1]=p;name->tframe.kps[2]=s;\
		memset(&(name->payload),0,sizeof(HYYRobotBase::PayLoad));

/**
 * @brief 导入工具
 * @param name 工具数据指针名（与数据同名）
 */
#define IMPORTTOOL(name) \
		HYYRobotBase::tool _____##name##__; \
		HYYRobotBase::toolptr name=&(_____##name##__);\
		if (0!=HYYRobotBase::gettool(#name,name))\
		{name=NULL;}

/**
 * @brief 设置工件
 * @param name 工件数据指针名
 * @param hold 工件是否手持（0:非手持；1:手持）
 * @param usrframe 用户坐标系描述（robposeptr类型）
 * @param workframe 工件坐标系描述（robposeptr类型）
 */
#define SETWOBJ(name,hold,usrframe,workframe) \
		HYYRobotBase::wobj _____##name##__; \
		HYYRobotBase::wobjptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->addhold=0;\
		name->ufmec=0;\
		name->uframe=*usrframe;\
		name->oframe=*workframe;\
		memset(&(name->aframe),0,sizeof(HYYRobotBase::robpose));

/**
 * @brief 设置工件
 * @param name 工件数据指针名
 * @param hold 工件是否手持（0:非手持；1:手持）
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SETWOBJ_FRAME(name,hold,x,y,z,k,p,s) \
		HYYRobotBase::wobj _____##name##__; \
		HYYRobotBase::wobjptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->addhold=0;\
		name->ufmec=0;\
		memset(&(name->uframe),0,sizeof(HYYRobotBase::robpose));\
		memset(&(name->aframe),0,sizeof(HYYRobotBase::robpose));\
		name->oframe.xyz[0]=x;name->oframe.xyz[1]=y;name->oframe.xyz[2]=z;\
		name->oframe.kps[0]=k;name->oframe.kps[1]=p;name->oframe.kps[2]=s;

/**
 * @brief 在现有工件基础上补充附加轴坐标系
 * @param name 工件数据指针名(必须存在)
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SUPWOBJ_AFRAME(name,x,y,z,k,p,s) \
		name->addhold=1;\
		name->aframe.xyz[0]=x;name->aframe.xyz[1]=y;name->aframe.xyz[2]=z;\
		name->aframe.kps[0]=k;name->aframe.kps[1]=p;name->aframe.kps[2]=s;


/**
 * @brief 设置工件
 * @param name 工件数据指针名
 * @param hold 工件是否手持（0:非手持；1:手持）
 * @param usrframe 用户坐标系描述（robposeptr类型）
 * @param workframe 工件坐标系描述（robposeptr类型）
 * @param addframe 附加轴组描述（robposeptr类型）
 */
#define SETWOBJ_ALL(name,hold,usrframe,workframe,addframe) \
		HYYRobotBase::wobj _____##name##__; \
		HYYRobotBase::wobjptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->addhold=1;\
		name->ufmec=0;\
		name->uframe=*usrframe;\
		name->oframe=*workframe;\
		name->aframe=*addframe;

/**
 * @brief 导入工件
 * @param name 工件数据指针名（与数据同名）
 */
#define IMPORTWOBJ(name) \
		HYYRobotBase::wobj _____##name##__; \
		HYYRobotBase::wobjptr name=&(_____##name##__);\
		if (0!=HYYRobotBase::getwobj(#name,name))\
		{name=NULL;}

/**
 * @brief 设置负载
 * @param name 负载数据指针名
 * @param m 负载质量（0.001kg）
 * @param cmx 质心x坐标（mm）
 * @param cmy 质心y坐标（mm）
 * @param cmz 质心z坐标（mm）
 * @param Ixx 相对质心xx惯性（0.001*m^2*kg）
 * @param Ixy 相对质心xy惯性（0.001*m^2*kg）
 * @param Ixz 相对质心xz惯性（0.001*m^2*kg）
 * @param Iyy 相对质心yy惯性（0.001*m^2*kg）
 * @param Iyz 相对质心yz惯性（0.001*m^2*kg）
 * @param Izz 相对质心zz惯性（0.001*m^2*kg）
 */
#define SETPAYLOAD(name,m,cmx,cmy,cmz,Ixx,Ixy,Ixz,Iyy,Iyz,Izz) \
		HYYRobotBase::PayLoad _____##name##__; \
		HYYRobotBase::PayLoadptr name=&(_____##name##__);\
		HYYRobotBase::initPayLoad(name, m, cmx, cmy, cmz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz,0);

/**
 * @brief 导入负载
 * @param name 负载数据指针名（与数据同名）
 */
#define IMPORTPAYLOAD(name) \
		HYYRobotBase::PayLoad _____##name##__; \
		HYYRobotBase::PayLoadptr name=&(_____##name##__);\
		if (0!=HYYRobotBase::getPayLoad(name,#name))\
		{name=NULL;}

#else

//数据操作定义
/**
 * @brief 设置7轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 * @param v5 5关节位置（rad）
 * @param v6 6关节位置（rad）
 * @param v7 7关节位置（rad）
 */
#define SETJOINT7(name,v1,v2,v3,v4,v5,v6,v7)  \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->angle[4]=v5;name->angle[5]=v6;name->angle[6]=v7;name->dof=7;

/**
 * @brief 设置6轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 * @param v5 5关节位置（rad）
 * @param v6 6关节位置（rad）
 */
#define SETJOINT6(name,v1,v2,v3,v4,v5,v6)  \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->angle[4]=v5;name->angle[5]=v6;name->dof=6;

/**
 * @brief 设置5轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 * @param v5 5关节位置（rad）
 */
#define SETJOINT5(name,v1,v2,v3,v4,v5)  \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->angle[4]=v5;name->dof=5;

/**
 * @brief 设置4轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 * @param v4 4关节位置（rad）
 */
#define SETJOINT4(name,v1,v2,v3,v4)  \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->angle[3]=v4;name->dof=4;

/**
 * @brief 设置3轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 * @param v3 3关节位置（rad）
 */
#define SETJOINT3(name,v1,v2,v3)  \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->angle[2]=v3;name->dof=3;

/**
 * @brief 设置2轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 * @param v2 2关节位置（rad）
 */
#define SETJOINT2(name,v1,v2)  \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->angle[1]=v2;name->dof=2;

/**
 * @brief 设置1轴关节位置
 * @param name 关节数据指针名
 * @param v1 1关节位置（rad）
 */
#define SETJOINT1(name,v1)  \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		name->angle[0]=v1;name->dof=1;

/**
 * @brief 导入关节数据
 * @param name 关节数据指针名（与数据同名）
 */
#define IMPORTJOINT(name) \
		robjoint _____##name##__; \
		robjointptr name=&(_____##name##__);\
		if(0!=getrobjoint(#name,name))\
		{name=NULL;}

/**
 * @brief 设置笛卡尔位置
 * @param name 笛卡尔数据指针名
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SETPOSE(name,x,y,z,k,p,s)  \
		robpose _____##name##__; \
		robposeptr name=&(_____##name##__);\
		name->xyz[0]=x;name->xyz[1]=y;name->xyz[2]=z;name->kps[0]=k;name->kps[1]=p;name->kps[2]=s;

/**
 * @brief 导入笛卡尔数据
 * @param name 笛卡尔数据指针名（与数据同名）
 */
#define IMPORTPOSE(name) \
		robpose _____##name##__; \
		robposeptr name=&(_____##name##__);\
		if (0!=getrobpose(#name,name))\
		{name=NULL;}

/**
 * @brief 设置速度百分比
 * @param name 速度数据指针名
 * @param v 速度百分比（0~1）
 */
#define SETSPEEDPER(name,v)  \
		speed _____##name##__; \
		speedptr name=&(_____##name##__);\
		name->per[0]=v;name->per[1]=v;name->per[2]=v;name->per[3]=v;name->per[4]=v;name->per[5]=v;name->per[6]=v;name->per[7]=v;name->per[8]=v;name->per[9]=v;name->per_flag=0;name->dof=ROBOT_MAX_DOF;\
		name->tcp=v;name->orl=v;name->tcp_flag=0;

/**
 * @brief 设置速度(rad/s 或 mm/s）
 * @param name 速度数据指针名
 * @param rot 关节或笛卡尔旋转速度（rad/s）
 * @param tra 笛卡尔平动速度（mm/s）
 */
#define SETSPEED(name,rot,tra)  \
		speed _____##name##__; \
		speedptr name=&(_____##name##__);\
		name->per[0]=rot;name->per[1]=rot;name->per[2]=rot;name->per[3]=rot;name->per[4]=rot;name->per[5]=rot;name->per[6]=rot;name->per[7]=rot;name->per[8]=rot;name->per[9]=rot;name->per_flag=1;name->dof=ROBOT_MAX_DOF;\
		name->tcp=tra;name->orl=rot;name->tcp_flag=1;

/**
 * @brief 设置速度时间
 * @param name 速度数据指针名
 * @param t 时间（s）
 */
#define SETSPEEDTIME(name,t)  \
		speed _____##name##__; \
		speedptr name=&(_____##name##__);\
		name->per[0]=t;name->per[1]=t;name->per[2]=t;name->per[3]=t;name->per[4]=t;name->per[5]=t;name->per[6]=t;name->per[7]=t;name->per[8]=t;name->per[9]=t;name->per_flag=3;name->dof=ROBOT_MAX_DOF;\
		name->tcp=t;name->orl=t;name->tcp_flag=3;

/**
 * @brief 导入速度
 * @param name 速度数据指针名（与数据同名）
 */
#define IMPORTSPEED(name) \
		speed _____##name##__; \
		speedptr name=&(_____##name##__);\
		if(0!=getspeed(#name,name))\
		{name=NULL;}

/**
 * @brief 设置转弯区百分比
 * @param name 转弯区数据指针名
 * @param v 转弯区百分比（0~1）
 */
#define SETZONE(name,v) \
		zone _____##name##__; \
		zoneptr name=&(_____##name##__);\
		name->zone_flag=1;\
		name->zone_size=v;

/**
 * @brief 设置转弯区大小（物理量）
 * @param name 转弯区数据指针名
 * @param v 转弯区大小（物理量）
 */
#define SETZONESIZE(name,v) \
		zone _____##name##__; \
		zoneptr name=&(_____##name##__);\
		name->zone_flag=2;\
		name->zone_size=v;

/**
 * @brief 导入转弯区
 * @param name 转弯区数据指针名（与数据同名）
 */
#define IMPORTZONE(name) \
		zone _____##name##__; \
		zoneptr name=&(_____##name##__);\
		if (0!=getzone(#name,name))\
		{name=NULL;}

/**
 * @brief 设置工具
 * @param name 工具数据指针名
 * @param hold 工具是否手持（0:非手持；1:手持）
 * @param pose 工具坐标系描述（robposeptr类型）
 * @param load 工具负载描述（PayLoadptr类型）
 */
#define SETTOOL(name,hold,pose,load) \
		tool _____##name##__; \
		toolptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->tframe=*pose; \
		name->payload=*load;

/**
 * @brief 设置工具
 * @param name 工具数据指针名
 * @param hold 工具是否手持（0:非手持；1:手持）
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SETTOOL_FRAME(name,hold,x,y,z,k,p,s) \
		tool _____##name##__; \
		toolptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->tframe.xyz[0]=x;name->tframe.xyz[1]=y;name->tframe.xyz[2]=z;\
		name->tframe.kps[0]=k;name->tframe.kps[1]=p;name->tframe.kps[2]=s;\
		memset(&(name->payload),0,sizeof(PayLoad));

/**
 * @brief 导入工具
 * @param name 工具数据指针名（与数据同名）
 */
#define IMPORTTOOL(name) \
		tool _____##name##__; \
		toolptr name=&(_____##name##__);\
		if (0!=gettool(#name,name))\
		{name=NULL;}

/**
 * @brief 设置工件
 * @param name 工件数据指针名
 * @param hold 工件是否手持（0:非手持；1:手持）
 * @param usrframe 用户坐标系描述（robposeptr类型）
 * @param workframe 工件坐标系描述（robposeptr类型）
 */
#define SETWOBJ(name,hold,usrframe,workframe) \
		wobj _____##name##__; \
		wobjptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->addhold=0;\
		name->ufmec=0;\
		name->uframe=*usrframe;\
		name->oframe=*workframe;\
		memset(&(name->aframe),0,sizeof(robpose));

/**
 * @brief 设置工件
 * @param name 工件数据指针名
 * @param hold 工件是否手持（0:非手持；1:手持）
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SETWOBJ_FRAME(name,hold,x,y,z,k,p,s) \
		wobj _____##name##__; \
		wobjptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->addhold=0;\
		name->ufmec=0;\
		memset(&(name->uframe),0,sizeof(robpose));\
		memset(&(name->aframe),0,sizeof(robpose));\
		name->oframe.xyz[0]=x;name->oframe.xyz[1]=y;name->oframe.xyz[2]=z;\
		name->oframe.kps[0]=k;name->oframe.kps[1]=p;name->oframe.kps[2]=s;

/**
 * @brief 在现有工件基础上补充附加轴坐标系
 * @param name 工件数据指针名(必须存在)
 * @param x x轴位置（mm）
 * @param y y轴位置（mm）
 * @param z z轴位置（mm）
 * @param k 绕x轴位置（rad）
 * @param p 绕y轴位置（rad）
 * @param s 绕z轴位置（rad）
 */
#define SUPWOBJ_AFRAME(name,x,y,z,k,p,s) \
		name->addhold=1;\
		name->aframe.xyz[0]=x;name->aframe.xyz[1]=y;name->aframe.xyz[2]=z;\
		name->aframe.kps[0]=k;name->aframe.kps[1]=p;name->aframe.kps[2]=s;

/**
 * @brief 设置工件
 * @param name 工件数据指针名
 * @param hold 工件是否手持（0:非手持；1:手持）
 * @param usrframe 用户坐标系描述（robposeptr类型）
 * @param workframe 工件坐标系描述（robposeptr类型）
 * @param addframe 附加轴组描述（robposeptr类型）
 */
#define SETWOBJ_ALL(name,hold,usrframe,workframe,addframe) \
		wobj _____##name##__; \
		wobjptr name=&(_____##name##__);\
		name->robhold=hold;\
		name->addhold=1;\
		name->ufmec=0;\
		name->uframe=*usrframe;\
		name->oframe=*workframe;\
		name->aframe=*addframe;

/**
 * @brief 导入工件
 * @param name 工件数据指针名（与数据同名）
 */
#define IMPORTWOBJ(name) \
		wobj _____##name##__; \
		wobjptr name=&(_____##name##__);\
		if (0!=getwobj(#name,name))\
		{name=NULL;}

/**
 * @brief 设置负载
 * @param name 负载数据指针名
 * @param m 负载质量（0.001kg）
 * @param cmx 质心x坐标（mm）
 * @param cmy 质心y坐标（mm）
 * @param cmz 质心z坐标（mm）
 * @param Ixx 相对质心xx惯性（0.001*m^2*kg）
 * @param Ixy 相对质心xy惯性（0.001*m^2*kg）
 * @param Ixz 相对质心xz惯性（0.001*m^2*kg）
 * @param Iyy 相对质心yy惯性（0.001*m^2*kg）
 * @param Iyz 相对质心yz惯性（0.001*m^2*kg）
 * @param Izz 相对质心zz惯性（0.001*m^2*kg）
 */
#define SETPAYLOAD(name,m,cmx,cmy,cmz,Ixx,Ixy,Ixz,Iyy,Iyz,Izz) \
		PayLoad _____##name##__; \
		PayLoadptr name=&(_____##name##__);\
		initPayLoad(name, m, cmx, cmy, cmz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz,0);

/**
 * @brief 导入负载
 * @param name 负载数据指针名（与数据同名）
 */
#define IMPORTPAYLOAD(name) \
		PayLoad _____##name##__; \
		PayLoadptr name=&(_____##name##__);\
		if (0!=getPayLoad(name,#name))\
		{name=NULL;}

#endif





#ifdef __cplusplus
}
}
#endif


#endif /* ROBOTSTRUCT_H_ */
