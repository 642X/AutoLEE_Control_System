/**
 * @file StackInterface.h
 *
 * @brief  码垛
 * @author hanbing
 * @version 11.0.0
 * @date 2021-5-25
 *
 */

#ifndef STACKINTERFACE_H_
#define STACKINTERFACE_H_
#include "Base/robotStruct.h"
#include "Move/MovePlan.h"
#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

#define STACK_FLOOR_MAXSUM   9999		///<最大层数
#define  STACK_GRAPHIC_MAXSUM   9		///<最大图形编号
#define STACK_NAME_MAX_SIZE 100         ///<最大字符数目

/*-------------------------------------------------------------------------*/
/**
@brief	码垛垛形数据
*/
/*-------------------------------------------------------------------------*/
typedef struct StackTechnologyGraphicData{
	unsigned int  id; ///<图形id
	unsigned int  Type;///<图形类型 ,0:行列型;1:纵横交错型2:回字型
	unsigned int  Xnum;	///<x方向工件数目
	unsigned int  Ynum;	///<y方向工件数目
	int RotationAngleWhole; ///<整体旋转角度（0、90、-90、180）
	int RotationAngleSingle;	///<单个工件旋转角度（0、90、-90、180）
}StackTechnologyGraphicData;


/*-------------------------------------------------------------------------*/
/**
@brief	码垛工艺数据
*/
/*-------------------------------------------------------------------------*/
typedef struct StackTechnologyData{
	char name[STACK_NAME_MAX_SIZE];	///<工艺名称
	double WorkpieceDataX;	///<工件宽
	double WorkpieceDataY;///<工件长
	double WorkpieceDataZ;///<工件高
	double GapDataX;	///<工件x方向的间隙
	double GapDataY;///<工件y方向的间隙
	unsigned int FloorGraphicNum[STACK_FLOOR_MAXSUM];	///<每层图形编号(1~STACK_GRAPHIC_MAXSUM)
	unsigned int  FloorSum;///<层数
	StackTechnologyGraphicData Griaphic[STACK_GRAPHIC_MAXSUM];	///<图形设置
	unsigned int  GriaphicSum;	///<图形个数
	unsigned int  CurrentWorkpieceID;///<码垛起始工件id
}StackTechnologyData;

/*-------------------------------------------------------------------------*/
/**
@brief	码垛工艺信息
*/
/*-------------------------------------------------------------------------*/
typedef struct StackTechnology{
	char name[STACK_NAME_MAX_SIZE]; ///<工艺名称
	unsigned int CurrentWorkpieceID; ///<当前工件id
	unsigned int TotalWorkpieceNum;	///<工件总数
	unsigned int IsPush; ///<是否为码垛，1:码垛；0:拆垛
	double deviation[6]; ///<偏移数据
}StackTechnology;

/**
 * @brief 写工艺数据至系统
 *
 * @param name 工艺名称
 * @param std 工艺数据
 * @return int 0:成功；否则失败
 */
extern int write_stack_technology(const char* name, StackTechnologyData* std);

/**
 * @brief 从系统读工艺数据
 *
 * @param name 工艺名称
 * @param std 返回工艺数据
 * @return int 0:成功；否则失败
 */
extern int read_stack_technology(const char* name, StackTechnologyData* std);

/**
 * @brief 删除工艺数据
 *
 * @param name 工艺名称
 * @return int 0:成功；否则失败
 */
extern int delete_stack_technology(const char* name);

/**
 * @brief 浏览某层工件位置或工件数目
 *
 * @param name 工艺名称
 * @param floor_id 所浏览的层id
 * @param deviation 工件位置及姿态数据
 * @param layers_workpiece_sum 所浏览层工件数目
 * @return int 0:成功；小于0失败（当deviation为NULL， 返回所在层的工件数目, 此情况layers_workpiece_sum无意义）
 */
extern int browse_stack_technology(const char* name,unsigned int floor_id, double (*deviation)[6],unsigned int  layers_workpiece_sum);

/**
 * @brief 初始化工艺
 *
 * @param st 工艺变量
 * @param std 工艺数据
 * @param IsPush 0: 拆垛；1:码垛
 * @return int 0:成功；否则失败
 */
extern int StackTechnologyInit(StackTechnology* st, StackTechnologyData* std, int IsPush);

/**
 * @brief 清空初始码垛工件id(从启始工件号码垛)
 *
 * @param st 工艺变量
 */
extern void StackTechnologyClearCurrentWorkpieceNum(StackTechnology* st);

/**
 * @brief 设置初始码垛工件id
 *
 * @param st 工艺变量
 * @param CurrentWorkpieceID 工件id
 * @return int 0:成功；否则失败
 */
extern int StackTechnologySetCurrentWorkpieceNum(StackTechnology* st, unsigned int CurrentWorkpieceID);

/**
 * @brief 获取当前码垛工件id
 *
 * @param st 工艺变量
 * @return int 当前码垛工件id
 */
extern int StackTechnologyGetCurrentWorkpieceID(StackTechnology* st);

/**
 * @brief 获取总码垛工件数目
 *
 * @param st 工艺变量
 * @return int 工件数目
 */
extern int StackTechnologyGetTotalWorkpieceSum(StackTechnology* st);

/**
 * @brief 码垛状态
 *
 * @param st 工艺变量
 * @return int 0:码垛完成；大于0:当前码垛工件id
 */
extern int StackTechnologyState(StackTechnology* st);

/**
 * @brief 移动到入口点
 *
 * @param st 工艺变量
 * @param rpose 入口点位姿
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int StackTechnologyMoveEnterPosition(StackTechnology* st, robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj );

/**
 * @brief 移动到辅助点
 *
 * @param st 工艺变量
 * @param rpose 辅助点位姿
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int StackTechMoveAuxiliaryPosition(StackTechnology* st, robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj );

/**
 * @brief 移动到放置点
 *
 * @param st 工艺变量
 * @param rpose 放置点位姿
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int StackTechMoveWorkpiecePosition(StackTechnology* st, robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj );

/**
 * @brief 导入工艺
 * @param name 工艺数据指针名（与数据同名）
 * @param IsPush 0: 拆垛；1:码垛
 */
#define IMPORTSTACK(name,IsPush) \
		StackTechnologyData _____##name##data__; \
		StackTechnology _____##name##__; \
		StackTechnology* name=&(_____##name##__);\
		if (0!=read_stack_technology(#name, &(_____##name##data__)))\
		{name=NULL;}\
		if (0!=StackTechnologyInit(name, &(_____##name##data__), IsPush))\
		{name=NULL;}

#ifdef __cplusplus
}
}
#endif

#endif /* STACKINTERFACE_H_ */
