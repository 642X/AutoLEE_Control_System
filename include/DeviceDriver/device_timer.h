/**
 * @file device_timer.h
 *
 * @brief  机器人控制系统时钟接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-08
 *
 *利用initUserTimer()初始化用户定时器数据，设置所采用的定时器索引（0,1,......,9）。代码同一运行期内仅能有一个定时器被使用。 同一索引下userTimer()与userTimerE()为同一定时器。
 */
#ifndef DEVICE_TIMER_H_
#define DEVICE_TIMER_H_




#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/*-------------------------------------------------------------------------*/
/**
 * @brief	时钟数据结构

	保存时钟数据
 */
/*-------------------------------------------------------------------------*/
typedef struct RTimer{
	int index;   ///< 定时器索引（0,1,......,9）,默认提供十个定时器， 定时周期为总线周期
	int cycle_times;///< 总线周期的整数倍数，大于1时可实现总线周期整数倍定时
}RTimer;

/**
 * @brief 初始化时钟，实现总线整数倍时钟定时
 *
 * @param timer 时钟数据
 * @param index 时钟索引
 * @param cycle_times 基础总线周期的整数倍
 * @return int 0:成功；其他：失败
 */
extern int initUserTimer(RTimer* timer, int index, int cycle_times);

/**
 * @brief 定时，按照设置的RTimer.cycle_times返回，定时时间未到，函数阻塞
 *
 * @param timer 时钟数据
 */
extern void userTimer(RTimer* timer);

/**
 * @brief 定时，无论RTimer.cycle_times为多少，按照总线时间返回，总线周期内阻塞，当到达RTimer.cycle_times设置的定时周期返回值为ture(1),否则，为false(0)
 *
 * @param timer 时钟数据
 * @return int 1:已到达定时时间，0:未到达定时时间
 */
extern int userTimerE(RTimer* timer);

#ifdef __cplusplus
}
}
#endif


#endif /* DEVICE_TIMER_H_ */
