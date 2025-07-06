/**
 * @file readData.h
 *
 * @brief  从文本文件读取数据（文本数据逗号间隔），不影响实时通讯
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-9
 *
 */
#ifndef READDATA_H_
#define READDATA_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif
/**
 * @brief 创建读取数据
 *
 * @param name 数据名字(对应数据文件名字)
 * @param type 数据类型  0:int,  1:double
 * @param row 数据队列的最大行数（不是所要存数据的最大组数。RReadData()读数据和加载文件数据是同步进行的，所以该数也没有必要过大，大小根据系统读文件速度而定，一般100左右足够）
 * @param col 数据的列数
 * @return int 0:成功，其他：失败
 */
extern int CreateReadData1(const char* name, int type, int row, int col);

/**
 * @brief 释放读取数据
 *
 * @param name 数据名字(对应数据文件名字)
 * @return int 0:成功，其他：失败
 */
extern int DeleteReadData(const char* name);

/**
 * @brief 从文件读取数据
 *
 * @param name 数据名字(对应数据文件名字)
 * @param data 返回读取的数据
 * @return int 0:成功，其他：失败
 */
extern int  RReadData(const char* name, void* data );

/**
 * @brief 从文件读取数据
 *
 * @param name 数据名字(对应数据文件名字)
 * @param data 返回读取的数据
 * @param col 读取数据的列数，不大于创建时指定的列数
 * @return int 0:成功，-1 队列已满, -2数据空间不存在
 */
extern int  RReadData1(const char* name, void* data, int col);

/**
 * @brief 从文件读取数据（无需创建数据空间）
 *
 * @param name 数据名字(对应数据文件名字)
 * @param data 返回读取的数据（默认数据类型为double）
 * @param col 读取数据的列数
 * @return int 0:成功，-1 队列已满, -2数据空间不存在, -3:空间创建失败
 */
extern int RReadDataFast(const char* name, void* data, int col);

/**
 * @brief 从文件读取数据（无需创建数据空间）
 *
 * @param name 数据名字(对应数据文件名字)
 * @param type 数据类型  0:int,  1:double
 * @param row: 数据队列的最大行数（过小可能存在数据丢失现象）
 * @param col: 数据的列数
 * @param data 返回读取的数据
 * @return int 0:成功，-1 队列已满, -2数据空间不存在, -3:空间创建失败
 */
extern int RReadDataFast1(const char* name,int type, int row, int col, void* data );


#ifdef __cplusplus
}
}
#endif


#endif /* READDATA_H_ */
