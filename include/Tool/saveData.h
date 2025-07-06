/**
 * @file saveData.h
 *
 * @brief  写数据到文本文件（文本数据逗号间隔），不影响实时通讯
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-9
 *
 */
#ifndef INCLUDE_SAVEDATA_H_
#define INCLUDE_SAVEDATA_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif
/**
 * @brief 创建保存数据
 *
 * @param name 数据名字(对应数据文件名字)
 * @param type 数据类型  0:int,  1:double
 * @param row  数据队列的最大行数（过小可能存在数据丢失现象,但不是所要存数据的最大组数。因为利用RSaveData()写数据和保存数据到文件是同步进行的，所以该数也没有必要过大，大小根据系统写文件速度而定，一般100左右足够）
 * @param col 数据的列数
 * @return int 0:成功，其他：失败
 */
extern int CreateSaveData1(const char* name, int type, int row, int col);

/**
 * @brief 释放保存数据
 *
 * @param name 数据名字(对应数据文件名字)
 * @return int 0:成功，其他：失败
 */
extern int DeleteSaveData(const char* name);

/**
 * @brief 保存数据到文件
 *
 * @param name 数据名字(对应数据文件名字)
 * @param data 写入的数据
 * @return int 0:成功，其他：失败
 */
extern int  RSaveData(const char* name, void* data );

/**
 * @brief 保存数据到文件
 *
 * @param name 数据名字(对应数据文件名字)
 * @param data 写入的数据
 * @param col 读取数据的列数，不大于创建时指定的列数
 * @return int 0:成功，其他：失败
 */
extern int  RSaveData1(const char* name, void* data, int col);

/**
 * @brief 保存数据到文件（无需创建数据空间）
 *
 * @param name 数据名字(对应数据文件名字)
 * @param data 写入的数据（默认数据类型为double）
 * @param col 数据的列数
 * @return int 0:成功，-1 队列已满, -2数据空间不存在, -3:空间创建失败
 */
extern int RSaveDataFast(const char* name, void* data, int col);

/**
 * @brief 保存数据到文件（无需创建数据空间）
 *
 * @param name 数据名字(对应数据文件名字)
 * @param type 数据类型  0:int,  1:double
 * @param row: 数据队列的最大行数（过小可能存在数据丢失现象）
 * @param col: 数据的列数
 * @param data  写入的数据
 * @return int 0:成功，-1 队列已满, -2数据空间不存在, -3:空间创建失败
 */
extern int RSaveDataFast1(const char* name,int type, int row, int col, void* data );


#ifdef __cplusplus
}
}
#endif


#endif /* INCLUDE_SAVEDATA_H_ */
