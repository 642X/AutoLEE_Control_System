/**
* @file Communication.h
*
* @brief  机器人通信接口函数
* @author hanbing
* @version 11.0.0
* @date 2020-03-31
*
*/

#ifndef INCLUDE_COMMUNICATION_H_
#define INCLUDE_COMMUNICATION_H_
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


#ifdef __cplusplus
namespace HYYRobotBase
{
	extern "C" {
#endif

typedef uint8_t byte; //!<byte

/**
* @brief 创建tcp server（阻塞）
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int SocketCreate(const char *ip, int port, const char *sName);

/**
* @brief 创建tcp client（阻塞）
*
* @param ip ip地址
* @param port 端口号
* @param sName client名称
* @return int 成功返回0，错误返回其他
*/
extern int ClientCreate(const char *ip, int port, const char *sName);

/**
* @brief 创建tcp server（非阻塞）
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param timeout 最大阻塞时间（us），负数为阻塞
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int SocketCreate1(const char* ip, int port, int timeout, const char* sName);

/**
* @brief 创建tcp client（非阻塞）
*
* @param ip ip地址
* @param port 端口号
* @param timeout 最大阻塞时间（us），负数为阻塞
* @param sName client名称
* @return int 成功返回0，错误返回其他
*/
extern int ClientCreate1(const char* ip, int port, int timeout, const char* sName);

/**
* @brief 关闭socket（含server和client）
*
* @param sName server名称或client名称
* @return int 成功返回0 ，错误返回其他
*/
extern int SocketClose(const char *sName);

/**
* @brief TCP发送Byte型数据
*
* @param data Byte型数据段
* @param len 数据段长度
* @param sName socket名称
* @return int 成功返回接收长度，错误返回值小于零
*/
extern int SocketSendByteI(byte *data, int len, const char *sName);

/**
* @brief TCP接收Byte型数据
*
* @param data Byte型数据段
* @param len 数据段长度
* @param sName socket名称
* @return int 成功返回接收数据长度，错误返回值小于零
*/
extern int SocketRecvByteI(byte *data, int len, const char *sName);


/**
* @brief TCP发送数据
*
* @param header 协议头
* @param header_format 协议头格式
* @param hf_len 协议头长度
* @param data_int int型数组
* @param data_float float型数组
* @param data_format 数据格式
* @param df_len 数据段长度
* @param sName socket名称
* @return int 成功返回接收数据长度，错误返回值小于零
*/
extern int SocketSendByteII(int *header, int(*header_format)[2], int hf_len,
	int *data_int, float *data_float, int(*data_format)[2], int df_len, const char *sName);

/**
* @brief TCP接收数据
*
* @param header 协议头
* @param header_format 协议头格式
* @param hf_len 协议头长度
* @param data_int int型数组
* @param data_float float型数组
* @param data_format 数据格式
* @param df_len 数据段长度
* @param sName socket名称
* @return int 成功返回接收数据长度，错误返回值小于零
*/
extern int SocketRecvByteII(int *header, int(*header_format)[2], int hf_len,
	int *data_int, float *data_float, int(*data_format)[2], int df_len, const char *sName);


/**
* @brief 通过TCP发送一个byte型数据
*
* @param data byte型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketSendByte(byte data, const char *sName);

/**
* @brief 通过TCPt接收一个byte型数据
*
* @param data byte型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketRecvByte(byte *data, const char *sName);

/**
* @brief 通过TCP发送一个String型数据
*
* @param data String型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketSendString(const char *data, const char *sName);

/**
* @brief 通过TCP接收一个String型数据
*
* @param data String型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketRecvString(char *data, const char *sName);

/**
* @brief 通过TCP发送一个double型数据
*
* @param data double型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketSendDouble(double data, const char *sName);

/**
* @brief 通过TCP接收一个double型数据
*
* @param data double型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketRecvDouble(double *data, const char *sName);

/**
* @brief 通过TCP发送一个int型数据
*
* @param data int型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketSendInt(int data, const char *sName);

/**
* @brief 通过TCP接收一个int型数据
*
* @param data int型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int SocketRecvInt(int *data, const char *sName);

/**
* @brief 通过TCP发送一个byte型数组
*
* @param data byte型数组
* @param n 数组长度
* @param sName socket名称
* @return int 成功>0，错误返回其他
*/
extern int SocketSendByteArray(byte *data, int n, const char *sName);

/**
* @brief 通过TCP接收一个byte型数组
*
* @param data byte型数组
* @param sName socket名称
* @return int 成功>0，错误返回其他
*/
extern int SocketRecvByteArray(byte *data, const char *sName);

/**
* @brief 通过TCP发送一个double型数组
*
* @param data double型数组
* @param n 数组长度
* @param sName socket名称
* @return int 成功>0，错误返回其他
*/
extern int SocketSendDoubleArray(double *data, int n, const char *sName);

/**
* @brief 通过TCP接收一个double型数组
*
* @param data double型数组
* @param sName  socket名称
* @return int 成功>0，错误返回其他
*/
extern int SocketRecvDoubleArray(double *data, const char *sName);

/**
* @brief 通过TCP发送一个int型数组
*
* @param data int型数组
* @param n 数组长度
* @param sName socket名称
* @return int 成功>0，错误返回其他
*/
extern int SocketSendIntArray(int *data, int n, const char *sName);

/**
* @brief 通过TCP接收一个int型数组
*
* @param data  int型数组
* @param sName socket名称
* @return int 成功>0，错误返回其他
*/
extern int SocketRecvIntArray(int *data, const char *sName);

/**
* @brief 创建udp server（阻塞）
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int UDPServerCreate(const char* ip, int port, const char* sName);

/**
* @brief 创建udp client（阻塞）
*
* @param ip ip地址
* @param port 端口号
* @param sName client名称
* @return int 成功返回0，错误返回其他
*/
extern int UDPClientCreate(const char* ip, int port, const char* sName);

/**
* @brief 创建udp server（非阻塞）
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param timeout 最大阻塞时间（us），负数为阻塞
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int UDPServerCreate1(const char* ip, int port, int timeout, const char* sName);

/**
* @brief 创建udp client（非阻塞）
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param timeout 最大阻塞时间（us），负数为阻塞
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int UDPClientCreate1(const char* ip, int port, int timeout, const char* sName);

/**
* @brief 通过UDP发送byte型数组
*
* @param data byte型数组
* @param len 数组长度
* @param sName socket名称
* @return int 成功返回接收长度，错误返回值小于零
*/
extern int UDPSendByteI(byte *data, int len, const char *sName);

/**
* @brief 通过UDP接收byte型数组
*
* @param data byte型数组
* @param len 数组长度
* @param sName socket名称
* @return int 成功返回接收长度，错误返回值小于零
*/
extern int UDPRecvByteI(byte *data, int len, const char *sName);

/**
* @brief UDP发送数据
*
* @param header 协议头
* @param header_format 协议头格式
* @param hf_len 协议头长度
* @param data_int int型数组
* @param data_float float型数组
* @param data_format 数据格式
* @param df_len 数据段长度
* @param sName socket名称
* @return int 成功返回接收数据长度，错误返回值小于零
*/
extern int UDPSendByteII(int *header, int(*header_format)[2], int hf_len,
	int *data_int, float *data_float, int(*data_format)[2], int df_len, const char *sName);

/**
* @brief UDP接收数据
*
* @param header 协议头
* @param header_format 协议头格式
* @param hf_len 协议头长度
* @param data_int int型数组
* @param data_float float型数组
* @param data_format 数据格式
* @param df_len 数据段长度
* @param sName socket名称
* @return int 成功返回接收数据长度，错误返回值小于零
*/
extern int UDPRecvByteII(int *header, int(*header_format)[2], int hf_len,
	int *data_int, float *data_float, int(*data_format)[2], int df_len, const char *sName);

/**
* @brief UDP发送byte型数据
*
* @param data byte型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int UDPSendByte(byte data, const char *sName);

/**
* @brief UDP接收byte型数据
*
* @param data byte型数据
* @param sName socket名称
* @return int  成功返回1，错误返回其他
*/
extern int UDPRecvByte(byte *data, const char *sName);

/**
* @brief 通过UDP发送一个string型数据
*
* @param data string型数据
* @param sName socket名称
* @return int 成功返回字符串长度，错误返回其他
*/
extern int UDPSendString(char *data, const char *sName);

/**
* @brief 通过UDP接收一个string型数据
*
* @param data string型数据
* @param sName  socket名称
* @return int 成功返回字符串长度，错误返回其他
*/
extern int UDPRecvString(char *data, const char *sName);

/**
* @brief 通过UDP发送一个double型数据
*
* @param data double型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int UDPSendDouble(double data, const char *sName);

/**
* @brief 通过UDP接收一个double型数据
*
* @param data double型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int UDPRecvDouble(double *data, const char *sName);

/**
* @brief 通过UDP发送一个int型数据
*
* @param data int型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int UDPSendInt(int data, const char *sName);

/**
* @brief 通过UDP接收一个int型数据
*
* @param data int型数据
* @param sName socket名称
* @return int 成功返回1，错误返回其他
*/
extern int UDPRecvInt(int *data, const char *sName);

/**
* @brief 通过UDP发送一个byte型数组
*
* @param data byte型数组
* @param n 数组长度
* @param sName socket名称
* @return int 成功返回数据，错误返回其他
*/
extern int UDPSendByteArray(int *data, int n, const char *sName);

/**
* @brief 通过UDP接收一个byte型数组
*
* @param data byte型数组
* @param sName socket名称
* @return int 成功返回数据长度，错误返回其他
*/
extern int UDPRecvByteArray(int *data, const char *sName);

/**
* @brief 通过UDP发送一个double型数组
*
* @param data double型数组
* @param n 数组长度
* @param sName socket名称
* @return int 成功返回数据长度，错误返回其他
*/
extern int UDPSendDoubleArray(double *data, int n, const char *sName);

/**
* @brief 通过UDP接收一个double型数组
*
* @param data double型数组
* @param sName socket名称
* @return int 成功返回数据长度，错误返回其他
*/
extern int UDPRecvDoubleArray(double *data, const char *sName);

/**
* @brief 通过UDP发送一个int型数组
*
* @param data int型数组
* @param n 数组长度
* @param sName socket名称
* @return int 成功返回数据长度，错误返回其他
*/
extern int UDPSendIntArray(int *data, int n, const char *sName);

/**
* @brief 通过UDP接收一个int型数组
*
* @param data int型数组
* @param sName socket名称
* @return int 成功返回数据长度，错误返回其他
*/
extern int UDPRecvIntArray(int *data, const char *sName);

/**
* @brief 创建tcp并发服务器（阻塞执行）
*
* @param ip 服务器ip,NULL为本机ip
* @param port 端口号
* @param deal_function 服务器处理函数，其入参为socket文件描述符(有数据后被调用，其内部通过调用read()函数获取数据)
* @return int 0：成功；其他失败
*/
int TCPConcurrentServer(const char* ip, int port, int(*deal_function)(int));

/**
* @brief 创建tcp并发服务器（阻塞执行）
*
* @param ip 服务器ip,NULL为本机ip
* @param port 端口号
* @param input_deal_function 服务器处理函数，其入参为socket文件描述符(有数据后被调用，其内部通过调用读函数获取数据)
* @param custom_deal_function 服务器处理函数，其入参为socket文件描述符(client连接后被调用，断开后被强制停止)
* @return int 0：成功；其他失败
*/
int TCPConcurrentServer1(const char* ip, int port, int(*input_deal_function)(int), int(*custom_deal_function)(int));

/**
* @brief 设置通信协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param offset 当前协议偏移，也就是当前数据缓存中协议的长度
* @param name 数据名称
* @param value 64位无符号整型协议数据
* @return  返回协议数据长度
*/
extern uint16_t SetProtocolData(uint8_t* buf,uint16_t offset, const char* name, uint64_t value);

/**
* @brief 设置通信协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param offset 当前协议偏移，也就是当前数据缓存中协议的长度
* @param name 数据名称
* @param value 整型协议数据
* @return  返回协议数据长度
*/
extern uint16_t SetProtocolDataInt(uint8_t* buf,uint16_t offset, const char* name, int value);

/**
* @brief 设置通信协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param offset 当前协议偏移，也就是当前数据缓存中协议的长度
* @param name 数据名称
* @param value 双精度浮点协议数据
* @return  返回协议数据长度
*/
extern int16_t SetProtocolDataDouble(uint8_t* buf,uint16_t offset, const char* name, double value);

/**
* @brief 设置通信协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param offset 当前协议偏移，也就是当前数据缓存中协议的长度
* @param name 数据名称
* @param value 字符串数据
* @return  返回协议数据长度
*/
extern uint16_t SetProtocolDataString(uint8_t* buf,uint16_t offset, const char* name, const char* value);

/**
* @brief 判断通信协议格式
*
* @param buf 协议数据缓冲区，存放协议数据
* @param buf_n 协议长度
* @return  协议格式是否正确，0：错误；1：正确
*/
extern int IsProtocolRight(const uint8_t* buf, uint16_t buf_n);

/**
* @brief 解析协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param name 数据名称
* @param value 返回64位无符号整型协议数据
* @return  0：解析成功，其他解析失败
*/
extern int GetProtocolData(const uint8_t* buf, const char* name, uint64_t* value);

/**
* @brief 解析协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param name 数据名称
* @param value 返回整型协议数据
* @return  0：解析成功，其他解析失败
*/
extern int GetProtocolDataInt(const uint8_t* buf, const char* name, int* value);

/**
* @brief 解析协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param name 数据名称
* @param value 返回双精度浮点协议数据
* @return  0：解析成功，其他解析失败
*/
extern int GetProtocolDataDouble(const uint8_t* buf, const char* name, double* value);

/**
* @brief 解析协议数据
*
* @param buf 协议数据缓冲区，存放协议数据
* @param name 数据名称
* @param value 返回字符串数据
* @return  0：解析成功，其他解析失败
*/
extern int GetProtocolDataString(const uint8_t* buf, const char* name, char* value);

#ifdef __cplusplus
}
}
#endif

#endif /* INCLUDE_COMMUNICATION_H_ */
