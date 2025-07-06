/**
 * @file filter_interface.h
 *
 * @brief  滤波接口
 * @author chenchen
 * @version 11.0.0
 * @date 2020-09-11
 *
 */

#ifndef FILTER_INTERFACE_H_
#define FILTER_INTERFACE_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

#define ONLINEFILTERNUM 10//!< 滑动均值滤波器数组长度
#define MoveAverageFilter_BUF 2000//!< 滑动均值滤波的缓冲区大小


/*-------------------------------------------------------------------------*/
/**
  @brief	滑动均值滤波

	滑动均值滤波器
 */
/*-------------------------------------------------------------------------*/
typedef struct MoveAverageFilter{
	int buf_num;///< 滑动均值滤波器数组长度
	double buf[MoveAverageFilter_BUF];///< 定义滑动均值滤波的缓冲区
	int _index;///< 索引
	int one_flag;///< 初始标识
}MoveAverageFilter;

/*-------------------------------------------------------------------------*/
/**
  @brief	滑动均值滤波

	滑动均值滤波器数组
 */
/*-------------------------------------------------------------------------*/
typedef struct MoveAverageFilters{
	MoveAverageFilter maf[ONLINEFILTERNUM];///< 滑动均值滤波器数组
	int dof; ///< 维度
}MoveAverageFilters;

/**
* @brief 创建在线滑动均值滤波器
*
* @param maf 滑动均值滤波器
* @param buf_num 缓存区数量
*/
extern void init_move_average_filter_online(MoveAverageFilter* maf, int buf_num);

/**
* @brief 在线滑动均值滤波
*
* @param maf 滑动均值滤波器
* @param value 待滤波数据
* @return double 滤波后数据
*/
extern double move_average_filter_online(MoveAverageFilter* maf, double value);

/**
* @brief 创建多个在线滑动均值滤波器
*
* @param mafs 滑动均值滤波器数组对象
* @param buf_num 缓存区数量
* @param _dof 数据数组维度
*/
extern void init_move_average_filter_onlines(MoveAverageFilters* mafs, int buf_num, int _dof);

/**
* @brief 多维在线滑动均值滤波
*
* @param mafs 滑动均值滤波器数组对象
* @param value_in 待滤波数据数组
* @param value_out 返回滤波后数据数组
* @param _dof 数据维度
*/
extern void move_average_filter_onlines(MoveAverageFilters* mafs, double* value_in, double* value_out, int _dof);



#define MATRIX_MAX 30//!< 卡尔曼滤波矩阵最大维度

/*-------------------------------------------------------------------------*/
/**
  @brief	矩阵的结构体声明

	声明一个卡尔曼滤波器所需要用到的参数的矩阵结构定义
 */
/*-------------------------------------------------------------------------*/
typedef struct RMATRIX{
	double data[MATRIX_MAX][MATRIX_MAX]; ///< RMATRIX数组
	int row; ///< 行
	int col; ///< 列
}RMATRIX;

/*-------------------------------------------------------------------------*/
/**
  @brief	卡尔曼滤波

	卡尔曼滤波器
 */
/*-------------------------------------------------------------------------*/
typedef struct KalmanFilter{
	RMATRIX A; ///<系统动力学矩阵，尺寸为(n, n)
	RMATRIX B; ///<输入矩阵，尺寸为(n,1)
	RMATRIX C; ///< 输出矩阵，尺寸为(m, n)
	RMATRIX Q; ///< 过程噪声的协方差矩阵，尺寸为(n, n)
	RMATRIX R; ///< 测量噪声的协方差矩阵，尺寸为(m, m)
	RMATRIX P; ///< 估计误差的协方差矩阵，尺寸为(n, n)
	RMATRIX K; ///< 卡尔曼矩阵中间系数
	RMATRIX I; ///< 卡尔曼矩阵中间系数
	RMATRIX P0; ///< 初始协方差矩阵
	RMATRIX x0; ///< 初始状态
	int m; ///< 矩阵(行)
	int n; ///< 矩阵（列）
	double dt; ///< 数据间隔周期
	RMATRIX x_hat;///< 状态数据
	RMATRIX x_hat_new;///< 状态数据

	RMATRIX AT; ///< 滤波中间数据
	RMATRIX CT;///< 滤波中间数据
	RMATRIX y;///< 滤波输出数据
	RMATRIX tmp;///< 临时变量
	RMATRIX tmp1;///< 临时变量
}KalmanFilter;

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param kf 初始化卡尔曼滤波器
* @param dt 采样周期
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P0  估计误差的协方差矩阵初值，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
*/
extern void initKalmanFilter(KalmanFilter* kf,double dt, RMATRIX* A, RMATRIX* C, RMATRIX* Q, RMATRIX* R, RMATRIX* P0, RMATRIX* x0);

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param kf 初始化卡尔曼滤波器
* @param dt 采样周期
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param B  输入矩阵，尺寸为(n,1)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P0  估计误差的协方差矩阵初值，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
*/
extern void KalmanFilterInit(KalmanFilter* kf,double dt, RMATRIX* A, RMATRIX* B, RMATRIX* C, RMATRIX* Q, RMATRIX* R, RMATRIX* P0, RMATRIX* x0);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器 
* @param y 测量值
* @return RMATRIX* 滤波后状态数据
*/
extern RMATRIX* KalmanFilter_update(KalmanFilter* kf, RMATRIX* y);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器 
* @param y 测量值
* @param u 输入值
* @return RMATRIX* 滤波后状态数据
*/
extern RMATRIX* KalmanFilterUpdate(KalmanFilter* kf, RMATRIX* y, double u);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param A 系统动力学矩阵，尺寸为(n,n)
* @return RMATRIX* 滤波后状态数据
*/
extern RMATRIX* KalmanFilter_update1(KalmanFilter* kf, RMATRIX* y, RMATRIX* A);

/**
* @brief 获取卡尔曼滤波器的状态
*
* @param kf 创建的卡尔曼滤波器
* @return RMATRIX* 状态数据
*/
extern RMATRIX* getKalmanFilter_state(KalmanFilter* kf);

/**
* @brief 获取卡尔曼滤波器的输出
*
* @param kf 创建的卡尔曼滤波器
* @return RMATRIX* 滤波后输出数据
*/
extern RMATRIX* getKalmanFilter_out(KalmanFilter* kf);

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param kf 初始化卡尔曼滤波器
* @param dt 采样周期
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P  估计误差的协方差矩阵，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
* @param m 输入矩阵尺寸
* @param n 输入矩阵尺寸
*/
extern void initKalmanFilter_d(KalmanFilter* kf,double dt, double* A, double* C, double* Q, double* R, double* P, double* x0, int m, int n);

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param kf 初始化卡尔曼滤波器
* @param dt 采样周期
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param B  输入矩阵，尺寸为(n,1)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P  估计误差的协方差矩阵，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
* @param m 输入矩阵尺寸
* @param n 输入矩阵尺寸
*/
extern void KalmanFilterInit_d(KalmanFilter* kf,double dt, double* A, double* B, double* C, double* Q, double* R, double* P, double* x0, int m, int n);

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param kf 初始化卡尔曼滤波器
* @param dt 采样周期
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P  估计误差的协方差矩阵，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
* @param m 输入矩阵尺寸
* @param n 输入矩阵尺寸
*/
extern void initKalmanFilter_d1(KalmanFilter* kf,double dt, double** A, double** C, double** Q, double** R, double** P, double* x0, int m, int n);

/**
* @brief 使用指定的矩阵参数来创建卡尔曼滤波器
*
* @param kf 初始化卡尔曼滤波器
* @param dt 采样周期
* @param A  系统动力学矩阵，尺寸为(n,n)
* @param B  输入矩阵，尺寸为(n,1)
* @param C  输出矩阵，尺寸为(m,n)
* @param Q  过程噪声的协方差矩阵，尺寸为(n,n)
* @param R  测量噪声的协方差矩阵，尺寸为(m,m)
* @param P  估计误差的协方差矩阵，尺寸为(n,n)
* @param x0 初始状态，输入尺寸为(n,1)
* @param m 输入矩阵尺寸
* @param n 输入矩阵尺寸
*/
extern void KalmanFilterInit_d1(KalmanFilter* kf,double dt, double** A, double* B,double** C, double** Q, double** R, double** P, double* x0, int m, int n);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param filter_state 返回滤波后状态数据，允许NULL（使用内部输区）
* @return double* 状滤波后状态数据
*/
extern double* KalmanFilter_update_d(KalmanFilter* kf, double* y, double* filter_state);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param u 输入值
* @param filter_state 返回滤波后状态数据，允许NULL（使用内部输区）
* @return double* 状滤波后状态数据
*/
extern double* KalmanFilterUpdate_d(KalmanFilter* kf, double* y,double u, double* filter_state);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param A 系统动力学矩阵(一维数组形式)
* @param filter_state 返回滤波后状态数据，允许NULL（使用内部输区）
* @return double* 状滤波后状态数据
*/
extern double* KalmanFilter_update1_d(KalmanFilter* kf, double* y, double* A, double* filter_state);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param A 系统动力学矩阵(一维数组形式)
* @param u 输入值
* @param filter_state 返回滤波后状态数据，允许NULL（使用内部输区）
* @return double* 状滤波后状态数据
*/
extern double* KalmanFilterUpdate1_d(KalmanFilter* kf, double* y, double* A,double u, double* filter_state);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param A 系统动力学矩阵(二维数组形式)
* @param filter_state 返回滤波后状态数据，允许NULL（使用内部输区）
* @return double* 状滤波后状态数据
*/
extern double* KalmanFilter_update1_d1(KalmanFilter* kf, double* y, double** A, double* filter_state);

/**
* @brief 卡尔曼滤波器参数更新
*
* @param kf 创建的卡尔曼滤波器
* @param y 测量值
* @param A 系统动力学矩阵(二维数组形式)
* @param u 输入值
* @param filter_state 返回滤波后状态数据，允许NULL（使用内部输区）
* @return double* 状滤波后状态数据
*/
double* KalmanFilterUpdate1_d1(KalmanFilter* kf, double* y, double** A, double u,double* filter_state);

/**
* @brief 获取卡尔曼滤波器的状态
*
* @param kf 创建的卡尔曼滤波器
* @param state 返回状态数据，允许NULL（使用内部输区）
* @return double* 状态数据
*/
extern double* getKalmanFilter_state_d(KalmanFilter* kf, double* state);

/**
* @brief 获取卡尔曼滤波器的输出
*
* @param kf 创建的卡尔曼滤波器
* @param out 返回输出数据，允许NULL（使用内部输区）
* @return double* 滤波后输出数据
*/
extern double* getKalmanFilter_out_d(KalmanFilter* kf, double* out);

/**
* @brief 系统动力学矩阵, 状态为位置，速度，加速度组成的向量
*
* @param A 系统动力学矩阵，尺寸为(n, n)
* @param dt 采样周期
* @param n 输入矩阵尺寸
* @return double* 系统动力学矩阵
*/
extern double* getKalmanFilterA(double* A, double dt, int n);

/**
* @brief 输出矩阵, 状态为位置，速度，加速度组成的向量
*
* @param C 输出矩阵，尺寸为(m, n)
* @param dt 采样周期
* @param m 输入矩阵尺寸
* @return double* 输出矩阵
*/
extern double* getKalmanFilterC(double* C, double dt, int m);

/**
* @brief 过程噪声的协方差矩阵, 状态为位置，速度，加速度组成的向量
*
* @param Q 过程噪声的协方差矩阵，尺寸为(n, n)
* @param value 过程噪声数据
* @param n 输入矩阵尺寸
* @return double* 过程噪声的协方差矩阵
*/
extern double* getKalmanFilterQ(double* Q, double value, int n);

/**
* @brief 测量噪声的协方差矩阵, 状态为位置，速度，加速度组成的向量
*
* @param R 测量噪声的协方差矩阵，尺寸为(m, m)
* @param value 测量噪声数据
* @param m 输入矩阵尺寸
* @return double* 测量噪声的协方差矩阵
*/
extern double* getKalmanFilterR(double* R, double value, int m);

/**
* @brief 估计误差的协方差矩阵, 状态为位置，速度，加速度组成的向量
*
* @param P 估计误差的协方差矩阵，尺寸为(n, n)
* @param value 估计误差数据
* @param n 输入矩阵尺寸
* @return double* 估计误差的协方差矩阵
*/
extern double* getKalmanFilterP(double* P, double value, int n);

#define ARRAY_DIM 50  ///< 这个参数必须大于等于2*MAX_POLE_COUNT，因为一些滤波器的多项式使用2*NumPoles进行定义

/*-------------------------------------------------------------------------*/
/**
  @brief 多种无限脉冲响应滤波器
 */
/*-------------------------------------------------------------------------*/
typedef enum TIIRPassTypes {
	iirLPF,  ///< 无限脉冲响应低通滤波
	iirHPF,  ///< 无限脉冲响应高通滤波
	iirBPF,  ///< 无限脉冲响应带通滤波
	iirNOTCH,  ///< 无限脉冲响应陷波滤波
	iirALLPASS ///< 无限脉冲响应全通滤波
}TIIRPassTypes;

/*-------------------------------------------------------------------------*/
/**
@brief	滤波系数

*/
/*-------------------------------------------------------------------------*/
typedef struct TIIRCoeff {
	double a0[ARRAY_DIM]; ///< 系数
	double a1[ARRAY_DIM]; ///< 系数
	double a2[ARRAY_DIM]; ///< 系数
	double a3[ARRAY_DIM]; ///< 系数
	double a4[ARRAY_DIM]; ///< 系数
	double b0[ARRAY_DIM]; ///< 系数
	double b1[ARRAY_DIM]; ///< 系数
	double b2[ARRAY_DIM]; ///< 系数
	double b3[ARRAY_DIM]; ///< 系数
	double b4[ARRAY_DIM]; ///< 系数
	int NumSections; ///< 片段数量
}TIIRCoeff;

/*-------------------------------------------------------------------------*/
/**
  @brief 可用的滤波器
 */
/*-------------------------------------------------------------------------*/
typedef enum TFilterPoly {
	BUTTERWORTH,  ///< 巴特沃斯滤波器
	GAUSSIAN,  ///< 高斯滤波器
	BESSEL,  ///< 贝赛尔滤波器
	ADJUSTABLE,  ///< 可调滤波器
	CHEBYSHEV, ///< 切比雪夫滤波器
	INVERSE_CHEBY, ///< 反切比雪夫滤波器
	PAPOULIS,  ///< 帕普利斯滤波器
	ELLIPTIC,  ///< 考尔滤波器
	NOT_IIR ///< 测试用
}TFilterPoly;

/*-------------------------------------------------------------------------*/
/**
  @brief	TIIRF的参数定义

	需要定义滤波类型（高通、低通等等）、截止频率、带宽、增益、滤波器原型、极点数量、通带波纹、阻带衰减、过渡带宽调整系数
 */
/*-------------------------------------------------------------------------*/
typedef struct TIIRFilterParams {
	TIIRPassTypes IIRPassType;     ///< 定义滤波类型：低通，高通，等等
	double OmegaC;                 ///< 截止频率。无限脉冲响应滤波器的3dB角频率用于低通和高通滤波，中心频率用于带通和陷波滤波
	double BW;                     ///< 带宽。无限脉冲响应滤波器的3dB带宽用于带通和陷波滤波
	double dBGain;                 ///< 滤波器的增益

	///< 定义了要使用的低通滤波原型
	TFilterPoly ProtoType;  ///< 如Butterworth, Cheby等等
	int NumPoles;           ///< 极点数量*/
	double Ripple;          ///< 椭圆滤波和切比雪夫滤波的通带波纹
	double StopBanddB;      ///< 椭圆滤波和逆切比雪夫的阻带衰减（db）
	double Gamma;           ///< 可调整高斯滤波的过渡带宽系数，-1 <= Gamma <= 1 
}TIIRFilterParams;

/*-------------------------------------------------------------------------*/
/**
  @brief	TIIRF定义

	需要定义滤波类型（高通、低通等等）、截止频率、带宽、增益、滤波器原型、极点数量、通带波纹、阻带衰减、过渡带宽调整系数
 */
/*-------------------------------------------------------------------------*/
typedef struct TIIRFilter{
	TIIRFilterParams IIRFilt;  ///< 在IIRFilterCode.h中进行了定义
	TIIRCoeff IIRCoeff; ///<滤波系数
	double RegX1[ARRAY_DIM]; ///<中间数据
	double RegX2[ARRAY_DIM]; ///<中间数据
	double RegY1[ARRAY_DIM]; ///<中间数据
	double RegY2[ARRAY_DIM]; ///<中间数据
	int flag_one;///<滤波初始表示
}TIIRFilter;

/*-------------------------------------------------------------------------*/
/**
  @brief	TIIRF数组

	数组最大容量为是个TIIRF
 */
/*-------------------------------------------------------------------------*/
typedef struct TIIRFilters{
	TIIRFilter irrfiler[10];  ///< IIRF数组
	int n; ///< 数据维度
}TIIRFilters;

/**
* @brief 初始化滤波器
*
* @param iirfiler 滤波器数据
* @param types 滤波类型
* @param cornerFreq 截至频率
* @param sampleFreq 采样频率
*/
extern void initTIIRFilter(TIIRFilter* iirfiler, TIIRPassTypes types, double cornerFreq, double sampleFreq);

/**
* @brief 滤波
*
* @param iirfiler 滤波器数据
* @param Signal 输入数据
* @return double 返回滤波后数据
*/
extern double IIRFilter(TIIRFilter* iirfiler, double Signal);

/**
* @brief 初始化多维滤波器
*
* @param iirfilers 滤波器数据数组
* @param types 滤波类型
* @param cornerFreq 截至频率
* @param sampleFreq 采样频率
* @param n 数据维度
*/
extern void initTIIRFilters(TIIRFilters* iirfilers, TIIRPassTypes types, double cornerFreq, double sampleFreq, int n);

/**
* @brief 多维滤波
*
* @param iirfilers 滤波器数据数组
* @param in 输入数据数组
* @param out 返回滤波后数据数组
*/
extern void IIRFilters(TIIRFilters* iirfilers, double* in, double* out);

/**
* @brief 零相位滤波（离线滤波）
*
* @param iirfiler 滤波器数据
* @param Signal_in 输入数据数组
* @param Signal_out 返回滤波后数据数组
* @param n 数据维度
* @return double 返回滤波后数据地址
*/
extern double* IIRFilterZeroPhase(TIIRFilter* iirfiler, double* Signal_in, double* Signal_out, int n);


#ifdef __cplusplus
}
}
#endif


#endif /* FILTER_INTERFACE_H_ */
