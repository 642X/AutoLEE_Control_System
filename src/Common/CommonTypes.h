#ifndef SIAT_COMMON_TYPES_H
#define SIAT_COMMON_TYPES_H

#include <string>
#include <optional>
#include <unordered_map>
#include <algorithm>
#include <Eigen/Dense>
namespace SIAT {



/**
 * @brief 三维向量模板结构，支持基本运算与 Eigen 转换。
 *
 * 用于替代裸数组表示的三维向量（如位置、力等），具备以下特性：
 * - 支持使用 `x`, `y`, `z` 成员直接访问
 * - 支持 `[]` 下标访问（读写）
 * - 支持向量加法、减法、数乘等运算符
 * - 支持与 `Eigen::Vector3<T>` 的互相转换
 *
 * 示例用法：
 * 
 *   Vec3d v1(1.0, 2.0, 3.0);
 *   Vec3d v2(0.5, 1.0, -1.0);
 *   Vec3d sum = v1 + v2;
 *   v1[0] = 10.0;  // 修改 x 分量
 *
 *   // 与 Eigen::Vector3d 转换
 *   Eigen::Vector3d ev = v1.toEigen();
 *   Vec3d v3(ev);  // 从 Eigen::Vector3d 构造
 */
template<typename T>
struct Vec3 {
    T x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    // 从 Eigen::Vector3<T> 构造
    explicit Vec3(const Eigen::Matrix<T, 3, 1>& v) : x(v[0]), y(v[1]), z(v[2]) {}

    // 转换为 Eigen::Vector3<T>
    Eigen::Matrix<T, 3, 1> toEigen() const {
        return Eigen::Matrix<T, 3, 1>(x, y, z);
    }

    // 下标访问（可读写）
    T& operator[](int idx) {
        switch (idx) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("Vec3 index out of range");
        }
    }

    // 下标访问（只读）
    const T& operator[](int idx) const {
        switch (idx) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("Vec3 index out of range");
        }
    }

    // 向量加减乘除运算
    Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator*(T scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vec3 operator/(T scalar) const { return {x / scalar, y / scalar, z / scalar}; }

    Vec3& operator+=(const Vec3& other) { x += other.x; y += other.y; z += other.z; return *this; }
    Vec3& operator-=(const Vec3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
    Vec3& operator*=(T scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
    Vec3& operator/=(T scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }
};


using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;
using Vec3i = Vec3<int>;


enum class CommandType {
    kUnknown = 0,
    kStart,
    kStop,
    kPause,
    kReset
};

struct State {
    Eigen::Vector3d pos; // 位置向量
    Eigen::Matrix3d rot; // 旋转矩阵
};

struct State02 {
    Eigen::Vector3d pos; // 位置向量
    Eigen::Quaterniond rot; // 四元数表示的旋转
};

struct ForceTorque {
    SIAT::Vec3d force;  ///< 力传感器读数，单位：N
    SIAT::Vec3d torque; ///< 力矩传感器读数，单位：Nm
};


} // namespace SIAT

#endif // SIAT_COMMON_TYPES_H
