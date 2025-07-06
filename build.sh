#!/bin/bash
set -e

BUILD_DIR="build"

# 设置环境变量（按需）
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=$(pwd)/libtorch/lib:$LD_LIBRARY_PATH

# 删除旧构建目录
if [ -d "$BUILD_DIR" ]; then
    echo "Removing old build directory..."
    rm -rf "$BUILD_DIR"
fi

# 创建并进入构建目录
mkdir "$BUILD_DIR"
cd "$BUILD_DIR"

# 初始化构建
echo "[INFO] Initializing build directory..."
cmake -DCMAKE_BUILD_TYPE=Debug ..

# 编译
echo "[INFO] Building RobotSystem..."
cmake --build . -- -j$(nproc)

# if [ -f RobotSystem ]; then
#     echo "[INFO] Setting cap_sys_nice on RobotSystem..."
#     sudo setcap cap_sys_nice=eip RobotSystem
# else
#     echo "[WARNING] RobotSystem not found! Skipping setcap."
#     ls -l
# fi

