#!/bin/bash
set -e  # 任何出错都立即退出

# 设置 pkg-config 路径
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH

BUILD_DIR=build

# 删除旧的构建目录
if [ -d "$BUILD_DIR" ]; then
    echo "Removing old build directory..."
    rm -rf "$BUILD_DIR"
fi

# 创建并进入新目录
mkdir "$BUILD_DIR"
cd "$BUILD_DIR"

# 配置 + 编译（启用调试模式）
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../deploy ..

# 使用 `nproc` 来并行化构建（如果可用），否则使用默认的并行数
JOBS=$(command -v nproc &>/dev/null && nproc || echo 4)
make -j$JOBS

make install
