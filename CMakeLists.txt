cmake_minimum_required(VERSION 3.10)
project(RobotSystem VERSION 1.0.0 LANGUAGES CXX)


# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
# 设置 libtorch 路径
set(LIBTORCH_DIR ${CMAKE_SOURCE_DIR}/libtorch)
set(CMAKE_PREFIX_PATH ${LIBTORCH_DIR})

# 查找 libtorch
find_package(Torch REQUIRED)

# 使用 pkg-config 查找 pinocchio 和 hpp-fcl（确保 /opt/openrobots/lib/pkgconfig 已在 PKG_CONFIG_PATH 中）
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)
pkg_check_modules(HPP_FCL REQUIRED hpp-fcl)

# 自动添加src文件夹下所有 .cpp 文件
set(SRC_FILES 
    src/Log/Logger.cpp
    src/Common/tools.cpp
    src/RLModel/RLModel.cpp
    src/RobotController/RLRobotController.cpp
    src/RobotController/RLZMPRobotController.cpp
    src/RobotCommunication/RobotCommunication.cpp
    src/RobotSystem.cpp
)

set(MAIN_FILES 
    src/demo/main.cpp
)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES} ${MAIN_FILES})


# 添加 lib，pinocchio 和 hpp-fcl 的库目录
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/lib
    ${PINOCCHIO_LIBRARY_DIRS}
    ${HPP_FCL_LIBRARY_DIRS}
    ${LIBTORCH_DIR}/lib
)

# 添加头文件路径
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${PINOCCHIO_INCLUDE_DIRS}
    ${HPP_FCL_INCLUDE_DIRS}
    src
    src/Common  # 添加 CommonTypes.h 路径
    src/Log     # 添加 Logger.h 路径
    src/RLModel # 添加 RLModel.h 路径
)


# 链接所有依赖库
target_link_libraries(${PROJECT_NAME}
    ${TORCH_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
    ${HPP_FCL_LIBRARIES}
    RobotControl
    HYYRobotTechnology
    BusCommunication
    modbus
    nlopt
    xml2
    pthread
    rt
    dl
)


# 设置安装路径结构
include(GNUInstallDirs)

# 安装头文件（如果你想装 public headers，也可以写 include/*.h）
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# 安装自定义共享库（假设你编译了它们）
install(FILES
    ${CMAKE_SOURCE_DIR}/lib/libRobotControl.so
    ${CMAKE_SOURCE_DIR}/lib/libBusCommunication.so
    ${CMAKE_SOURCE_DIR}/lib/libHYYRobotTechnology.so
    ${CMAKE_SOURCE_DIR}/lib/libmodbus.so.5
    ${CMAKE_SOURCE_DIR}/lib/libnlopt.so.0
    ${CMAKE_SOURCE_DIR}/lib/libxml2.so.2
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# 安装 libtorch 的 .so 文件
file(GLOB TORCH_LIBS "${LIBTORCH_DIR}/lib/*.so*")
install(FILES ${TORCH_LIBS} DESTINATION ${CMAKE_INSTALL_LIBDIR})

# 如果目标机器没有安装 pinocchio/hpp-fcl，取消注释以下几行：
install(FILES
    /opt/openrobots/lib/libpinocchio.so.2.7.0
    /opt/openrobots/lib/libhpp-fcl.so
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 安装启动脚本（部署后可直接 ./run.sh 运行）
install(PROGRAMS ${CMAKE_SOURCE_DIR}/run.sh DESTINATION .)

# 安装资源目录 system 到 share/RobotSystem/system/
install(DIRECTORY ${CMAKE_SOURCE_DIR}/system/
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/system
        FILES_MATCHING PATTERN "*.*")

# 判断当前构建类型
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Debug mode: skipping RPATH setting")
else()
    # 设置 RPATH 使其指向 ../lib，相对可执行文件
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "$ORIGIN/../lib"
        BUILD_WITH_INSTALL_RPATH TRUE
        BUILD_RPATH "${LIBTORCH_DIR}/lib"
    )
    message(STATUS "RPATH set for non-Debug build")
endif()


# 导出 compile_commands.json，方便 clangd / LSP 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 输出调试信息
message(STATUS "Pinocchio include: ${PINOCCHIO_INCLUDE_DIRS}")
message(STATUS "Pinocchio libs: ${PINOCCHIO_LIBRARIES}")
message(STATUS "hpp-fcl include: ${HPP_FCL_INCLUDE_DIRS}")
message(STATUS "hpp-fcl libs: ${HPP_FCL_LIBRARIES}")
message(STATUS "Pinocchio lib dirs: ${PINOCCHIO_LIBRARY_DIRS}")
message(STATUS "hpp-fcl lib dirs: ${HPP_FCL_LIBRARY_DIRS}")
