cmake_minimum_required(VERSION 3.16)
project(armor_pnp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
#  CLion 使用要点
#
#  1. 打开工程：File -> Open -> 选 armor_pnp 目录
#  2. 运行前改工作目录：
#     Run -> Edit Configurations -> Working directory = $ProjectFileDir$
#  3. 切到 Release（Debug 下推理慢 5~10 倍）：
#     右上角下拉框 Debug -> Release
# ============================================================

# ---------- OpenCV ----------
# 如果系统装的 OpenCV 在 /usr/local，取消下面注释
# set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")

find_package(OpenCV 4 REQUIRED COMPONENTS
             core imgproc imgcodecs highgui dnn calib3d)
message(STATUS "OpenCV 版本: ${OpenCV_VERSION}")
message(STATUS "OpenCV 路径: ${OpenCV_DIR}")

# ---------- 海康 MVS SDK ----------
set(MVS_ROOT "/opt/MVS" CACHE PATH "MVS SDK 根目录")
find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h
          HINTS "${MVS_ROOT}/include")
find_library(MVS_LIBRARY NAMES MvCameraControl
             HINTS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib/aarch64")

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
    message(FATAL_ERROR
        "找不到海康 MVS SDK。\n"
        "  默认查找路径: /opt/MVS\n"
        "  如果你的 SDK 装在别处，用:\n"
        "    cmake .. -DMVS_ROOT=/你的/路径\n"
        "  Ubuntu 安装包一般解压到 /opt/MVS")
endif()

message(STATUS "MVS 头文件: ${MVS_INCLUDE_DIR}")
message(STATUS "MVS 库: ${MVS_LIBRARY}")

# ---------- 头文件路径 ----------
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${MVS_INCLUDE_DIR})

# ---------- 主程序 ----------
add_executable(armor_pnp
    src/main.cpp
    src/pnp_solver.cpp
    src/kalman_tracker.cpp
    src/camera_calib.cpp
)
target_link_libraries(armor_pnp PRIVATE ${MVS_LIBRARY} ${OpenCV_LIBS})

# 运行时能找到 MVS 的 .so，不用手动 export LD_LIBRARY_PATH
get_filename_component(MVS_RUNTIME_DIR "${MVS_LIBRARY}" DIRECTORY)
set_target_properties(armor_pnp PROPERTIES BUILD_RPATH "${MVS_RUNTIME_DIR}")

# ---------- 标定工具（可选，不需要海康相机） ----------
# 用普通摄像头或图片标定，生成 calib.yml
if(EXISTS "${CMAKE_SOURCE_DIR}/tools/calibrate.cpp")
    add_executable(calibrate
        tools/calibrate.cpp
        src/camera_calib.cpp
    )
    target_link_libraries(calibrate PRIVATE ${OpenCV_LIBS})
    target_compile_options(calibrate PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# ---------- 编译警告 ----------
target_compile_options(armor_pnp PRIVATE -Wall -Wextra -Wno-unused-parameter)

# CLion 部分版本会自动使用这个属性设工作目录
set_target_properties(armor_pnp PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
