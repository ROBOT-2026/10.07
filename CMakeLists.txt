# ============================================================================
# CMakeLists.txt — 大恒相机拉流 + rm_log 日志 demo
#
# 构建（DAHENG_SDK_DIR 缺省为 ~/Downloads，会自动递归查找 GxIAPI.h 和
# libgxiapi.so，也可显式指定完整 SDK 所在目录）:
#   mkdir build && cd build
#   cmake .. -DDAHENG_SDK_DIR=/home/robot/Downloads
#   make -j
#
# 运行（连好相机即可；CMake 已把 SDK 库目录写进 rpath，无需再 export
# LD_LIBRARY_PATH）:
#   ./daheng_demo                按默认序列号打开
#   ./daheng_demo <相机序列号>    按指定序列号打开
# ============================================================================

cmake_minimum_required(VERSION 3.16)
project(daheng_camera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Threads REQUIRED)

# ---- 大恒 Galaxy SDK：递归自动定位头文件与库 ----
if(NOT DAHENG_SDK_DIR)
  set(DAHENG_SDK_DIR "$ENV{HOME}/Downloads")   # 默认去 ~/Downloads 下找
endif()
file(GLOB_RECURSE GX_HEADERS_LIST "${DAHENG_SDK_DIR}/GxIAPI.h")
file(GLOB_RECURSE GX_LIBS_LIST
     "${DAHENG_SDK_DIR}/libgxiapi.so"
     "${DAHENG_SDK_DIR}/libgxapi.so"
     "${DAHENG_SDK_DIR}/libGxIAPI.so")
if(NOT GX_HEADERS_LIST OR NOT GX_LIBS_LIST)
  message(FATAL_ERROR "在 ${DAHENG_SDK_DIR} 下未找到 GxIAPI.h / libgxiapi.so, "
                      "请 -DDAHENG_SDK_DIR=<完整 SDK 所在目录>（如 ~/Downloads）")
endif()

# 若磁盘上同时存在多份 SDK（例如解压出来的旧版 demo 头文件），
# 优先选 Galaxy_camera 官方目录（inc/ 与 lib/）里的，避免误用旧版本
set(GX_HEADER_PATH "")
foreach(h ${GX_HEADERS_LIST})
  if(h MATCHES "Galaxy_camera/inc")
    set(GX_HEADER_PATH ${h})
    break()
  endif()
endforeach()
if(NOT GX_HEADER_PATH)
  list(GET GX_HEADERS_LIST 0 GX_HEADER_PATH)
endif()
get_filename_component(GX_INCLUDE_DIR "${GX_HEADER_PATH}" DIRECTORY)

set(GX_LIB_PATH "")
foreach(l ${GX_LIBS_LIST})
  if(l MATCHES "Galaxy_camera/lib")
    set(GX_LIB_PATH ${l})
    break()
  endif()
endforeach()
if(NOT GX_LIB_PATH)
  list(GET GX_LIBS_LIST 0 GX_LIB_PATH)
endif()
get_filename_component(GX_LIB_DIR "${GX_LIB_PATH}" DIRECTORY)

message(STATUS "Found GxIAPI.h  : ${GX_HEADER_PATH}")
message(STATUS "Found libgxiapi : ${GX_LIB_PATH}")

# ---- rm_log 日志库（封装 spdlog，header-only）----
set(UTIL_DIR "${CMAKE_CURRENT_LIST_DIR}/util")

add_executable(daheng_demo
  daheng_demo.cpp
  armor_detector.cpp
  "${UTIL_DIR}/rm_log/src/rm_log.cpp"
)
target_include_directories(daheng_demo PRIVATE
  ${GX_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  "${UTIL_DIR}"
  "${UTIL_DIR}/rm_log/include"
)
target_link_libraries(daheng_demo PRIVATE
  ${OpenCV_LIBS}
  Threads::Threads
  "${GX_LIB_PATH}"
)

# 把 SDK 库目录写进 rpath：编译完直接 ./daheng_demo 就能跑，
# 省掉 export LD_LIBRARY_PATH（libgxiapi.so 还依赖同目录下的
# libavformat 等库，rpath 一并解决）
set_target_properties(daheng_demo PROPERTIES
  BUILD_RPATH "${GX_LIB_DIR}"
)
