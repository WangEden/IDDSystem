cmake_minimum_required(VERSION 3.20.0)
project(visual_window)

add_definitions(-DAPI_EXPORTS)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

# 包含头文件
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugin)
include_directories(/usr/local/cuda/targets/aarch64-linux/include)
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(/usr/local/cuda/targets/aarch64-linux/lib)

# 查找依赖库
find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5 COMPONENTS Network REQUIRED)
find_package(OpenCV)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
MESSAGE(STATUS "OpenCV version: " ${OpenCV_VERSION})

# -----------------------------------------------------------------------------
# 编译 myplugins 动态库
file(GLOB PLUGIN_SRC ${PROJECT_SOURCE_DIR}/plugin/*.cu)
add_library(myplugins SHARED ${PLUGIN_SRC})
target_link_libraries(myplugins nvinfer cudart)

# -----------------------------------------------------------------------------
# 编译 test 应用（简单版）
add_executable(test "test/test.cpp")
target_link_libraries(test
    ${OpenCV_LIBS}
)

# -----------------------------------------------------------------------------
# 编译 visual_window 应用（正式版）

# 查找源文件
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.cu)
file(GLOB MAIN_SRC "./*.cpp")
# list(REMOVE_ITEM MAIN_SRC "./test.cpp")
file(GLOB HEAD_FILES "./*.h" "./include/*.h")
file(GLOB UI_FILES "./*.ui")

# Qt处理UI
qt5_wrap_ui(WRAP_FILES ${UI_FILES})

# 添加资源文件
set(RCC_FILES rew.qrc)

# 创建可执行程序
add_executable(${PROJECT_NAME}
    ${MAIN_SRC}
    ${SRC_FILES}
    ${HEAD_FILES}
    ${RCC_FILES}
    ${WRAP_FILES}
)

# 连接库
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::Network
    X11
    nvinfer
    cudart
    myplugins
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

