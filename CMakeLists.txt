cmake_minimum_required(VERSION 3.16)

project(cartPole LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(
    MUJOCO_ROOT
    "/home/shiroha_pantsu/miniconda3/envs/robot/lib/python3.12/site-packages/mujoco/"
    CACHE PATH
    "mujoco root directory"
)

# 引入
find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS "${MUJOCO_ROOT}/include"
)

add_library(mujoco SHARED IMPORTED)

set_target_properties(mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_ROOT}/libmujoco.so.3.14.0"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)

find_package(glfw3 REQUIRED CONFIG)
find_package(Eigen3 REQUIRED CONFIG)

# 编译库文件
add_library(
    controller
    STATIC
    src/controller.cpp
)

add_library(
    simulation
    STATIC
    src/simulation.cpp
)

# 编译可执行程序
add_executable(
    simu_cartPole
    src/main.cpp
)

# 关联头文件
target_include_directories(
    controller
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(
    simulation
    PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# 关联外部依赖
target_link_libraries(
    controller
    PUBLIC 
    Eigen3::Eigen
)

target_link_libraries(
    simulation
    PUBLIC
    controller mujoco glfw
)

target_link_libraries(
    simu_cartPole
    PRIVATE 
    controller
    simulation
)

set_target_properties(simu_cartPole PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
    BUILD_RPATH ${MUJOCO_ROOT}
)