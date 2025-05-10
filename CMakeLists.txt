cmake_minimum_required(VERSION 3.10)
project(Fast_RNRR VERSION 1.0.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖库
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(NanoFlann REQUIRED)
find_package(OpenMesh REQUIRED)

# 添加编译选项
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OPENMP)
endif()

# 设置包含目录
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${NANOFLANN_INCLUDE_DIR}
    ${OPENMESH_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
)

# 收集源文件
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "src/tools/*.cpp"
    "src/geodesic/*.cpp"
)

add_library(Fast_RNRR STATIC ${SOURCES})

set_target_properties(Fast_RNRR PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "include/Fast_RNRR/INonRigidReg.h"
)

target_link_libraries(Fast_RNRR
    OpenMeshCore
    OpenMeshTools
)

add_definitions(-D_USE_MATH_DEFINES)

install(TARGETS Fast_RNRR
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)
