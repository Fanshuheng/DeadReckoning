cmake_minimum_required(VERSION 3.13)
project(dead_reckoning)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/include/eigen3")
include_directories(
        ${PROJECT_SOURCE_DIR}
)

add_executable(${PROJECT_NAME}_node
        ./main.cpp
        ./DeadReckoning.cpp
        ./ClimbotKinematics.cpp
        ./ClimberKinematics5D.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${EIGEN3_LIBS}
)