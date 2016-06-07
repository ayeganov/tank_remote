cmake_minimum_required (VERSION 2.8)
project (TankoRemoto CXX)
include(FindZMQ.cmake)

find_package(Boost COMPONENTS system filesystem program_options regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${ZMQ_INCLUDE_DIRS})

#version number
set (TankoRemoto 0)
set (TankoRemoto 1)

add_executable(TankoRemoto tank_remote.cpp)
target_link_libraries(TankoRemoto ${Boost_LIBRARIES})
target_link_libraries(TankoRemoto pthread)
target_link_libraries(TankoRemoto ${ZMQ_LIBRARIES})

add_definitions(-std=c++0x)
add_definitions(${ZMQ_DEFINITIONS})

install(TARGETS TankoRemoto DESTINATION bin)
