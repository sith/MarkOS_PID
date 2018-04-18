cmake_minimum_required(VERSION 3.9)
project(MarkOS_PID)


add_library(${PROJECT_NAME} src/PID.cpp src/PID.h)

target_include_directories(${PROJECT_NAME} PUBLIC src)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MarkOS_commons)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MarkOS_cycle)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MarkOS_time)

add_subdirectory(test)

install(DIRECTORY "src/"
        DESTINATION "/Users/oleksandrabaukh/workspace/libs/MarkOS/MarkOS_PID")
