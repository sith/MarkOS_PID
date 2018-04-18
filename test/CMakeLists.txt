cmake_minimum_required(VERSION 3.10)
project(MarkOS_PID_tests)

add_executable(${PROJECT_NAME}
        main.cpp
        PIDTest.cpp PIDTest.h)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MarkOS_PID)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MarkOS_commons)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MarkOS_cycle)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC MarkOS_time)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC gtest gtest_main)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC gmock gmock_main)
