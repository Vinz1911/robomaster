cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(robomaster)

option(BUILD_RUN_TESTS "build with testing" OFF)
find_package(Threads REQUIRED)

# Set C++ standards
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SRC_LIST src/can.cpp src/handler.cpp src/utils.cpp src/queue.cpp src/robomaster.cpp src/data.cpp src/message.cpp src/payload.cpp)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Build shared library and demo
add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_THREAD_LIBS_INIT} atomic)

# Build demo project
add_executable(${PROJECT_NAME}_demo examples/main.cpp)
target_link_libraries(${PROJECT_NAME}_demo PRIVATE ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# Installation directories and rules
set(INSTALL_LIB_DIR lib)
set(INSTALL_INCLUDE_DIR include)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${INSTALL_LIB_DIR} ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION ${INSTALL_INCLUDE_DIR})

# Build with test's
if(BUILD_RUN_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    add_executable(run_tests tests/main_test.cpp tests/data_test.cpp tests/message_test.cpp tests/utils_test.cpp tests/queue_test.cpp)
    target_link_libraries(run_tests PRIVATE GTest::GTest ${PROJECT_NAME})
    add_test(NAME run_tests COMMAND run_tests)
endif()

