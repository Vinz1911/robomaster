cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(robomaster)

option(BUILD_RUN_TESTS "build with testing" OFF)
find_package(Threads REQUIRED)

# Source files
set(SRC_LIST src/canbus.cpp src/handler.cpp src/utils.cpp src/queue.cpp src/robomaster.cpp src/data.cpp src/message.cpp)

# Build shared library
add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 23)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Example
add_executable(${PROJECT_NAME}_demo examples/main.cpp)
target_link_libraries(${PROJECT_NAME}_demo PRIVATE ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# Installation directories
set(INSTALL_LIB_DIR lib)
set(INSTALL_INCLUDE_DIR include)

# Installation rules
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${INSTALL_LIB_DIR} ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${INSTALL_INCLUDE_DIR})

if(BUILD_RUN_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()

    add_executable(run_tests
            tests/main_test.cpp
            tests/data_test.cpp
            tests/message_test.cpp
            tests/utils_test.cpp
            tests/queue_test.cpp)

    target_link_libraries(run_tests PRIVATE GTest::GTest ${PROJECT_NAME})
    add_test(NAME run_tests COMMAND run_tests)
endif()

