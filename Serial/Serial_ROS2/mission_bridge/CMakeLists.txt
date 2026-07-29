cmake_minimum_required(VERSION 3.5)
project(mission_bridge C CXX)

# 默认使用 C++17 编译
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 复用仓库已有的 common 日志模块和 Serial_32 传输层帧解析代码
set(MISSION_BRIDGE_COMMON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/common.c)
set(MISSION_BRIDGE_SERIAL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../Serial_32/src/Serial.c)

# Serial.c 以 SERIAL_HOST_TEST 编译，脱离 STM32 HAL 依赖
set_source_files_properties(${MISSION_BRIDGE_SERIAL_SRC}
  PROPERTIES COMPILE_DEFINITIONS SERIAL_HOST_TEST)

add_library(mission_bridge_core
  src/protocol.cpp
  src/bridge_state.cpp
  ${MISSION_BRIDGE_COMMON_SRC}
  ${MISSION_BRIDGE_SERIAL_SRC}
)

target_include_directories(mission_bridge_core PUBLIC
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../Serial_32/include
)

add_executable(mission_bridge_node
  src/mission_bridge_node.cpp
  src/serial_port.cpp
)
target_link_libraries(mission_bridge_node mission_bridge_core)
target_include_directories(mission_bridge_node PRIVATE include)
ament_target_dependencies(mission_bridge_node rclcpp std_msgs)

if(BUILD_TESTING)
  enable_testing()
  foreach(test_name protocol crc session heartbeat start serial_connection)
    add_executable(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} mission_bridge_core)
    add_test(NAME test_${test_name} COMMAND test_${test_name})
  endforeach()
  add_test(
    NAME test_source_contract
    COMMAND ${CMAKE_COMMAND}
      -DNODE_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/mission_bridge_node.cpp
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_source_contract.cmake)
endif()

install(TARGETS mission_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
