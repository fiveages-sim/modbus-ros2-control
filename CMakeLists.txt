cmake_minimum_required(VERSION 3.16)
project(modbus_ros2_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        gripper_hardware_common
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

# Find libmodbus
find_library(MODBUS_LIBRARY
    NAMES modbus
    PATHS /usr/lib /usr/local/lib
)

if (NOT MODBUS_LIBRARY)
    message(FATAL_ERROR "libmodbus not found. Please install libmodbus-dev")
endif ()

## COMPILE
add_library(
        ${PROJECT_NAME}
        SHARED
        src/communicator/modbus_rtu_communicator.cpp
        src/grippers/modbus_gripper_base.cpp
        src/grippers/changingtek_gripper.cpp
        src/modbus_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link libmodbus
target_link_libraries(${PROJECT_NAME} PRIVATE ${MODBUS_LIBRARY})

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface modbus_ros2_control.xml)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

