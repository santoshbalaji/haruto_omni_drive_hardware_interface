cmake_minimum_required(VERSION 3.16)
project(haruto_omni_drive_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  haruto_omni_drive_hardware_interface
  SHARED
  src/serial_control.cpp
  src/omni_drive_system_interface.cpp
)

target_compile_features(haruto_omni_drive_hardware_interface PUBLIC cxx_std_17)

target_include_directories(haruto_omni_drive_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/haruto_omni_drive_hardware_interface>
)

ament_target_dependencies(
  haruto_omni_drive_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(haruto_omni_drive_hardware_interface omni_drive_hardware_interface_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/haruto_omni_drive_hardware_interface
)

install(TARGETS haruto_omni_drive_hardware_interface
  EXPORT haruto_omni_drive_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# ament_export_targets(export_haruto_omni_drive_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
