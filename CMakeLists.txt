cmake_minimum_required(VERSION 3.1)

project(PhidgetPressureSensorPlugin LANGUAGES CXX VERSION 1.0.0)
set(CMAKE_CXX_VERSION 17)

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
install(FILES etc/PhidgetPressureSensorPlugin.yaml DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

# TODO export this library
add_library(PhidgetPressureSensorDAQ SHARED src/PhidgetPressureSensorDAQ.cpp src/PhidgetPressureSensorDAQ.h)
target_include_directories(PhidgetPressureSensorDAQ PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
pkg_check_modules(libphidget22 REQUIRED IMPORTED_TARGET libphidget22)
target_link_libraries(PhidgetPressureSensorDAQ PUBLIC mc_rtc::mc_control)
target_link_libraries(PhidgetPressureSensorDAQ PUBLIC PkgConfig::libphidget22)

install(
  TARGETS PhidgetPressureSensorDAQ
  EXPORT ${TARGETS_EXPORT_NAME}
  )

set(plugin_SRC
  src/PhidgetPressureSensorPlugin.cpp
  )

set(plugin_HDR
  src/PhidgetPressureSensorPlugin.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(PhidgetPressureSensorPlugin "${plugin_SRC}" "${plugin_HDR}")
set(AUTOLOAD_PhidgetPressureSensorPlugin_PLUGIN CACHE INTERNAL OFF)
target_link_libraries(PhidgetPressureSensorPlugin PUBLIC PhidgetPressureSensorDAQ)

if(BUILD_TESTING)
 add_subdirectory(tests)
endif()
