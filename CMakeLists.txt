cmake_minimum_required(VERSION 3.16)
project(custom_interactive_markers LANGUAGES CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
    -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
    -Werror=missing-braces)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(interactive_markers REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

# Automoc and moc((meta objects complier) file generation
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/custom_interactive_markers/mt_rviz_ui.hpp
)

# Add the library for the RViz plugin
add_library(mt_rviz_ui SHARED src/mt_rviz_ui.cpp ${MOC_FILES})

# Specify the include directories
target_include_directories(mt_rviz_ui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
find_package(interactive_markers REQUIRED)




# Link dependencies
ament_target_dependencies(mt_rviz_ui
  rclcpp
  rclcpp_components
  rviz_common
  rviz_rendering
  pluginlib   
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  interactive_markers
  visualization_msgs
)

# Link Qt5
target_link_libraries(mt_rviz_ui Qt5::Widgets)

# Install the library and XML files
install(TARGETS mt_rviz_ui
  EXPORT export_custom_interactive_markers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install the RViz plugin description
install(FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export interfaces and plugin description
ament_export_include_directories(include)
ament_export_targets(export_custom_interactive_markers)
#pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

