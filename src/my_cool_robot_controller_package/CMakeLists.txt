# Add my_controller_file library related compile commands
generate_parameter_library(my_controller_file_parameters
  src/my_controller_file.yaml
  include/my_cool_robot_controller_package/validate_my_controller_file_parameters.hpp
)
add_library(
  my_controller_file
  SHARED
  src/my_controller_file.cpp
)
target_include_directories(my_controller_file PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(my_controller_file my_controller_file_parameters)
ament_target_dependencies(my_controller_file ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(my_controller_file PRIVATE "MY_CONTROLLER_FILE_BUILDING_DLL")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gmock(test_load_my_controller_file test/test_load_my_controller_file.cpp)
  target_include_directories(test_load_my_controller_file PRIVATE include)
  ament_target_dependencies(
    test_load_my_controller_file
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_my_controller_file test/test_my_controller_file.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/my_controller_file_params.yaml)
  target_include_directories(test_my_controller_file PRIVATE include)
  target_link_libraries(test_my_controller_file my_controller_file)
  ament_target_dependencies(
    test_my_controller_file
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(test_my_controller_file_preceeding test/test_my_controller_file_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/my_controller_file_preceeding_params.yaml)
  target_include_directories(test_my_controller_file_preceeding PRIVATE include)
  target_link_libraries(test_my_controller_file_preceeding my_controller_file)
  ament_target_dependencies(
    test_my_controller_file_preceeding
    controller_interface
    hardware_interface
  )
endif()
ament_export_libraries(
  my_controller_file
)

ament_package()
