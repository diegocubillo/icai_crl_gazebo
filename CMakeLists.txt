cmake_minimum_required(VERSION 3.5)

project(icai_crl_gazebo)


# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(icai_crl_description REQUIRED)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

# Following 'add_library' directive defines a library target named 'md25_plugin'.
# The 'SHARED' keyword indicates that a shared library should be compiled, and
# is followed by the list of source files for the target.
add_library(md25_plugin
  SHARED
  plugins/md25_plugin.cc
)

# Following 'target_include_directories' directive specifies 'include' as the
# include directory to use when compiling the 'md25_plugin' target.
# The 'PRIVATE' keyword specifies that this directive will populate the
# INCLUDE_DIRECTORIES property for the 'md25_plugin' target.
target_include_directories(
  md25_plugin PRIVATE include
)

# Following 'target_link_libraries' directive specifies to use the 
# gz-sim library when linking the 'md25_plugin' target.
# The 'PRIVATE' keyword stipulates that the gz-sim library will not
# automatically be included if the 'md25_plugin' target were to
# be linked to anoter target. 
# ${GZ_SIM_VER} is substituted by the value that is was set to above.
target_link_libraries(md25_plugin PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER})




# Following 'install' directive ensures that the compiled libraries
# of the 'md25_plugin' and 'FullSystem' targets will be copied
# to the subfolder 'lib/ros_gz_example_gazebo' of the install directory.
install(
  TARGETS md25_plugin
  DESTINATION lib/${PROJECT_NAME}
)


# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/ros_gz_example_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)


# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# set(ament_cmake_copyright_FOUND TRUE)
# set(ament_cmake_copyright_FOUND TRUE) (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'source install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()