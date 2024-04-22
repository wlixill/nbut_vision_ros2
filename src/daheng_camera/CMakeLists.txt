# ====================================####
# ====Set project basic properties====####
# ====================================####
cmake_minimum_required(VERSION 3.8)
project(daheng_camera)

set(CMAKE_CXX_STANDARD 14) # Use C++14
set(CMAKE_CXX_EXTENSIONS OFF) # No compiler extension
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Prevent the compiler from using other
#add_definitions(-Wall -Werror) # Open all wall and werror
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -fsanitize=address")

# ####==================####
# ####====Custom set====####
# ####==================####
#file(GLOB MAIN ${CMAKE_CURRENT_SOURCE_DIR} "src/daheng_camera_node.cpp")
file(GLOB MAIN ${CMAKE_CURRENT_SOURCE_DIR} "src/*.cpp")
set(REGISTER_CLASS_NODE daheng_camera::AcquisitionNode)

# ================================================####
# ====Automatically find external dependencies====####
# ================================================####
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ==========================================####
# ====Automatically add executable files====####
# ==========================================####
# ament_auto_add_executable(${PROJECT_NAME} ${MAIN})
ament_auto_add_library(${PROJECT_NAME} SHARED ${MAIN})
rclcpp_components_register_node(
${PROJECT_NAME} # Register Ros2  executable node
PLUGIN ${REGISTER_CLASS_NODE} EXECUTABLE ${PROJECT_NAME}_node)

# =============================####
# ====Manually adding links====####
# =============================####
target_link_libraries(${PROJECT_NAME} gxiapi) # dahen api

# ==================####
# ====Build test====####
# ==================####
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint
       ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================####
# ====Automatic install====####
# =========================####
ament_auto_package(INSTALL_TO_SHARE config launch)
