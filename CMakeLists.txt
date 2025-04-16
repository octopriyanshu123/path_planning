# cmake_minimum_required(VERSION 3.8)
# project(path_planning)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)

# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# find_package(geometry_msgs REQUIRED)



# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# # add_executable(rrt_star_node src/rrt_star.cpp)
# # ament_target_dependencies(rrt_star_node rclcpp)

# install(
#   DIRECTORY include launch
#   DESTINATION share/${PROJECT_NAME}
# )
# include_directories(
#   include 
#   )

# # #Collect all .cpp files in src/
# # file(GLOB FILES "src/*.cpp")

# # # Loop through each file
# # foreach(FILE ${FILES})
# #   # Get the filename without directory
# #   get_filename_component(FILENAME ${FILE} NAME_WE)

# #   # Create executable
# #   add_executable(${FILENAME}_node ${FILE})

# #   # Link with dependencies
# #   ament_target_dependencies(${FILENAME}_node
# #     rclcpp
# #     std_msgs
# #     geometry_msgs
# #   )

# #   # Install executable
# #   install(TARGETS
# #     ${FILENAME}_node
# #     DESTINATION lib/${PROJECT_NAME}
# #   )
# # endforeach()



# # Loop through each file

#   # Get the filename without directory

#   # Create executable
#   add_library(RRT SHARED src/rrt_star.cpp) 
#   # Add dependencies to the library
#   ament_target_dependencies(RRT
#   rclcpp
#   std_msgs
#   geometry_msgs
#   )


#   add_executable(main_node src/main.cpp)
#   target_link_libraries(main_node RRT )

#   # Link with dependencies
#   ament_target_dependencies(main_node
#     rclcpp
#     std_msgs
#     geometry_msgs
#   )

#   # Install executable
#   install(TARGETS
#     main_node
#     DESTINATION lib/${PROJECT_NAME}
#   )


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()



cmake_minimum_required(VERSION 3.8)
project(path_planning)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Install headers and launch files
install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

# Create shared library
add_library(RRT SHARED src/rrt_star.cpp)

# Add dependencies to the library
ament_target_dependencies(RRT
  rclcpp
  std_msgs
  geometry_msgs
)

# Create executable
add_executable(main_node src/main.cpp)
target_link_libraries(main_node RRT)

# Add dependencies to the executable
ament_target_dependencies(main_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Install executable
# Install executable and shared library
install(TARGETS
  main_node
  RRT
  DESTINATION lib/${PROJECT_NAME}
)
# Linting and testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
