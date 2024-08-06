cmake_minimum_required(VERSION 3.5)
project(oradar_lidar)

#=======================================
# Compile setup (CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD COLCON)

set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-O3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
add_definitions(-DROS2_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)

set(CMAKE_CXX_STANDARD 23)

set(GCC_WARNINGS
    -Wall
    -Wextra # reasonable and standard
    -Wshadow # warn the user if a variable declaration shadows one from a parent context
    -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor. This helps
    # catch hard to track down memory errors
    -Wold-style-cast # warn for c-style casts
    -Wcast-align # warn for potential performance problem casts
    -Wunused # warn on anything being unused
    -Woverloaded-virtual # warn if you overload (not override) a virtual function
    -Wpedantic # warn if non-standard C++ is used
    -Wconversion # warn on type conversions that may lose data
    -Wsign-conversion # warn on sign conversions
    -Wnull-dereference # warn if a null dereference is detected
    -Wdouble-promotion # warn if float is implicit promoted to double
    -Wformat=2 # warn on security issues around functions that format output (ie printf)
    -Wimplicit-fallthrough # warn on statements that fallthrough without an explicit annotation
    -Wmisleading-indentation # warn if indentation implies blocks where blocks do not exist
    -Wduplicated-cond # warn if if / else chain has duplicated conditions
    -Wduplicated-branches # warn if if / else branches have duplicated code
    -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
    -Wuseless-cast # warn if you perform a cast to the same type
    -Wsuggest-override # warn if an overridden member function is not marked 'override' or 'final'
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(COMPILER_WARNINGS ${GCC_WARNINGS})
endif()

include_directories(${rclcpp_INCLUDE_DIRS})

# Bin and Install
set(ORADAR_SDK_DIR "sdk")
add_subdirectory(${ORADAR_SDK_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${ORADAR_SDK_DIR} include)

add_executable(oradar_scan ${CMAKE_CURRENT_SOURCE_DIR}/src/oradar_scan_node.cpp)

#Ros2#
ament_target_dependencies(oradar_scan rclcpp sensor_msgs)
target_link_libraries(oradar_scan oradar_sdk pthread)

install(TARGETS oradar_scan DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz2 DESTINATION share/${PROJECT_NAME}/)

ament_package()
