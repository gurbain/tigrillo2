cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PROJECT_NAME "Tigrillo2Plugin")

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rapidjson REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
#include_directories(${RAPIDJSON_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find the packages generated in this project
set(tigrillo_io_DIR  ${PROJECT_SOURCE_DIR}/../../devel/share/tigrillo_io/cmake/)
set(tigrillo_ctrl_DIR  ${PROJECT_SOURCE_DIR}/../../devel/share/tigrillo_ctrl/cmake/)
message(STATUS "Looking for tigrillo_io cmake in ${tigrillo_io_DIR}")
message(STATUS "Looking for tigrillo_ctrl cmake in ${tigrillo_ctrl_DIR}")
find_package(tigrillo_io REQUIRED)
find_package(tigrillo_ctrl REQUIRED)

include_directories(${tigrillo_io_INCLUDE_DIRS})
include_directories(${tigrillo_ctrl_INCLUDE_DIRS})
link_directories(${tigrillo_io_LIBRARY_DIRS})
link_directories(${tigrillo_ctrl_LIBRARY_DIRS})

# Build our plugin
add_library(${PROJECT_NAME} SHARED tigrillo_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_libraries} ${roscpp_LIBRARIES})

# Install library
install(TARGETS ${PROJECT_NAME} DESTINATION .)

# Build the stand-alone test program
#add_executable(test_tigrillo tigrillo_plugin_test.cc)

# if (${gazebo_VERSION_MAJOR} LESS 6)
#   include(FindBoost)
#   find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#   target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# else()
#   target_link_libraries(vel ${GAZEBO_LIBRARIES})
# endif()
