cmake_minimum_required(VERSION 2.8.10)
project(violet)

# Dependencies
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roslib
  roscpp
  std_msgs
  std_srvs
  tf
  violet_msgs
  violet_srvs
)
find_package(Eigen REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roslib roscpp std_msgs std_srvs tf violet_msgs violet_srvs
  DEPENDS Eigen
)

option(WITH_CLOSED_WORLD_KB "Compile with closed world knowledge base" ON)
option(SEGMENTATION_TRANSFORM_POINT "Transform points coming into segmentation" OFF)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

if(WITH_CLOSED_WORLD_KB)
  add_definitions(-DCLOSED_WORLD_KB)
endif()

if(SEGMENTATION_TRANSFORM_POINT)
    add_definitions(-DSEGMENTATION_TRANSFORM_POINT)
endif()

# Add all input sources
aux_source_directory(src/inputsources INPUT_SRCS)

add_executable(violet
               src/global_parameters.cpp
               src/util.cpp
               src/predicate.cpp
               src/object_info.cpp
               src/defined_predicates.cpp
               src/input_source_manager.cpp
               src/knowledge_base.cpp
               src/bayesian_fusion.cpp
               src/object_database.cpp
               src/object_catalog.cpp
               src/violet_main.cpp
               ${INPUT_SRCS})

add_dependencies(violet ${catkin_EXPORTED_TARGETS})
target_link_libraries(violet ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(violet_visualization_marker_publisher visualization/violet_visualization_marker_publisher.cpp)
set_property(SOURCE visualization/violet_visualization_marker_publisher.cpp APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
add_dependencies(violet_visualization_marker_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(violet_visualization_marker_publisher ${catkin_LIBRARIES})

add_executable(violet_predefined_object_loader src/predefined_object_loader.cpp)
add_dependencies(violet_predefined_object_loader ${catkin_EXPORTED_TARGETS})
target_link_libraries(violet_predefined_object_loader ${catkin_LIBRARIES})
