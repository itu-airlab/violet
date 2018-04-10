cmake_minimum_required(VERSION 2.8.3)
project(violet_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

set(MESSAGE_FILES 
  ObjectInfo.msg;
  ObjectProperty.msg
  DetectionInfo.msg
  Object.msg
  Predicate.msg
  WorldState.msg
)

foreach(MESSAGE MESSAGE_FILES)
  list(APPEND MESSAGE_INSTALL_FILES "msg/${MESSAGE}")
endforeach(MESSAGE)
  
# Generate messages in the 'msg' folder
add_message_files(FILES ${MESSAGE_FILES})
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs )

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  ${MESSAGE_INSTALL_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
