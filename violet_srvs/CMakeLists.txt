cmake_minimum_required(VERSION 2.8.3)
project(violet_srvs)

find_package(catkin REQUIRED COMPONENTS message_generation std_srvs)

set(SERVICE_FILES
  RegisterSource.srv
)

foreach(SERVICE SERVICE_FILES)
  list(APPEND SERVICE_INSTALL_FILES "msg/${SERVICE}")
endforeach(SERVICE)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ${SERVICE_FILES}
)

generate_messages(DEPENDENCIES std_srvs)
catkin_package(CATKIN_DEPENDS message_runtime std_srvs)

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
)

install(FILES
  ${SERVICE_INSTALL_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

