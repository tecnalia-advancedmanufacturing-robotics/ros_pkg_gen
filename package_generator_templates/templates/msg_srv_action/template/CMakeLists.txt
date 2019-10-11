cmake_minimum_required(VERSION 2.8.3)
project({packageName})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  {foralldependencies}
  {dependencyName}
  {endforalldependencies}
)

# protected region additional user defined REQUIREMENTS begin #
# protected region additional user defined REQUIREMENTS end #



################################################
## Declare ROS messages, services and actions ##
################################################

{ifmsg}
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  {forallmsg}
  {name}.msg
  {endforallmsg}
)

{endifmsg}
{ifsrv}
## Generate services in the 'srv' folder
add_service_files(
  FILES
  {forallsrv}
  {name}.srv
  {endforallsrv}
)

{endifsrv}
{ifaction}
## Generate actions in the 'action' folder
add_action_files(
   FILES
  {forallaction}
  {name}.action
  {endforallaction}
 )

{endifaction}

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    {foralldependencies}
    {dependencyName}
    {endforalldependencies}
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    {foralldependencies}
    {dependencyName}
    {endforalldependencies}
    message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

# protected region user Cmake macros begin #
# protected region user Cmake macros end #
