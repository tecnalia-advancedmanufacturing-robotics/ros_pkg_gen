cmake_minimum_required(VERSION 2.8.3)
project({packageName})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  {foralldependencies}
  {dependencyName}
  {endforalldependencies}
)

# protected region additional user defined REQUIREMENTS begin #
# protected region additional user defined REQUIREMENTS end #

{ifdynParameter}
{forallnodes}
if ({packageName}_dyn_rec_files)
  list(APPEND {packageName}_dyn_rec_files cfg/{nodeName}.cfg)
  # message("Append of cfg/{nodeName}.cfg")
else ()
  set({packageName}_dyn_rec_files cfg/{nodeName}.cfg)
  # message("Setting with cfg/{nodeName}.cfg")
endif ()
{endforallnodes}

# message("Test: ${{packageName}_dyn_rec_files}")
if ({packageName}_dyn_rec_files)
  generate_dynamic_reconfigure_options(${{packageName}_dyn_rec_files})
endif ()
{endifdynParameter}

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/src
  CATKIN_DEPENDS
    roscpp
    {foralldependencies}
    {dependencyName}
    {endforalldependencies}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare cpp executable
{forallnodes}
add_executable({nodeName} ros/src/{nodeName}_ros.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
{ifdynParameter}
add_dependencies({nodeName} ${PROJECT_NAME}_gencfg)
add_dependencies({nodeName} ${${PROJECT_NAME}_EXPORTED_TARGETS})
{endifdynParameter}
## Specify libraries to link a library or executable target against
target_link_libraries({nodeName} ${catkin_LIBRARIES}
)
{endforallnodes}
# protected region additional user defined BUILD STATEMENTS begin #
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation
{forallnodes}
install(TARGETS {nodeName}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
{endforallnodes}

# protected region user Cmake macros begin #
# protected region user Cmake macros end #
