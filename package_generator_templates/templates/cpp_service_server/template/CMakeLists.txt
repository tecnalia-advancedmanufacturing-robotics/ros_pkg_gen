cmake_minimum_required(VERSION 2.8.3)
project({packageName})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  {foralldependencies}
  {dependencyName}
  {endforalldependencies}
)

# protected region additional user defined REQUIREMENTS begin #
# protected region additional user defined REQUIREMENTS end #

{ifdynParameter}
{forallcomponent}
if ({packageName}_dyn_rec_files)
  list(APPEND {packageName}_dyn_rec_files cfg/{componentName}.cfg)
  # message("Append of cfg/{componentName}.cfg")
else ()
  set({packageName}_dyn_rec_files cfg/{componentName}.cfg)
  # message("Setting with cfg/{componentName}.cfg")
endif ()
{endforallcomponent}

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
    {foralldependencies}
    {dependencyName}
    {endforalldependencies}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})

## Declare cpp executable
{forallcomponent}
add_executable({componentName} ros/src/{componentName}_ros.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
{ifdynParameter}
add_dependencies({componentName} ${PROJECT_NAME}_gencfg)
add_dependencies({componentName} ${${PROJECT_NAME}_EXPORTED_TARGETS})
{endifdynParameter}
## Specify libraries to link a library or executable target against
target_link_libraries({componentName} ${catkin_LIBRARIES}
  # protected region additional user defined LINK STATEMENTS begin #
  # protected region additional user defined LINK STATEMENTS end #
)
{endforallcomponent}
# protected region additional user defined BUILD STATEMENTS begin #
# protected region additional user defined BUILD STATEMENTS end #

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
{forallcomponent}
{ifdynParameter}
  add_rostest(test/test_{componentName}_dyn_param.test)
{endifdynParameter}
{endforallcomponent}
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
{forallcomponent}
install(TARGETS {componentName}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
{endforallcomponent}

# protected region user Cmake macros begin #
# protected region user Cmake macros end #
