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
include_directories(${catkin_INCLUDE_DIRS})

# protected region additional user defined BUILD STATEMENTS begin #
# protected region additional user defined BUILD STATEMENTS end #

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
{forallcomponent}
{ifpublisher}
  add_rostest(test/test_{componentPackage}_{componentNode}_pub.test)
{endifpublisher}
{ifcyclicPublisher}
  add_rostest(test/test_{componentPackage}_{componentNode}_pub.test)
{endifcyclicPublisher}
{ifdynParameter}
  add_rostest(test/test_{componentPackage}_{componentNode}_dyn_param.test)
{endifdynParameter}
{endforallcomponent}
endif()

# protected region user Cmake macros begin #
# protected region user Cmake macros end #
