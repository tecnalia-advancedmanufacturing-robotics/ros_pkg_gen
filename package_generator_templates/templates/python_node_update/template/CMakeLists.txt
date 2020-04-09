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

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
  CATKIN_DEPENDS
    rospy
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

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
{forallcomponent}
{ifpublisher}
  add_rostest(test/test_{componentName}_hz_loop.test)
{endifpublisher}
{endforallcomponent}
endif()

#############
## Install ##
#############
catkin_install_python(PROGRAMS
                      {forallcomponent}
                      scripts/{componentName}
                      {endforallcomponent}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# protected region user Cmake macros begin #
# protected region user Cmake macros end #
