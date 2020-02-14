cmake_minimum_required(VERSION 2.8.3)
project(package_generator_templates)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
    templates
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
