# package generator

## Overview

This package contains the needed python scripts for generating a package, using an xml package description, that defines the interface of a package following a component pattern.

The available patterns are available in the package [package_generator_templates][../package_generator_templates/README.md].

**Author(s): Anthony Remazeilles**

**Maintainer: Anthony Remazeilles, anthony.remazeilles@tecnalia.com**

**Affiliation: Tecnalia Research and Innovation, Spain**

## Installation and Usage

See the upper [README.md][../README.md]

## Content

The package contains the following scripts:

* [package_xml_parser.py](package_generator/src/package_generator/package_xml_parser.py): is responsible of parsing an xml package description.
  An example of such xml description is provided in [tests\extended.ros_package](package_generator/tests/extended.ros_package).
* [code_generator.py](package_generator/src/package_generator/code_generator.py): from a xml node description and a template of ros file, it generates the corresponding file.
  An example of template of ros file is provided in [sandbox/template/package/ros/src/node_ros.cpp](package_generator/sandbox/template/package/ros/src/node_ros.cpp)
* [generate_package.py](package_generator/src/package_generator/generate_package.py): given an directory containing a set of template file, and a xml package description, generates the whole related code.
* [file_update_management.py](package_generator/src/package_generator/file_update_management.py): contains needed tools for enabling update of already created package

Some testing functionalities that can be mentioned::

* A test file for the xml parser is presented in [tests/test_xml_parser.py](package_generator/tests/test_xml_parser.py).
 It loads the ros package description in the same repo, [tests/extended.ros_package](package_generator/tests/extended.ros_package) and parses it.

* A test file for the code generation in [tests/test_code_generator.py](package_generator/tests/test_code_generator.py).
 It launches a set of generation, including all the files we can encounter in a ROS package.
 When files are store it is done in a directory in `/tmp/`

* A test file for a whole package generation in [tests/test_package_generator.py](package_generator/tests\test_package_generator.py).
 It generates a whole package, which is then ready to be built.
 Be aware that the created ROS package is placed in the directory containing the ROS package `package_generator` (to be improved later on).