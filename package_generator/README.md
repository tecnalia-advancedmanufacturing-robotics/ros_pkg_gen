# package generator

## Overview

This package contains the needed python scripts for generating a package, using an xml package description, that defines the interface of a package following a component pattern.

The available patterns are available in the package [package_generator_templates](../package_generator_templates/README.md).

**Author**: Anthony Remazeilles

**Maintainer**: Anthony Remazeilles, anthony.remazeilles@tecnalia.com

**Affiliation**: Tecnalia Research and Innovation, Spain

**License**: This project is under the NPOSL-3.0 License.
See [LICENSE.md](../LICENSE.md) for more details.

## Installation and Usage

See example of use in the upper [README.md](../README.md).

Get information of the basic template characteristics in the [package_generator_templates Readme](../package_generator_templates/README.md).

## Content

The package contains the following scripts:

* [enhanced_object.py](package_generator/src/package_generator/enhanced_object.py): basic class providing utilities used in derivated classes.
* [template_spec.py](package_generator/src/package_generator/template_spec.py): load the spec of a given package template (xml dictionnary and additional Designer functions).
* [package_xml_parser.py](package_generator/src/package_generator/package_xml_parser.py): is responsible of parsing an xml package description.
  An example of such xml description is provided in [package_generator/tests/data/demo.ros_package](package_generator/tests/data/demo.ros_package).
* [code_generator.py](package_generator/src/package_generator/code_generator.py): from a xml node description and a template of file, it generates the corresponding file.
  An example of template of ros file is provided in [../package_generator_templates/templates/cpp_node_update/template/ros/src/component_ros.cpp](../package_generator_templates/templates/cpp_node_update/template/ros/src/component_ros.cpp)
* [generate_package.py](package_generator/src/package_generator/generate_package.py): given a directory containing a set of template file, and a xml package description, generates the whole related code.
* [file_update_management.py](package_generator/src/package_generator/file_update_management.py): contains needed tools for enabling update of already created package.
