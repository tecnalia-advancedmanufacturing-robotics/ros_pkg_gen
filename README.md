# Package generator

The following packages are about the automatic creation of ROS packages (so far c++ and python).

To do so the input needed are:

* A _package template_: a template is created by a _template Designer_.
  It contains the list of files to be automatically generated.
  The _Developer_ just has to select the template that best fits his needs.
* An _interface specification_: a xml file describing mainly the interface of the package to create, according to the template.
  This file is to be filled by the _Developer_, using the interface proposed by the template, according to the concrete needs of the package he is willing to create

**Author**: Anthony Remazeilles

**Maintainer**: Anthony Remazeilles, anthony.remazeilles@tecnalia.com

**Affiliation** : Tecnalia Research and Innovation, Spain

**License**: This project is under the NPOSL-3.0 License.
See [LICENSE.md](LICENSE.md) for more details.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=qsNkYGQBW8U
" target="_blank"><img src="http://img.youtube.com/vi/qsNkYGQBW8U/0.jpg"
alt="ROS Package Generator demo" width="560" height="315" border="0" /></a>

## Getting started

### Prerequisites

We assume [`ROS`][ros] is installed on the machine.
Code is developed and tested so far under `ROS indigo` and `ROS kinetic`.

[ros]: http://www.ros.org/

### Installing

The installation procedure follows the standard operations as any ROS package does.

```shell
# Assuming ~/catkin_ws is the workspace in which the repository has been downloaded
cd ~/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### Use

We assume we are at the ROS workspace root, and that the current git repository is accessible from the ROS workspace.

```shell
source devel/setup.bash
# go to the place where we would like to place the new package
cd src
# create a ROS interface xml file
gedit my_new_package_spec.ros_package
# launch the code generation (for c++ package)
rosrun package_generator generate_package my_new_package_spec.ros_package cpp_node_update
# launch the code generation (for python package)
rosrun package_generator generate_package my_new_package_spec.ros_package python_node_update
```

The template can be indicated either as:

* an absolute path to a template directory
* a relative path from the currently active directory
* a directory name assumed to be existing in the template package

The expected content of the xml file and the behavior of the generated code is described in [template package readme][template_readme].
Please take 5 minutes to read it.

You can generate automatically the structure of the xml file using the script `generate_xml_skel`:

```shell
rosrun package_generator generate_xml_skel cpp_node_update my_new_package_spec.ros_package
```

A basic structure of template specification (to be then filled) is then generated, based on the template considered.

[template_readme]: package_generator_templates/README.md

## content

### `package_generator`

Generates a node / package content based on a xml description.

See the dedicated [README](package_generator/README.md) for more details on its content.

### `package_generator_templates`

Gathers the package templates currently defined.
So far, it only contains two templates, one for python code and another for C++.
Both follow the same pattern, based on the concept of a central update loop at a given frequency (inspired from BRICS models).

More details in the dedicated [Readme file](package_generator_templates/README.md)

## reminders

* to remap a topic with rosrun:

```shell
rosrun great_multi_package_pub_sub node_sub sub_int:=/pub_in
```

* for services, the current implementation is configured to enable initial remapping

```shell
rosrun great_package_services node_service_server _service_trigger_server_remap:=/service_client
```

* for the action, one can use:

```shell
rosrun actionlib axclient.py /do_action
```

* actions can be also remapped :

```shell
rosrun great_package_action_client node_action_client _ac_use_action_remap:=do_action
```

## Future work

* Short term:
  * generate xml skeleton testing (in particular for error cases)
  * Describe how a Designer can create a template
  * see how to handle list and map from parameter server
  * check if remap for service client is implemented
  * update: if the requested file is generated, refuse it
  * apply more code static checking
  * if no dependency is provided, the system is not accepting doign the generation.
    Check if (i) th message is appropriate, (ii) if the generation should be performed anyhow
  * Decide on the insertion per default to `cmake_module` in the required packages
  * Enhance executable definition in CMakeLists when a library is to be added
  * Handle user-provided dependencies for find_package
  * Remove sentance _This file is to be edited by the Developer_
  * Consider Forcing to "to require cmake 3.0.2 for Kinetic"
  * Ensure all class methods are camelCased.
* Longer Term:
  * Consider use of Jinja

## Acknowledgements

This development is supported by the European Union’s Horizon 2020 project [ROSIN][rosin_website].
This project has received funding from the European Union’s Horizon 2020 research and innovation programme under
grant agreement No 732287.

The opinions and arguments expressed reflect only the author‘s view and reflect in no way the European Commission‘s opinions.
The European Commission is not responsible for any use that may be made of the information it contains.

[![ROSIN website][rosin_logo]][rosin_website]

[rosin_logo]: http://rosin-project.eu/wp-content/uploads/2017/03/Logo_ROSIN_CMYK-Website.png
[rosin_website]: http://rosin-project.eu/ "Go to website"
