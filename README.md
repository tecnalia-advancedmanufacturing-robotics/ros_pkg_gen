# Package generator

The following packages are about the automatic creation of ROS packages (so far C++ and python).

To do so the input needed are:

* A _package template_: a template is created by a _template Designer_.
  It contains the list of files to be automatically generated.
  The _Developer_ just has to select the template that best fits his needs.
* An _interface specification_: a xml file describing mainly the interface of the package to create, according to the template.
  This file is to be filled by the _Developer_, using the interface proposed by the template, according to the concrete needs of the package he is willing to create

**Author & Maintainer**: Anthony Remazeilles, anthony.remazeilles@tecnalia.com

**Affiliation** : Tecnalia Research and Innovation, Spain

**License**: This project is under the NPOSL-3.0 License.
See [LICENSE.md](LICENSE.md) for more details.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=qsNkYGQBW8U
" target="_blank"><img src="http://img.youtube.com/vi/qsNkYGQBW8U/0.jpg"
alt="ROS Package Generator demo" width="560" height="315" border="0" /></a>

Note that the video is related to version `1.0.0`.
It should be updated... soonish?

**Publications**:
If you use this work in an academic context, please cite the following publication(s):

A. Remazeilles, J. Azpiazu: Towards an Advanced ROS Package Generator.
International Conference on Informatics in Control, Automation and Robotics,
ICINCO’2019
([pdf](https://www.insticc.org/Primoris/Resources/PaperPdf.ashx?idPaper=78340)).

```
@inproceedings{Remazeilles2019,
  author = {Remazeilles, A. and Azpiazu, J.},
  booktitle = {International Conference on Informatics in Control, Automation and Robotics, ICINCO'2019},
  title = {Towards an Advanced ROS Package Generator},
  year = {2019},
  address = {Praga, Czech Republic}
}
```

Note that the paper is related to version `1.0.0`.
The code has evolved since then.
See the modifications in file [ChangeLog.md](ChangeLog.md).
Main changes are:

* The main element generated in a package is now named `component` (used to be `node`).
  This term is more generic, and enables considering more package types (version `2.0.0`).
* In addition to the custom generator, [jinja generator](https://jinja.palletsprojects.com/en/2.10.x/) can also be used for designing new templates (`2.1.0`).
* The package template is not anymore provided as input parameters, but is now directly inserted into the package specification file (`3.0.0`).

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
We also suppose that we know the package template we would like to use.

```shell
source devel/setup.bash
# go to the place where we would like to place the new package
cd src
# create a ROS interface xml file, related to the template cpp_node_update
rosrun package_generator generate_xml_skel cpp_node_update my_new_package_spec.ros_package
# edit the file to insert the ROS interface we are interested in
gedit my_new_package_spec.ros_package
# launch the code generation
rosrun package_generator generate_package my_new_package_spec.ros_package
# open the generated package to insert the node logic.
```

The expected content of the xml file `my_new_package_spec.ros_package` and
the behaviour of the generated code is described in [template package readme][template_readme].
Please take 5 minutes to read it.
The script `generate_xml_skel` provides the interface file skeleton
that the Developer as to adjust based on his needs.

The template name is corresponding to the template folder name within the
[package_generator_templates](package_generator_templates/templates).
Option for indicating template located elsewhere will be soon resumed.

[template_readme]: package_generator_templates/README.md

## Content

* [`package_generator`](package_generator/README.md):
  generates a package content based on a xml description.

* [`package_generator_templates`](package_generator_templates/README.md):
  list of available package templates.

## Reminders

* to remap a topic or a service with `rosrun`:

```shell
rosrun great_multi_package_pub_sub node_sub sub_int:=/pub_in
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
  * see how to handle list and map from parameter server
  * update: if the requested file is generated, refuse it
  * apply more code static checking
  * if no dependency is provided, the system is not accepting doing the generation.
    Check if (i) the message is appropriate, (ii) if the generation should be performed anyhow
  * Decide on the insertion per default to `cmake_module` in the required packages
  * Enhance executable definition in CMakeLists when a library is to be added
  * Handle user-provided dependencies for find_package
  * Remove sentence _This file is to be edited by the Developer_
  * Consider Forcing to "to require cmake 3.0.2 for Kinetic"
  * Ensure all class methods are camelCased.

## Acknowledgements

This development is supported by the European Union’s Horizon 2020 project [ROSIN][rosin_website].
This project has received funding from the European Union’s Horizon 2020 research and innovation programme under
grant agreement No 732287.

The opinions and arguments expressed reflect only the author‘s view and reflect in no way the European Commission‘s opinions.
The European Commission is not responsible for any use that may be made of the information it contains.

[![ROSIN website][rosin_logo]][rosin_website]

[rosin_logo]: http://rosin-project.eu/wp-content/uploads/2017/03/Logo_ROSIN_CMYK-Website.png
[rosin_website]: http://rosin-project.eu/ "Go to website"
