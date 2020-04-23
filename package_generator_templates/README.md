# Package generator templates

This package gathers templates proposed for the `package_generator`.

**Author & Maintainer**: Anthony Remazeilles, anthony.remazeilles@tecnalia.com

**Affiliation**: Tecnalia Research and Innovation, Spain

**License**: This project is under the Apache 2.0 License.
See [LICENSE.md](../LICENSE.md) for more details.

## Getting Started

### Use

This package is a ROS wrapper to ease the access to the different templates by the [package_generator](../README.md).

As detailed there, when the code generation is launched,

```shell
rosrun package_generator generate_package my_new_package_spec.ros_package
```

a tag is searched in the xml specification file `my_new_package_spec.ros_package` that specifies the template to be used.
Its value is the name of the template folder within the [templates](templates) directory of this package.

### Content

* [templates](templates): the current set of templates implemented.
* [README_designer.md](README_designer.md) provides guidelines for creating new package templates.

### Available templates

* [cpp_node_update](templates/cpp_node_update/README.md): C++ node handling at a given frequency incoming messages (subscription) for generating and sending result messages.
   Also gives access to all ROS interface.
* `python_node_update`: similar life-cycle for python (see previous description).
* [cpp_service_server](templates/cpp_service_server/README.md): C++ node acting as a service server.
* `python_service_server`: similar life-cycle for python.
* [msg_srv_action](templates/msg_srv_action/README.md): Basic layers for creating an interface package
* [nodetest](templates/nodetest/README.md): to create a package dedicated to testing.

## Q & A

**_Where should I write my code?_**

In the implementation file, anywhere and _only_ where special text indicates it.
In C++ files:

```cpp
/* protected region user additional functions begin */
/* protected region user additional functions end */
```

or in python.

```python
# protected region user additional functions begin #
# protected region user additional functions end #
```

The code **should be in between** these contribution markers, and the two **bounding comment lines should not be touched**.

This could be problematic in case of interface update: only the _"code protected"_ is maintained.

----

**_Can I change the other files, or write out of the protected areas?_**

Yes you can!
But then you do not follow the proposed pattern (or the pattern should be updated).

**Warning**: if doing so, do not update the package using the `package_generator`: any change out of the protected areas will be erased (but not totally lost, see below).

----

**_I created an interface. I would like to update some components (remove some, add some, rename some, ...). Can I do it?_**

Yes you can !

You just recall the package generator, as you did it the first time.
If it finds a folder package with the same name, it will assume that this is a package update.

If you stick contributing within the protected areas, then all your contributions will be maintained in the updated version.

Note also that if you remove an interface from the xml, the Developer contribution related to that interface (like a service callback function) will be also removed from the developer file.

Note finally that every time a package update is detected, the original package is temporally backed up in `/tmp`.
That gives you additional chance to recover you package state before the update took place, in case needed.

----

**_I want to update my package, but it contains files not present in the template_**

This can occur for example if you add a launch folder (current templates do not cover launch files).

To maintain specific files during the update, the procedure is to add a file `.gen_maintain` at the root of the package folder.
This file is searched and processed during the update.
Any directory and files mentioned there will be restored.
As an example:

```shell
launch/my_launch_file.launch
data/
```

will restore your file `launch/my_launch_file.launch` as well as all the content of the folder `data`.

Each component to restore is to be placed on a new line.

Note that this tool should only be used to maintain files or folders not handled by the template.
Otherwise, the package is likely to be corrupted.
