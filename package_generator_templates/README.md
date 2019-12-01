# Package generator templates

This package gathers templates proposed for the `package_generator`.
So far a single package pattern is implemented, declined in its C++ and python version, named `cpp_node_update` and `python_node_update`.

**Author**: Anthony Remazeilles

**Maintainer**: Anthony Remazeilles, anthony.remazeilles@tecnalia.com

**Affiliation**: Tecnalia Research and Innovation, Spain

**License**: This project is under the NPOSL-3.0 License.
See [LICENSE.md](../LICENSE.md) for more details.

## Getting Started

### Prerequisites

The component is assuming `ROS` is installed on the machine.

### Installing

The installation procedure follows the standard operations as any ROS package does.

### Use

This package is mainly a wrapper to ease the access to the different templates.
It is mainly used through the [package_generator](../README.md).

As detailed there, when the code generation is launched,

```shell
rosrun package_generator generate_package my_new_package_spec.ros_package python_node_update
```

The last argument (here `python_node_update`) can refer to a template supposed to be present in the `templates` directory of this package.

### Content

* [templates](templates): the current set of templates implemented.
* [samples](samples): a set of package spec for both C++ and python templates, as examples.
* [README_designer.md](README_designer.md) provides guidelines for creating new package templates.

## The `{cpp|python}_node_update` patterns

Both are jointly detailed since they both refer to the same node pattern.

They both generate nodes in which incoming messages (through subscription) are processed at a given frequency, and resulting in out-coming messages (through publication) sent at the same frequency.

If the pattern mainly restricts the node update policy for the publish/subscribe mechanism, it also enables to use _any_ of the other ROS communication means (i.e actions, services, tf).

The ROS interface and the core node intelligence are explicitly separated:

* in the C++ version, the ROS interface is in `ros/src/[node_name]_ros.cpp`, and the node implementation is in `common/src/[node_name]_common.cpp`
* in the python version, the ROS interface is in `src/[package_name]/[node_name]_ros.py`, and the implementation in `src/[package_name]/[node_name]_impl.py`
* The ROS interface class handles the creation of all needed ROS interface.
* You **should** only complete the node implementation files.

### Implementation file

The files `common/src/[node_name]_common.cpp` (C++) or `src/[package_name]/[node_name]_impl.py` (python) need to be filled by the Developer.

It contains different classes automatically created:

| Class | Description |
| ----- | ----------- |
| `[NodeName]Config` | contains parameter variables (read from `rosparam` or dynamically adjusted) |
| `[NodeName]Data` | contains the received messages and the messages to send |
| `[NodeName]Passthrough` | gathers ROS interface components _violating_ the interface / implementation paradigm (such as topic management out of the update concept, actions, ...) |
| `[NodeName]Impl` | contains the Developer implementation of the node |

**Only** class `[NodeName]Impl` should be modified.
It is provided with two methods (described from the C++ pattern):

| Method | Description |
| ------ | ----------- |
| `void configure([NodeName]Config config)` | enables to initialize the object given the configuration parameters contained in `config` |
| `void update([NodeName]Data &data, [NodeName]Config config)` | `data` contains the latest messages received by the ROS interface. The Developer should put in that same object the messages to be sent to the ROS world after the `update` method call. Parameter variables are accessible with object `config`|

The method `update` is called through the ROS file at the frequency specified by the Developer in the xml spec (see next section).

## Package & Nodes specification

The package and nodes are described by their interface.
This interface is defined in a `xml` file, following the syntax presented in this example.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<package name="py_dummy_package" author="anthony" author_email="anthony@todo.todo" description="Dummy package containing various interface" license="TODO">
   <node name="first_node" frequency="50.0">
       <publisher name="is_config_changed" type="std_msgs::Bool" description="Inform whether the config changed"/>
       <publisher name="complete_name" type="std_msgs::String" description="name and surname"/>
       <subscriber name="sub_surname" type="std_msgs::String" description="to receive the person surname"/>
       <serviceServer name="srv_display_name" type="std_srvs::SetBool" description="to print the name"/>
       <parameter name="name" type="std::string" value="Empty" description="default person name"/>
       <parameter name="generate_mail_format" type="bool" value="1" description="whether mail format is used or not"/>
       <actionServer name="count_char" type="actionlib::Test" description="count the letters in the generated name"/>
   </node>
   <depend>std_msgs</depend>
   <depend>std_srvs</depend>
   <depend>actionlib</depend>
</package>
```

All accepted (and required) attributes of a package tag are presented in the previous example.

A node is described by the attributes `name` and `frequency` (the update frequency, i.e the frequency of message subscription and publication).

Then is introduced the ROS interface provided by the node.
The current pattern provides the following interfaces:

* `publisher` and `subscriber` refer to the standard message publication and subscription.
   Note that the access to received messages and the publication of messages is driven by the update frequency.
* `parameter` and `dynParameter` are parameters respectively obtained from `rosparam`, and the dynamical server.
   At each update call, they will be provided to the node implementation.
* `serviceServer` and `serviceClient` refer to the management of ROS services.
   Note that the service client is indicated to complete the node interface. Since the service can be directly called by the node implementation, its handler has been placed in `[NodeName]Passthrough`.
* `actionServer` and `actionClient`: Similarly, `actionServer` is defined in the `[NodeName]Passthrough` component, since the Developer may be willing to send action messages within the action callback.
* `listener` and `broadcaster`: informs the node will either listen to `tf` or publish transforms to `tf`.
   Both variables are then available from the `[NodeName]Passthrough` component.
* `directSubscriber` and `directPublisher` are not handled through the update mechanism.
  They are added to satisfy impatient Developers that consider they can not wait a period for receiving a message or sending another.
  These components (implemented as traditional subscriber / publisher) are defined in the `[NodeName]Passthrough`.

More spec:

* All ROS interface tags are described by the attributes `name`, `type`, and `description`.
  * The `name` should be in format `node_name`.
  * The `type` should be using the c++ format as seen in the example.
* Only `parameter`, `listener` and `broadcaster` differ:
  * For the `parameter`tag, accepted `type` are `{'std::string', 'string', 'int', 'double', 'bool'}`.
   A default value is to be provided as well.
* For the `listener` and `broadcaster`, only attributes `name` and `description` are expected.

## Q & A

**_Where should I wrote my code?_**

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
