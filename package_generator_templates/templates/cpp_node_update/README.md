# cpp_node_update

Template for a C++ node handling at a given frequency incoming messages
(subscription) for generating and sending result messages.

As the first template proposed, it also gives access to _all_ standard
ROS interfaces, i.e. topic subscription & publication, services, actions,
static & dynamic parameters, and tf.

## Pattern description

Both C++ and python versions follow the same pattern described here.

This template generates nodes in which incoming messages (through subscription)
are processed at a given frequency, and resulting in out-coming messages
(through publication) sent at the same frequency.

If the pattern mainly restricts the node update policy for the publish /subscribe mechanism, it also enables to use _any_ of the other ROS communication means (i.e actions, services, tf).

The ROS interface and the core node intelligence are explicitly separated:

* In the C++ version, the ROS interface is in `ros/src/[node_name]_ros.cpp`, and the node implementation is in `common/src/[node_name]_common.cpp`
* In the python version, the ROS interface is in `src/[package_name]/[node_name]_ros.py`, and the implementation in `src/[package_name]/[node_name]_impl.py`
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
   <component name="first_node" frequency="50.0">
       <publisher name="is_config_changed" type="std_msgs::Bool" description="Inform whether the config changed"/>
       <publisher name="complete_name" type="std_msgs::String" description="name and surname"/>
       <subscriber name="sub_surname" type="std_msgs::String" description="to receive the person surname"/>
       <serviceServer name="srv_display_name" type="std_srvs::SetBool" description="to print the name"/>
       <parameter name="name" type="std::string" value="Empty" description="default person name"/>
       <parameter name="generate_mail_format" type="bool" value="1" description="whether mail format is used or not"/>
       <actionServer name="count_char" type="actionlib::Test" description="count the letters in the generated name"/>
   </component>
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
