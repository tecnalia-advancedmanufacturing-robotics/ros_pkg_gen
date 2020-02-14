# cpp_service_server

C++ node acting as a service server.

## Pattern description

Both C++ and python versions follow the same pattern described here.

Following the _separation of concern_,

* `ros/src/[nodename]_ros_.cpp`: implementation of the ROS interface.
* `common/src/[nodename]_common_.cpp`: implementation of the service callbacks.

The Developer should focus on the second file. It contains different classes automatically created:

| Class | Description |
| ----- | ----------- |
| `[NodeName]Config` | contains parameter variables (read from `rosparam` or dynamically adjusted) |
| `[NodeName]Passthrough` | gathers ROS components accessible for the services |
| `[NodeName]Impl` | contains the Developer implementation of the node |

Class `[NodeName]Impl` contains a constructor, and configure methods to be filled by the Developer.
It also contains a callback per service server defined by the Developer.

In class `[NodeName]Passthrough`, the Developer will have access to the transform listener and broadcaster, publisher, subscriber and service client he required access to.
