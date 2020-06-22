# nodetest template

Template for generating pure node testing packages.

Relies on the testing tools provided by [rostest](http://wiki.ros.org/rostest).

## XML specification

The dictionary used in the xml specification is defined in [config/dictionary.html].
To generate an empty spec file, one can use:

```shell
rosrun package_generator generate_xml_skel nodetest filename.ros_package
```

### Package attributes

The required are the standard ones: `name`, `author`, `author_email`, `description`, `license` and `copyright`.

### Component attributes

Here a _component_ refers to a _node_ we want to test

* `package`: package to which the node belongs to.
* `name`: name of the node to test

### Component interface

**publisher**: to verify that a message is at least published once.

* `name`: topic name to which the message is being published.

**cyclicPublisher**: to verify a message is being published at a given frequency

* `name`: topic name to which the message is being published.
* `frequency`: expected publication frequency

**dynParameter**: to verify a dynamic parameter is being defined by the component

* `name`: name of the dynamic parameter

**service**: to check the output generated given an input.

* `name`: name of the service to test.

>[!WARNING]
> the generated file in test folder has to be edited to define the input and output values, in python dictionary format.

The test generated can be copied and paste to add more cases with different input-output value.
Note that if the package generator is called again, introduced changes will be erased (temporally stored in `/tmp` as indicated)

**filter**: to test a filter node behaviour, in which a message is published after processing a received message.

* `topic_in`: input topic name (where we will publish a teat message).
* `topic_out`: output topic name (where we expect receiving an answer).

>[!WARNING]
> the generated file in test folder has to be edited to define the input and output values, in python dictionary format.

The test generated can be copied and paste to add more cases with different input-output value.
Note that if the package generator is called again, introduced changes will be erased (temporally stored in `/tmp` as indicated)
