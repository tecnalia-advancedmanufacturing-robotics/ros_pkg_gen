# {packageName}

## General description of the package

<!--- protected region package descripion begin -->
{packageDescription}
<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/{componentName}.png" width="300px" />-->

{forallcomponent}
## Node: {componentName}

Update frequency: {componentFrequency} Hz.
{iflistener}

This node is using `\tf` to get transform information.
{endiflistener}
{ifbroadcaster}

This node is using `\tf` to broadcast transforms.
{endifbroadcaster}

<!--- protected region {componentName} begin -->
<!--- protected region {componentName} end -->
{ifparameter}

### Static Parameters

All static parameters can be set through the command line:

```shell
rosrun {packageName} {componentName} [param_name]:=[new_value]
```

{endifparameter}
{forallparameter}
`{name}` *({type}, default: {apply-get_py_param_value})*
<!--- protected region param {name} begin -->
{description}
<!--- protected region param {name} end -->
{endforallparameter}
{ifdynParameter}

### Dynamic Parameters

All dynamic parameters can be set through the command line:

```shell
rosrun {packageName} {componentName} _[param_name]:=[new_value]
```

{endifdynParameter}
{foralldynParameter}
`{name}` *({type}, default: {apply-get_py_param_value})*
<!--- protected region param {name} begin -->
{description}
<!--- protected region param {name} end -->
{endforalldynParameter}
{ifpublisher}

### Published Topics

A topic can be remapped from the command line:

```shell
rosrun {packageName} {componentName} [old_name]:=[new_name]
```

{endifpublisher}
{forallpublisher}
`{name}` *({type})*
<!--- protected region publisher {name} begin -->
{description}
<!--- protected region publisher {name} end -->
{endforallpublisher}
{ifsubscriber}

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun {packageName} {componentName} [old_name]:=[new_name]
```

{endifsubscriber}
{forallsubscriber}
`{name}` *({type})*
<!--- protected region {name} begin -->
<!--- protected region {name} end -->
{endforallsubscriber}
{ifserviceServer}

### Services proposed

A remapping of the service name is made possible at node launch:

```shell
rosrun {packageName} {componentName} _[old_name]_remap:=/[new_name]
```

{endifserviceServer}
{forallserviceServer}
`{name}` *({type})*
<!--- protected region service server {name} begin -->
{description}
<!--- protected region service server {name} end -->
{endforallserviceServer}
{ifserviceClient}

### Services used

A remapping of the service name is made possible at node launch:

```shell
rosrun {packageName} {componentName} _[old_name]_remap:=/[new_name]
```

{endifserviceClient}
{forallserviceClient}
`{name}` *({type})*
<!--- protected region service client {name} begin -->
{description}
<!--- protected region service client {name} end -->
{endforallserviceClient}
{ifactionServer}

### Action proposed

A simple action launched can be obtained with:

```shell
rosrun actionlib axclient.py /do_action
```

{endifactionServer}
{forallactionServer}
`{name}` *({type})*
<!--- protected region action server {name} begin -->
{description}
<!--- protected region action server {name} end -->
{endforallactionServer}
{ifactionClient}

### Action used

Any action client name can be readjusted at node launch:

```shell
rosrun {packageName} {componentName} _[old_name]_remap:=[new_name]
```

{endifactionClient}
{forallactionClient}
`{name}` *({type})*
<!--- protected region action client {name} begin -->
{description}
<!--- protected region action client {name} end -->
{endforallactionClient}
{ifdirectPublisher}

### Direct Publishers

These publishers are not handled through the update loop.
Their publication frequency is thus unknown

{endifdirectPublisher}
{foralldirectPublisher}
`{name}` *({type})*
<!--- protected region direct publisher {name} begin -->
{description}
<!--- protected region direct publisher {name} end -->
{endforalldirectPublisher}
{ifdirectSubscriber}

### Direct Subscribers

These subscribers are not handled through the update loop.
The subscription handler is triggered as soon as a message arrives.

{endifdirectSubscriber}
{foralldirectSubscriber}
`{name}` *({type})*
<!--- protected region direct subscriber {name} begin -->
{description}
<!--- protected region direct subscriber {name} end -->
{endforalldirectSubscriber}

{endforallcomponent}

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
