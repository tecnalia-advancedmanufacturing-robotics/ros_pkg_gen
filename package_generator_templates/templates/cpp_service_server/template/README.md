# {packageName}

## General description of the package

<!--- protected region package description begin -->
{packageDescription}
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/{componentName}.png" width="300px" />-->
{forallcomponent}

## Node: {componentName}

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

{ifserviceServer}

### Services proposed

Any service name can be adjusted using the ROS remapping functionality:

```shell
rosrun {packageName} {componentName} [old_name]:=[new_name]
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

Any service name can be adjusted using the ROS remapping functionality:

```shell
rosrun {packageName} {componentName} [old_name]:=[new_name]
```

{endifserviceClient}
{forallserviceClient}
`{name}` *({type})*
<!--- protected region service client {name} begin -->
{description}
<!--- protected region service client {name} end -->
{endforallserviceClient}
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

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
