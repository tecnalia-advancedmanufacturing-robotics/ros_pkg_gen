extended
====================

# General description of the package
The extended package
<!--- protected region package descripion begin -->
<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/node_extended.png" width="300px" />-->

# Node: node_extended
Update frequency: 100.0 Hz.

<!--- protected region node_extended begin -->
<!--- protected region node_extended end -->

## Static Parameters

All static parameters can be set through the command line:
```
rosrun extended node_extended [param_name]:=[new_value]
```
`param_one` *(std::string, default: "Empty")*
<!--- protected region param param_one begin -->
Critical string parameter
<!--- protected region param param_one end -->
`param_two` *(bool, default: True)*
<!--- protected region param param_two begin -->

<!--- protected region param param_two end -->

## Published Topics

A topic can be remapped from the command line:
```
rosrun extended node_extended [old_name]:=[new_name]
```

`pub` *(std_msgs::Bool)*
<!--- protected region publisher pub begin -->
boolean publisher
<!--- protected region publisher pub end -->

## Subscribed Topics

A topic can be remapped from the command line:
```
rosrun extended node_extended [old_name]:=[new_name]
```

`sub_in` *(std_msgs::String)*
<!--- protected region sub_in begin -->
<!--- protected region sub_in end -->

## Services proposed

A remapping of the service name is made possible at node launch:

```
rosrun extended node_extended _[old_name]_remap:=/[new_name]
```

`service_server` *(std_srvs::SetBool)*
<!--- protected region service server service_server begin -->
great service used
<!--- protected region service server service_server end -->

## Services used

A remapping of the service name is made possible at node launch:

```
rosrun extended node_extended _[old_name]_remap:=/[new_name]
```

`service_client` *(std_srvs::Trigger)*
<!--- protected region service client service_client begin -->
great service proposed
<!--- protected region service client service_client end -->

## Action proposed

A simple action launched can be obtained with:

```
rosrun actionlib axclient.py /do_action
```

`action_server` *(actionlib::Test)*
<!--- protected region action server action_server begin -->
great action proposed to ROS world
<!--- protected region action server action_server end -->

## Action used
Any action client direaction can be readjusted at node launch:

```
rosrun extended node_extended _[old_name]_remap:=[new_name]
```
`action_client` *(actionlib::Test)*
<!--- protected region action client action_client begin -->
great action from ROS used
<!--- protected region action client action_client end -->

