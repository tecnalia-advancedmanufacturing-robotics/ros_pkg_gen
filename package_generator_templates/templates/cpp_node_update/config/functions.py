#!/usr/bin/env python
"""
@package package_generator_template
@file functions.py
@author Anthony Remazeilles
@brief List of aditional functions that can be used in the template

Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""


def get_package_type(context):
    """extract the package the type belong to

    Args:
        context (dict): context (related to an interface description)

    Returns:
        str: the package extracted from the type key

    Examples:
        >>> context = {name: "misc", type={std_msgs::String}, desc="an interface"}
        >>> get_package_type(context)
        std_msgs
    """
    return context['type'].split("::")[0]


def get_class_type(context):
    """extract the Class related to the type, in C++ format

    Args:
        context (dict): context (related to an interface description)

    Returns:
        str: The class extracted from the type

    Examples:
        >>> context = {name: "misc", type={std_msgs::String}, desc="an interface"}
        >>> get_class_type(context)
        String
    """
    return context['type'].split("::")[1]


def get_python_type(context):
    """extract the type of an object in python format

    Args:
        context (dict): context (related to an interface description)

    Returns:
        str: the type of the interface written in python format

    Examples:
        >>> context = {name: "misc", type={std_msgs::String}, desc="an interface"}
        >>> get_python_type(context)
        "std_msgs.String"
    """
    return context['type'].replace("::", ".")


def get_cpp_path(context):
    """convert the type content into a path format

    Args:
        context (dict): context (related to an interface description)

    Returns:
        str: conversion in cpp path format of the type

    Examples:
        >>> context = {name: "misc", type={std_msgs::String}, desc="an interface"}
        >>> get_cpp_path(context)
        "std_msgs/String"
    """
    return context['type'].replace("::", "/")


def get_py_param_type(context):
    """convert a param type into python accepted format.
    Related to the formats accepted by the parameter under dynamic reconfigure

    Args:
        context (dict): context (related to an interface description)

    Returns:
        str: variable type in python format related to param components

    Examples:
        >>> context = {desc="an interface" name="misc" type="std::string"/>
        >>> get_py_param_type(context)
        "str_t"
    """
    param_type = context['type']
    if param_type not in ['std::string', 'string', 'int', 'double', 'bool']:
        raise SyntaxError("Invalid type for param {}".format(param_type))
    if param_type in ['std::string', 'string']:
        return 'str_t'
    if param_type == 'int':
        return 'int_t'
    if param_type == 'double':
        return 'double_t'
    if param_type == 'bool':
        return 'bool_t'


def get_py_param_value(context):
    """Extract the value of a parameter.
    Dedicated to parameters under dynamic reconfigure

    Args:
        context (dict): context (related to an param description)

    Returns:
        str: The value of the parameter as a string

    Examples:
        >>> context = {desc="an interface" type="std::string" value="Hello"/>
        >>> get_py_param_value(context)
        "\"Hello\""
        >>> context = {desc="an interface" type="bool" value="1"/>
        >>> get_py_param_value(context)
        "True"
    """
    param_type = context['type']
    if param_type not in ['std::string', 'string', 'int', 'double', 'bool']:
        msg = "Invalid type for param {}".format(param_type)
        msg += "\n autorized type: std::string, string, int, double, bool]"
        raise SyntaxError(msg)
    if param_type in ['std::string', 'string']:
        return "\"{}\"".format(context['value'])
    if param_type == 'bool':
        return "{}".format(str2bool(context['value']))
    return context['value']


def get_cpp_param_value(context):
    """Extract the value of a parameter, in cpp format.
    Similar to get_py_param_value, except for booleans that are true or false

    Args:
        context (dict): context (related to an param description)

    Returns:
        str: The value of the parameter as a string

    Examples:
        >>> context = {desc="an interface" type="bool" value="1"/>
        >>> get_py_param_type(context)
        "true"
    """
    param_type = context['type']
    if param_type not in ['std::string', 'string', 'int', 'double', 'bool']:
        msg = "Invalid type for param {}".format(param_type)
        msg += "\n autorized type: [std::string, string, int, double, bool]"
        raise SyntaxError(msg)
    if param_type in ['std::string', 'string']:
        return "\"{}\"".format(context['value'])
    if param_type == 'bool':
        if str2bool(context['value']):
            return "true"
        else:
            return "false"
    return context['value']


def str2bool(strg):
    """convert a string into boolean value
    Several format are accepted for True.
    What is not True is considered as False

    Args:
        strg (str): string containing a boolean value

    Returns:
        Bool: corresponding boolean value
    """
    return strg.lower() in ("yes", "true", "t", "1")


def capitalized_node_name(context):
    """return the node name in capitalized format

    Args:
        context (dict): complete package and node transformation

    Returns:
        str: node name in capitalized format, underscore being removed.

    Examples:
        >>> context = {nodeName="another_node" frecuency="100"/>
        >>> capitalized_node_name(context)
        "AnotherNode"
    """
    return context['nodeName'].title().replace("_", "")


def dependencies_from_template():
    """provides the dependencies required by the template

    Returns:
        list: list of ROS package dependency required by the template
    """
    return []


def dependencies_from_interface(interface_name, context):
    """return package dependencies according to the interface name

    Args:
        interface_name (str): name of the interface to consider
        context (list): list of dependencies related to such interface
    """
    list_dep = []

    type_field = ['publisher', 'directPublisher', 'directSubscriber',
                  'subscriber', 'serviceClient', 'serviceServer',
                  'actionServer', 'actionClient']

    if interface_name in type_field:
        pkg_dep = get_package_type(context)
        list_dep.append(pkg_dep)

    if interface_name in ['actionClient', 'actionServer']:
        list_dep.append('actionlib')
        list_dep.append('actionlib_msgs')

    if interface_name in ['listener', 'broadcaster']:
        list_dep.append('tf')

    if interface_name == 'dynParameter':
        list_dep.append('dynamic_reconfigure')

    return list_dep
