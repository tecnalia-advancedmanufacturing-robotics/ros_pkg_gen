#!/usr/bin/env python
"""
@package package_generator_template
@file functions.py
@author Anthony Remazeilles
@brief List of aditional functions that can be used in the template

Copyright (C) 2020 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.
"""


def get_name_no_py(context):
    """return the component name without .py extension (if existing)

    Args:
        context (dict): complete package and component transformation

    Returns:
        str: component name without possible .py extension.

    Examples:
        >>> get_name_no_py({'componentName':"nopy"})
        >>> 'nopy'
         >>> get_name_no_py({'componentName':"nopy.py"})
        >>> 'nopy'
    """

    return context['componentName'].replace('.py', '')


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
        context (dict): attributes assigned by the User to such instance

    Returns:
        list: List of dependencies that should be added according to
              the interface used and the attributes values
    """

    list_dep = []

    if interface_name in ['service', 'filter']:
        list_dep.append('rostest_node_interface_validation')
    return list_dep

