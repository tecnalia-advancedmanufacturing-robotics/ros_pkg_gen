#!/usr/bin/env python
"""
@package package_generator_template
@file functions.py
@author Anthony Remazeilles
@brief List of aditional functions that can be used in the template

Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
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
        str: The type of the interface written in path style

    Examples:
        >>> context = {name: "misc", type={std_msgs::String}, desc="an interface"}
        >>> get_cpp_path(context)
        "std_msgs/String"
    """
    return context['type'].replace("::", "/")
