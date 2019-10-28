#!/usr/bin/env python
"""
@package {packageName}
@file {nodeName}_ros.py
@author {packageAuthor}
@brief {packageDescription}

Copyright (C) {packageCopyright}
{packageLicense}
"""

from copy import deepcopy
import rospy
{ifdynParameter}
from dynamic_reconfigure.server import Server
from {packageName}.cfg import {nodeName}Config
{endifdynParameter}

# ROS message & services includes
{forallserviceServer}
from {apply-get_package_type}.srv import {apply-get_class_type}
{endforallserviceServer}
# other includes
from {packageName} import {nodeName}_impl


class {apply-capitalized_node_name}ROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_config_ = {nodeName}_impl.{apply-capitalized_node_name}Config()
        self.component_implementation_ = {nodeName}_impl.{apply-capitalized_node_name}Implementation()

        {ifdynParameter}
        # preparing dynamic reconfigure mechanism
        srv = Server({nodeName}Config, self.configure_callback)
        {endifdynParameter}
        {ifparameter}
        # handling parameters from the parameter server
        {endifparameter}
        {forallparameter}
        self.component_config_.{name} = rospy.get_param("~{name}", {apply-get_py_param_value})
        {endforallparameter}
        {ifdynParameter}
        # handling dynamic parameters
        {endifdynParameter}
        {foralldynParameter}
        self.component_config_.{name} = rospy.get_param("{name}", {apply-get_py_param_value})
        {endforalldynParameter}
        {ifserviceServer}
        # handling service servers
        {endifserviceServer}
        {forallserviceServer}
        self.{name}_ = rospy.Service('{name}', {apply-get_class_type}, self.component_implementation_.callback_{name})
        {endforallserviceServer}

    {ifdynParameter}
    def configure_callback(self, config, level):
        """
        callback on the change of parameters dynamically adjustable
        """
        {foralldynParameter}
        self.component_config_.{name} = config.{name}
        {endforalldynParameter}
        return config

    {endifdynParameter}
    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("{nodeName}", anonymous=False)

    node = {apply-capitalized_node_name}ROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.spin()
