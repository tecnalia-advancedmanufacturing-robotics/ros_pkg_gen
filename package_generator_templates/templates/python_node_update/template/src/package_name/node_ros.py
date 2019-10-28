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
{ifactionServer}
import actionlib
{endifactionServer}
{ifactionClient}
import actionlib
{endifactionClient}
{ifdynParameter}
from dynamic_reconfigure.server import Server
from {packageName}.cfg import {nodeName}Config
{endifdynParameter}

# ROS message & services includes
{forallpublisher}
from {apply-get_package_type}.msg import {apply-get_class_type}
{endforallpublisher}
{forallsubscriber}
from {apply-get_package_type}.msg import {apply-get_class_type}
{endforallsubscriber}
{foralldirectPublisher}
from {apply-get_package_type}.msg import {apply-get_class_type}
{endforalldirectPublisher}
{foralldirectSubscriber}
from {apply-get_package_type}.msg import {apply-get_class_type}
{endforalldirectSubscriber}
{forallserviceServer}
from {apply-get_package_type}.srv import {apply-get_class_type}
{endforallserviceServer}
{forallserviceClient}
from {apply-get_package_type}.srv import {apply-get_class_type}
{endforallserviceClient}
{forallactionServer}
from {apply-get_package_type}.msg import {apply-get_class_type}Action
{endforallactionServer}
{forallactionClient}
from {apply-get_package_type}.msg import {apply-get_class_type}Action
{endforallactionClient}

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
        self.component_data_ = {nodeName}_impl.{apply-capitalized_node_name}Data()
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
        {ifpublisher}
        # handling publishers
        {endifpublisher}
        {forallpublisher}
        self.{name}_ = rospy.Publisher('{name}', {apply-get_class_type}, queue_size=1)
        {endforallpublisher}
        {ifsubscriber}
        # handling subscribers
        {endifsubscriber}
        {forallsubscriber}
        self.{name}_ = rospy.Subscriber('{name}', {apply-get_class_type}, self.topic_callback_{name})
        {endforallsubscriber}
        {ifdirectPublisher}
        # Handling direct publisher
        {foralldirectPublisher}
        self.component_implementation_.passthrough.pub_{name} = rospy.Publisher('{name}', {apply-get_class_type}, queue_size=1)
        {endforalldirectPublisher}
        {endifdirectPublisher}
        {ifdirectSubscriber}
        # Handling direct subscriber
        {foralldirectSubscriber}
        self.component_implementation_.passthrough.sub_{name} = rospy.Subscriber('{name}',
                                                                                 {apply-get_class_type},
                                                                                 self.component_implementation_.direct_topic_callback_{name})
        {endforalldirectSubscriber}
        {endifdirectSubscriber}
        {ifserviceServer}
        # handling service servers
        {endifserviceServer}
        {forallserviceServer}
        self.{name}_ = rospy.Service('{name}', {apply-get_class_type}, self.component_implementation_.callback_{name})
        {endforallserviceServer}
        {ifserviceClient}
        # handling service clients
        {endifserviceClient}
        {forallserviceClient}
        self.component_implementation_.passthrough.client_{name} = rospy.ServiceProxy('{name}',
                                                                                      {apply-get_class_type});
        {endforallserviceClient}
        {forallactionServer}
        # to enable action name adjustment when loading the node
        remap = rospy.get_param("~{name}_remap", "{name}")
        self.component_implementation_.passthrough.as_{name} = actionlib.SimpleActionServer(remap,
                                                                                                {apply-get_class_type}Action,
                                                                                                execute_cb=self.component_implementation_.callback_{name},
                                                                                                auto_start=False)
        self.component_implementation_.passthrough.as_{name}.start()
        {endforallactionServer}
        {forallactionClient}
        # to enable action name adjustment when loading the node
        remap = rospy.get_param("~{name}_remap", "{name}")
        self.component_implementation_.passthrough.ac_{name} = actionlib.SimpleActionClient(remap,{apply-get_class_type}Action)
        rospy.loginfo("Waiting for action server {}".format(remap))
        self.component_implementation_.passthrough.ac_{name}.wait_for_server()
        {endforallactionClient}

    {forallsubscriber}
    def topic_callback_{name}(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_{name} = msg
        self.component_data_.in_{name}_updated = True

    {endforallsubscriber}
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

    def activate_all_output(self):
        """
        activate all defined output
        """
        {forallpublisher}
        self.component_data_.out_{name}_active = True
        {endforallpublisher}
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        {forallsubscriber}
        self.component_data_.in_{name}_updated = False
        {endforallsubscriber}
        pass

    def update(self, event):
        """
        @brief update function

        @param      self The object
        @param      event The event

        @return { description_of_the_return_value }
        """
        self.activate_all_output()
        config = deepcopy(self.component_config_)
        data = deepcopy(self.component_data_)
        self.set_all_input_read()
        self.component_implementation_.update(data, config)
        {ifpublisher}

        try:
            {forallpublisher}
            self.component_data_.out_{name}_active = data.out_{name}_active
            self.component_data_.out_{name} = data.out_{name}
            if self.component_data_.out_{name}_active:
                self.{name}_.publish(self.component_data_.out_{name})
            {endforallpublisher}
        except rospy.ROSException as error:
            rospy.logerr("Exception: {}".format(error))
        {endifpublisher}


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

    rospy.Timer(rospy.Duration(1.0 / {nodeFrequency}), node.update)
    rospy.spin()
