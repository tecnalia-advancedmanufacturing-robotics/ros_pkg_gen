#!/usr/bin/env python3
"""
@package package_generator
@file enhanced_object.py
@author Anthony Remazeilles
@brief Common tools for python classes

Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license
"""

import inspect
from termcolor import colored


class EnhancedObject(object):
    """Enhanced Object with advanced log tool

    Attributes:
        name_ (str): name of the object
    """
    def __init__(self, name="EnhancedObject"):
        """class constructor

        Args:
            name (str, optional): name of the object
        """
        self.name_ = name

    def log(self, text):
        """display log message with the class name in parameter
        text the string to display

        Args:
            text (str): message to print
        """
        print ("[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text)

    def log_warn(self, text):
        """display warn message with the class name in parameter
        text the string to display

        Args:
            text (str): warn message
        """
        msg = "[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text
        print (colored(msg, 'yellow'))

    def log_error(self, text):
        """display warn message with the class name in parameter
        text the string to display

        Args:
            text (str): error message
        """
        msg = "[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text
        print (colored(msg, 'red'))
