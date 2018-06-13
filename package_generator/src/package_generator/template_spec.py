#!/usr/bin/env python
"""
@package package_generator
@file generate_dict.py
@author Anthony Remazeilles
@brief generate the disctionnary for the package generator

Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""
from package_generator.enhanced_object import EnhancedObject
import yaml
import pprint
import os

class TemplateSpec(EnhancedObject):
    """Contains a the set of configuration info for a template
       This information is completing the folder containing the files to
       generate

    Attributes:
        dep_from_interface_ (function): to get the dependencies related to
            a given interface
        dep_from_template_ (function): to get the package dependencies
            due to the template itself
        dico_ (dict): the package dictionary, expected to be used by
            the developer in the xml file
        transformation_functions_ (dict): list of additional functions that are
            provided by the Designer to complete the basic instructions
            directly deduced from the dictionary

    """
    def __init__(self, name="TemplateSpec"):
        """Basic constructor

        Args:
            name (str, optional): object name
        """
        # call super class constructor
        super(TemplateSpec, self).__init__(name)

        self.dico_ = dict()
        self.transformation_functions_ = dict()
        self.dep_from_template_ = None
        self.dep_from_interface_ = None

    def load_spec(self, folder_path):
        """Load all configuration information from a given folder path
            We are expecting to find in that folder the files:
             * dico.yaml: dictionary to be used within the Developer xml file.
             * functions.py: set of additional functions the code generator
               could rely on

        Args:
            folder_path (str): folder into which the expected files are searched

        Returns:
            Bool: True on success

        Warnings:
            The presence of the function file is not set mandatory
        """

        # confirm file existence

        abs_name = folder_path + "/" + "dictionary.yaml"
        if not os.path.isfile(abs_name):
            self.log_error("Missing expected dictionary " + abs_name)
            return False

        if not self.load_yaml_desc(abs_name):
            self.log_error("Configuration aborted")
            return False

        abs_name = folder_path + "/" + "functions.py"
        if not os.path.isfile(abs_name):
            self.log_warn("Missing expected file " + abs_name)
            return True

        if not self.load_functions(abs_name):
            self.log_error("Configuration aborted")
            return False
        return True

    def load_yaml_desc(self, yaml_file):
        """Load the dictionary to be used by the Developer and defining most of
            the generation instructions provided to the Designer

        Args:
            yaml_file (str): yaml file containing that dictionary

        Returns:
            Bool: Operation success
        """
        try:
            with open(yaml_file, 'r') as open_file:
                self.dico_ = yaml.load(open_file)
        except IOError, err:
            self.log_error("IO Error: {}".format(err))
            return False
        except yaml.parser.ParserError, err:
            self.log_error("Parsing Error detected: {}".format(err))
            return False

        self.log("Data read: | \n {}".format(self.dico_))
        pprint.pprint(self.dico_)
        return True

    def load_functions(self, py_file):
        """Load the set of additional functions to be used at the generation

        Args:
            py_file (str): filename

        Returns:
            Bool: Operation success
        """
        try:
            with open(py_file) as open_file:
                exec(open_file.read(), self.transformation_functions_)
        except IOError, err:
            self.log_error("IO Error: {}".format(err))
            return False

        self.log("functions found: {}".format(self.transformation_functions_.keys()))
        # remove keys that are not associated to functions
        del self.transformation_functions_['__doc__']
        del self.transformation_functions_['__builtins__']

        # checking package dependencies related functions.
        fun_name = 'dependencies_from_template'
        if fun_name in self.transformation_functions_.keys():
            self.dep_from_template_ = self.transformation_functions_[fun_name]
            del self.transformation_functions_[fun_name]
        else:
            self.dep_from_template_ = None
        fun_name = 'dependencies_from_interface'
        if fun_name in self.transformation_functions_.keys():
            self.dep_from_interface_ = self.transformation_functions_[fun_name]
            del self.transformation_functions_[fun_name]
        else:
            self.dep_from_interface_ = None

        return True

if __name__ == "__main__":
    """
    main
    """
    print "Hello World"

    TSPEC = TemplateSpec()
    CFG_PATH = "../../../package_generator_templates/templates/cpp_node_update/config/"

    TSPEC.load_spec(CFG_PATH)
    print "Done"

