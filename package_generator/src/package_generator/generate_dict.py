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
import importlib

class GenerateDictionary(EnhancedObject):

    def __init__(self, name="GenerateDictionary"):
        # call super class constructor
        super(GenerateDictionary, self).__init__(name)

        self.log("Hello world")
        self.spec_ = dict()
        self.transformation_functions_ = dict()

    def load_yaml_desc(self, yaml_file):

        try:
            with open(yaml_file, 'r') as open_file:
                self.spec_ = yaml.load(open_file)
        except IOError, err:
            self.log_error("IO Error: {}".format(err))
            return False
        except yaml.parser.ParserError, err:
            self.log_error("Parsing Error detected: {}".format(err))
            return False

        self.log("Data read: | \n {}".format(self.spec_))
        pprint.pprint(self.spec_)
        return True

    def load_transformation_functions(self, py_file):
        # see https://pymotw.com/3/importlib/
        # see https://copyninja.info/blog/dynamic-module-loading.html
        # https://www.blog.pythonlibrary.org/2012/07/31/advanced-python-how-to-dynamically-load-modules-or-classes/
        # from https://stackoverflow.com/questions/67631/how-to-import-a-module-given-the-full-path/67693
        # best solution would be:
        # module = importlib.import_module(py_file)

    def generate_lambdas(self):

        lambda_package = dict()
        for item in self.spec_["package_attributes"]:
            self.log(item)
            tag = "package" + item.title()
            lambda_package[tag] = lambda: self.debug_fun(item)

        print lambda_package

        for item in self.spec_["package_attributes"]:
            tag = "package" + item.title()
            print lambda_package[tag]()

    def debug_fun(self, par):
        return "Requesting for {}".format(par)

if __name__ == "__main__":
    """
    main
    """
    print "Hello World"

    gen_dict = GenerateDictionary()

    gen_dict.load_yaml_desc("dico.yaml")
    gen_dict.generate_lambdas()
    print "Done"

