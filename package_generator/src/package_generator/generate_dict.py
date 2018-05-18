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

class GenerateDictionnary(EnhancedObject):

    def __init__(self, name="GenerateDictionnary"):
        # call super class constructor
        super(GenerateDictionnary, self).__init__(name)

        self.log("Hello world")
        self.spec_ = dict()

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

    gen_dict = GenerateDictionnary()

    gen_dict.load_yaml_desc("dico.yaml")
    gen_dict.generate_lambdas()
    print "Done"

