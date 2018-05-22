#!/usr/bin/env python
"""
@package package_generator
@file test_dic_generator.py
@author Anthony Remazeilles
@brief test the dictionary generator

Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt

To run a single test, type:
python test_code_generator.py CodeGeneratorTest.test_apply_function
"""

import unittest
import os

import generate_dict

class GenerateDictTest(unittest.TestCase):

    def setUp(self):
        file_content = """\
package_attributes: ["name", "author", "author_email", "description", "license"]
node_attributes: ["name", "frequency"]

node_interface:
    publisher: ["name", "type", "description"]
    directPublisher: ["name", "type", "description"]
    subscriber: ["name", "type", "description"]
    directSubscriber: ["name", "type", "description"]
    serviceClient: ["name", "type", "description"]
    serviceServer: ["name", "type", "description"]
    parameter: ["name", "type", "value", "description"]
    dynParameter: ["name", "type", "value", "description"]
    actionServer: ["name", "type", "description"]
    actionClient: ["name", "type", "description"]
    listener: ["name", "description"]
    broadcaster: ["name", "description"]
"""

        self.dir_name = "/tmp/test_dic_generator"
        if not os.path.exists(self.dir_name):
            print "Creating the repo {}".format(self.dir_name)
            os.makedirs(self.dir_name)

        # print "File content: \n{}".format(file_content)

        self.filename = self.dir_name + "/dico.yaml"
        with open(self.filename, 'w') as open_file:
            open_file.write(file_content)

        self.gen_dict = generate_dict.GenerateDictionnary()

    def test_undef_file(self):
        self.assertFalse(self.gen_dict.load_yaml_desc("unknown_file.yaml"))

    def test_yaml_error(self):
        bad_file = """bad_content: ["name", "author","""
        filename = self.dir_name + "/bad_dico.yaml"
        with open(filename, 'w') as open_file:
            open_file.write(bad_file)

        self.assertFalse(self.gen_dict.load_yaml_desc(filename))

    def test_correct_file(self):
        self.assertTrue(self.gen_dict.load_yaml_desc(self.filename))

if __name__ == '__main__':
    print "test_dic_generator -- begin"
    unittest.main()
    print "test_dic_generator -- end"