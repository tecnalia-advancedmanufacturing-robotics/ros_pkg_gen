#!/usr/bin/env python
"""
@package package_generator
@file test_dic_generator.py
@author Anthony Remazeilles
@brief test the dictionary generator

Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).

To run a single test, type:
python test_code_generator.py CodeGeneratorTest.test_apply_function
"""

import unittest
import os
import rospkg

from package_generator.template_spec import TemplateSpec

class GenerateDictTest(unittest.TestCase):

    def setUp(self):

        rospack = rospkg.RosPack()
        node_path = rospack.get_path('package_generator_templates')

        self.dir_template_spec = node_path + "/templates/cpp_node_update/config/"
        self.spec = TemplateSpec()

        # creating a temporary repo for trials
        self.dir_name = "/tmp/test_template_spec"
        if not os.path.exists(self.dir_name):
            print "Creating the repo {}".format(self.dir_name)
            os.makedirs(self.dir_name)


    def test_undef_files(self):
        self.assertFalse(self.spec.load_yaml_desc("unknown_file.yaml"))
        self.assertFalse(self.spec.load_functions("unknown_file.py"))

    def test_yaml_error(self):
        bad_file = """bad_content: ["name", "author","""
        filename = self.dir_name + "/bad_dico.yaml"
        with open(filename, 'w') as open_file:
            open_file.write(bad_file)

        self.assertFalse(self.spec.load_yaml_desc(filename))

    def test_correct_folder(self):
        self.assertTrue(self.spec.load_spec(self.dir_template_spec))

if __name__ == '__main__':
    print "test_dic_generator -- begin"
    unittest.main()
    print "test_dic_generator -- end"
