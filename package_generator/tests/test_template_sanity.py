#!/usr/bin/env python
"""
@package package_generator
@file test_template_sanity.py
@author Anthony Remazeilles
@brief test the template sanity checker

Copyright (C) 2019 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

# TODO should these packages added to the packages.xml?
import unittest
import os
import shutil
import datetime
import rospkg

from package_generator.generate_package import PackageGenerator
from package_generator.code_generator import CodeGenerator
from package_generator.package_xml_parser import PackageXMLParser


class TemplateSanityTest(unittest.TestCase):

    counter_ = 0
    def setUp(self):

        rospack = rospkg.RosPack()
        path_pkg = rospack.get_path('package_generator_templates')
        self.path_templates_ = path_pkg + "/templates/"

        now = datetime.datetime.now()
        str_date = now.strftime("%H_%M_%S")
        self.dir_name = "/tmp/test_package_generator/sanity_{}_{}".format(str_date, TemplateSanityTest.counter_)
        TemplateSanityTest.counter_ += 1
        if not os.path.exists(self.dir_name):
            os.makedirs(self.dir_name)

    # todo: to be enabled, we should be able to set a sanity_check providing a path to the template
    def test_empty_template(self):

        gen = PackageGenerator()

        self.assertFalse(gen.template_sanity_check(self.dir_name))

    def test_no_template_file(self):
        path_from = self.path_templates_ + "/cpp_node_update/config/dictionary.yaml"
        path_to = self.dir_name + "/config/"

        os.makedirs(path_to)
        shutil.copy(path_from, path_to)

        os.makedirs(self.dir_name + "/template")

        gen = PackageGenerator()
        self.assertTrue(gen.template_sanity_check(self.dir_name))

    def test_for(self):

        path_from = self.path_templates_ + "/cpp_node_update/config/dictionary.yaml"
        path_to = self.dir_name + "/config/"
        os.makedirs(path_to)
        shutil.copy(path_from, path_to)

        os.makedirs(self.dir_name + "/template")

        filename = self.dir_name + "/template/onefile.cpp"
        file_content = (
            'So far so good' '\n'
            '{forallpublisher}' '\n'
            'name={name}' '\n')

        with open(filename, 'w') as openfile:
            openfile.write(file_content)

        gen = PackageGenerator()
        self.assertFalse(gen.template_sanity_check(self.dir_name))

    def test_if(self):

        path_from = self.path_templates_ + "/cpp_node_update/config/dictionary.yaml"
        path_to = self.dir_name + "/config/"
        os.makedirs(path_to)
        shutil.copy(path_from, path_to)

        os.makedirs(self.dir_name + "/template")

        filename = self.dir_name + "/template/onefile.cpp"
        file_content = (
            'So far so good' '\n'
            '{ifpublisher}' '\n'
            'name={name}' '\n')

        with open(filename, 'w') as openfile:
            openfile.write(file_content)

        gen = PackageGenerator()

        self.assertFalse(gen.template_sanity_check(self.dir_name))

    def test_single_template(self):

        gen = PackageGenerator()

        self.assertTrue(gen.template_sanity_check("cpp_node_update"))


# To launch a given test:
# python test_template_sanity.py TemplateSanityTest.test_if
# To launch all test: python -m unittest discover -s tests -v

if __name__ == '__main__':
    unittest.main()
