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
import rospkg

from package_generator.generate_package import PackageGenerator
from package_generator.code_generator import CodeGenerator
from package_generator.package_xml_parser import PackageXMLParser
from package_generator.template_spec import TemplateSpec


class TemplateSanityTest(unittest.TestCase):

    def setUp(self):

        rospack = rospkg.RosPack()
        path_pkg = rospack.get_path('package_generator_templates')
        self.path_templates_ = path_pkg + "/templates/"

    def test_single_template(self):

        gen = PackageGenerator()

        path_template = self.path_templates_ + "cpp_node_update/"
        self.assertTrue(gen.set_package_template(path_template))

        self.assertTrue(gen.template_sanity_check())


# To launch a given test:
# python -m unittest test_module.TestClass


if __name__ == '__main__':
    unittest.main()
