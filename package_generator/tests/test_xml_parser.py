#!/usr/bin/env python
"""
@package package_generator
@file test_xml_parser.py
@author Anthony Remazeilles
@brief test the xml parser

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

import unittest
import rospkg
from package_generator.package_xml_parser import PackageXMLParser
from package_generator.template_spec import TemplateSpec


class PackageXMLParserTest(unittest.TestCase):
    """
    tests for xml parser functions
    """

    def test_parsing(self):
        """
        check if a parsing goes well
        Mainly implemented as an example of use
        """

        rospack = rospkg.RosPack()
        node_path = rospack.get_path('package_generator_templates')

        # the current example only contains the dictionary
        dir_template_spec = node_path + "/templates/cpp_node_update/config/"
        spec = TemplateSpec()

        self.assertTrue(spec.load_spec(dir_template_spec))

        package_parser = PackageXMLParser()
        self.assertTrue(package_parser.set_template_spec(spec))

        rospack = rospkg.RosPack()
        node_path = rospack.get_path('package_generator')

        file_xml = node_path + '/tests/data/demo.ros_package'

        self.assertTrue(package_parser.load(file_xml))
        # todo check the content read and compare with an expected output.


if __name__ == '__main__':
    print "test_xml_parser -- begin"
    unittest.main()
    print "test_xml_parser -- end"
