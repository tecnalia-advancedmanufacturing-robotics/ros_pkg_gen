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
import os
import rospkg
from package_generator.package_xml_parser import PackageXMLParser
from package_generator.template_spec import TemplateSpec


class PackageXMLParserTest(unittest.TestCase):
    """
    tests for xml parser functions
    """

    def setUp(self):
        """Common initialiation for all tests
        """

        self.rospack = rospkg.RosPack()
        self.package_parser = PackageXMLParser()

        self.dir_storage = "/tmp/test_package_generator"
        if not os.path.exists(self.dir_storage):
            print "Creating folder {}".format(self.dir_storage)
            os.makedirs(self.dir_storage)

    def test_parsing(self):
        """
        check if a parsing goes well
        Mainly implemented as an example of use
        """

        node_path = self.rospack.get_path('package_generator_templates')

        # the current example only contains the dictionary
        dir_template_spec = node_path + "/templates/cpp_node_update/config/"
        spec = TemplateSpec()

        self.assertTrue(spec.load_spec(dir_template_spec))

        self.assertTrue(self.package_parser.set_template_spec(spec))

        node_path = self.rospack.get_path('package_generator')

        file_xml = node_path + '/tests/data/demo.ros_package'

        self.assertTrue(self.package_parser.load(file_xml))
        # todo check the content read and compare with an expected output.

    def test_template_reading(self):

        file_content = (
            '<?xml version="1.0" encoding="UTF-8"?>' '\n'
            '<nothing>interesting</nothing>' '\n'
        )
        filename = self.dir_storage + "/no_template.ros_package"

        with open(filename, 'w') as open_file:
            open_file.write(file_content)

        self.assertIsNone(self.package_parser.get_template(filename))

        node_path = self.rospack.get_path('package_generator')

        file_xml = node_path + '/tests/data/demo.ros_package'

        self.assertEqual(self.package_parser.get_template(file_xml),
                         'cpp_node_update')


if __name__ == '__main__':
    print "test_xml_parser -- begin"
    unittest.main()
    print "test_xml_parser -- end"
