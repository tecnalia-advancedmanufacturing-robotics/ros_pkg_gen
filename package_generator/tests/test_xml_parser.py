#!/usr/bin/env python
"""
@package package_generator
@file test_xml_parser.py
@author Anthony Remazeilles
@brief test the xml parser

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import unittest
import rospkg
from package_generator.package_xml_parser import PackageXMLParser

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
        node_path = rospack.get_path('package_generator')

        package_parser = PackageXMLParser()
        self.assertTrue(package_parser.load(node_path + '/tests/extended.ros_package'))


if __name__ == '__main__':
    print "test_xml_parser -- begin"
    unittest.main()
    print "test_xml_parser -- end"
