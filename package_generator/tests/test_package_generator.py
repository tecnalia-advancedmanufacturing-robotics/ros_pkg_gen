#!/usr/bin/env python
"""
@package package_generator
@file test_package_generator.py
@author Anthony Remazeilles
@brief test the code generator

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

import unittest
import os

from package_generator.generate_package import PackageGenerator
from package_generator.code_generator import CodeGenerator
from package_generator.package_xml_parser import PackageXMLParser
from package_generator.template_spec import TemplateSpec

class PackageGeneratorTest(unittest.TestCase):
    """Tests proposed for the Package generator module
    """

    def setUp(self):
        """
        Common initialization for all tester
        """

        file_content = (
            '<?xml version="1.0" encoding="UTF-8"?>' '\n'
            '<package name="great_package" author="anthony" author_email="anthony@todo.todo" description="The extended package" license="TODO" copyright="TRI">' '\n'
            '   <node name="node_extended" frequency="100.0">' '\n'
            '       <publisher name="pub" type="std_msgs::Bool" description=""/>' '\n'
            '       <publisher name="pub_second" type="std_msgs::String" description=""/>' '\n'
            '       <subscriber name="sub_in" type="std_msgs::String" description=""/>' '\n'
            '       <serviceClient name="service_client" type="std_srvs::Trigger" description=""/>' '\n'
            '       <serviceServer name="service_server" type="std_srvs::SetBool" description=""/>' '\n'
            '       <parameter name="param_one" type="std::string" value="Empty" description=""/>'  '\n'
            '       <parameter name="param_two" type="bool" value="1" description=""/>'  '\n'
            '       <actionServer name="action_server" type="bride_tutorials::TriggerPublish" description=""/>' '\n'
            '       <actionClient name="action_client" type="bride_tutorials::TriggerPublish" description=""/>' '\n'
            '   </node>'  '\n'
            '<depend>std_msgs</depend>'  '\n'
            '<depend>std_srvs</depend>'  '\n'
            '<depend>bride_tutorials</depend>'  '\n'
            '</package> ' '\n')

        file_content_multi = (
            '<?xml version="1.0" encoding="UTF-8"?>' '\n'
            '<package name="great_multi_node_package" author="anthony" author_email="anthony@todo.todo" description="The extended package" license="TODO" copyright="TRI">' '\n'
            '   <node name="node_extended" frequency="100.0">' '\n'
            '       <publisher name="pub" eventHandler="" type="std_msgs::Bool" description=""/>' '\n'
            '       <publisher name="pub_second" eventHandler="" type="std_msgs::String" description=""/>' '\n'
            '       <subscriber name="sub_in" type="std_msgs::String" description=""/>' '\n'
            '       <serviceClient name="service_client" type="std_srvs::Trigger" description=""/>' '\n'
            '       <serviceServer name="service_server" type="std_srvs::SetBool" description=""/>' '\n'
            '       <parameter name="param_one" type="std::string" value="Empty" description=""/>'  '\n'
            '       <parameter name="param_two" type="bool" value="1" description=""/>'  '\n'
            '       <actionServer name="action_server" type="bride_tutorials::TriggerPublish" description=""/>' '\n'
            '       <actionClient name="action_client" type="bride_tutorials::TriggerPublish" description=""/>' '\n'
            '   </node>'  '\n'
            '   <node name="second_node" frequency="100.0">' '\n'
            '       <publisher name="pub_bool" eventHandler="" type="std_msgs::Bool" description=""/>' '\n'
            '       <publisher name="pub_string" eventHandler="" type="std_msgs::String" description="" />' '\n'
            '       <subscriber name="sub_string" type="std_msgs::String" description=""/>' '\n'
            '       <parameter name="param_string" type="std::string" value="Empty" description=""/>'  '\n'
            '       <parameter name="param_bool" type="bool" value="1" description=""/>'  '\n'
            '   </node>'  '\n'

            '<depend>std_msgs</depend>'  '\n'
            '<depend>std_srvs</depend>'  '\n'
            '<depend>bride_tutorials</depend>'  '\n'
            '</package> ' '\n')

        # print "File content: \n{}".format(file_content)

        self.dir_name = "/tmp/test_package_generator"
        if not os.path.exists(self.dir_name):
            print "Creating the repo {}".format(self.dir_name)
            os.makedirs(self.dir_name)

        filename = self.dir_name + "/node_spec.ros_package"
        with open(filename, 'w') as file_opened:
            file_opened.write(file_content)

        self.package_spec_ = filename

        filename = self.dir_name + "/multi_node_spec.ros_package"
        with open(filename, 'w') as file_opened:
            file_opened.write(file_content_multi)

        self.multi_node_package_spec_ = filename

        import rospkg
        rospack = rospkg.RosPack()
        self.node_path_ = rospack.get_path('package_generator_templates')
        self.path_template_ = self.node_path_ + "/templates/cpp_node_update/"

    def test_package_generator(self):
        """direct call to the package genator component, with appropriate info
        """

        gen = PackageGenerator()

        self.assertTrue(gen.set_package_template(self.path_template_))

        output_path = '/tmp/test_package_generation'
        # to generate in the ros workspace
        # output_path = os.path.normpath(self.node_path_ + "/../")
        print "Package to be placed in {}".format(output_path)

        if not os.path.exists(output_path):
            os.makedirs(output_path)
        self.assertTrue(gen.generate_package(self.package_spec_, output_path))


    def test_package_generator_multi_nodes(self):
        """direct call to the package genator component, with appropriate info
        """

        gen = PackageGenerator()

        self.assertTrue(gen.set_package_template(self.path_template_))

        output_path = '/tmp/test_multi_node_package_generation'
        # to generate in the ros workspace
        # output_path = os.path.normpath(self.node_path_ + "/../")
        print "Package to be placed in {}".format(output_path)

        if not os.path.exists(output_path):
            os.makedirs(output_path)
        self.assertTrue(gen.generate_package(self.multi_node_package_spec_, output_path))

    def test_readme_multi_nodes(self):
        """test the generation of a readme file involving multiple nodes spec.
        """

        spec = TemplateSpec()
        self.assertTrue(spec.load_spec(self.path_template_ + "config"))

        xml_parser = PackageXMLParser()
        self.assertTrue(xml_parser.set_template_spec(spec))
        self.assertTrue(xml_parser.load(self.multi_node_package_spec_))

        file_generator = CodeGenerator()
        self.assertTrue(file_generator.configure(xml_parser, spec))

        readme_file = self.path_template_ + "template/README.md"
        output_file = self.dir_name + "/test_README.md"

        self.assertTrue(file_generator.generate_file(readme_file, output_file))

    def test_cmake_multi_nodes(self):
        """test generation of a CMakeLists file involving multiple nodes spec.
        """

        spec = TemplateSpec()
        self.assertTrue(spec.load_spec(self.path_template_ + "config"))

        xml_parser = PackageXMLParser()
        self.assertTrue(xml_parser.set_template_spec(spec))
        self.assertTrue(xml_parser.load(self.multi_node_package_spec_))

        file_generator = CodeGenerator()
        self.assertTrue(file_generator.configure(xml_parser, spec))

        readme_file = self.path_template_ + "template/CMakeLists.txt"
        output_file = self.dir_name + "/test_CMakeLists.txt"

        self.assertTrue(file_generator.generate_file(readme_file, output_file))

    def test_package_multi_nodes(self):
        """test generation of a package file involving multiple nodes spec.
        """

        spec = TemplateSpec()
        self.assertTrue(spec.load_spec(self.path_template_ + "config"))

        xml_parser = PackageXMLParser()
        self.assertTrue(xml_parser.set_template_spec(spec))
        self.assertTrue(xml_parser.load(self.multi_node_package_spec_))

        file_generator = CodeGenerator()
        self.assertTrue(file_generator.configure(xml_parser, spec))

        readme_file = self.path_template_ + "template/package.xml"
        output_file = self.dir_name + "/test_package.xml"

        self.assertTrue(file_generator.generate_file(readme_file, output_file))


if __name__ == '__main__':
    print "test_package_generator -- begin"
    unittest.main()
    print "test_package_generator -- end"
