#!/usr/bin/env python
"""
@package package_generator
@file test_code_generator.py
@author Anthony Remazeilles
@brief test the code generator

Copyright (C) 2019 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).

To run a single test, type:
python test_jinja_generator.py JinjaGeneratorTest.test_is_launching
To launch all use:
python -m unittest discover --verbose
"""

import unittest
import os
import rospkg

from package_generator.package_xml_parser import PackageXMLParser
from package_generator.jinja_generator import JinjaGenerator
from package_generator.template_spec import TemplateSpec
from package_generator.code_generator import CodeGenerator


class JinjaGeneratorTest(unittest.TestCase):
    """
    tests for the code generator
    """

    def setUp(self):
        """
        Common initialization for all tester
        """
        file_content = (
            '<?xml version="1.0" encoding="UTF-8"?>' '\n'
            '<package name="great_package" author="anthony" author_email="anthony@todo.todo" description="The extended package" license="TODO" copyright="TRI">' '\n'
            '   <component name="node_extended" frequency="100.0">' '\n'
            '       <publisher name="pub" type="std_msgs::Bool" description=""/>' '\n'
            '       <publisher name="pub_second" type="std_msgs::String" description=""/>' '\n'
            '       <subscriber name="sub_in" type="std_msgs::String" description="" />' '\n'
            '       <serviceClient name="service_client" type="std_srvs::Trigger" description=""/>' '\n'
            '       <serviceServer name="service_server" type="std_srvs::SetBool" description=""/>' '\n'
            '       <parameter name="param_one" type="std::string" value="Empty" description=""/>'  '\n'
            '       <parameter name="param_two" type="bool" value="1" description=""/>'  '\n'
            '       <actionServer name="action_server" type="bride_tutorials::TriggerPublish" description=""/>' '\n'
            '       <actionClient name="action_client" type="bride_tutorials::TriggerPublish" description=""/>' '\n'
            '   </component>' '\n'
            '<depend>std_msgs</depend>' '\n'
            '<depend>std_srvs</depend>' '\n'
            '<depend>bride_tutorials</depend>' '\n'
            '<depend>actionlib</depend>' '\n'
            '<depend>actionlib_msgs</depend>' '\n'
            '<depend>roscpp</depend>' '\n'
            '</package>' '\n')

        # print "File content: \n{}".format(file_content)

        self.dir_name = "/tmp/test_package_generator"
        if not os.path.exists(self.dir_name):
            print "Creating the repo {}".format(self.dir_name)
            os.makedirs(self.dir_name)

        file_xml = self.dir_name + "/node_spec.ros_package"
        with open(file_xml, 'w') as open_file:
            open_file.write(file_content)

        rospack = rospkg.RosPack()
        self.template_path_ = rospack.get_path('package_generator_templates')
        self.template_path_ += "/templates/cpp_node_update/"

        self.spec = TemplateSpec()
        self.assertTrue(self.spec.load_spec(self.template_path_ + "config"))

        self.xml_parser = PackageXMLParser()
        self.assertTrue(self.xml_parser.set_template_spec(self.spec))
        self.assertTrue(self.xml_parser.load(file_xml))

        self.generator_ = JinjaGenerator()
        self.assertTrue(self.generator_.configure(self.xml_parser, self.spec))

    def test_is_launching(self):
        """
        Confirm that Jinja does not interfere with custom tags
        """
        filename = self.dir_name + "/template_test_apply_function.cpp"
        file_content = (
            '{forallpublisher}' '\n'
            'name={name}' '\n'
            'pkg={apply-get_package_type} ' '\n'
            'type={apply-get_class_type}' '\n'
            'in one line: {name}: {apply-get_python_type}' '\n'
            'cpp path: {name}: {apply-get_cpp_path}' '\n'
            'more tricky: {name}: {apply-get_cpp_path} {unknowntag}' '\n'
            'more tricky: {name}: {apply-get_cpp_path} {apply-unknown}' '\n'
            '\n'
            '{endforallpublisher}')

        with open(filename, 'w') as openfile:
            openfile.write(file_content)

        self.assertTrue(self.generator_.generate_disk_file(filename))

        for generated, groundtruth in zip(self.generator_.rendered_,
                                          file_content.splitlines()):
            # print "Comparing |{}| with |{}|".format(generated, groundtruth)
            self.assertEqual(generated, groundtruth)

    def test_jinja(self):
        filename = self.dir_name + "/template_test_apply_function.cpp"

        file_content = (
            '{forallpublisher}' '\n'
            'name={{active_node}}' '\n'
            'pkg={{package.name}}' '\n'
            '{% for item in components[0].interface.publisher %}' '\n'
            'publisher: {{ item.name }}' '\n'
            '{% endfor %}' '\n'
            '{% if components[0].interface.publisher %}' '\n'
            'At least one publisher ' '\n'
            '{% endif %}' '\n'
            '{% if components[0].interface.directPublisher %}' '\n'
            'At least one direct publisher ' '\n'
            '{% else %}' '\n'
            'No direct publisher ' '\n'
            '{% endif %}' '\n'
            '{endforallpublisher}' '\n')

        expected_output = (
            '{forallpublisher}' '\n'
            'name=0' '\n'
            'pkg=great_package' '\n'
            'publisher: pub' '\n'
            'publisher: pub_second' '\n'
            'At least one publisher ' '\n'
            'No direct publisher ' '\n'
            '{endforallpublisher}')

        with open(filename, 'w') as openfile:
            openfile.write(file_content)

        self.assertTrue(self.generator_.generate_disk_file(filename))
        for generated, groundtruth in zip(self.generator_.rendered_,
                                          expected_output.splitlines()):
            self.assertEqual(generated, groundtruth)

    def test_custom_on_jinja(self):
        filename = self.dir_name + "/template_test_apply_function.cpp"

        file_content = (
            '{ifpublisher}' '\n'
            'name={{active_node}}' '\n'
            'pkg={{package.name}}' '\n'
            '{% for item in components[0].interface.publisher %}' '\n'
            'publisher: {{ item.name }}' '\n'
            '{% endfor %}' '\n'
            '' '\n'
            '{% if components[0].interface.publisher %}' '\n'
            'At least one publisher ' '\n'
            '{% endif %}' '\n'
            '{% if components[0].interface.directPublisher %}' '\n'
            'At least one direct publisher ' '\n'
            '{% else %}' '\n'
            'No direct publisher ' '\n'
            '{% endif %}' '\n'
            '{endifpublisher}' '\n')

        expected_output = (
            'name=0' '\n'
            'pkg=great_package' '\n'
            'publisher: pub' '\n'
            'publisher: pub_second' '\n'
            '' '\n'
            'At least one publisher ' '\n'
            'No direct publisher ' '\n')

        custom_generator = CodeGenerator()
        self.assertTrue(custom_generator.configure(self.xml_parser, self.spec))
        custom_generator.reset_output_file()

        with open(filename, 'w') as openfile:
            openfile.write(file_content)

        self.assertTrue(custom_generator.process_file(filename))

        print "Custom generation \n"
        for line in custom_generator.rendered_:
            print line

        self.assertTrue(self.generator_.generate_open_file(custom_generator.rendered_))

        # print "Rendered file \n"
        # print self.generator_.rendered_

        for generated, groundtruth in zip(self.generator_.rendered_,
                                          expected_output.splitlines()):
            print "Comparing |{}| with |{}|".format(generated, groundtruth)

            self.assertEqual(generated, groundtruth)

    def test_write_file(self):
        file_content = (
            'name={{active_node}}' '\n'
            'pkg={{package.name}}' '\n'
            '{% for item in components[0].interface.publisher %}' '\n'
            'publisher: {{ item.name }}' '\n'
            '{% endfor %}' '\n'
            '{% if components[0].interface.publisher %}' '\n'
            'At least one publisher ' '\n'
            '{% endif %}' '\n'
            '{% if components[0].interface.directPublisher %}' '\n'
            'At least one direct publisher ' '\n'
            '{% else %}' '\n'
            'No direct publisher ' '\n'
            '{% endif %}')
        expected_output = (
            'name=0' '\n'
            'pkg=great_package' '\n'
            'publisher: pub' '\n'
            'publisher: pub_second' '\n'
            'At least one publisher ' '\n'
            'No direct publisher ' '\n')
        template_name = self.dir_name + "/jinja_template.txt"
        with open(template_name, 'w') as openfile:
            openfile.write(file_content)
        rendered_name = self.dir_name + "/jinja_rendered.txt"

        self.assertTrue(self.generator_.generate_disk_file(template_name, rendered_name))
        self.assertTrue(os.path.isfile(rendered_name))

        lines = list()
        with open(rendered_name) as input_file:
            for line in input_file:
                lines.append(line.rstrip('\n'))

        for generated, groundtruth in zip(lines,
                                          expected_output.splitlines()):
            # print "Comparing |{}| with |{}|".format(generated, groundtruth)

            self.assertEqual(generated, groundtruth)
        # todo check write request with bad filename


if __name__ == '__main__':
    print "test_jinja_generator -- begin"
    unittest.main()
    print "test_jina_generator -- end"
