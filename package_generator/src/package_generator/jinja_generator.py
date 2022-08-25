#!/usr/bin/env python3
"""
@package package_generator
@file jinja_generator.py
@author Anthony Remazeilles
@brief parse a template file, and generate the related file

Copyright (C) 2019 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.
"""

from package_generator.enhanced_object import EnhancedObject

# Todo should these import be maintained?
from package_generator.template_spec import TemplateSpec
from package_generator.package_xml_parser import PackageXMLParser
import jinja2


class JinjaGenerator(EnhancedObject):
    """Handles the file generation using jinja generator

    Attributes:
        rendered_ (list): list of str lines rendered by the generator
        spec_ (TemplateSpec): Package template spec
        xml_parser_ (PackageXMLParser): Spec of the package to generate
    """

    def __init__(self, name="JinjaGenerator"):
        """Basic constructor

        Args:
            name (str, optional): Component name
        """
        # call super class constructor
        super(JinjaGenerator, self).__init__(name)
        self.xml_parser_ = None
        self.spec_ = None
        self.rendered_ = None

    def configure(self, parser, spec):
        """Set the template and package specification

        Args:
            parser (PackageXMLParser): Spec of the package to generate
            spec (TemplateSpec): Package template spec

        Returns:
            Bool: Operation success
        """
        self.xml_parser_ = parser
        self.spec_ = spec
        return True

    def generate_disk_file(self, template_file, output_file=None, force_write=None):
        """Render a file whose template is read from the disk

        Args:
            template_file (str): template filename
            output_file (None, optional): name of the file to be stored
            force_write (None, optional): whether the file should be written if empty

        Returns:
            Bool: Operation success
        """
        # creating the dictionnary
        active_comp = self.xml_parser_.active_comp_
        comp_attributes = self.xml_parser_.data_comp_[active_comp]['attributes']
        comp_interface = self.xml_parser_.data_comp_[active_comp]['interface']
        context = {
            'package': self.xml_parser_.data_pack_,
            'components': self.xml_parser_.data_comp_,
            'active_comp': active_comp,
            'dependencies': self.xml_parser_.data_depend_,
            'attributes': comp_attributes,
            'interface': comp_interface
        }
        # print "Check components {}".format(context["components"])

        with open(template_file) as file_:
            template = jinja2.Template(file_.read(), trim_blocks=True)

        # Rendered element is a unique string containing all content
        # code generator is generating a list of string instead
        # todo handle exceptions, here or in upper layers.
        str_rendered = template.render(context)

        # convert it in the list of string format:
        self.rendered_ = str_rendered.splitlines()
        if not output_file:
            return True

        if force_write or self.rendered_:
            return self.write_rendered_file(output_file)
        # print "Rendered: \n{}".format(self.rendered_)
        return True

    def generate_open_file(self, template_file, output_file=None, force_write=None):
        """Render a file whose template is provided as parameter

        Args:
            template_file (list): list of the str lines to be rendered.
            output_file (None, optional): name of the file to be stored
            force_write (None, optional): whether the file should be written if empty

        Returns:
            Bool: Operation success
        """
        # template_file is expected to be a unique string, and not a list of string
        # as is generated by the custom generator

        # creating the dictionnary
        active_comp = self.xml_parser_.active_comp_
        comp_attributes = self.xml_parser_.data_comp_[active_comp]['attributes']
        comp_interface = self.xml_parser_.data_comp_[active_comp]['interface']
        context = {
            'package': self.xml_parser_.data_pack_,
            'components': self.xml_parser_.data_comp_,
            'active_comp': active_comp,
            'dependencies': self.xml_parser_.data_depend_,
            'attributes': comp_attributes,
            'interface': comp_interface
        }
        # print "Check components {}".format(context["components"])

        str_template = "\n".join(template_file)

        template = jinja2.Template(str_template, trim_blocks=True)
        # todo handle exceptions, here or in upper layers.
        str_rendered = template.render(context)

        # convert it in the list of string format:
        self.rendered_ = str_rendered.splitlines()

        if not output_file:
            return True

        if force_write or self.rendered_:
            return self.write_rendered_file(output_file)
        # print "Rendered: \n{}".format(self.rendered_)
        return True

    def write_rendered_file(self, filename):
        """write the generated file

        Args:
            filename (str): filename of the file to be stored

        Returns:
            Bool: Operation sucess
        """
        try:
            with open(filename, 'w') as out_file:
                for item in self.rendered_:
                    out_file.write(item + '\n')
                # todo could this addition be removed?
                # if self.rendered_:
                #    out_file.write('\n')

        except IOError:
            self.log_error("Prb while opening output file {}".format(filename))
            return False
        return True

    def check_template_file(self, filename, is_filename=True):
        """check if the template file is valid

        Args:
            filename {str} --  str (either absolute filename or file content)

        Keyword Arguments:
            is_filename {bool} -- to distinguish the two cases (default: {True})

        Returns:
            bool -- check success
        """
        env = jinja2.Environment()
        # self.log("Checking file: {}".format(filename))

        if is_filename:
            try:
                with open(filename) as template:
                    env.parse(template.read())
            except jinja2.exceptions.TemplateSyntaxError as err:
                self.log_warn("Syntax error: line {}: {}".format(err.lineno, err))
                return False
            return True

        # file provided is not a filename, but the outcome of another generator
        str_template = "\n".join(filename)
        try:
            env.parse(str_template)
        except jinja2.exceptions.TemplateSyntaxError as err:
            self.log_warn("Syntax error: line {}: {}".format(err.lineno, err))
            return False
        return True

# todo:
# * Handle error situations
# * update jinja_generator docstrings
# * handle the template sanity check (if Jinja used)
