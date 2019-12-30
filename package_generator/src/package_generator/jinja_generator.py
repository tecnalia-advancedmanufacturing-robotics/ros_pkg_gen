#!/usr/bin/env python
"""
@package package_generator
@file jinja_generator.py
@author Anthony Remazeilles
@brief parse a template file, and generate the related file

Copyright (C) 2019 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

import jinja2
from package_generator.enhanced_object import EnhancedObject


class JinjaGenerator(EnhancedObject):

    def __init__(self, name="JinjaGenerator"):
        # call super class constructor
        super(JinjaGenerator, self).__init__(name)
        self.xml_parser_ = None
        self.spec_ = None
        self.rendered_ = None

    def configure(self, parser, spec):
        self.xml_parser_ = parser
        self.spec_ = spec
        return True

    def generate_disk_file(self, template_file):

        # creating the dictionnary

        context = dict()
        context["package"] = self.xml_parser_.data_pack_
        context["components"] = self.xml_parser_.data_comp_
        context["active_node"] = self.xml_parser_.active_comp_

        # print "Check components {}".format(context["components"])

        with open(template_file) as file_:
            template = jinja2.Template(file_.read(), trim_blocks=True)

        self.rendered_ = template.render(context)

        # print "Rendered: \n{}".format(self.rendered_)
        return True

    def generate_open_file(self, template_file):

        # creating the dictionnary

        context = dict()
        context["package"] = self.xml_parser_.data_pack_
        context["components"] = self.xml_parser_.data_comp_
        context["active_node"] = self.xml_parser_.active_comp_

        # print "Check components {}".format(context["components"])

        template = jinja2.Template(template_file, trim_blocks=True)

        self.rendered_ = template.render(context)

        # print "Rendered: \n{}".format(self.rendered_)
        return True
