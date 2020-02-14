#!/usr/bin/env python
"""
@package package_generator
@file package_xml_parser.py
@author Anthony Remazeilles
@brief parse an xml file describing a package content

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.

"""

from termcolor import colored
import sys
import os
import rospkg
import xml.etree.cElementTree as ET
from xml.dom import minidom

from package_generator.enhanced_object import EnhancedObject
from package_generator.template_spec import TemplateSpec


def remove_empty_line(text):
    """Remove empty line within a multiline string

    Args:
        text (str): Mutliline string to process

    Returns:
        str: String with empty lines removed
    """
    res = list()
    for line in text.splitlines():
        if line.strip():
            res.append(line)
    return res


# TODO look at http://stackoverflow.com/questions/299588/validating-with-an-xml-schema-in-python
# for improving the xml format validation
class PackageXMLParser(EnhancedObject):
    """load a package description and prepare appropriate access structure

    Attributes:
        active_comp_ (int): id of the current active component (if several defined)
        data_comp_ (list): list of component specification
        data_depend_ (list): list of package dependency
        data_pack_ (dict): specifications of the package
        is_dependency_complete_ (bool): whether dependencies were automatically added
        root_ (TYPE): root of the xml tree
        spec_ (TemplateSpec): Template specification
    """

    def get_template(self, filename):

        try:
            tree = ET.ElementTree(file=filename)
        except IOError:
            self.log_error("Prb while opening file {}".format(filename))
            return None
        except ET.ParseError as error:
            self.log_error("Prb while parsing file: {}:".format(error))
            return None
        root = tree.getroot()

        if 'template' not in root.attrib.keys():
            self.log_error("Missing template tag in file {}".format(filename))
            return None
        return root.attrib['template']

    def __init__(self, name="PackageXMLParser"):
        """object constructor

        Args:
            name (str, optional): component name
        """
        #  call super class constructor
        super(PackageXMLParser, self).__init__(name)

        self.root_ = None
        self.spec_ = None
        self.data_pack_ = dict()
        self.data_depend_ = list()
        self.data_comp_ = list()
        self.active_comp_ = -1
        self.is_dependency_complete_ = True

    def __repr__(self):
        """Print object state
        """
        msg = "object_state: \n"
        msg += "package spec: {}\n".format(self.data_pack_)
        msg += "dependencies: {}\n".format(self.data_depend_)
        msg += "components: {} \n".format(len(self.data_comp_))
        for num, item in enumerate(self.data_comp_):
            msg += "item {}\n".format(num)
            msg += "attributes: {}\n".format(item['attributes'])
            msg += "interface:\n".format()
            for elt in item['interface'].items():
                msg += "\t {}\n".format(elt)

        return msg

    def set_template_spec(self, spec):
        """set the template specifications

        Args:
            spec (TemplateSpec): Object containing the template specs.

        Returns:
            Bool: operation success
        """
        expected_keys = ['package_attributes', 'component_interface', 'component_attributes']

        for item in expected_keys:
            if item not in spec.dico_:
                self.log_error("Missing key {} in provided dictionary".format(item))
                return False
        self.spec_ = spec
        return True

    # TODO: see how to put warning messages in the comment.
    # TODO: check the root tag is package
    def load(self, filename):
        """load a xml description provided in a file

        Args:
            filename (str): path of the file containing the xl description

        Returns:
            Bool: Operation sucess
        Warning:
            on success the active component is placed on the first one
        """
        if self.spec_ is None:
            msg_err = "Cannot load a package description without template spec"
            self.log_error(msg_err)
            return False
        if self.spec_.dico_ is None:
            self.log("Cannot load a package description without dictionary")
            return False

        # self.log("Parsing file: {}".format(filename))

        try:
            tree = ET.ElementTree(file=filename)
        except IOError:
            self.log_error("Prb while opening file {}".format(filename))
            return False
        except ET.ParseError as error:
            self.log_error("Prb while parsing file: {}:".format(error))
            return False

        self.root_ = tree.getroot()

        try:
            is_ok = self.load_all_spec()
        except AssertionError, err:
            self.log_error("Prb while parsing the file: {}".format(err.args))
            return False

        if not self.sanity_check():
            return False

        is_ok = self.extend_dependencies()

        if is_ok:
            self.active_comp_ = 0
        return is_ok

    def sanity_check(self):
        """Make sure the loaded spec is complete
           All fields are to be filled.
        Returns:
            Bool -- operation success
        """

        is_ok = True
        for [key, val] in self.data_pack_.items():
            if not val or not val.strip():
                self.log_error("Package attribute {} undefined".format(key))
                is_ok = False

        for num, one_comp in enumerate(self.data_comp_):
            for [key, val] in one_comp['attributes'].items():
                if not val or not val.strip():
                    self.log_error("Component {} attribute {} undefined".format(num, key))
                    is_ok = False
            for [type_interface, list_interface] in one_comp['interface'].items():
                for num_item, item in enumerate(list_interface):
                    for key, val in item.items():
                        if not val or not val.strip():
                            self.log_error("Key {} of {}[{}] undefined".format(key, type_interface, num_item))
                            is_ok = False

        return is_ok

    def extend_dependencies(self):
        """Check if dependencies required by the package spec and
            the used interface is effectively defined.

        Returns:
            Bool: Operation success
        """

        # will gather package name and component requiring it
        pkg_dependencies = dict()

        # check if dependencies are defined directly at the template level
        if self.spec_.dep_from_template_ is not None:
            try:
                pkg_dep = self.spec_.dep_from_template_()
            except TypeError as err:
                err_msg = "Error while calling external function dep_from_template. \n"
                err_msg += " exception: {} \n".format(err)
                err_msg += " Discarding dependency checking"
                self.log_error(err_msg)
                pkg_dep = list()
            # self.log("template dependencies: \n {}".format(pkg_dep))

            for one_pack in pkg_dep:
                if one_pack not in pkg_dependencies:
                    pkg_dependencies[one_pack] = list()
                pkg_dependencies[one_pack].append('template')

        # look now at the dependencies related to available interfaces
        if self.spec_.dep_from_interface_ is not None:
            for comp in self.data_comp_:
                # self.log("Checking comp with interface: \n {}".format(comp['interface']))
                for one_interface in comp['interface']:
                    for item in comp['interface'][one_interface]:
                        try:
                            pkg_dep = self.spec_.dep_from_interface_(one_interface, item)
                        except TypeError as err:
                            err_msg = "Error while calling dep_from_interface external function. \n"
                            err_msg += " input param : {}, {} \n".format(one_interface, item)
                            err_msg += " exception: {} \n".format(err)
                            err_msg += " Discarding dependency checking"
                            self.log_error(err_msg)
                            pkg_dep = list()
                        # self.log("Found dependency: {}".format(pkg_dep))
                        for one_pack in pkg_dep:
                            if one_pack not in pkg_dependencies:
                                pkg_dependencies[one_pack] = list()
                            pkg_dependencies[one_pack].append("{}".format(item))

        # all required dependencies gathered.
        # Now we check if they are provided by the Developer.
        # self.log("Detected dependencies {}".format(pkg_dependencies.keys()))
        missing_dep = dict()
        for dependency in pkg_dependencies:
            if dependency not in self.data_depend_:
                missing_dep[dependency] = pkg_dependencies[dependency]
        # self.log("List of missing dependencies {}".format(missing_dep))

        if missing_dep:
            self.is_dependency_complete_ = False
        for missing in missing_dep:
            msg_err = "Dependency {} not listed in xml file \n".format(missing)
            for item in missing_dep[missing]:
                msg_err += "\t Required by {}. \n".format(item)
            msg_err += "Dependency automatically added"
            self.log_warn(msg_err)
            self.data_depend_.append(missing)
            # adding the dependency to the xml tree
            ET.SubElement(self.root_, "depend").text = missing

        return True

    def load_package_attribute(self):
        """Check and get the package attributes
        """
        # self.log("Package attributes: \n{}".format(self.root_.attrib))

        attributes_package = self.spec_.dico_['package_attributes']

        for attrib in attributes_package:
            assert attrib in self.root_.attrib.keys(), "Missing required attrib {} for package description".format(attrib)
            # self.log("package attribute {} set to {}".format(attrib, self.root_.attrib[attrib]))
            self.data_pack_[attrib] = self.root_.attrib[attrib]

        for attrib in self.root_.attrib.keys():
            if attrib not in attributes_package:
                self.log_warn("Provided attrib {} ignored".format(attrib))

    def load_one_comp_interface(self, xml_interface):
        """Check and store a component interface

        Args:
            xml_interface (TYPE): xml spec of the comp

        Returns:
            dict: the comp interface
        """
        # self.log("Checking comp interface {}".format(xml_interface.tag))

        interface_comp = self.spec_.dico_['component_interface'].keys()

        assert xml_interface.tag in interface_comp, "Unknown interface {}".format(xml_interface.tag)

        interface_spec = dict()
        interface_spec["type"] = xml_interface.tag
        interface_spec["attributes"] = dict()

        attributes = self.spec_.dico_['component_interface'][xml_interface.tag]

        for attrib in attributes:
            assert attrib in xml_interface.attrib.keys(), 'Missing required attribute {} for {} interface. Check line {}'.format(attrib, xml_interface.tag, xml_interface.attrib)
            interface_spec["attributes"][attrib] = xml_interface.attrib[attrib]

        # self.log("Requested interface for {} correct".format(xml_interface.tag))

        for attrib in xml_interface.attrib.keys():
            if attrib not in attributes:
                self.log_warn("Provided attrib {} of interface {} ignored (check {})".format(attrib, xml_interface.tag, xml_interface.attrib))
        return interface_spec

    def load_comp_spec(self, xml_comp):
        """Load the component specification

        Args:
            xml_comp (TYPE): xml description of the comp

        Returns:
            dict: uploaded comp attributes and possible interface
        """
        # TODO check as well component names

        loc_data_comp = dict()
        loc_data_comp['attributes'] = dict()

        attributes_comp = self.spec_.dico_['component_attributes']

        for attrib in attributes_comp:
            assert attrib in xml_comp.attrib.keys(), "Missing required attribute {} for comp description".format(attrib)
            loc_data_comp['attributes'][attrib] = xml_comp.attrib[attrib]

        # self.log("Requested comp description correct")

        for attrib in xml_comp.attrib.keys():
            if attrib not in attributes_comp:
                self.log_warn("Provided attrib {} ignored".format(attrib))

        interface_comp = self.spec_.dico_['component_interface'].keys()

        # self.log("Check: comp interface is: {}".format(interface_comp))

        loc_data_comp['interface'] = dict()
        for item in interface_comp:
            loc_data_comp['interface'][item] = list()

        for child in xml_comp:
            # self.log("Checking for {}".format(child))
            child_interface = self.load_one_comp_interface(child)
            # self.log("Adding entry for type {}".format(child_interface["type"]))
            # self.log("Within: {}".format(loc_data_comp['interface']))

            loc_data_comp['interface'][child_interface["type"]].append(child_interface["attributes"])

        return loc_data_comp

    def load_one_dependency(self, xml_dep):
        """Add a dependency

        Args:
            xml_dep (TYPE): Description
        """
        assert xml_dep.text, "Missing dependency text"
        self.data_depend_.append(xml_dep.text)

    def load_child_spec(self, xml_item):
        """Load the spec of a child element in the tree

        Args:
            xml_item (TYPE): xml item to look at
        """
        tag = xml_item.tag
        if tag == "component":
            self.data_comp_.append(self.load_comp_spec(xml_item))
        elif tag == "depend":
            self.load_one_dependency(xml_item)
        else:
            self.log_error("Unknown tag {}".format(tag))

    def load_all_spec(self):
        """
        checking the sanity of the xml file

        Returns:
            Bool: Operation sucess
        """

        if not self.root_:
            return False

        self.load_package_attribute()

        for child in self.root_:
            self.load_child_spec(child)

        # TODO should not always return true!
        return True

    def get_comp_spec(self):
        """Return all components spec

        Returns:
            list: All comps description
        """
        return self.data_comp_

    def get_package_spec(self):
        """Return the package spec

        Returns:
            dict: list of attributes of the package
        """
        return self.data_pack_

    def get_dependency_spec(self):
        """Get all the dependencies defined

        Returns:
            List: all dependencies defined
        """
        return self.data_depend_

    def get_number_comps(self):
        """Returns the number of components being defined

        Returns:
            int: number of components
        """
        return len(self.data_comp_)

    def set_active_comp(self, comp_id):
        """set the active component be handled

        Args:
            comp_id (int): active comp number

        Returns:
            Bool: True if the operation succeeded
        """

        if comp_id < 0:
            self.log_error("comp id ({}) should be >= 0".format(comp_id))
            return False
        if not self.data_comp_:
            self.log_error("No specification read so far")
            return False
        if comp_id >= len(self.data_comp_):
            msg = "comp id ({}) should be < {}".format(comp_id,
                                                       len(self.data_comp_))
            self.log_error(msg)
            return False

        self.active_comp_ = comp_id
        return True

    def get_active_comp_spec(self):
        """Provide all the spec of a given component

        Returns:
            TYPE: Description
        """
        assert self.active_comp_ != -1, "No active component defined"
        assert self.active_comp_ < len(self.data_comp_), "Active component {} should be less than {}".format(self.active_comp_,
                                                                                                        len(self.data_comp_))
        return self.data_comp_[self.active_comp_]

    def write_xml(self, filename):
        """write the xml into a file

        Args:
            filename (str): filename to use

        Returns:
            Bool: True on success
        """
        self.log("Writting xml into file {}".format(filename))

        str_tmp = ET.tostring(self.root_, 'utf-8')

        reparsed = minidom.parseString(str_tmp)
        res = reparsed.toprettyxml(indent="  ", encoding="utf-8")
        res = remove_empty_line(res)

        # with open("Output.txt", "w") as text_file:
        #     text_file.write(res2)
        with open(filename, 'w') as file_handler:
            for item in res:
                file_handler.write("{}\n".format(item))
        return True

    def generate_xml_from_spec(self, template_name, filename):
        """Generate an xml file based on the template dictionary

        Args:
            template_name (str): name of the template of interest
            filename (str): filename where xml skeleton is to be written

        Returns:
            Bool: Operation success
        """

        if self.spec_ is None:
            self.log_error("Template spec is missing")
            return False

        xml_pack = ET.Element('package')
        for item in self.spec_.dico_['package_attributes']:
            xml_pack.set(item, '')
        # todo should we add the attribute template to the config?
        xml_pack.set("template", template_name)

        xml_comp = ET.SubElement(xml_pack, "component")
        for item in self.spec_.dico_['component_attributes']:
            xml_comp.set(item, '')

        for item in self.spec_.dico_['component_interface']:
            xml_one_interface = ET.SubElement(xml_comp, item)
            for item_attrib in self.spec_.dico_['component_interface'][item]:
                xml_one_interface.set(item_attrib, '')

        if self.spec_.dep_from_template_ is None:
            xml_dep = ET.SubElement(xml_pack, "depend")
            xml_dep.text = ""
        else:
            try:
                pkg_dep = self.spec_.dep_from_template_()
            except TypeError as err:
                err_msg = "Error while calling external function dep_from_template. \n"
                err_msg += " exception: {} \n".format(err)
                err_msg += " Discarding dependency checking"
                self.log_error(err_msg)
                pkg_dep = [""]

            for item in pkg_dep:
                xml_dep = ET.SubElement(xml_pack, "depend")
                xml_dep.text = item

        data_string = ET.tostring(xml_pack, 'utf-8')
        reparsed = minidom.parseString(data_string)
        res = reparsed.toprettyxml(indent="  ", encoding="utf-8")

        # self.log("generating template: \n {} \n".format(res))

        with open(filename, 'w') as file_handler:
            file_handler.write("{}".format(res))
        self.log("XML skeleton written in file {}".format(filename))
        self.log("Edit the file, remove or comment unused interface")
        return True

    def set_empty_spec(self):
        """Generate an empty package spec file (for template checking)

        Returns:
            [Bool] -- True on success
        """

        if self.spec_ is None:
            self.log_error("Template spec is missing")
            return False

        # setting package attrbutes
        attributes_package = self.spec_.dico_['package_attributes']

        self.data_pack_ = dict()

        for attrib in attributes_package:
            self.data_pack_[attrib] = ""

        # setting one comp
        attributes_comp = self.spec_.dico_['component_attributes']

        comp_spec = dict()
        comp_spec["attributes"] = dict()

        for attrib in attributes_comp:
            comp_spec["attributes"][attrib] = ""

        interface_comp = self.spec_.dico_['component_interface'].keys()
        comp_spec["interface"] = dict()

        for item in interface_comp:
            comp_spec['interface'][item] = list()

            # TODO can we make a loop on key and key value?
            attributes = self.spec_.dico_['component_interface'][item]

            fake_interface = dict()
            for one_attr in attributes:
                fake_interface[one_attr] = ""
            comp_spec['interface'][item].append(fake_interface)

        self.data_comp_ = list()
        self.data_comp_.append(comp_spec)
        self.data_depend_ = list()
        self.active_comp_ = 0
        self.is_dependency_complete_ = True
        return True


USAGE = """ usage: generate_xml_skel package_template xml_skeleton
package_template : name of the template to use
xml_skeleton: xml description of the component(s) interface

Packages template: either one defined in package `package_generator_templates`,
                   either a path to a local one.
"""


def main_generate_xml():
    """Generate a xml package description based on a template spec

    Returns:
        int: negative value on error
    """
    rospack = rospkg.RosPack()

    try:
        default_templates_path = rospack.get_path('package_generator_templates')
        default_templates_path += "/templates/"
    except rospkg.common.ResourceNotFound as error:
        msg = "Package package_generator_templates not found in rospack"
        print colored(msg, "yellow")
        print colored("{}".format(error), "yellow")
        default_templates_path = None

    available_templates = None
    # look for the templates available
    if default_templates_path is not None:
        available_templates = os.listdir(default_templates_path)

    if len(sys.argv) != 3:
        print colored("Wrong input parameters !", "red")
        print colored(USAGE, "yellow")
        if available_templates is not None:
            msg = "Available templates are: {}"
            print colored(msg.format(available_templates), 'yellow')
        print "Bye bye"
        return -1

    path_template = sys.argv[1]
    package_spec = sys.argv[2]

    # check first if the provided filename is correct
    # we do not accept the name to be an existing file

    if os.path.isfile(package_spec):
        print colored("file {} already exists".format(package_spec), "red")
        print colored("Please select another filename, or remove the file")
        print "Bye Bye"
        return -1

    # if relative path or absolute path, check the containing folder exists
    folder = os.path.dirname(package_spec)

    if folder and not os.path.isdir(folder):
        msg_err = "file {} cannot be created in {}".format(package_spec, folder)
        print colored(msg_err, "red")
        print colored("Please adjust parameter")
        print "Bye Bye"
        return -1

    # searching for the template location
    if os.path.isabs(path_template):
        print "Loading model from absolute path {}".format(path_template)
    else:
        # relative path.
        # either from the current path, or from the template package
        path_current = os.getcwd()
        path_attempt = path_current + "/" + path_template

        if os.path.isdir(path_attempt):
            path_template = path_attempt
            print "Loading template: {}".format(path_template)
        else:
            if path_template in available_templates:
                path_template = default_templates_path + path_template
                msg = "Loading template: {}"
                print msg.format(path_template)
            else:
                msg = "Template name not found in package_generator_templates"
                print colored(msg, "red")
                print colored("Please verify your setup", "red")
                return -1

    package_parser = PackageXMLParser()

    dir_template_spec = path_template + "/config/"
    spec = TemplateSpec()

    if not spec.load_spec(dir_template_spec):
        print colored("Could not load the template spec", "red")
        return -1

    # print "Setting the template spec to \n {}".format(spec.dico_)
    if not package_parser.set_template_spec(spec):
        print colored("Prb while setting the parser dictionary", "red")
        return -1

    package_parser.generate_xml_from_spec(os.path.basename(path_template), package_spec)

    print colored("Bye bye", "green")
    return 0
