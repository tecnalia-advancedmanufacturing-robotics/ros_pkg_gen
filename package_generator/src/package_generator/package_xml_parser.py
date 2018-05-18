#!/usr/bin/env python
"""
@package package_generator
@file package_xml_parser.py
@author Anthony Remazeilles
@brief parse an xml file describing a package content

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import rospy
import xml.etree.cElementTree as ET
from xml.dom import minidom

from package_generator.generate_dict import GenerateDictionnary
from package_generator.enhanced_object import EnhancedObject


def str2bool(strg):
    """Summary

    Args:
        strg (str): string containing a boolean value

    Returns:
        Bool: corresponding boolean value
    """
    return strg.lower() in ("yes", "true", "t", "1")

def remove_empty_line(text):
    res = list()
    for line in text.splitlines():
        if line.strip():
            res.append(line)
    return res

# todo look at http://stackoverflow.com/questions/299588/validating-with-an-xml-schema-in-python
# for improving the xml format validation

class PackageXMLParser(EnhancedObject):
    """load a package description and prepare appropriate access structure

    Attributes:
        active_node_ (int): id of the current active node (if several defined)
        data_depend_ (list): list of package dependency
        data_node_ (list): list of node specification
        data_pack_ (dict): specifications of the package
        dico_ (GenerateDictionnary): Dictionnary expected for node description
        is_dependency_complete_ (bool): whether dependencies were automatically added
        root_ (TYPE): root of the xml tree

    """
    def __init__(self, name="PackageXMLParser"):
        #  call super class constructor
        super(PackageXMLParser, self).__init__(name)

        self.root_ = None
        self.dico_ = None
        self.data_pack_ = dict()
        self.data_depend_ = list()
        self.data_node_ = list()
        self.active_node_ = -1
        self.is_dependency_complete_ = True

    def set_dictionnary(self, dico):
        """set the dictionnary to be used for parsing package spec

        Args:
            dico (GenerateDictionnary): Object containing the specs.
        """
        self.dico_ = dico

    # todo: see how to put warning messages in the comment.
    def load(self, filename):
        """load a xml description provided in a file

        Args:
            filename (TYPE): Description

        Returns:
            Bool: true if it succeeded

        Deleted Parameters:
            Warning: on success the active node is placed on the first one
        """
        if self.dico_ is None:
            self.log("Cannot load a package decsription without dictionnary")
            return False
        self.log("Parsing file: {}".format(filename))

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
            is_ok = self.sanity_check(verbose=True)
        except AssertionError, err:
            self.log_error("Prb while parsing the file: {}".format(err.args))
            return False
        # todo if the previous operation return false,
        # we should not do the following
        is_ok = self.extend_dependencies()
        if is_ok:
            self.print_xml_parsed()
            self.active_node_ = 0
        return is_ok

    # todo: using the same type tag for all interfaces would ease the process.
    def extend_dependencies(self):
        for node in self.data_node_:
            # can not loop this way, since all interface do not share
            # the same type tag
            # for type_interface in  self.node_interface_:
            #    if type_interface == 'parameter':
            #        continue
            #    if node['interface'][type_interface]:

            # gathering the dependencies provided by the interface
            # for helping the developer, we collect the dependency,
            # interface type, and interface name

            pkg_dependencies = dict()

            if node['interface']["publisher"]:
                for item in node['interface']["publisher"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "publisher", "name":item["name"]})

            if node['interface']["directPublisher"]:
                for item in node['interface']["directPublisher"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "directPublisher", "name":item["name"]})

            if node['interface']["directSubscriber"]:
                for item in node['interface']["directSubscriber"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "directSubscriber", "name":item["name"]})


            if node['interface']["subscriber"]:
                for item in node['interface']["subscriber"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "subscriber", "name":item["name"]})

            if node['interface']["serviceClient"]:
                for item in node['interface']["serviceClient"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "serviceClient", "name":item["name"]})

            if node['interface']["serviceServer"]:
                for item in node['interface']["serviceServer"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "serviceServer", "name":item["name"]})

            if node['interface']["actionServer"]:
                for item in node['interface']["actionServer"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "actionServer", "name":item["name"]})

            if node['interface']["actionClient"]:
                for item in node['interface']["actionClient"]:
                    pkg_dep = item['type'].split("::")[0]
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "actionClient", "name":item["name"]})

            if node['interface']["dynParameter"]:
                pkg_dep = 'dynamic_reconfigure'
                if pkg_dep not in pkg_dependencies:
                    pkg_dependencies[pkg_dep] = list()
                pkg_dependencies[pkg_dep].append({"type_interface": "dynParameter", "name":"dyn_recon"})
            else:
                # todo in the current setup, we always add the dependency
                # to dynamic reconfigure, even if we do not use it
                pkg_dep = 'dynamic_reconfigure'
                if pkg_dep not in self.data_depend_:
                    self.log_warn("Dependency on {} added for building".format(pkg_dep))
                    self.data_depend_.append(pkg_dep)

            if node['interface']["actionServer"] or node['interface']["actionClient"]:
                pkg_deps = ['actionlib', 'actionlib_msgs']

                for pkg_dep in pkg_deps:
                    if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                    pkg_dependencies[pkg_dep].append({"type_interface": "action", "name":"action"})

            if node['interface']["listener"] or node['interface']["broadcaster"]:
                pkg_dep = 'tf'
                if pkg_dep not in pkg_dependencies:
                        pkg_dependencies[pkg_dep] = list()
                pkg_dependencies[pkg_dep].append({"type_interface": "tf", "name":"tf"})

            self.log("List of detected dependencies {}".format(pkg_dependencies))
            missing_dep = dict()
            for dependency in pkg_dependencies:
                if dependency not in self.data_depend_:
                    missing_dep[dependency] = pkg_dependencies[dependency]
            self.log("List of missing dependencies {}".format(missing_dep))

            if missing_dep:
                self.is_dependency_complete_ = False
            for missing in missing_dep:
                self.log_error("Dependency on {} not listed in xml file".format(missing))
                self.log_error("Required at least for node {}".format(node["attributes"]["name"]))
                for item in missing_dep[missing]:
                    self.log_error("Used in interface {} named {}".format(item["type_interface"], item["name"]))
                self.data_depend_.append(missing)
                # adding the dependency to the xml tree
                ET.SubElement(self.root_,"depend").text=missing

        return True

    def sanity_check_package_attribute(self):
        """Check the package attributes are the ones expected
        """
        self.log("Package attributes: \n{}".format(self.root_.attrib))

        attributes_package = self.dico_.spec_['package_attributes']

        for attrib in attributes_package:
            assert attrib in self.root_.attrib.keys(), "Missing required attrib {} for package description".format(attrib)
            # self.log("package attribute {} set to {}".format(attrib, self.root_.attrib[attrib]))
            self.data_pack_[attrib] = self.root_.attrib[attrib]

        # self.log("Requested package description correct")

        for attrib in self.root_.attrib.keys():
            if attrib not in attributes_package:
                self.log_warn("Provided attrib {} ignored".format(attrib))

    def sanity_check_node_interface(self, xml_interface):
        """Check and store a node interface

        Args:
            xml_interface (TYPE): xml spec of the node

        Returns:
            dict: the node interface
        """
        self.log("Checking node interface {}".format(xml_interface.tag))

        interface_node = self.dico_.spec_['node_interface'].keys()

        assert xml_interface.tag in interface_node, "Unknown interface {}".format(xml_interface.tag)

        interface_spec = dict()
        interface_spec["type"] = xml_interface.tag
        interface_spec["attributes"] = dict()

        attributes = self.dico_.spec_['node_interface'][xml_interface.tag]

        for attrib in attributes:
            # print "Checking for attributes {}".format(attrib)
            assert attrib in xml_interface.attrib.keys(), 'Missing required attribute {} for {} interface. Check line {}'.format(attrib, xml_interface.tag, xml_interface.attrib)
            interface_spec["attributes"][attrib] = xml_interface.attrib[attrib]

        # self.log("Requested interface for {} correct".format(xml_interface.tag))

        for attrib in xml_interface.attrib.keys():
            if attrib not in attributes:
                self.log_warn("Provided attrib {} of interface {} ignored (check {})".format(attrib, xml_interface.tag, xml_interface.attrib))
        return interface_spec

    def sanity_check_node(self, xml_node):
        # todo check as well component names

        loc_data_node = dict()
        loc_data_node['attributes'] = dict()

        attributes_node = self.dico_.spec_['node_attributes']

        for attrib in attributes_node:
            assert attrib in xml_node.attrib.keys(), "Missing required attribute {} for node description".format(attrib)
            loc_data_node['attributes'][attrib] = xml_node.attrib[attrib]

        # self.log("Requested node description correct")

        for attrib in xml_node.attrib.keys():
            if attrib not in attributes_node:
                self.log_warn("Provided attrib {} ignored".format(attrib))

        interface_node = self.dico_.spec_['node_interface'].keys()

        self.log("Check: node interface is: {}".format(interface_node))

        loc_data_node['interface'] = dict()
        for item in interface_node:
            loc_data_node['interface'][item] = list()

        for child in xml_node:
            # self.log("Checking for {}".format(child))
            child_interface = self.sanity_check_node_interface(child)
            # self.log("Adding entry for type {}".format(child_interface["type"]))
            # self.log("Within: {}".format(loc_data_node['interface']))

            loc_data_node['interface'][child_interface["type"]].append(child_interface["attributes"])

        return loc_data_node

    def sanity_check_dependency(self, xml_dep):
        """Summary

        Args:
            xml_dep (TYPE): Description
        """
        assert xml_dep.text, "Missing dependency text"
        self.data_depend_.append(xml_dep.text)

    def sanity_check_child(self, xml_item):
        tag = xml_item.tag
        if tag == "node":
            self.data_node_.append(self.sanity_check_node(xml_item))
        elif tag == "depend":
            self.sanity_check_dependency(xml_item)
        else:
            self.log_error("Unknown tag {}".format(tag))

    def sanity_check(self, verbose=False):
        """
        checking the sanity of the xml file
        @param verbose whether additional info is being displayed
        """
        if verbose:
            self.log("XML sanity check")

        if not self.root_:
            return False

        self.sanity_check_package_attribute()

        for child in self.root_:
            self.sanity_check_child(child)

        # todo should not always return true!
        return True

    def print_xml_parsed(self):
        self.log("**************")
        self.log("XML parsed: ")
        self.log("**************")
        self.log("{}".format(self.data_pack_))
        self.log("**************")
        self.log("{}".format(self.data_depend_))
        self.log("**************")
        self.log("{}".format(self.data_node_))
        return True

    def get_nodes_spec(self):
        return self.data_node_

    def get_package_spec(self):
        return self.data_pack_

    def get_dependency_spec(self):
        return self.data_depend_

    def get_number_nodes(self):
        """Returns the number of nodes being defined

        Returns:
            int: number of nodes
        """
        return len(self.data_node_)

    def set_active_node(self, node_id):
        """set the active node be handled

        Args:
            node_id (int): active node number

        Returns:
            Bool: True if the operation succeeded
        """

        if node_id < 0:
            self.log_error("node id ({}) should be >= 0".format(node_id))
            return False
        if not self.data_node_:
            self.log_error("No specification read so far")
            return False
        if node_id >= len(self.data_node_):
            msg = "node id ({}) should be < {}".format(node_id,
                                                       len(self.data_node_))
            self.log_error(msg)
            return False

        self.active_node_ = node_id
        return True

    def get_active_node_spec(self):
        assert self.active_node_ != -1, "No active node defined"
        assert self.active_node_ < len(self.data_node_), "Active node {} should be less then {}".format(self.active_node_, len(self.data_node_))
        return self.data_node_[self.active_node_]

    def write_xml(self, filename):
        """write the xml into a file

        Args:
            filename (str): filename to use

        Returns:
            Bool: True on success
        """
        self.log("Writting xml into file {}".format(filename))

        st = ET.tostring(self.root_, 'utf-8')

        reparsed= minidom.parseString(st)
        res = reparsed.toprettyxml(indent="  ", encoding="utf-8")
        res = remove_empty_line(res)

        # with open("Output.txt", "w") as text_file:
        #     text_file.write(res2)
        with open(filename, 'w') as file_handler:
            for item in res:
                file_handler.write("{}\n".format(item))
        return True

# todo the content of the main should be adapted to be a sanity check instead
def main():
    print "package xml parser trial"
    #rospy.init_node('package_xml_parser', anonymous=True)
    #rospy.loginfo("Package description sanity check")

    package_parser = PackageXMLParser()
    import rospkg
    rospack = rospkg.RosPack()
    node_path = rospack.get_path('package_generator')

    file_dico = node_path + "/sandbox/dico.yaml"
    dico = GenerateDictionnary()

    if not dico.load_yaml_desc(file_dico):
        print "Could not load the dictionnary"
        return

    package_parser.set_dictionnary(dico)

    filename = node_path + '/tests/extended.ros_package'
    rospy.loginfo("Loading xml file {}".format(filename))
    if package_parser.load(filename):
        print "File loaded with success"
        print "Rewritting the file"
        if not package_parser.write_xml("debug_ros_xml.xml"):
            print "could not write the xml file"
    else:
        print "Prb while loading the xml file"

    print "Bye bye"
