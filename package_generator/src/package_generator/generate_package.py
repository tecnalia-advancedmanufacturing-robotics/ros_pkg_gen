#!/usr/bin/env python
"""
@package package_generator
@file package_generator.py
@author Anthony Remazeilles
@brief given a template ros package structure,
 generates the package according to the xml definition

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

from termcolor import colored
import os
import datetime
import shutil
import sys
import rospkg

from package_generator.code_generator import CodeGenerator
from package_generator.package_xml_parser import PackageXMLParser
from package_generator.file_update_management import GeneratedFileAnalysis
from package_generator.enhanced_object import EnhancedObject
from package_generator.template_spec import TemplateSpec


class PackageGenerator(EnhancedObject):
    """Handle the genration of a whole package

    Attributes:
        file_generator_ (TYPE): Description
        package_path_ (TYPE): Description
        path_pkg_backup_ (TYPE): Description
        spec_ (TemplateSpec): configuration of the template model
        template_path_ (TYPE): Description
        xml_parser_ (TYPE): Description
    """

    def __init__(self, name="PackageGenerator"):
        """ Intialisation of the object

        Args:
            name (str, optional): Name of the component, for printing aspect
        """
        #  call super class constructor
        super(PackageGenerator, self).__init__(name)

        # path to the template to use
        self.template_path_ = None
        # base location of the package to create
        self.package_path_ = None
        # parser of the package description
        self.xml_parser_ = None
        # config parameter provide with the template
        self.spec_ = None
        # generic file generator
        self.file_generator_ = None
        # if the package already existed, location of the package backup
        self.path_pkg_backup_ = None

    def set_package_template(self, template_path):
        """set the package template

        Args:
            template_path (str): path to the package template

        Returns:
            Bool: True if basic sanity checks are done with success
        """
        if not os.path.exists(template_path):
            msg = "Template path ({}) is incorrect ".format(template_path)
            self.log_error(msg)
            return False

        if not os.path.isdir(template_path):
            msg = "Template path ({}) is not a directory ".format(template_path)
            self.log_error(msg)
            return False

        # check if minimum information is present.

        details = """A template should contain:
    * config/dictionary.yaml : the dictionary to be used
    * [optional] config/functions.py : additional functions used in the generation
    * template/* set of elements to be generated
Revise the template, and compare to examples
        """

        is_ok = True
        # check for directories
        required_folders = ["config", "template"]
        for item in required_folders:
            req_folder = template_path + "/" + item
            if not os.path.isdir(req_folder):
                msg_err = "Error \n Expecting to have folder " + item
                msg_err += " in " + template_path
                self.log_error(msg_err)
                is_ok = False

        # check for files
        required_files = ["config/dictionary.yaml"]
        for item in required_files:
            req_file = template_path + "/" + item
            if not os.path.isfile(req_file):
                msg_err = "Error.\n Expecting to have file " + item
                msg_err += " in " + template_path
                self.log_error(msg_err)
                is_ok = False

        if not is_ok:
            self.log_error("\n{}".format(details))
            return False

        self.template_path_ = template_path
        return True

    def generate_package(self, package_desc, output_path):
        """launches the package generation

        Args:
            package_desc (str): xml file containing the package description
            output_path (str): directory into which the package is created

        Returns:
            Bool: True if the operation succeeded
        """
        self.path_pkg_backup_ = None

        if not os.path.exists(output_path):
            msg_err = "Expected package path ({}) is incorrect ".format(output_path)
            self.log_error(msg_err)
            return False

        if not os.path.isdir(output_path):
            msg_err = "Expected package path ({}) not a directory ".format(output_path)
            self.log_error(msg_err)
            return False

        self.spec_ = TemplateSpec()
        self.xml_parser_ = PackageXMLParser()
        self.file_generator_ = CodeGenerator()

        dir_template_spec = self.template_path_ + "/config/"
        if not self.spec_.load_spec(dir_template_spec):
            self.log_error("Could not load the template spec")
            return False

        if not self.xml_parser_.set_template_spec(self.spec_):
            msg_err = "Package spec not compatible with xml parser expectations"
            self.log_error(msg_err)
            return False

        if not self.xml_parser_.load(package_desc):
            msg_err = "Prb while parsing xml file {}".format(package_desc)
            self.log_error(msg_err)
            return False

        # self.xml_parser_.print_xml_parsed()

        if not self.file_generator_.configure(self.xml_parser_, self.spec_):
            return False

        package_name = self.xml_parser_.get_package_spec()["name"]

        self.package_path_ = output_path + "/" + package_name

        if os.path.exists(self.package_path_):
            self.log_warn("Package {} already exists".format(self.package_path_))
            # moving preexisting code.
            # generating dir name using date
            now = datetime.datetime.now()
            str_date = now.strftime("%Y_%m_%d_%H_%M_%S")
            self.path_pkg_backup_ = "/tmp/{}_{}".format(os.path.basename(self.package_path_), str_date)
            self.log_warn("Original package temporally stored in {}".format(self.path_pkg_backup_))
            # TODO check if the move succeeded
            shutil.move(self.package_path_, self.path_pkg_backup_)
        else:
            self.log("Package to be created in {}".format(self.package_path_))
        os.makedirs(self.package_path_)

        package_content = os.listdir(self.template_path_ + "/template")
        nb_node = self.xml_parser_.get_number_nodes()
        self.log("Number of nodes defined: {}".format(nb_node))

        # launching the file generation, except common files
        self.log("Generating files specific to nodes")
        is_ok = True
        for id_node in range(nb_node):
            # self.log_error("Handling node {}".format(id_node))
            if not self.xml_parser_.set_active_node(id_node):
                is_ok = False
                break
            node_name = self.xml_parser_.data_node_[id_node]["attributes"]["name"]
            self.log_warn("Handling files for node {}".format(node_name))

            # reconfiguring the generator to adjust to the new active node
            if not self.file_generator_.configure(self.xml_parser_, self.spec_):
                is_ok = False
                break

            for item in package_content:
                file_generic = self.template_path_ + '/template/' + item
                is_ok = self.generate_content(file_generic, False)
                if not is_ok:
                    break
        if not is_ok:
            return False

        self.log("Generating files common to all nodes")
        is_ok = True
        # todo should we do something with the active_node file?
        for item in package_content:
            file_generic = self.template_path_ + '/template/' + item
            is_ok = self.generate_content(file_generic, True)
            if not is_ok:
                break

        if not is_ok:
            return False

        # we store the model into the directory model
        path = self.package_path_ + "/model"
        if not os.path.exists(path):
            os.makedirs(path)
        path += "/" + package_name + ".ros_package"

        if self.xml_parser_.is_dependency_complete_:
            try:
                if os.path.abspath(package_desc) == os.path.abspath(path):
                    # self.log_warn("Using generated model...")
                    stored_desc = self.path_pkg_backup_ + "/model/" + package_name + ".ros_package"
                    # self.log("check {} is absolute: {}".format(package_desc, os.path.abspath(package_desc)))
                    shutil.copyfile(stored_desc, path)
                else:
                    shutil.copyfile(package_desc, path)
                self.log("Package model saved in: {}".format(path))
            except IOError as error:
                self.log_error("Could not store model file: {}".format(error))
        else:
            # some dependencies were automatically added
            # model needs to be rewritten
            try:
                self.xml_parser_.write_xml(path)
                self.log("Package model updated & saved in: {}".format(path))
            except IOError as error:
                self.log_error("Could not store model file: {}".format(error))

        is_ok = self.handle_maintained_files()

        return is_ok

    def handle_maintained_files(self):
        """ Restore file Developer requests to maintain
        Assuming these patterns are defined in file .gen_maintain
        Returns:
            Bool: True on sucess
        """
        # check for files to be maintained
        if self.path_pkg_backup_ is None:
            # package just created, no maintained file
            return True

        filename_rel = ".gen_maintain"
        filename_abs = self.path_pkg_backup_ + "/" + filename_rel
        if os.path.exists(filename_abs) and os.path.isfile(filename_abs):
            self.log("Checking content to maintain after update")
        else:
            self.log("no maintained file defined in previous package version")
            return True

        with open(filename_abs) as open_file:
            for line in open_file:
                line = line.rstrip('\n')
                if not line:
                    continue
                path_abs = self.path_pkg_backup_ + "/" + line

                if not os.path.exists(path_abs):
                    msg = "Content {} not found. Revise {} content"
                    self.log_error(msg.format(line, filename_abs))
                    continue
                new_path = self.package_path_ + "/" + line

                if os.path.isfile(path_abs):
                    try:
                        self.log("Restoring file {}".format(line))
                        # check if directories needs to be created
                        dirname = os.path.dirname(line)
                        # self.log("dirname is : {}".format(dirname))
                        if dirname:
                            path_abs_dir = self.package_path_ + "/" + dirname
                            if not os.path.isdir(path_abs_dir):
                                os.makedirs(path_abs_dir)

                        shutil.copyfile(path_abs, new_path)
                    except IOError as error:
                        msg = "Could not restore a file: {}"
                        self.log_error(msg.format(error))
                    continue

                if os.path.isdir(path_abs):
                    try:
                        self.log("Restoring folder {}".format(line))
                        shutil.copytree(path_abs, new_path)
                    except IOError as error:
                        msg = "Could not restore folder: {}"
                        self.log_error(msg.format(error))
                    continue
                self.log_error("Unkown statement {}".format(line))

        # restoring the maintained content file
        try:
            self.log("Restoring file {}".format(filename_rel))
            new_path = self.package_path_ + "/" + filename_rel
            shutil.copyfile(filename_abs, new_path)
        except IOError as error:
            msg = "Could not restore file: {}"
            self.log_error(msg.format(error))
        return True

    def generate_content(self, input_item, is_generic):
        """recursive function enabling the generation and storage of a file
            or all content of a directory

        Args:
            input_item (str): path to a file or a directory to be generated
            is_generic (Bool): specify if we handle generic files or node specific ones

        Returns:
            Todo: how to handle the file name, when several nodes are defined.
        """
        is_ok = True
        # get the package name
        package_name = self.xml_parser_.get_package_spec()["name"]

        rel_path = os.path.relpath(input_item, self.template_path_ + "/template")
        # check if the name 'package_name is present in the name'
        rel_path = rel_path.replace('package_name', package_name)
        output_item = self.package_path_ + '/' + rel_path

        if os.path.isdir(input_item):
            # self.log("Handling folder {}".format(rel_path))
            if os.path.exists(output_item):
                # self.log_warn("Directory {} already exists".format(output_item))
                pass
            else:
                # self.log("Creating directory {}".format(rel_path))
                os.makedirs(output_item)

            # Generation loop
            for item in os.listdir(input_item):
                is_ok = self.generate_content(input_item + '/' + item, is_generic)
                if not is_ok:
                    break
            return is_ok

        # file to be generated.
        # we check if "node" is in the filename (ie not a generic file).
        basename = os.path.basename(rel_path)

        if 'node' in basename:
            # this is a node specific file. Flag is_generic defines our strategy
            if is_generic:
                # nothing to be done, we just skip it
                return True
        else:
            # this is a generic file, common to all nodes
            if not is_generic:
                # we do nothing
                return True

        self.log("Handling template file {}".format(rel_path))
        # The only difference between generic and no generic is that
        # in the generic case we have to instanciate the node name
        if 'node' in basename:
            # This is a node specific file. We adjust the filename
            basename = basename.replace('node', self.xml_parser_.get_active_node_spec()["attributes"]["name"])
            # the rest is common

        rel_path = os.path.join(os.path.dirname(rel_path), basename)
        output_item = os.path.join(os.path.dirname(output_item), basename)

        # Normally an empty file should not be written
        # The exception is currently only for the special python file __init__.py
        is_write_forced = (os.path.basename(output_item) == '__init__.py')

        if self.path_pkg_backup_ is None:
            self.log("Generating file {} in {}".format(rel_path, output_item))
            is_ok = self.file_generator_.generate_file(input_item, output_item,
                                                       force_write=is_write_forced)

            return self.handle_status_and_advise(input_item, output_item, is_ok)

        # A previous version of the package exists
        # Checking if an update is necessary
        previous_filename = os.path.join(self.path_pkg_backup_, rel_path)

        # Check 1: does this file exist?
        if not os.path.isfile(previous_filename):
            msg = "File {} not previously existing. Just write it"
            self.log_warn(msg.format(rel_path))

            is_ok = self.file_generator_.generate_file(input_item,
                                                       output_item,
                                                       is_write_forced)
            return self.handle_status_and_advise(input_item,
                                                 output_item,
                                                 is_ok)

        # File already existing. Processing previous version
        is_update_needed = False
        file_analyzor = GeneratedFileAnalysis()
        is_ok = file_analyzor.extract_protected_region(previous_filename)
        if is_ok:
            # Check if Developer inserted any contribution
            if file_analyzor.extracted_areas_:
                # contribution found, merge needed
                is_update_needed = True
            else:
                self.log("No Developer contribution found")
        else:
            msg = "prb while extracting protected area in {}"
            self.log_error(msg.format(previous_filename))
            self.log_error("Previous file to be manually merged, sorry")

        # now we know if an update is needed
        if is_ok and is_update_needed:
            # self.log("Updating file {} in {}".format(rel_path, output_item))
            self.log("Updating file {}".format(rel_path))

            is_ok = self.file_generator_.generate_file(input_item)
            if is_ok:
                if self.file_generator_.get_len_gen_file() == 0:
                    msg = "New generated file empty. No code maintained from previous version"
                    self.log_warn(msg)
                    # we write it if forced
                    if is_write_forced:
                        is_ok = self.file_generator_.write_output_file(output_item)
                else:
                    self.log("Merging with previous version")
                    self.file_generator_.buffer_ = file_analyzor.update_file(self.file_generator_.buffer_)
                    is_ok = self.file_generator_.write_output_file(output_item)
                return self.handle_status_and_advise(input_item, output_item, is_ok)

        # Although the file existed before, we do not have to maintain it
        is_ok = self.file_generator_.generate_file(input_item, output_item, is_write_forced)
        return self.handle_status_and_advise(input_item, output_item, is_ok)

    def handle_status_and_advise(self, input_file, output_file, gen_flag):
        """Depending on the file generation process outcome,
           Adjust file status and inform user

        Args:
            input_file (str): path  of the template file used
            output_file (TYPE): path of the generated file
            gen_flag (Bool): Success of the generation process

        Returns:
            Bool: True on success of the file generation
        """
        if not gen_flag:
            msg = "Prb while generating file {}".format(output_file)
            self.log_error(msg)
            return False
        # so the file generation went well
        if self.file_generator_.get_len_gen_file() == 0:
            # Only file __init__.py is kept empty
            if os.path.basename(output_file) != '__init__.py':
                msg = "File {} not written since empty".format(output_file)
                self.log_warn(msg)
                self.log_warn("Check: {}".format(os.path.basename(output_file)))
                return True
        # file has content
        file_status = os.stat(input_file)
        os.chmod(output_file, file_status.st_mode)
        # self.log("File {} handled".format(input_file))
        self.log("File handled")
        self.log("*********************************")
        return True

    def template_sanity_check(self):
        """ Perform the package sanity check
        """

        # TODO list number of files in template
        # Extracting all components from the template
        file_list = list()
        dir_list = list()

        path_root_template = self.template_path_ + "/template"

        for (root, dirs, files) in os.walk(path_root_template):
            # print "check {}: dir {}, files: {}".format(root, dirs, files)

            if os.path.samefile(root, path_root_template):
                for item in files:
                    file_list.append(item)
                for item in dirs:
                    dir_list.append(item)
            else:
                rel_path = os.path.relpath(root, path_root_template)
                for item in files:
                    file_list.append(rel_path + "/" + item)
                for item in dirs:
                    dir_list.append(rel_path + "/" + item)

        # print ("Dirs: ")
        # print("\n".join(dir_list))

        # print("Files: ")
        # print("\n".join(file_list))

        # setting the needed component.
        self.spec_ = TemplateSpec()
        self.xml_parser_ = PackageXMLParser()
        self.file_generator_ = CodeGenerator()

        dir_template_spec = self.template_path_ + "/config/"
        if not self.spec_.load_spec(dir_template_spec):
            self.log_error("Could not load the template spec")
            return False

        if not self.xml_parser_.set_template_spec(self.spec_):
            msg_err = "Package spec not compatible with xml parser expectations"
            self.log_error(msg_err)
            return False

        if not self.xml_parser_.set_empty_spec():
            msg_err = "Failed generating empty spec"
            self.log_error(msg_err)
            return False

        if not self.file_generator_.configure(self.xml_parser_, self.spec_):
            return False

        is_ok = True

        for item in file_list:
            self.log("Checking file: {}".format(item))
            item_abs = path_root_template + "/" + item
            is_ok = self.file_generator_.check_template_file(item_abs)

        if is_ok:
            self.log("No error detected")
        else:
            self.log_error("Revise the template")
        return is_ok


USAGE = """ usage: generate_package package_spec package_template
package_spec: xml description of the node(s) interface
package_template: name of the template to use

Packages template: either one defined in package `package_generator_templates`,
                   either a path to a local one.
"""


def main():
    """
    @brief Entry point of the package.
    Generates a package, given a specified structure
    @return nothing

    Returns:
        int: negative value on error
    """

    # checking available templates
    rospack = rospkg.RosPack()
    try:
        node_path = rospack.get_path('package_generator_templates')
        default_templates_path = node_path + "/templates/"
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

    package_spec = sys.argv[1]
    path_template = sys.argv[2]

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
            print "Loading template from path {}".format(path_template)
        else:
            if path_template in available_templates:
                path_template = default_templates_path + path_template
                msg = "Loading template from template package: {}"
                print msg.format(path_template)
            else:
                msg = "Template name not found in package_generator_templates"
                print colored(msg, "red")
                print colored("Please verify your setup", "red")
                return -1

    gen = PackageGenerator()

    if not gen.set_package_template(path_template):
        print colored("Not able to load the template at:", "red")
        print colored(path_template, "red")
        print "Bye"
        return -1

    if not gen.generate_package(package_spec, path_current):
        print colored("Prb while generating the package", "red")
        return -1
    else:
        print colored("Package generated", "green")
    print "Bye bye"
