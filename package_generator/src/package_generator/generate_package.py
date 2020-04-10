#!/usr/bin/env python
"""
@package package_generator
@file package_generator.py
@author Anthony Remazeilles
@brief given a template ros package structure,
 generates the package according to the xml definition

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.

"""

import os
import datetime
import shutil
import sys
import rospkg

from package_generator.code_generator import CodeGenerator
from package_generator.jinja_generator import JinjaGenerator
from package_generator.package_xml_parser import PackageXMLParser
from package_generator.file_update_management import GeneratedFileAnalysis
from package_generator.enhanced_object import EnhancedObject
from package_generator.template_spec import TemplateSpec
from termcolor import colored


class PackageGenerator(EnhancedObject):
    """Handle the genration of a whole package

    Attributes:
        file_generator_ (CodeGenerator): custom generator
        jinja_generator_ (JinjaGenerator): generator based on jinja
        package_path_ (str): base location of the package to create
        path_pkg_backup_ (str): if the package already existed, location of the package backup
        spec_ (TemplateSpec): configuration of the template model
        template_path_ (str): path to the template to use
        xml_parser_ (PackageXMLParser): parser of the package description
    """

    def __init__(self, name="PackageGenerator"):
        """Intialisation of the object

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
        # jinja-based generator
        self.jinja_generator_ = None
        # if the package already existed, location of the package backup
        self.path_pkg_backup_ = None

    def check_template_structure(self, template_path):
        """Check a provided path refers to a valid template structure

        Args:
            template_path (str): path to the package template

        Returns:
            Bool: True if basic sanity checks are successful
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
    * config/functions.py [optional] : additional functions used in the generation
    * config/generator.py [optional] : generator list (custom, jinja) default is custom
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

        return True

    def get_template_info(self):
        """Get information about the available package templates

        Returns:
            list: tuple with [absolute package path, list of package names]
        """
        rospack = rospkg.RosPack()
        path_template = rospack.get_path('package_generator_templates')
        path_template += "/templates/"
        template_names = os.listdir(path_template)

        return [path_template, template_names]

    def generate_package(self, package_desc, output_path):
        """launches the package generation

        Args:
            package_desc (str): xml file containing the package description
            output_path (str): directory into which the package is created

        Returns:
            Bool: True if the operation succeeded
        """
        if not os.path.exists(output_path):
            msg_err = "Incorrect desired package path ({})".format(output_path)
            self.log_error(msg_err)
            return False

        if not os.path.isdir(output_path):
            msg_err = "Desired package path ({}) not a directory ".format(output_path)
            self.log_error(msg_err)
            return False

        # Initialising needed components
        # todo bring it to the constructor?
        self.spec_ = TemplateSpec()
        self.xml_parser_ = PackageXMLParser()
        self.file_generator_ = CodeGenerator()
        self.jinja_generator_ = JinjaGenerator()

        # Start finding the template

        template = self.xml_parser_.get_template(package_desc)
        if template is None:
            return False

        # Locate template location
        try:
            [all_template_path, template_names] = self.get_template_info()
        except rospkg.common.ResourceNotFound as error:
            msg = "Package package_generator_templates not found in rospack"
            self.log_error(msg)
            self.log_error(error)
            return False
        except OSError as error:
            msg = "No template dounf in package_generator_templates"
            self.log_error(msg)
            self.log_error(error)
            return False

        if template not in template_names:
            msg = "Template requested: {} unknown".format(template)
            self.log_error(msg)
            msg = "Available templates: {}".format(template_names)
            self.log_error(msg)
            return False

        template_path = all_template_path + "/" + template

        # confirm this is a template...
        if not self.check_template_structure(template_path):
            msg = "Please revise template structure"
            self.log_error(msg)
            return False

        # template localized, ready to work!
        self.template_path_ = template_path
        self.path_pkg_backup_ = None

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

        # todo why only the custom generator is configured?
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

        nb_comp = self.xml_parser_.get_number_comps()
        self.log("Number of components defined: {}".format(nb_comp))

        if not self.generate_content():
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

    def generate_one_file(self, template_file, result_file, force_write):
        """Generate a template file, depending on the generators to be used

        Args:
            template_file (str): template filename
            result_file (str): filename to store the result (unless is None)
            force_write (str): force the writting of empty files (if not, files is not written)

        Returns:
            Bool: True on success
        """

        generator = dict()
        generator["custom"] = self.file_generator_
        generator["jinja"] = self.jinja_generator_

        if len(self.spec_.generators_) == 1:
            return generator[self.spec_.generators_[0]].generate_disk_file(template_file,
                                                                           result_file,
                                                                           force_write)
        # two generators are to be used
        gen_one = generator[self.spec_.generators_[0]]
        gen_two = generator[self.spec_.generators_[1]]

        is_ok = gen_one.generate_disk_file(template_file)

        if not is_ok:
            return False

        return gen_two.generate_open_file(gen_one.rendered_,
                                          result_file,
                                          force_write)

    def check_template_file(self, template_file):
        """Generate a template file, depending on the generators to be used

        Args:
            template_file (str): template filename
            result_file (str): filename to store the result (unless is None)
            force_write (str): force the writting of empty files (if not, files is not written)

        Returns:
            Bool: True on success
        """

        generator = dict()
        generator["custom"] = self.file_generator_
        generator["jinja"] = self.jinja_generator_

        if len(self.spec_.generators_) == 1:
            # self.log("Check with Generator {}".format(self.spec_.generators_[0]))
            return generator[self.spec_.generators_[0]].check_template_file(template_file)

        # two generators are to be used
        gen_one = generator[self.spec_.generators_[0]]
        gen_two = generator[self.spec_.generators_[1]]

        # self.log("Check with Generator {}".format(self.spec_.generators_[0]))

        is_ok = gen_one.check_template_file(template_file)

        if not is_ok:
            return False

        # self.log("Check with Generator {}".format(self.spec_.generators_[1]))

        if self.spec_.generators_[1] == "jinja":
            return gen_two.check_template_file(gen_one.rendered_, is_filename=False)
        return gen_two.check_template_file(template_file)

    def write_generated_file(self, result_file):
        """Write a generated file

        Args:
            result_file (str): filename to store the file.

        Returns:
            Bool: True on success
        """
        generator = dict()
        generator["custom"] = self.file_generator_
        generator["jinja"] = self.jinja_generator_

        return generator[self.spec_.generators_[-1]].write_rendered_file(result_file)

    def get_generated_file(self):
        """Get the generated files

        Returns:
            list: list of of each line of the generated file
        """
        generator = dict()
        generator["custom"] = self.file_generator_
        generator["jinja"] = self.jinja_generator_

        return generator[self.spec_.generators_[-1]].rendered_

    def set_generated_file(self, l_file):
        """set the generated file

        Args:
            l_file (list): list of of each line of the generated file
        """
        generator = dict()
        generator["custom"] = self.file_generator_
        generator["jinja"] = self.jinja_generator_

        generator[self.spec_.generators_[-1]].rendered_ = l_file

    def handle_maintained_files(self):
        """Restore file Developer requests to maintain
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

    def handle_status_and_advise(self, input_file, output_file, gen_flag):
        """Depending on the file generation process outcome,
           Adjust file status and inform user

        Args:
            input_file (str): path  of the template file used
            output_file (str): path of the generated file
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

    def generate_content(self):
        """Generation and storage of all content

        Returns:
            Bool -- True on success
        """
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

        # Looking at final directory and filenames
        package_name = self.xml_parser_.get_package_spec()["name"]
        nb_comp = self.xml_parser_.get_number_comps()
        comps_name = [self.xml_parser_.data_comp_[id_comp]["attributes"]["name"] for id_comp in range(nb_comp)]

        self.log("Generating all folders")

        tmp = list()
        for item in dir_list:
            item = item.replace('package_name', package_name)
            if 'component' in item:
                for one_name in comps_name:
                    tmp.append(item.replace('component', one_name))
            else:
                tmp.append(item)
        dir_list = tmp

        for item in dir_list:
            path_folder = self.package_path_ + "/" + item
            if not os.path.exists(path_folder):
                os.makedirs(path_folder)

        generation_list = list()
        # File preparation: storing [template filename, new filename, comp id]
        for item in file_list:

            new_item = item.replace('package_name', package_name)
            if 'component' in item:
                for num, one_name in enumerate(comps_name):
                    generation_list.append([item,
                                            new_item.replace('component',
                                                             one_name),
                                            num])
            else:
                # todo if no component active I should not set one
                generation_list.append([item, new_item, 0])

        is_ok = True
        # self.log("\nFiles generation plan: ")
        for item in generation_list:
            [template_file, result_file, comp_id] = item
            self.log("{} --> {}".format(template_file, result_file))

            if not self.xml_parser_.set_active_comp(comp_id):
                return False

            # reconfiguring the generator to adjust to the new active component
            # todo configure already called in generate_package function. Check why
            if not self.file_generator_.configure(self.xml_parser_, self.spec_):
                return False
            if not self.jinja_generator_.configure(self.xml_parser_, self.spec_):
                return False

            # Normally an empty file should not be written
            # The exception is currently only for the special python file __init__.py
            is_write_forced = (os.path.basename(result_file) == '__init__.py')

            result_file = self.package_path_ + "/" + result_file
            template_file = self.template_path_ + '/template/' + template_file

            if self.path_pkg_backup_ is None:
                self.log("Generating file {}".format(result_file))

                is_ok = self.generate_one_file(template_file,
                                               result_file,
                                               is_write_forced)

                if self.handle_status_and_advise(template_file,
                                                 result_file,
                                                 is_ok):
                    continue
                else:
                    return False

            # A previous version of the package exists
            # Checking if an update is necessary
            rel_path = os.path.relpath(result_file, package_name)
            previous_filename = os.path.join(self.path_pkg_backup_, rel_path)

            # Check 1: does this file exist?
            if not os.path.isfile(previous_filename):
                msg = "File {} not previously existing. Just write it"
                self.log_warn(msg.format(rel_path))

                is_ok = self.generate_one_file(template_file,
                                               result_file,
                                               is_write_forced)
                if self.handle_status_and_advise(template_file,
                                                 result_file,
                                                 is_ok):
                    continue
                else:
                    return False
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

                is_ok = self.generate_one_file(template_file, None, None)
                if not is_ok:
                    return False

                # todo handle this in case jinja is involved.
                l_gen = self.get_generated_file()
                if not l_gen:
                    msg = "New generated file empty. No code maintained from previous version"
                    self.log_warn(msg)
                    # we write it if forced
                    if is_write_forced:
                        is_ok = self.write_generated_file(result_file)
                else:
                    self.log("Merging with previous version")
                    l_gen = file_analyzor.update_file(l_gen)
                    self.set_generated_file(l_gen)
                    is_ok = self.write_generated_file(result_file)

                if self.handle_status_and_advise(template_file,
                                                 result_file,
                                                 is_ok):
                    continue
                else:
                    return False

            # Although the file existed before, we do not have to maintain it
            is_ok = self.generate_one_file(template_file, result_file, is_write_forced)
            if self.handle_status_and_advise(template_file, result_file, is_ok):
                continue
            else:
                return False
        return True

    def template_sanity_check(self, template):
        """Perform the package sanity check

        Returns:
            Bool: True on success
        """

        # Locate template location
        try:
            [all_template_path, template_names] = self.get_template_info()
        except rospkg.common.ResourceNotFound as error:
            msg = "Package package_generator_templates not found in rospack"
            self.log_error(msg)
            self.log_error(error)
            return False
        except OSError as error:
            msg = "No template found in package_generator_templates"
            self.log_error(msg)
            self.log_error(error)
            return False

        is_template_found = False
        template_path = None

        if template in template_names:
            is_template_found = True
            template_path = all_template_path + "/" + template
        else:
            self.log("Could not find template {} in {}".format(template, all_template_path))
            # check if the template provided is a relative path, and not a package in the repo
            if os.path.isabs(template):
                self.log("Loading template from absolute path {}".format(template))
                is_template_found = True
                template_path = template
            else:
                # relative path ?
                template_path = os.getcwd() + "/" + template

                if os.path.isdir(template_path):
                    self.log("Loading template from path {}".format(template_path))
                    is_template_found = True

        if not is_template_found:
            msg = "Template requested: {} unknown".format(template)
            self.log_error(msg)
            msg = "Available templates: {}".format(template_names)
            self.log_error(msg)
            return False

        # confirm this is a template...
        if not self.check_template_structure(template_path):
            msg = "Please revise template structure"
            self.log_error(msg)
            return False

        # TODO list number of files in template
        # Extracting all components from the template
        file_list = list()
        dir_list = list()

        path_root_template = template_path + "/template"

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
        self.jinja_generator_ = JinjaGenerator()

        dir_template_spec = template_path + "/config/"
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

        if not self.jinja_generator_.configure(self.xml_parser_, self.spec_):
            return False

        is_ok = True

        for item in file_list:
            self.log("Checking file: {}".format(item))
            item_abs = path_root_template + "/" + item
            is_ok = self.check_template_file(item_abs)
            if not is_ok:
                break
        if is_ok:
            self.log("No error detected")
        else:
            self.log_error("Revise the template")
        return is_ok


# todo complete the usage description with available templates
# and with existing commands
USAGE_GEN = """ usage: generate_package [package_spec]

package_spec: xml description of the component(s) interface

"""


def main():
    """
    @brief Entry point of the package.
    Generates a package, given a specified structure
    @return nothing

    Returns:
        int: negative value on error
    """

    gen = PackageGenerator()

    if len(sys.argv) != 2:
        print colored("Wrong input parameters !", "red")
        print colored(USAGE_GEN, "yellow")

        try:
            [_, template_names] = gen.get_template_info()
        except rospkg.common.ResourceNotFound as error:
            msg = "Package package_generator_templates not found in rospack"
            print colored(msg, 'red')
            print colored(error, 'red')
            return -1
        except OSError as error:
            msg = "No template found in package_generator_templates"
            print colored(msg, 'red')
            print colored(error, 'red')
            return -1

        msg = "Available templates are: {}"
        print colored(msg.format(template_names), 'yellow')
        print "Bye bye"
        return -1

    package_spec = sys.argv[1]
    path_current = os.getcwd()

    if not gen.generate_package(package_spec, path_current):
        print colored("Prb while generating the package", "red")
        return -1
    else:
        print colored("Package generated", "green")
    print "Bye bye"
    return 0


USAGE_CHECK = """ usage: check_template package_template
package_template: name of the template to check

Packages template: either one defined in package `package_generator_templates`,
                   either a path to a local one.
"""


def main_check():
    """
    @brief Entry point of the package.
    Check a template structure, as provided

    Returns:
        int: negative value on error
    """
    gen = PackageGenerator()

    if len(sys.argv) != 2:
        print colored("Wrong input parameters !", "red")
        print colored(USAGE_CHECK, "yellow")

        try:
            [_, template_names] = gen.get_template_info()
        except rospkg.common.ResourceNotFound as error:
            msg = "Package package_generator_templates not found in rospack"
            print colored(msg, 'red')
            print colored(error, 'red')
            return -1
        except OSError as error:
            msg = "No template found in package_generator_templates"
            print colored(msg, 'red')
            print colored(error, 'red')
            return -1

        msg = "Available templates are: {}"
        print colored(msg.format(template_names), 'yellow')
        print "Bye bye"
        return -1

    template_name = sys.argv[1]
    if not gen.template_sanity_check(template_name):
        print colored("Issue detected in template", "red")
        return -1
    else:
        print colored("No issue detected", "green")
    print "Bye bye"
    return 0
