#!/usr/bin/env python
"""
@package package_generator
@file code_generator.py
@author Anthony Remazeilles
@brief parse a ros template file, and generate the related file

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

import re
from package_generator.package_xml_parser import PackageXMLParser
from package_generator.enhanced_object import EnhancedObject
from package_generator.template_spec import TemplateSpec


def convert(line, delimiter=None, **kwargs):
    """equivalant to format, with provided delimiter
       and without issue with unknown keys

    Args:
        line (str): string to process
        delimiter (list, optional): two char to segment the tags
        **kwargs: dictionary to use

    Returns:
        str: the converted string
    """

    if delimiter is None:
        delimiter = ['{', '}']
    result_line = line

    for key, value in kwargs.iteritems():
        splitted = result_line.split(delimiter[0] + key + delimiter[1])

        acc = splitted[0]

        if len(splitted) > 1:
            for item in splitted[1:]:
                acc += value + item

        result_line = acc

    return result_line


class CodeGenerator(EnhancedObject):
    """class responsible of the generation of a single file

    Attributes:
        dep_spec_ (list): list of dependency of the package
        do_generate_ (bool): False used for template check
        comp_spec_ (list): spec of each component
        package_spec_ (dict): specification of the package
        spec_ (TemplateSpec): specification of the template
        rendered_ (list): will contain the generated file
        transformation_ (dict): mapping instruction tag to generation function
        transformation_functions_ (dict): mapping inst. fun to gen. fun
        transformation_loop_ (dict): mapping instruction flow to gen. functions
        xml_parser_ (PackageXMLParser): Parsed xml Developer specification

    """
    def __init__(self, name="CodeGenerator"):
        #  call super class constructor
        super(CodeGenerator, self).__init__(name)

        self.transformation_ = dict()
        self.transformation_loop_ = dict()
        self.transformation_functions_ = dict()

        self.spec_ = None
        self.rendered_ = list()
        self.xml_parser_ = None
        self.comp_spec_ = None
        self.package_spec_ = None
        self.dep_spec_ = None
        self.do_generate_ = True

    def configure(self, parser, spec):
        """set the required information to configure the generator

        Args:
            parser (PackageXMLParser): XML component description parsed
            spec (TemplateSpec): configuration of the template

        Returns:
            Bool: Operation success
        """
        try:
            self.xml_parser_ = parser
            self.get_xml_parsing()

            self.spec_ = spec

            # generating the tags
            self.generate_simple_tags()
            self.generate_flow_tags()
            self.generate_apply_functions()
            self.rendered_[:] = []
        except AssertionError, err:
            self.log_error("Prb during configuration: {}".format(err))
            return False
        return True

    # TODO empty self.transformation_ before/when entering here
    def generate_simple_tags(self):
        """From the template dictionnary, generate the simple tags expected
        To be used in the template
        all package attribute will have a tag like "packageAttribute"
        all component attributes will have a tag like "componentAttribute"
        """
        package_attributes = self.spec_.dico_['package_attributes']

        for attrib_pack in package_attributes:
            tag = "package" + attrib_pack.title().replace("_", "")
            self.transformation_[tag] = self.get_package_attr(attrib_pack)

        comp_attributes = self.spec_.dico_['component_attributes']

        for attrib_comp in comp_attributes:
            tag = "component" + attrib_comp.title().replace("_", "")
            self.transformation_[tag] = self.get_comp_attr(attrib_comp)

        # self.log_error("Generated tags: \n {}".format(self.transformation_.keys()))

    # TODO empty self.transformation_loop_ before/when entering here
    def generate_flow_tags(self):
        """Generate the conditional and loop tags.
        The definiion is based on the possible interfaces of a given component
        each possible interface will have defined tags like "ifinterface"
        and "forallinterface"
        """
        comp_interface = self.spec_.dico_['component_interface'].keys()

        lambda_for = lambda d: lambda t: self.convert_forall(d, t)
        lambda_if = lambda u: lambda v: self.convert_if(u, v)
        lambda_for_deps = lambda text: self.convert_forall_dependencies(text)
        lambda_for_comps = lambda text: self.convert_forall_comps(text)

        for item in comp_interface:
            # self.log_warn("Adding tag for {}".format(item))
            tag = "forall" + item
            self.transformation_loop_[tag] = lambda_for(item)
            tag = "if" + item
            self.transformation_loop_[tag] = lambda_if(item)

        # TODO check how to make this even generic,
        # we should not assume these names are provided
        self.transformation_loop_['foralldependencies'] = lambda_for_deps
        self.transformation_loop_['forallcomponent'] = lambda_for_comps

    # TODO empty self.transformation_functions_ before/when entering here
    def generate_apply_functions(self):
        """Get the transformation functions defined with the template config
        TODO : is it worth doing so?
        """
        for fun in self.spec_.transformation_functions_:
            self.transformation_functions_[fun] = self.spec_.transformation_functions_[fun]

    def get_xml_parsing(self):
        """ set the xml parser, and extract the relevant input from it
        """
        assert self.xml_parser_ is not None, "No xml data defined"
        self.comp_spec_ = self.xml_parser_.get_active_comp_spec()
        self.package_spec_ = self.xml_parser_.get_package_spec()
        self.dep_spec_ = self.xml_parser_.get_dependency_spec()

        assert self.comp_spec_, "No component specification"
        assert self.package_spec_, "No package specification"
        assert self.dep_spec_ is not None, "No dependency specification"

    def reset_output_file(self):
        """Reset the internal buffer used to accumulate generated code
        """
        self.rendered_ = list()

    def get_len_gen_file(self):
        """Returns the number of lines of the generated file

        Returns:
            Int: number of lines in the file
        """
        return len(self.rendered_)

    def write_rendered_file(self, filename=None):
        """write the generated code

        Args:
            filename (None, optional): file to save to. If None, printed on screen

        Returns:
            Bool: True if the operation succeeded.
        """
        if filename is None:
            self.log("Resulting file:")
            print"----"
            for item in self.rendered_:
                print item
            print"----"
            return True

        try:
            # self.log("Storing {} lines in {}".format(len(self.rendered_), filename))
            out_file = open(filename, 'w')
            for item in self.rendered_:
                # print "printing {}".format(item)
                out_file.write(item + '\n')
            out_file.close()

        except IOError:
            self.log_error("Prb while opening output file {}".format(filename))
            return False
        return True

    def process_file(self, template_filename):
        """
        process a file with tags

        Args:
            template_filename (str): pathfile

        Returns:
            Bools: True on sucess
        """
        # self.log("Generating file {}".format(template_filename))

        if self.xml_parser_ is None:
            self.log_error("XML parser not defined")
            return False
        if self.spec_ is None:
            self.log_error("Template spec missing")
            return False

        lines_in_file = list()
        try:
            with open(template_filename) as input_file:
                for line in input_file:
                    lines_in_file.append(line.rstrip('\n'))
        except IOError:
            self.log_error("Prb while opening file {}".format(template_filename))
            return False
        # self.log("File to process has {} lines".format(len(lines_in_file)))

        # generate an iterator on the enumerated content of the lines list
        iter_enum_lines = iter(enumerate(lines_in_file, start=1))
        return self.process_input(iter_enum_lines)

    def generate_disk_file(self, file_template, output_file=None, force_write=False):
        """generate a file and store it where specified

        Args:
            file_template (str): template to follow
            output_file (str): where to store the generated code
            force_write (bool, optional): if set, forces the file writting even if empty

        Returns:
            Bool: True on success
        """
        self.reset_output_file()
        self.do_generate_ = True
        if not self.process_file(file_template):
            return False

        if output_file is None:
            return True

        nb_line = self.get_len_gen_file()

        # We decide not writting a file if it is empty, unless forced to do so
        if force_write or nb_line > 0:
            return self.write_rendered_file(output_file)
        return True

    def generate_open_file(self, file_template, output_file=None, force_write=False):
        """generate a file and store it where specified

        Args:
            file_template (list): template file as a list of string (per lines)
            output_file (str): where to store the generated code
            force_write (bool, optional): if set, forces the file writting even if empty

        Returns:
            Bool: True on success
        """
        self.reset_output_file()
        self.do_generate_ = True

        # generate an iterator on the enumerated content of the lines list
        iter_enum_lines = iter(enumerate(file_template, start=1))
        if not self.process_input(iter_enum_lines):
            return False

        if output_file is None:
            return True

        nb_line = self.get_len_gen_file()

        # self.log_error("File length: {}".format(nb_line))
        # We decide not writting a file if it is empty, unless forced to do so
        if force_write or nb_line > 0:
            return self.write_rendered_file(output_file)
        return True

    def check_template_file(self, file_template):
        """Check a template file (without generating it)

        Arguments:
            file_template {string} -- template file name

        Returns:
            [Bool] -- True if the file is correct
        """
        self.reset_output_file()

        self.do_generate_ = False
        if not self.process_file(file_template):
            return False
        return True

    def get_all_tags(self, line):
        """Find all tags in a given line

        Arguments:
            line {string} -- Line to process

        Returns:
            [type] -- list of matches found
        """
        # TODO check the return format
        matches = [[m.group(0)[1:-1], m.start()] for m in re.finditer(r'{\w+}', line)]

        # print "Found matches: {}".format(matches)
        return matches

    def get_all_tags_pattern(self, root_pattern, line):
        # TODO are these 2 functions needed?
        # self.log("Processing line {}".format(line))
        instances = re.finditer(r'\{' + root_pattern + r'-\w+}', line)
        matches = [[m.group(0)[1:-1], m.start()] for m in instances]

        # print "Found matches: {}".format(matches)
        return matches

    def process_input(self, iter_enum_lines):
        """Summary

        Args:
            iter_enum_lines (iterator): set of lines to process

        Returns:
            Boolean: operation suceess

        Raises:
            SyntaxError: when an item can not be corectly translated
        """
        try:
            is_ok = True
            while is_ok:
                num_line, line = iter_enum_lines.next()
                # self.log("Processing line [{}]: {}".format(num_line, line))

                # look for apply generator
                # TODO presense of loop / conditional tag not handled.
                matches = self.get_all_tags_pattern("apply", line)
                for m in matches:
                    operation = m[0].split("-")[1]
                    # self.log("operation is: {}".format(operation))
                    # make sure the operation exist
                    if operation not in self.transformation_functions_:
                        self.log_warn("Operator {} unknown and thus discarded".format(operation))
                        continue
                    try:
                        if self.do_generate_:
                            res = self.transformation_functions_[operation](self.transformation_)
                        else:
                            res = ""
                    except Exception as err:
                        self.log_error("Exception detected in processing apply on line {} \n {}".format(line, err))
                        raise
                    # self.log_warn("result would be {}".format(res))
                    # self.log_warn("Here we are")
                    line = convert(line, **{m[0]: res})

                # look for the other tags
                matches = self.get_all_tags(line)
                if not matches:
                    self.rendered_.append(line)
                    num_line += 1
                    continue

                # self.log("the line has a tag")
                # check for a loop tag
                loop_tag_found = False
                tags = [tag for tag, _ in matches]

                for item in self.transformation_loop_:
                    loop_tag_found = loop_tag_found or item in tags
                    end_item = 'end' + item
                    loop_tag_found = loop_tag_found or end_item in tags

                if len(matches) > 1:
                    # multiple matches.
                    # we make simple checks according to authorized operations
                    assert not loop_tag_found, "Multiple tag with loop found. Not yet implemented"

                # todo(Anthony) if the tag is the same, we can avoid two calls

                if not loop_tag_found:
                    for tag, indent in matches:
                        # self.log("found tag |{}| at line {}:col {}".format(tag, num_line, indent))

                        if tag in self.transformation_:
                            replacement = self.transformation_[tag]
                            line = convert(line, **{tag: replacement})
                        else:
                            # todo here we could publish the line
                            self.log_warn("tag {} on line {} not processed".format(tag, num_line))
                    # check if the line is empty
                    # that would be due to a if tag that is not defined.
                    if line and (not line.isspace()):
                        self.rendered_.append(line)
                    # else:
                    #    print colored("Line empty: |{}|".format(line), "blue")

                    # TODO why do I increase num_line here? Isn t it done automatically?
                    num_line += 1
                    continue

                if loop_tag_found:
                    tag, indent = matches[0]
                    # self.log("tag {} in transformation_loop".format(tag))

                    # looking for the end of the loop
                    search_tag = "end" + tag
                    # print "searching for tag {}".format(search_tag)
                    tag_found = False

                    # todo check how to handle it on the same line
                    accumulated_lines = list()

                    try:
                        while not tag_found:
                            # todo: avoid using an accumulated line here.
                            _, sub_line = iter_enum_lines.next()
                            # self.log("Sub Checking line: {}".format(sub_line))

                            sub_matches = self.get_all_tags(sub_line)
                            subtags = [tagg for tagg, _ in sub_matches]

                            if search_tag in subtags:
                                tag_found = True
                                break
                            accumulated_lines.append(sub_line)
                            continue

                    except StopIteration, e:
                        error_msg = "missing closing tag {} openned on line {}".format(search_tag, num_line)
                        raise SyntaxError('Syntax ERROR',
                                          {'filename': 'unknown',
                                           'lineno': "{}".format(num_line),
                                           'offset': "{}".format(indent),
                                           'text': error_msg})

                    # tag found. We know have a bunch of line to process
                    iter_acc_lines = iter(enumerate(accumulated_lines,
                                                    start=num_line + 1))
                    is_ok = self.transformation_loop_[tag](iter_acc_lines)
                    continue
                raise SyntaxError('SyntaxError',
                                  {'filename': 'unknown',
                                   'lineno': "{}".format(num_line),
                                   'offset': "{}".format(indent),
                                   'text': "unknown tag {}".format(tag)})
        except AssertionError, err:
            self.log_error("Assertion Error on line {}: {}".format(num_line,
                                                                   err.args))
            return False
        except SyntaxError, err:
            self.log_error("Syntax Error on line {}: {}".format(num_line,
                                                                err.args))
            return False
        except StopIteration, err:
            # self.log("All file has been processed")
            return True
        except Exception, err:
            self.log_error("Error detected around line {}: {}".format(num_line,
                                                                      err.args))
            return False
        return False

    def get_package_attr(self, attr):
        """Return the value of a package attribute

        Args:
            attr (str): the attribute we are looking for

        Returns:
            str: The value associated to that attribute
        """
        return self.package_spec_[attr]

    def get_comp_attr(self, attr):
        """Retun the value of a component attribute

        Args:
            attr (str): the attribute we are looking for

        Returns:
            str: the associated value
        """
        return self.comp_spec_["attributes"][attr]

    # TODO remove that function that is not used anymore
    # function kept as exmaple if externalizing makes sense
    def get_include_interface(self):
        output = None

        include_set = set()

        for item in self.comp_spec_["interface"]["publisher"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.comp_spec_["interface"]["directPublisher"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.comp_spec_["interface"]["directSubscriber"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.comp_spec_["interface"]["subscriber"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.comp_spec_["interface"]["actionClient"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}Action.h>".format(str_path))

        for item in self.comp_spec_["interface"]["actionServer"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}Action.h>".format(str_path))

        for item in self.comp_spec_["interface"]["serviceServer"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        # todo this may be added to the ros core even though it does not use it.
        for item in self.comp_spec_["interface"]["serviceClient"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in include_set:
            if output is None:
                output = item
            else:
                output += '\n' + item

        if output is None:
            return ""
        else:
            return output

    # TODO error to be checked
    def convert_forall_dependencies(self, iter_text):
        """convert a code looping on each dependency defined.

        Args:
            iter_text (Iterator): listing to process

        Returns:
            Bool: operation success
        """
        output = list()
        listing = list(iter_text)
        # self.log("Handling text: \n {}".format(listing))
        for item in self.dep_spec_:
            dep_dict = {'dependencyName': item}
            # self.log("Handling dependency {}".format(item))
            for _, line in listing:
                # self.log("Handling line {}".format(line))
                line_processed = convert(line, **dep_dict)

                output.append(line_processed)

        self.rendered_ += output

        return True

    # TODO error should be checked
    def convert_forall(self, interface_type, text_it):
        """Convert a code for all instances of an interface type

        Args:
            interface_type (str): name of the interface we are interested in
            text_it (iter): iterator on [num_line, line] to be processed

        Returns:
            Bool: Operation success
        """
        output = list()

        # self.log("Handling text: \n {} with interface {}".format(text, interface_type))
        assert interface_type in self.comp_spec_["interface"], "Requested interface type {} unknown".format(interface_type)

        # computing all upperlayer spec
        # TODO this could be avoided and done once
        # We could only do this when item exists in any case
        context_upper = dict()
        for key in self.transformation_:
            context_upper[key] = self.transformation_[key]

        # listing stands for num_line, text
        listing = list(text_it)

        for item in self.comp_spec_["interface"][interface_type]:
            # self.log("Handling {} {}".format(interface_type, item))

            context_extended = item.copy()
            context_extended.update(context_upper)
            # self.log_warn("Extended item {}".format(context_extended))
            for num_line, line in listing:
                # self.log("Handling line {}".format(line))
                # TODO: there is a risk of inserting empty lines
                # maybe not, since imbricated cases are not considered yet
                # we can not have a for_all  if endif end_for_all
                line_processed = convert(line, **context_extended)

                # check for apply operator
                # TODO remove the loop on apply, that is useless
                matches = self.get_all_tags_pattern("apply", line_processed)
                if matches:
                    # self.log_warn("Found matches: {}".format(matches))
                    for m in matches:
                        operation = m[0].split("-")[1]
                        # self.log("operation is: {}".format(operation))
                        # make sure the operation exist
                        if operation not in self.transformation_functions_:
                            self.log_warn(" [{}] Operator {} unknown and thus discarded".format(num_line, operation))
                            continue
                        try:
                            if self.do_generate_:
                                res = self.transformation_functions_[operation](context_extended)
                            else:
                                res = ""
                        except Exception as e:
                            self.log_error("Exception detected in processing apply on line [{}]: {} \n {} \n with item {}".format(num_line, line, e, item))
                            raise
                        # self.log_warn("result would be {}".format(res))
                        # self.log_warn("Here we are")
                        line_processed = convert(line_processed, **{m[0]: res})

                output.append(line_processed)

        self.rendered_ += output
        # self.log_error("Sanity check: Is dictinnary extended?\n {}".format(self.comp_spec_["interface"][interface_type]))
        # self.log("created: {}".format(type(output)))
        # todo no false case?
        return True

    def convert_if(self, interface_type, it_text):
        """convert a code only if the given interface is beeing used

        Args:
            interface_type (str): interface name
            it_text (ITerator): listing to process if the interface is used.

        Returns:
            TYPE: Description
        """
        # self.log("Handling text: \n {}".format(text))
        if isinstance(interface_type, str):
            list_type = [interface_type]
        else:
            list_type = interface_type

        for item in list_type:
            assert item in self.comp_spec_["interface"], "Requested interface type {} unknown".format(item)

        is_one_defined = False

        for item in list_type:
            is_one_defined = is_one_defined or self.comp_spec_["interface"][item]

        if not is_one_defined:
            return True

        # if defined, we recall the generator...
        is_ok = self.process_input(it_text)

        return is_ok

    def convert_forall_comps(self, it_text):
        """Convert a code, looping on each component defined

        Args:
            it_text (Iterator): listing to process

        Returns:
            Bool: Operation success
        """
        # we copy locally the component spec to pop up at the end.
        bup = self.comp_spec_
        all_comp_spec = self.xml_parser_.get_comp_spec()

        listing = list(it_text)

        for item in all_comp_spec:
            self.comp_spec_ = item
            self.generate_simple_tags()

            is_ok = self.process_input(iter(listing))

            if not is_ok:
                break

        self.comp_spec_ = bup
        self.generate_simple_tags()
        return is_ok


def main():
    """main only defined for debugging purposes

    Returns:
        None: nothing
    """
    gen = CodeGenerator()
    xml_parser = PackageXMLParser()

    import rospkg
    rospack = rospkg.RosPack()
    dir_template_spec = rospack.get_path('package_generator_templates')

    dir_template_spec += "/templates/cpp_node_update/config"

    spec = TemplateSpec()

    if not spec.load_spec(dir_template_spec):
        print "Could not load the template spec"
        print "Bye"
        return

    if not xml_parser.set_template_spec(spec):
        print "template spec not compatible with parser requirements"
        print "Bye"
        return

    filename = rospack.get_path('package_generator')
    filename += '/tests/data/demo.ros_package'

    if not xml_parser.load(filename):
        print "Error while parsing the xml file {}".format(filename)
        print "Bye"
        return

    xml_parser.set_active_comp(0)
    gen.configure(xml_parser, spec)

    filename = rospack.get_path('package_generator_templates')

    filename += '/templates/cpp_node_update/template/README.md'

    gen.reset_output_file()
    if gen.process_file(filename):
        output_file = "README.md"
        gen.write_rendered_file(output_file)
        print "Output written in file {}".format(output_file)
    else:
        print "Debug!"


if __name__ == '__main__':
    main()
