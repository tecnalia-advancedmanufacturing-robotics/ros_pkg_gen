#!/usr/bin/env python
"""
@package package_generator
@file code_generator.py
@author Anthony Remazeilles
@brief parse a ros template file, and generate the related file

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""

from termcolor import colored
import inspect
import re
from package_generator.package_xml_parser import PackageXMLParser

import string

class FormatterNoKeyError(string.Formatter):
    """formatter not generating error when a key is unknown

    Attributes:
        default (TYPE): Description
    """
    def __init__(self, default='{{{0}}}'):
        self.default = default

    def get_value(self, key, args, kwds):
        if isinstance(key, str):
            return kwds.get(key, self.default.format(key))
        else:
            string.Formatter.get_value(key, args, kwds)

def convert(line, delimiter=['{', '}'], **kwargs):
    """equivalant to format, with provided delimiter
       and without issue with unknown keys

    Args:
        line (str): string to process
        delimiter (list, optional): two char to segment the tags
        **kwargs: dictionnary to use

    Returns:
        str: the converted string
    """
    result_line = line

    for key, value in kwargs.iteritems():
        # print "{} == {}".format(key, value)
        splitted = result_line.split(delimiter[0] + key + delimiter[1])
        # print "Splitted: {}".format(splitted)

        acc = splitted[0]
        # print splitted[1:]
        # print splitted[-1]

        if len(splitted) > 1:
            for item in splitted[1:]:
                acc += value + item

        result_line = acc

        # print "acc: {}".format(result_line)
        # print acc
    return result_line

def get_package_type(context):
    return context['type'].split("::")[0]

def get_class_type(context):
    return context['type'].split("::")[1]

def get_python_type(context):
    return context['type'].replace("::", ".")

def get_cpp_path(context):
    return context['type'].replace("::", "/")

def get_camelcase_name(context):
    # print "here we are: processing {}".format(context)
    # print "Node name: {}".format(context['name'])
    return context['name'].title().replace("_", "")

def get_py_param_type(context):
    param_type = context['type']
    if param_type not in ['std::string', 'string', 'int', 'double', 'bool']:
        raise SyntaxError("Invalid type for param {}".format(param_type))
    if param_type in ['std::string', 'string']:
        return 'str_t'
    if param_type == 'int':
        return 'int_t'
    if param_type == 'double':
        return 'double_t'
    if param_type == 'bool':
        return 'bool_t'

def str2bool(strg):
    """Summary

    Args:
        strg (str): string containing a boolean value

    Returns:
        Bool: corresponding boolean value
    """
    return strg.lower() in ("yes", "true", "t", "1")

# todo should this be merged with the previous one?
def get_py_param_value_to_remove(context):
    param_type = context['type']
    if param_type not in ['std::string', 'string', 'int', 'double', 'bool']:
        msg = "Invalid type for param {}".format(param_type)
        msg += "\n autorized type: std::string, string, int, double, bool]"
        raise SyntaxError(msg)
    if param_type in ['std::string', 'string']:
        return context['value']
    if param_type == 'int':
        return context['value']
    if param_type == 'double':
        return context['value']
    if param_type == 'bool':
        return "{}".format(str2bool(context['value']))

def get_py_param_value(context):
    param_type = context['type']
    if param_type not in ['std::string', 'string', 'int', 'double', 'bool']:
        msg = "Invalid type for param {}".format(param_type)
        msg += "\n autorized type: std::string, string, int, double, bool]"
        raise SyntaxError(msg)
    if param_type in ['std::string', 'string']:
        return "\"{}\"".format(context['value'])
    if param_type == 'bool':
        return "{}".format(str2bool(context['value']))
    return context['value']

def get_cpp_param_value(context):
    param_type = context['type']
    if param_type not in ['std::string', 'string', 'int', 'double', 'bool']:
        msg = "Invalid type for param {}".format(param_type)
        msg += "\n autorized type: [std::string, string, int, double, bool]"
        raise SyntaxError(msg)
    if param_type in ['std::string', 'string']:
        return "\"{}\"".format(context['value'])
    if param_type == 'bool':
        if str2bool(context['value']):
            return "true"
        else:
            return "false"
    return context['value']


class CodeGenerator(object):
    """class responsible of the generation of a whole ROS package

    Attributes:
        dep_spec_ (TYPE): Description
        formatter_ (TYPE): Description
        name_ (TYPE): Description
        nodes_spec_ (TYPE): Description
        package_spec_ (TYPE): Description
        tmp_buffer_ (TYPE): Description
        transformation_ (TYPE): Description
        transformation_loop_ (TYPE): Description
        xml_parser_ (TYPE): Description
    """
    def __init__(self, name="CodeGenerator"):
        self.name_ = name

        self.transformation_ = {
            'packageName' : lambda: self.get_package_attr("name"),
            'packageDescription' : lambda: self.get_package_attr("description"),
            'packageAuthor' : lambda: self.get_package_attr("author"),
            'packageAuthorEmail' : lambda: self.get_package_attr("author_email"),
            'packageLicense' : lambda: self.get_package_attr("license"),
            'nodeName' : lambda: self.get_node_attr("name"),
            "camelCaseNodeName": lambda: get_camelcase_name(self.nodes_spec_["attributes"]),
            'nodeFrequency' : lambda: self.get_node_attr("frequency")#,
        }

        self.transformation_loop_ = {
            'forallpublisher' : lambda text: self.get_loop_gen("publisher", text),
            'foralldirectpublisher' : lambda text: self.get_loop_gen("directPublisher", text),
            'foralldirectsubscriber' : lambda text: self.get_loop_gen("directSubscriber", text),
            'forallsubscriber' : lambda text: self.get_loop_gen("subscriber", text),
            'forallserviceServer' : lambda text: self.get_loop_gen("serviceServer", text),
            'forallserviceClient' : lambda text: self.get_loop_gen("serviceClient", text),
            'forallactionServer' : lambda text: self.get_loop_gen("actionServer", text),
            'forallactionClient' : lambda text: self.get_loop_gen("actionClient", text),
            'foralllistener' : lambda text: self.get_loop_gen("listener", text),
            'forallbroadcaster' : lambda text: self.get_loop_gen("broadcaster", text),
            'foralldependencies' : lambda text: self.get_loop_dependencies(text),
            'forallnodes' : lambda text: self.get_loop_nodes(text),
            'forallparam' : lambda text: self.get_loop_gen("parameter", text),
            'ifparam' : lambda text: self.get_if_defined("parameter", text),
            'foralldynParam' : lambda text: self.get_loop_gen("dynParameter", text),
            'ifdynParam' : lambda text: self.get_if_defined("dynParameter", text),
            'ifpublisher' : lambda text: self.get_if_defined("publisher", text),
            'ifdirectpublisher' : lambda text: self.get_if_defined("directPublisher", text),
            'ifdirectsubscriber' :  lambda text: self.get_if_defined("directSubscriber", text),
            'ifsubscriber' :  lambda text: self.get_if_defined("subscriber", text),
            'ifaction' : lambda text: self.get_if_defined(["actionClient", "actionServer"], text),
            'ifserviceServer' : lambda text: self.get_if_defined(["serviceServer"], text),
            'ifserviceClient' : lambda text: self.get_if_defined(["serviceClient"], text),
            'ifactionServer' : lambda text: self.get_if_defined(["actionServer"], text),
            'ifactionClient' : lambda text: self.get_if_defined(["actionClient"], text),
            'iflistener' : lambda text: self.get_if_defined(["listener"], text),
            'ifbroadcaster' : lambda text: self.get_if_defined(["broadcaster"], text)
        }

        self.transformation_functions_ = {
            'get_package_type' : lambda context : get_package_type(context),
            'get_class_type' : lambda context : get_class_type(context),
            'get_python_type' : lambda context : get_python_type(context),
            'get_cpp_path' : lambda context : get_cpp_path(context),
            'get_py_param_value': lambda context : get_py_param_value(context),
            'get_cpp_param_value': lambda context : get_cpp_param_value(context),
            'get_py_param_type': lambda context : get_py_param_type(context)
        }

        self.tmp_buffer_ = list()
        self.xml_parser_ = None
        self.nodes_spec_ = None
        self.package_spec_ = None
        self.dep_spec_ = None

        self.formatter_ = FormatterNoKeyError()

    def log(self, text):
        """ display log message with the class name in parameter
        text the string to display
        """
        print "[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text

    def log_warn(self, text):
        """ display warn message with the class name in parameter
        text the string to display
        """
        print colored("[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text, 'yellow')

    def log_error(self, text):
        """ display warn message with the class name in parameter
        text the string to display
        """
        print colored("[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text, 'red')

    def set_xml_parser(self, parser):
        """set the xml parser object

        Args:
            parser (PackageXMLParser): the parser of interest
        """
        self.xml_parser_ = parser
        self.get_xml_parsing()

    def get_xml_parsing(self):
        """ set the xml parser, and extract the relevant input from it
        """
        assert self.xml_parser_ is not None, "No xml data defined"
        self.nodes_spec_ = self.xml_parser_.get_active_node_spec()
        self.package_spec_ = self.xml_parser_.get_package_spec()
        self.dep_spec_ = self.xml_parser_.get_dependency_spec()

        assert self.nodes_spec_, "No nodes specification"
        assert self.package_spec_, "No package specification"
        assert self.dep_spec_, "No dependency specification"

    def reset_output_file(self):
        """Reset the internal buffer used to accumulate generated code
        """
        self.tmp_buffer_ = list()

    def add_output_line(self, line):
        """Add a line to the code generated buffer

        Args:
            line (str): line to be added
        """
        self.tmp_buffer_.append(line)

    def add_output_lines(self, buffer_line):
        """Add a list of lines to the code generated buffer

        Args:
            buffer_line (list): list of lines to be added
        """
        self.tmp_buffer_ += buffer_line

    def write_output_file(self, filename=None):
        """write the generated code

        Args:
            filename (None, optional): file to save to. If None, printed on screen

        Returns:
            Bool: True if the operation succeeded.
        """
        if filename is None:
            self.log("Resulting file:")
            print"----"
            for item in self.tmp_buffer_:
                print item
            print"----"
            return True

        try:
            # self.log("Storing {} lines in {}".format(len(self.tmp_buffer_), filename))
            out_file = open(filename, 'w')
            for item in self.tmp_buffer_:
                # print "printing {}".format(item)
                out_file.write(item + '\n')
            out_file.close()

        except IOError:
            self.log_error("Prb while opening the output file {}".format(filename))
            return False
        return True

    # def process_multi_line_string(self, stringfile):
    #     for item in stringfile.splitlines():
    #         self.add_output_line(self.process_line(item))

    def process_file(self, filename):
        """
        process a file with tags

        Args:
            filename (str): pathfile

        Returns:
            Bools: True on sucess
        """
        self.log("Generating file {}".format(filename))

        if self.xml_parser_ is None:
            self.log_error("XML parser not defined")
            return False
        lines_in_file = list()
        try:
            with open(filename) as input_file:
                for line in input_file:
                    line = line.rstrip('\n')
                    lines_in_file.append(line)
        except IOError:
            self.log_error("Prb while opening file {}".format(filename))
            return False
        # self.log("File to process contains {} lines".format(len(lines_in_file)))

        # generate an iterator on the enumerated content of the lines list
        iter_enum_lines = iter(enumerate(lines_in_file, start=1))
        return self.process_input(iter_enum_lines)

    def generate_file(self, file_template, output_file=None):
        """generate a file and store it where specified

        Args:
            file_template (str): template to follow
            output_file (str): where to store the generated code

        Returns:
            Bool: True on success
        """
        self.reset_output_file()

        if not self.process_file(file_template):
            return False

        if output_file is None:
            return True

        return self.write_output_file(output_file)

    def has_tag(self, line):
        """
        @brief Determines if a tag is present in the povide code line.

        @param      self The object
        @param      line The line to be parsed

        @return True if has tag, False otherwise.
        """
        found_tags = re.findall(r'{\w+}', line)
        assert len(found_tags) < 2, "Several tags {} on single line: |{}|".format(found_tags, line)

        return found_tags

    def get_tag(self, line):
        assert self.has_tag(line), "get_tag: no tag in input line |{}|".format(line)

        match = re.search(r'{\w+}', line)
        tag = match.group(0)[1:-1]

        return tag, match.start()

    def get_all_tags(self, line):

        matches = [[m.group(0)[1:-1], m.start()] for m in re.finditer(r'{\w+}', line)]

        # print "Found matches: {}".format(matches)
        return matches

    def get_all_tags_pattern(self, root_pattern, line):
        #self.log("Processing line {}".format(line))
        pattern = r'{%s-\w+}' % (root_pattern)
        # self.log("Pattern search: {}".format(pattern))
        instances = re.finditer(r'\{' + root_pattern + r'-\w+}', line)
        #instances = re.finditer(r'\{apply-\w+-\w+\}', line)
        #instances = re.finditer("%r"%pattern, line)
        matches = [[m.group(0)[1:-1], m.start()] for m in instances]

        #print "Found matches: {}".format(matches)
        return matches

    def process_input(self, iter_enum_lines):
        try:
            is_ok = True
            while is_ok:
                num_line, line = iter_enum_lines.next()
                # self.log("Processing line [{}]: {}".format(num_line, line))

                matches = self.get_all_tags(line)

                if not matches:
                    self.add_output_line(line)
                    num_line += 1
                    continue

                # the line has a tag
                # check for a loop tag
                loop_tag_found = False
                tags = [tag for tag, _ in matches]

                for item in self.transformation_loop_.keys():
                    loop_tag_found = loop_tag_found or item in tags
                    end_item = 'end' + item
                    loop_tag_found = loop_tag_found or end_item in tags

                if len(matches) > 1:
                    # multiple matches.
                    # we make simple checks according to authorize operations
                    assert not loop_tag_found, "Multiple tag with loop found. Not yet implemented"

                # todo(Anthony) if the tag is the same, we can avoid two calls

                if not loop_tag_found:
                    for tag, indent in matches:
                        #self.log("found tag |{}| at line {}:col {}".format(tag, num_line, indent))

                        if tag in self.transformation_.keys():
                            replacement = self.transformation_[tag]()
                            line = convert(line, **{tag: replacement})
                        else:
                            # todo here we could publish the line
                            self.log_warn("tag {} not processed".format(tag))
                    # check if the line is empty
                    # that would be due to a if tag that is not defined.
                    if line and (not line.isspace()):
                        self.add_output_line(line)
                    # else:
                    #    print colored("Line empty: |{}|".format(line), "blue")

                    # todo why do I increase num_line here? Isn t it done automatically?
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
                    #accumulated_line = line + '\n'
                    accumulated_line = ""

                    try:
                        while not tag_found:
                            # todo: avoid using an accumulated line here.
                            sub_num_line, sub_line = iter_enum_lines.next()
                            # print "Sub Checking line: {}".format(sub_line)

                            sub_matches = self.get_all_tags(sub_line)
                            subtags = [tagg for tagg, _ in sub_matches]

                            if not matches:
                                accumulated_line += sub_line + '\n'
                                continue
                            if search_tag in subtags:
                                tag_found = True
                                break
                            else:
                                accumulated_line += sub_line + '\n'
                                continue
                    except StopIteration, e:
                        error_msg = "missing closing tag {} openned on line {}".format(search_tag, num_line)
                        raise SyntaxError('Syntax ERROR',
                                          {'filename': 'unknown',
                                           'lineno': "{}".format(num_line),
                                           'offset': "{}".format(indent),
                                           'text': error_msg})

                    # tag found. We know have a bunch of line to process
                    # print "content to process in loop: \n{}".format(accumulated_line)
                    # todo: should the loop tag be returning the generated code, or directly write in the
                    # output? using process_loop imbricated, it is by default writting in it.

                    is_ok = self.transformation_loop_[tag](accumulated_line)

                    continue
                raise SyntaxError('SyntaxError',
                                  {'filename': 'unknown',
                                   'lineno': "{}".format(num_line),
                                   'offset': "{}".format(indent),
                                   'text': "unknown tag {}".format(tag)})
        except AssertionError, e:
            self.log_error("Assertion Error on line {}: {}".format(num_line, e.args))
            return False
        except SyntaxError, e:
            self.log_error("Syntax Error on line {}: {}".format(num_line, e.args))
            return False
        except StopIteration, e:
            # self.log("All file has been processed")
            return True
        except Exception, e:
            self.log_error("Error detected around line {}: {}".format(num_line, e.args))
        self.log_error("This should not be reached...")
        return False

    def get_package_attr(self, attr):
        return self.package_spec_[attr]

    def get_node_attr(self, attr):
        return self.nodes_spec_["attributes"][attr]

    # todo remove that function and use the apply tag instead
    # function kept in case it could be defined as an external function for simplification
    def get_include_interface(self):
        output = None

        include_set = set()

        # gathering interface of all interfaces
        # todo use smae tag to factorize.
        # relevant_interfaces = ["publisher", "subscriber", "actionClient", "actionServer", "actionServer", "serviceServer", "serviceClient"]
        # for item_interface in relevant_interfaces:
        #     for item in self.nodes_spec_["interface"][item_interfaces]:
        #         str_path = item['msg'].replace('::', '/')
        #         include_set.add("#include <{}.h>".format(str_path))

        for item in self.nodes_spec_["interface"]["publisher"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.nodes_spec_["interface"]["directPublisher"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.nodes_spec_["interface"]["directSubscriber"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.nodes_spec_["interface"]["subscriber"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        for item in self.nodes_spec_["interface"]["actionClient"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}Action.h>".format(str_path))

        for item in self.nodes_spec_["interface"]["actionServer"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}Action.h>".format(str_path))

        for item in self.nodes_spec_["interface"]["serviceServer"]:
            str_path = item['type'].replace('::', '/')
            include_set.add("#include <{}.h>".format(str_path))

        # todo this may be added to the ros core even though it does not use it.
        for item in self.nodes_spec_["interface"]["serviceClient"]:
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

    # todo how should be handled the dependency inherent to the structure used?
    # should it be manually added, or resulting in error?
    # manually handled for dyn rec & action, similar point of view to consider
    # for other types
    def get_loop_dependencies(self, text):
        output = list()
        # self.log("Handling text: \n {}".format(text))
        for item in  self.dep_spec_:
            dep_dict = {'dependencyName': item}
            #self.log("Handling dependency {}".format(item))
            for line in text.splitlines():
                # self.log("Handling line {}".format(line))
                line_processed = convert(line, **dep_dict)

                output.append(line_processed)

        self.add_output_lines(output)

        return True

    # todo we loose the access to all information.
    # Added manually the node name. More may be needed.
    # todo how to catch an error
    def get_loop_gen(self, interface_type, text):
        output = list()

        # self.log("Handling text: \n {} with interface {}".format(text, interface_type))
        node_name = self.nodes_spec_["attributes"]["name"]

        assert interface_type in self.nodes_spec_["interface"], "Requested interface type {} unknown".format(interface_type)

        # computing all upperlayer spec
        # todo this could be avoided and done once
        # todo here is not considered the aditional functions
        # todo we could aonly do this when item exists in any case
        context_upper = dict()
        for key in self.transformation_:
            context_upper[key] = self.transformation_[key]()

        for item in self.nodes_spec_["interface"][interface_type]:
            # self.log("Handling {} {}".format(interface_type, item))

            context_extended = item.copy()
            context_extended.update(context_upper)
            #self.log_warn("Extended item {}".format(context_extended))
            for line in text.splitlines():
                # self.log("Handling line {}".format(line))
                # todo: there is a risk of inserting empty lines
                # maybe not, since imbricated cases are not considered yet
                # we can not have a for_all  if endif end_for_all
                line_processed = convert(line, **context_extended)

                # check for apply operator
                # todo remove the loop on apply, that is useless
                matches = self.get_all_tags_pattern("apply", line_processed)
                if matches:
                    # self.log_warn("Found matches: {}".format(matches))
                    for m in matches:
                        operation = m[0].split("-")[1]
                        # self.log("operation is: {}".format(operation))
                        # make sure the operation exist
                        if operation not in self.transformation_functions_:
                            self.log_warn("Operator {} unknown and thus discarded".format(operation))
                            continue
                        try:
                            res = self.transformation_functions_[operation](context_extended)
                        except Exception as e:
                            self.log_error("Exception detected in processing apply on line {} \n {} \n with item {}".format(line, e, item))
                            raise
                        # self.log_warn("result would be {}".format(res))
                        # self.log_warn("Here we are")
                        line_processed = convert(line_processed, **{m[0]: res})

                output.append(line_processed)

        self.add_output_lines(output)
        # self.log_error("Sanity check: Is dictionnary extended?\n {}".format(self.nodes_spec_["interface"][interface_type]))
        # self.log("created: {}".format(type(output)))
        # todo no false case?
        return True

    # todo unify the data format. Here it is provided as a unique string?
    def get_if_defined(self, interface_type, text):

        # self.log("Handling text: \n {}".format(text))
        if isinstance(interface_type, str):
            list_type = [interface_type]
        else:
            list_type = interface_type

        for item in list_type:
            assert item in self.nodes_spec_["interface"], "Requested interface type {} unknown".format(item)

        is_one_defined = False

        for item in list_type:
            is_one_defined = is_one_defined or self.nodes_spec_["interface"][item]

        if not is_one_defined:
            return True

        # if defined, we recall the generator...

        input_line = list()
        for line in text.splitlines():
            line = line.rstrip('\n')
            input_line.append(line)

        # todo is it good starting at line number 1?
        iter_enum_lines = iter(enumerate(input_line, start=1))
        is_ok = self.process_input(iter_enum_lines)

        return is_ok

    def get_loop_nodes(self, text):

        # we copy locally the node spec to pop up at the end.
        bup = self.nodes_spec_
        all_node_spec = self.xml_parser_.get_nodes_spec()

        input_line = list()
        # todo: is this operation needed?
        for line in text.splitlines():
            line = line.rstrip('\n')
            input_line.append(line)

        for item in all_node_spec:
            self.nodes_spec_ = item

            iter_enum_lines = iter(enumerate(input_line, start=1))
            is_ok = self.process_input(iter_enum_lines)

            if not is_ok:
                break

        self.nodes_spec_ = bup
        return is_ok

def main():

    gen = CodeGenerator()
    xml_parser = PackageXMLParser()

    import rospkg
    rospack = rospkg.RosPack()
    node_path = rospack.get_path('package_generator')

    #filename = '/home/anthony/code/kinetic/rosin/sandbox/extended.ros_package'
    filename = node_path + '/tests/extended.ros_package'

    if not xml_parser.load(filename):
        print "Error while parsing the xml file {}".format(filename)
        print "Bye"
        return

    filename = node_path + '/sandbox/template_interface.cpp'
    gen.set_xml_parser(xml_parser)
    gen.reset_output_file()
    if gen.process_file(filename):
        output_file = "test.cpp"
        gen.write_output_file(output_file)
        print "Output written in file {}".format(output_file)
    else:
        print "Debug!"

if __name__ == '__main__':
    main()
