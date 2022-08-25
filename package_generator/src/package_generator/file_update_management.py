#!/usr/bin/env python3
"""
@package package_generator
@file file_update_management.py
@author Anthony Remazeilles
@brief responsible of the management of file update

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.
"""

from copy import deepcopy
from package_generator.enhanced_object import EnhancedObject

# define a protected line
# with all the current field
# define a prottected area
# should contain similar field that the protected line
# should gather the list of line within the tag area.
# begining and end of line as well


class ProtectedArea(object):
    """Definition of a protected area, as a list of code line

    Attributes:
        num_line_start_ (int): in original file, where the code starts
        num_line_stop_ (int): in original file, where the code finishes
        protected_lines_ (list): list of lines within that protected area
        tag_ (str): string ident of that protected line
    """

    def __init__(self):
        """Object constructor
        """
        self.num_line_start_ = -1
        self.num_line_stop_ = -1
        self.protected_lines_ = []
        self.tag_ = None

    def set(self, pl_start, pl_stop, all_lines):
        """set the content of a protected area

        Args:
            pl_start (ProtectedLine): comment codeline launching the protection
            pl_stop (ProtectedLine): comment codeline finishing the protection
            all_lines (list): all codelines protected

        Returns:
            Bool: True on success
        """
        assert pl_start.is_protected_line()
        assert pl_stop.is_protected_line()
        assert pl_start.protected_tag_ == pl_stop.protected_tag_
        assert pl_start.is_start_flag()
        assert pl_stop.is_stop_flag()
        # to be check with a single line of comment.
        # for the added value, check related comment in the following
        assert pl_start.num_line_ < pl_stop.num_line_

        self.num_line_start_ = pl_start.num_line_
        self.num_line_stop_ = pl_stop.num_line_
        self.tag_ = pl_start.protected_tag_

        # line num starts at 1
        # start: ok, since we skip the protected tag line
        # stop: we quit 1 lines to take it out
        self.protected_lines_ = all_lines[self.num_line_start_:self.num_line_stop_-1]
        return True


class ProtectedLine(EnhancedObject):
    """Description of a protected line

    Attributes:
        comment_format_ (list): 2 strings respectively for the begin / end of the comment
        delimitor_ (list): 2 string respectively to identify the beginining / end of the protected area
        line_ (TYPE): Description
        num_line_ (int): Description
        protected_tag_ (str): unique tag for the current comment
        protection_pattern_ (str): tag used to detected protected area start / stop

    Deleted Attributes:
        line_start_ (int): line at which the protected area start
    """
    def __init__(self, name="ProtectedLine"):
        """Object constructor

        Args:
            name (str, optional): Description
        """
        super(ProtectedLine, self).__init__(name)

        self.delimitor_ = []
        self.num_line_ = -1
        self.protected_tag_ = ""
        self.protection_pattern_ = ""
        self.comment_format_ = []
        self.line_ = None

    def __repr__(self):
        """return a string state of the object

        Returns:
            str: Object state
        """
        output = "<ProtectedLine "
        output += "delimitor: {} ".format(self.delimitor_)
        output += "num_line: {} ".format(self.num_line_)
        output += "protected_tag: {} ".format(self.protected_tag_)
        output += "protection_pattern: {} ".format(self.protection_pattern_)
        output += "comment_format: {} ".format(self.comment_format_)
        output += ">"
        return output

    def __str__(self):
        """return a string state of the object

        Returns:
            str: Object state
        """
        output = "<ProtectedLine "
        output += "delimitor: {} ".format(self.delimitor_)
        output += "num_line: {} ".format(self.num_line_)
        output += "protected_tag: {} ".format(self.protected_tag_)
        output += "protection_pattern: {} ".format(self.protection_pattern_)
        output += "comment_format: {} ".format(self.comment_format_)
        output += ">"
        return output

    def set_line_content(self, line):
        """set the line content

        Args:
            line (str): protected line to place in
        """
        self.line_ = line

    def is_protected_line(self, line=None):
        """check wether a line contains the protection tag

        Args:
            line (None, optional): string to check (if None internal one used)

        Returns:
            Bool: True if the line has the protection tag
        """
        if line is None:
            line = self.line_
        # print "Looking for {} in {}".format(self.protection_pattern_, line)
        return self.protection_pattern_ in line

    def is_start_flag(self, line=None):
        """check wether the flag in line corresponds to the protection begining

        Args:
            line (None, optional): string to check (if None internal one used)

        Returns:
            Bool: if it is a start flag
        """
        if line is None:
            line = self.line_

        assert len(self.delimitor_) == 2, "No delimitor defined for the protected region"
        if not self.is_protected_line(line):
            return False
        return self.delimitor_[0] in line

    def is_stop_flag(self, line=None):
        """check wether the flag in line corresponds to the protection end

        Args:
            line (None, optional): string to check (if None internal one used)

        Returns:
            Bool: if it is an end flag
        """
        if line is None:
            line = self.line_

        assert len(self.delimitor_) == 2, "No delimitor defined for the protected region"
        if not self.is_protected_line(line):
            return False
        return self.delimitor_[1] in line

    def deduce_comment_format(self, line=None):
        """extract the comment format of a protected area

        Args:
            line (None, optional): string to check (if None internal one used)

        Returns:
            list: string related to comment start and end.
        """
        if line is None:
            line = self.line_
        # self.log("*********************************")
        # self.log("Processing line: {}".format(line))
        # looking for the comment type
        self.comment_format_ = list()
        # the comment start format is found in the string before the protection pattern
        comment_string = line.split(self.protection_pattern_)[0]
        # self.log("Looking for begin comment marker within |{}|".format(comment_string))
        comment_string = comment_string.strip()
        if comment_string == "":
            self.log_error("Prb while searching the begin comment in line {}".format(line))

        self.comment_format_.append(comment_string)

        # the end is found in the string after the delimitor pattern
        # if the string is repeated, this may provide issues
        # one should instead take all the rest, not only the second element.
        # todo look at https://www.tutorialspoint.com/python/string_find.htm

        if self.is_start_flag(line):
            str_del = self.delimitor_[0]
        else:
            str_del = self.delimitor_[1]

        # search for the last instance of the area begin tag
        # if we may have several, it might be an issue by the way
        # To get the last, we search first occurence from the end, reversing both strings
        # to reverse a string: str[::-1]
        pos_in_str = len(line) - line[::-1].find(str_del[::-1])
        if pos_in_str == -1:
            self.log_error("Prb while searching pose of {} in line {}".format(self.delimitor_[0], line))
            return False
        # cut the string through slicing
        comment_string = line[pos_in_str:]

        # self.log("Looking for end comment marker within |{}|".format(comment_string))
        comment_string = comment_string.strip()
        if comment_string == "":
            self.log("Prb while searching the begin comment in line {}".format(line))

        self.comment_format_.append(comment_string)

        # self.log("comment format is: {}".format(self.comment_format_))

        # looking for the comment unique id
        tag_string = line.split(self.protection_pattern_)[1]
        # self.log("Looking for ident tag within |{}|".format(tag_string))
        tag_string = tag_string.strip()
        if tag_string == "":
            self.log_error("Prb while searching the comment id in line {}".format(line))
            return False

        # searching for the last occurence again,
        # but we want to slice before the delimitor
        pos_in_str = len(tag_string) - tag_string[::-1].find(str_del[::-1]) - len(str_del)
        if pos_in_str == -1:
            self.log_error("Prb while searching pose of {} in line {}".format(self.delimitor_[0], line))
            return False
        tag_string = tag_string[:pos_in_str]

        tag_string = tag_string.strip()
        if tag_string == "":
            self.log_error("Prb while searching the begin comment in line {}".format(line))
            return False

        self.protected_tag_ = tag_string
        # self.log("Protected tag set to: {}".format(self.protected_tag_))
        return True


class GeneratedFileAnalysis(EnhancedObject):
    """Handle a generated file, wrt to protected areas

    Attributes:
        extracted_areas_ (dict): set of protected areas extracted organized per area identifier
        protected_pattern_ (str): message indicating protected area start
        protected_tags_ (list): string related to begining and end of protected area
    """
    def __init__(self, name="GeneratedFileAnalysis",
                 protected_pattern="protected region",
                 protected_tags=None):
        """Class constructor

        Args:
            name (str, optional): Description
            protected_pattern (str, optional): Description
            protected_tags (None, optional): Description
        """
        super(GeneratedFileAnalysis, self).__init__(name)

        # pattern text used fro delimiting user code
        self.protected_pattern_ = protected_pattern
        # tag to delimit the begining and the end of a user code
        if protected_tags is None:
            self.protected_tags_ = ["begin", "end"]
        # areas extracted, stored wrt to the tag used.
        self.extracted_areas_ = dict()

    def extract_protected_region(self, filename):
        """Extract the protected areas from a file given in parameters

        Args:
            filename (str): file to open

        Returns:
            Bool: Operation success
        """
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

        self.extracted_areas_ = dict()
        all_protected_lines = list()

        protected_line = ProtectedLine()
        protected_line.protection_pattern_ = self.protected_pattern_
        protected_line.delimitor_ = [" begin ", " end "]

        for id_line, line in enumerate(lines_in_file, start=1):
            # self.log("Checking l. {}: {}".format(id_line, line))

            if not protected_line.is_protected_line(line):
                continue
            protected_line.set_line_content(line)
            pl_copy = deepcopy(protected_line)
            # self.log_warn("protection detected on line {}".format(id_line))
            if not pl_copy.deduce_comment_format():
                self.log_error("Not able to deduce comment type from line {}".format(line))
                return False
            pl_copy.num_line_ = id_line
            all_protected_lines.append(pl_copy)

        # self.log("End of the protection search")
        num_protected_lines = len(all_protected_lines)
        # self.log("Number of the protected lines: {}".format(num_protected_lines))

        # starting the sanity check on the protected  markers
        # sanity check 1: number should be even
        if num_protected_lines % 2 != 0:
            self.log_error("Unbalanced set of protection")
            # we do not return yet, to check where is the unbalancing in the file

        # todo: this could be done progressively by removing element from the list
        id_line = 0
        while id_line < num_protected_lines:
            # sanity check 2: the current one should be a start
            if not all_protected_lines[id_line].is_start_flag():
                with all_protected_lines[id_line] as pline:
                    msg_err = "Protected line({}) on line {}: not finding the begining of such zone"
                    self.log_error(msg_err.format(pline.protected_tag_, pline.num_line_))
                    return False
            # sanity check 3: an opening should be followed by a closure of same pattern
            if id_line == num_protected_lines - 1:
                msg_err = "Missing end tag of protected area {} started on line {}"
                pline = all_protected_lines[id_line]
                self.log_error(msg_err.format(pline.protected_tag_, pline.num_line_))

            if not all_protected_lines[id_line + 1].is_stop_flag():
                pline = all_protected_lines[id_line]
                plinep = all_protected_lines[id_line + 1]

                # with all_protected_lines[id_line], all_protected_lines[id_line + 1] as pl, pln:
                msg_err = "Protected area ({}) on line {}: expected an end of area, found a new area on line: {}"
                self.log_error(msg_err.format(pline.protected_tag_, pline.num_line_, plinep.num_line_))
                return False

            if all_protected_lines[id_line].protected_tag_ != all_protected_lines[id_line + 1].protected_tag_:
                pline = all_protected_lines[id_line]
                plinep = all_protected_lines[id_line + 1]

                msg_err = "A end is expected for protected area [{}] from line {}."
                self.log_error(msg_err.format(pline.protected_tag_, pline.num_line_))
                self.log_error("Found protected area [{}] on line {}".format(plinep.protected_tag_, plinep.num_line_))
                return False

            id_line += 2
        # Data correct, now we reduce the set to store the begining and the end

        # todo: this could be done taking out the element from the list progressively
        id_line = 0
        while id_line < num_protected_lines:
            pa = ProtectedArea()
            pa.set(all_protected_lines[id_line], all_protected_lines[id_line + 1], lines_in_file)
            self.extracted_areas_[pa.tag_] = pa

            id_line += 2

        self.log("Number of protected area collected: {}".format(len(self.extracted_areas_)))

        # checking areas without Developper contribution
        empty_keys = list()
        # self.log("Keys used")
        for key in self.extracted_areas_:
            # self.log("|{}|: ({}, {})".format(key,
            #                                 self.extracted_areas_[key].num_line_start_,
            #                                 self.extracted_areas_[key].num_line_stop_))
            if self.extracted_areas_[key].num_line_start_ == self.extracted_areas_[key].num_line_stop_ - 1:
                # self.log_warn("empty area")
                empty_keys.append(key)

        # removing empty keys
        for key in empty_keys:
            self.extracted_areas_.pop(key, None)
        if empty_keys:
            self.log("User contribution found: {}".format(len(self.extracted_areas_)))
        return True

    def update_file(self, file_in_list):
        """
        @brief given a file as a list, update with the stored content

        @param      file_in_list The initial file

        @return updated list if the operation succeeded

        Args:
            file_in_list (list): list of str lines to be processed

        Returns:
            list: the updated file
        """
        # self.log("File initial contains {} lines".format(len(file_in_list)))
        file_updated = list()
        # todo: this is repeated from extract_protected_region.
        # Does it makes sense to repeat it?
        protected_line = ProtectedLine()
        protected_line.protection_pattern_ = self.protected_pattern_
        protected_line.delimitor_ = ["begin", "end"]

        # content can be palced in Developera area in pattern
        # if contrib provided by the user, related content in template
        # # should be skipped.
        wait_for_end_area = False

        for _, line in enumerate(file_in_list, start=1):
            # self.log("Process line [{}]: {}".format(id_line, line))
            if not protected_line.is_protected_line(line):
                # not protected, we add it as it is
                if not wait_for_end_area:
                    file_updated.append(line)
                continue

            protected_line.set_line_content(line)

            if protected_line.is_stop_flag() and wait_for_end_area:
                wait_for_end_area = False
            # line protected. Only interested by start
            if not protected_line.is_start_flag():
                file_updated.append(line)
                continue
            # self.log_warn("Detected start of protected line")
            # self.log_error("Line: {}".format(line))

            # we detected a protected area start. Looking for user contribution

            if not protected_line.deduce_comment_format():
                self.log_error("Not able to deduce comment type from line {}".format(protected_line))
                continue
            tag = protected_line.protected_tag_
            # self.log_warn("Tag is {}".format(tag))

            file_updated.append(line)
            # search for the tag in the stored ones
            if tag in self.extracted_areas_:
                # get the stored lines
                # self.log_error("Protected code: ")
                for _, user_line in enumerate(self.extracted_areas_[tag].protected_lines_):
                    # self.log_error("Adding: [{}]: {}".format(tmp_id, user_line))
                    file_updated.append(user_line)
                if self.extracted_areas_[tag].protected_lines_:
                    # self.log_warn("user contribution!")
                    wait_for_end_area = True
                    # in that case, we advance file processing until we get the end flag.

                # self.log_error("Done!!!")
            # else:
            #     self.log_error("Tag {} not found in previous file".format(tag))

        # self.log("File updated contains {} lines".format(len(file_updated)))
        return file_updated
