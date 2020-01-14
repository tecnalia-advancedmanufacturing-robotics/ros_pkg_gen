#!/usr/bin/env python
"""
@package package_generator
@file test_update.py
@author Anthony Remazeilles
@brief test the management of update of existing node

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Non-Profit Open Software License 3.0 (NPOSL-3.0).
"""

import unittest
import os

from package_generator.file_update_management import GeneratedFileAnalysis


class UpdateTest(unittest.TestCase):
    """Tests proposed for the Package generator module
    """

    def setUp(self):
        """
        Common initialization for all tester
        """

        self.dir_name = "/tmp/test_package_generator/update"
        if not os.path.exists(self.dir_name):
            print "Creating the repo {}".format(self.dir_name)
            os.makedirs(self.dir_name)

    def test_detect_areas(self):
        """Check the detection of protected areas in code file
        """
        file_content = (
                   '# from CMakelists.txt' '\n'
         '# protected region additional user defined REQUIREMENTS begin #' '\n'
         'protected_01' '\n'
         '# protected region additional user defined REQUIREMENTS end #' '\n'
         'unprotected_02' '\n'
        #file_rest = (
         '# protected region additional user defined BUILD STATEMENTS begin #' '\n'
         'protected_03' '\n'
         '# protected region additional user defined BUILD STATEMENTS end #' '\n'
         'unprotected_04' '\n'
         '# protected region user Cmake macros begin #' '\n'
         'protected_05' '\n'
         '# protected region user Cmake macros end #' '\n'
         'unprotected_06' '\n'
         'from package.xml' '\n'
         '<!-- protected region additional package statements begin -->' '\n'
         'protected_07' '\n'
         '<!-- protected region additional package statements end -->' '\n'
         'from readme' '\n'
         '<!--- protected region package descripion begin -->' '\n'
         'protected_08' '\n'
         '<!--- protected region package descripion end -->' '\n'
         '<!--- protected region {name} begin -->' '\n'
         'protected_09' '\n'
         '<!--- protected region {name} end -->' '\n'
         'from node_common' '\n'
         '/* protected region user include files begin */' '\n'
         'protected_11' '\n'
         '/* protected region user include files end */' '\n'
         '/* protected region user member variables begin */' '\n'
         'protected_12' '\n'
         '/* protected region user member variables end */' '\n'
         '/* protected region user constructor begin */' '\n'
         'protected_13' '\n'
         '/* protected region user constructor end */' '\n'
         '/* protected region user configure begin */' '\n'
         'protected_14' '\n'
         '/* protected region user configure end */' '\n'
         '/* protected region user update begin */' '\n'
         'protected_15' '\n'
         '/* protected region user update end */' '\n'
         '/* protected region user implementation of action callback for TriggerPublisher begin */' '\n'
         'protected_16' '\n'
         '' '\n'
         'protected_17' '\n'
         '/* protected region user implementation of action callback for TriggerPublisher end */' '\n'
         '/* protected region user additional functions begin */' '\n'
         '/* protected region user additional functions end */' '\n')

        filename = self.dir_name + "/test_generated_file.cpp"

        with open(filename, 'w') as file_out:
            file_out.write(file_content)

        file_analyzor = GeneratedFileAnalysis()
        self.assertTrue(file_analyzor.extract_protected_region(filename))

        expected_output = {
            "additional user defined REQUIREMENTS": ["protected_01"],
            "additional user defined BUILD STATEMENTS": ['protected_03'],
            "user Cmake macros": ['protected_05'],
            "additional package statements": ['protected_07'],
            "package descripion": ['protected_08'],
            "{name}": ['protected_09'],
            "user include files": ['protected_11'],
            "user member variables": ['protected_12'],
            "user constructor": ['protected_13'],
            "user configure": ['protected_14'],
            "user update": ['protected_15'],
            "user implementation of action callback for TriggerPublisher": ['protected_16', '', 'protected_17']
        }

        for item in expected_output:
            self.assertTrue(item in file_analyzor.extracted_areas_)
            self.assertEquals(expected_output[item], file_analyzor.extracted_areas_[item].protected_lines_)
            # print file_analyzor.extracted_areas_[item].protected_lines_

        for item in file_analyzor.extracted_areas_:
            self.assertTrue(item in expected_output)


if __name__ == '__main__':
    print "test_update -- begin"
    unittest.main()
    print "test_update -- end"
