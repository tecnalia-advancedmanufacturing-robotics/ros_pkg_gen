#!/usr/bin/env python


# the question being how to handle a line with multiple tag in it, some being probably not yet solvable. 

dico = {"nodeName": "great_node",
        "frecuency": "125"}

dico_extended = {"nodeName": "great_node",
                 "frecuency": "125",
                 "yo": "yeah"}


line = 'void update({{nodeName}}_data &data, {{yo}}_config config)'

print line.format(**dico_extended)

#from __future__ import print_function
import string

class MyFormatter(string.Formatter):
    def __init__(self, default='{{{0}}}'):
        self.default=default

    def get_value(self, key, args, kwds):
        if isinstance(key, str):
            return kwds.get(key, self.default.format(key))
        else:
            Formatter.get_value(key, args, kwds)

fmt = MyFormatter()
print fmt.format(line, nodeName='What a Node')

print fmt.format(line, **dico)

import re
line = "{ {name}++}"
matches = [[m.group(0)[1:-1], m.start()] for m in re.finditer(r'{.*?}', line)]

print "Found matches: {}".format(matches)

# This does not work... damned
#line = ' {'
#print fmt.format(line, **dico)
#print line.format(**dico)

def convert(line, delimiter = ['{', '}'], **kwargs):
    result_line = line

    for key, value in kwargs.iteritems():
        # print "{} == {}".format(key, value)
        splitted = result_line.split(delimiter[0] + key + delimiter[1])
        # print "Splitted: {}".format(splitted)

        acc = splitted[0]

        #print splitted[1:]
        #print splitted[-1]

        if len(splitted) > 1:
            for item in splitted[1:]:
                acc += value + item

            #acc += splitted[-1]

        result_line = acc

        # print "acc: {}".format(result_line)
        # print acc
    return result_line

def test_convert():
    print convert("{nodeName} This {is a test {nodeName}, {frecuency} {yeah} }", **dico)

    print " ANNDDDD"

    print convert("{nodeName} This {is a test {nodeName}, {frecuency} {yeah} }{nodeName}", **dico)

    print " ANNDDDD"

    print convert("This {is a test  }", **dico)

def test_re():
    found_tags = re.findall(r'{\w+}', "Hello this is a {testYes}")
    print found_tags

    found_tags = re.findall(r'{\w+}', "{ Hello this is a {test}")
    print found_tags

    found_tags = re.findall(r'{\w+}', "{Hello this is a {test}")
    print found_tags

def test_iterator():
    element = ["one", "two", "three", "four", "five"]

    num_and_elements = enumerate(element, start=1)

    print num_and_elements

    iter_element = iter(num_and_elements)

    try:
        while True:
            elt = iter_element.next()

            print elt
    except StopIteration:
        print "this is the end"

if __name__ == '__main__':
    test_iterator()