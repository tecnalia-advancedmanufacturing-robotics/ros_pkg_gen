import importlib

# this only work if you are in the folder where is the file
# path are not expected otherwise
#module = importlib.import_module(py_file)


# see https://copyninja.info/blog/dynamic-module-loading.html
# https://www.blog.pythonlibrary.org/2012/07/31/advanced-python-how-to-dynamically-load-modules-or-classes/
# from https://stackoverflow.com/questions/67631/how-to-import-a-module-given-the-full-path/67693
# best solution would be:

module_path = "../../package_generator_templates/templates/cpp_node_update/config/functions.py"

module = dict()
with open(module_path) as f:
    exec(f.read(), module)

print "Keys found:"
print module.keys()

context = {'name': "misc", 'type':"std_msgs::String", 'desc':"an interface"}
res = module["get_package_type"](context)

print res