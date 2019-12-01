# Designing Package templates

This document describes how a _Template Designer_ can generate a package template.

Once generated, a template can be checked for some major issues, using:

```shell
rosrun package_generator check_template [template_name]
```

## Required Content

A package template is a set of files gathered into a directory.
The expected content is the following (referring to the [C++ template](templates/cpp_node_update) as illustration):

* `config` directory: contains specification information of the package (see [this example](templates/cpp_node_update/config))

* `template` directory, bunch of skeleton files that are used for the package generation.

### Config directory

The specification of a template is done with two files, `dictionary.yaml` and `functions.py`.

**`dictionary.yaml`**:

The dictionary file is basically the definition of the tags the _User_ can use in the xml file defining the package to generate.

As an example, from the [C++ template](templates/cpp_node_update/config/dictionary.yaml):

```yaml
package_attributes: ["name", "author", "author_email", "description", "license", "copyright"]
node_attributes: ["name", "frequency"]

node_interface:
    publisher: ["name", "type", "description"]
    directPublisher: ["name", "type", "description"]
    subscriber: ["name", "type", "description"]
    directSubscriber: ["name", "type", "description"]
    serviceClient: ["name", "type", "description"]
    serviceServer: ["name", "type", "description"]
    parameter: ["name", "type", "value", "description"]
    dynParameter: ["name", "type", "value", "description"]
    actionServer: ["name", "type", "description"]
    actionClient: ["name", "type", "description"]
    listener: ["name", "description"]
    broadcaster: ["name", "description"]
```

* `package_attributes`: the list of XML attributes the _User_ can provide to the required XML element `package`.
* `node_attributes`: the list of XML attributes the _User_ can provide to the required XML element `node`.
* `node_interface`: list of interface components authorized by the template.

The _Designer_ is only required to define these three keys `package_attributes` and `node_attributes` as lists, and `node_interface` as a dictionary.
The attribute names are completely defined by the Designer.

As said, the dictionary specifies the available XML elements and attributes for a _User_ when defining a package to create.
But it also defines different tags that will be available to the Designer when defining the package template.

**`functions.py`**:

This file must contain at least **two** functions:

```python
def dependencies_from_template():
    """provides the dependencies required by the template

    Returns:
        list: list of ROS package dependency required by the template
    """
    return ['rospy']
```

Function `dependencies_from_template` defines the dependencies that are mandatory for the package, according to the template used.
They will be automatically added to the ones provided by the `User` in the package specification.

```python
def dependencies_from_interface(interface_name, context):
    """return package dependencies according to the interface name

    Args:
        interface_name (str): name of the interface to consider
        context (dict): attributes assigned by the User to such instance

    Returns:
        list: List of dependencies that should be added according to
              the interface used and the attributes values
    """
    list_dep = []

    type_field = ['publisher', 'directPublisher', 'directSubscriber',
                  'subscriber', 'serviceClient', 'serviceServer',
                  'actionServer', 'actionClient']

    if interface_name in type_field:
        pkg_dep = get_package_type(context)
        list_dep.append(pkg_dep)

    if interface_name in ['actionClient', 'actionServer']:
        list_dep.append('actionlib')
        list_dep.append('actionlib_msgs')

    if interface_name in ['listener', 'broadcaster']:
        list_dep.append('tf')

    if interface_name == 'dynParameter':
        list_dep.append('dynamic_reconfigure')

    return list_dep
```

Function `dependencies_from_interface` defines the dependencies that are to be satisfied depending on the interface used by the `User` (parameter `interface_name`), as well as the values assigned by the `User` to this interface instance (parameter `context`).

Ideally that function should cover all the accepted interfaces as defined in file `dictionary.yaml`.

Notice that here we use a the function `get_package_type`, that is defined in that [same file][templates/cpp_node_update/config/functions.py].

Then the `Template Designer` may add additional functions that would be later on accessible during the code generation (see next section for more details).
For instance,

```python
def get_package_type(context):
    """extract the package the type belong to

    Args:
        context (dict): context (related to an interface description)

    Returns:
        str: the package extracted from the type key

    Examples:
        >>> context = {name: "misc", type={std_msgs::String}, desc="an interface"}
        >>> get_package_type(context)
        std_msgs
    """
    return context['type'].split("::")[0]
```

is a function that can be used to extract from the values assigned to the `type` attribute the ROS package to which belongs the type used.
In the provided example, the `type` value is set to `std_msgs::String`.
The call to that function would thus return `std_msgs`.

Note that all these functions **have to** be defined with a unique input parameter (named here `context`).
That variable is a dictionary that will contain all the attributes values provided by the `User` in its specification file.
These add-on functions must follow that structure to enable their automatic call during the code generation process.

### Template directory

The Template folder contains the skeleton of all files that will be deployed into the generated files.

All files are generated once, unless his name contains the string `node`.
In that case, the file will be generated for each node defined by the user.
The filename is then adjusted by replacing the string `node` with the node name.

#### Template file

A template file can be related to any language.
Each line of the template is reproduced, unless a specific instruction tag is encountered.

All instruction tags are of the form `"{instructionTag}"`.

**Package information**:

All package attributes can be accessed using the tags `{packageTag}`, where `Tag` is to be replaced by any of the attributes of a package, as defined in file `dictionary.yaml`.
Given the dictionnary example provided earlier, we would have access to instruction tags `{packageName}`, `{packageAuthor}`, `{packageAuthorEmail}`, `{poackageDescription}`, `{packageLicense}`, and `{packageCopyright}`.

**Node information**:

All nodes attributes can be accessed using the tags `{nodeTag}`, where `Tag` is to be replaced by any of the attributes of a node, as defined in file `dictionary.yaml`.
Given the dictionnary example provided earlier, we would have access to instruction tags `{nodeName}` and `{nodeFrequency}`.

To reproduce a bunch of codes for all nodes, use the following instructions:

```c++
{forallnodes}
...
{endforallnodes}
```

**Node interface access**:

To access a node interface access, we propose the following tools:

* Introduce a bunch of code **IF** at least one interface has been defined:

```
{iftag}
...
{endiftag}
```

* Introduce a bunch of code **FOR ALL** instances of a given instance:

```
{foralltag}
...
{endforalltag}
```

In both case, the word `tag` can be any of the interface name defined in `dictionary.yaml`.
With the above example, we would get such code generated for 12 interfaces, like `{iflistener}` and `{endiflistener}`, or `{foralldynParameter}` and `{endforalldynParameter}`.

In the loop instruction, where the code is repeated for each instance of the selected interface, we also get access to the interface attributes.
For example, with the above dictionary file,

```
{forallpublisher}
`{name}` *({type})*
{description}
{endforallpublisher}
```

tags `{name}`, `{type}`, and `{description}` would be replaced by the values entered by the `Developer` in each reproduction loop.

**Package Dependencies**:

Dependencies defined by the Developer can be accessed using the following tags:

```
{foralldependencies}
  <depend>{dependencyName}</depend>
{endforalldependencies}
```

With this illustrative instruction, the line will be repeated and completed for any of the dependencies defined by the _Developer_.

**Specific commands**:

All functions defined in previous file `functions.py` can be accessed in the template file using tag: `apply-function_name`, where `function_name` can be any of the functions present in that file `funtions.py`.
For instance, in the following code:

```
{forallpublisher}
from {apply-get_package_type}.msg import {apply-get_class_type}
{endforallpublisher}
```

The middle line will be repeated for each publisher defined.
The function `get_package_type` is previously defined.
If a publisher of type `std_msgs::String` is used, then this first function will return `std_msgs`, while the function `get_class_type` will provide the class name, i.e. `String`.
