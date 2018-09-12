# Designing Package templates

This document describes how a _Template Designer_ can generate a package template.

## Required Content

A package template is a set of files gathered into a directory.
The expected content is the following (referring to the [C++ template](templates/cpp_node_update) as illustration):

* `config` directory, which contains specification information of the package (see [this example](templates/cpp_node_update/config))

* `template` directory, which is the bunch of skeleton files that are used for the package generation.

### Config directory

The specification of a template is done with two files, `dictionary.yaml` and `functions.py`.

**`dictionary.yaml`**

The dictionary file is basically the definition of the tags the _User_ will be able to use when defining a package to generate.

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
* `node_interface`: the proposed interface component that can have a node.
  The `Designer` can define the interface name and the interface attributes.
  They will be used by the `User` to add interface elements, with the required attributes.

The _Designer_ is only required to define these three keys `package_attributes` and `node_attributes` as lists, and `node_interface` as a dictionary.
The attribute names are completely defined by the Designer.

As said, the dictionary specifies the available XML elements and attributes for a _User_ when defining a package to create.
But it also defines different tags that will be available to the Designer when defining the package template.

**`functions.py`**

