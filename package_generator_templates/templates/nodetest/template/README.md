# {packageName}

<!--- protected region package description begin -->
{packageDescription}
<!--- protected region package description end -->

**Author & Maintainer**: {packageAuthor}, {packageAuthorEmail}

**Copyright** : {packageCopyright}

**License**: {packageLicense}.

## Content
{forallcomponent}

## {componentName} testing

Node test for the component {componentName} from package {componentPackage}:

{ifpublisher}
* [test/{componentName}_pub.test](test/{componentName}_pub.test): to test existence of publication on a given topic
{endifpublisher}
{ifcyclicPublisher}
* [test/{componentName}_hz_loop.test](test/{componentName}_hz_loop.test): to verify cyclic publication frequency
{endifcyclicPublisher}
{ifdynParameter}
* [test/{componentName}_dyn_param.test](test/{componentName}_dyn_param.test): to verify existence of dynamic parameters
{endifdynParameter}
{endforallcomponent}

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
