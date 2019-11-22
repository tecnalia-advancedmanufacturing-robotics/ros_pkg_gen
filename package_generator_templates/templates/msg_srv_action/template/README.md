# {packageName}

## General description of the package

<!--- protected region package descripion begin -->
{packageDescription}
<!--- protected region package descripion end -->

{ifmsg}

### Messages

{endifmsg}
{forallmsg}
`{name}`
<!--- protected region msg {name} begin -->
{description}
<!--- protected region msg {name} end -->
{endforallmsg}
{ifsrv}

### Services

{endifsrv}
{forallsrv}
`{name}`
<!--- protected region srv {name} begin -->
{description}
<!--- protected region srv {name} end -->
{endforallsrv}
{ifaction}

### Actions

{endifaction}
{forallaction}
`{name}`
<!--- protected region msg {name} begin -->
{description}
<!--- protected region msg {name} end -->
{endforallaction}

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
