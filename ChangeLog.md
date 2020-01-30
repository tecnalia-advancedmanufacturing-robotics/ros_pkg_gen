# Package Generator ChangeLog

The versioning used is [Sentimental versioning][sentimental].

[sentimental]: http://sentimentalversioning.org/

## [3.0.0] 2020-01-30

### Modified

* The template name is not anymore provided as input parameter to `generate_package`.
  It is now a parameter to the `package` within the interface file.
  So that a generation / update call only requries the xml file, nothing more.

## [2.1.0] 2020-01-14

### Added

* It is now possible to use the [Jinja2](https://jinja.palletsprojects.com/en/2.10.x/) generator.
  A Template designer can select the custom and /or the jinja2 generator.
  This does not affect the Developer.

## [2.0.0] 2019-12-22

### Modified

* In the XML specification, term `node` is replaced by `component`.
  Remove _"term"_ restriction to nodes, and enables considering other packages types.
  This change is also affecting the package templates.

## [0.0.1]: 2019-12-09

* The latest version aligned with the [reference paper](https://www.insticc.org/Primoris/Resources/PaperPdf.ashx?idPaper=78340).
