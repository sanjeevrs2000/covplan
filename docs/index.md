# Introduction

This Python package generates a guidance trajectory for complete coverage in 2 dimensions. It can be used for operations where complete coverage of an Area of Interest (AoI) is required for applications in field robotics. This can also assist researchers in coverage path planning who want to compare it their algorithms  or want to develop new methods . It is an updated Python implementation of the coverage method discussed in [this paper](https://journals.sagepub.com/doi/full/10.5772/56248).

 It takes in only coordinates of points on the boundary of the area in consideration and returns the full trajectory as a list of latitude-longitude coordinates which makes it easily transferrable to potential applications. 

# Getting Started

It can be installed from PyPi by running `pip install covplan`. You can find the GitHub repository [here](https://github.com/sanjeevrs2000/covplan).

See [Usage](usage.md) for more information on how to use it and [About](about.md) for description about the method.

<!-- For full documentation visit [mkdocs.org](https://www.mkdocs.org).

## Commands

* `mkdocs new [dir-name]` - Create a new project.
* `mkdocs serve` - Start the live-reloading docs server.
* `mkdocs build` - Build the documentation site.
* `mkdocs -h` - Print help message and exit.

## Project layout

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files. -->
