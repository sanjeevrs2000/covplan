# Coverage Path Planning (CPP)

##What is it useful for?
This repository contains code for generating a guidance trajectory for 2D coverage. It can be used for operations where complete coverage of an Area of Interest (AoI) is required, for land or marine applications.

## How to use the program
To use the program, run 'coverage_path_planner.cpp()'. It returns a list of coordinates that constitute the path for complete coverage.
In the input file, describe the boundaries of the AoI using its lat-lon coordinates like this:
'''
lat1  lon1
lat2  lon2
...
lat1  lon1
NaN  NaN
'''
Ensure that the AoI is a closed polygon, by keeping the first coordinate the same as the last coordinate. Separate different polygons by including a NaN NaN at the end. Also ensure that the coordinates of the polygon are described clockwise, and counter-clockwise for any obstacles and forbidden regions.
