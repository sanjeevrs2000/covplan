# CovPlan: A Python package for coverage path planning

[![PyPI](https://img.shields.io/pypi/v/cc_pathplanner?color=blue&label=pypi)](https://pypi.org/project/covplan/0.1.0/)
[![PyPi license](https://img.shields.io/pypi/l/ansicolortags.svg)](https://pypi.org/project/cc-pathplanner/0.1.0/)


## Getting started
This repository contains a program which generates a guidance trajectory for complete 2D coverage. It can be used for operations where complete coverage of an Area of Interest (AoI) is required for various applications. It is an updated Python implementation of the method that was presented in [this paper](https://journals.sagepub.com/doi/full/10.5772/56248).

The package can be installed from PyPi by running `pip install covplan`. The full documentation can be found [here](https://covplan.readthedocs.io/).


## How to use it
To generate a guidance trajectory for full coverage, use the API `coverage_path_planner.covplan(input_file, params)`. It returns a list of coordinates that compose a path for complete coverage.
In the input file, describe the boundaries of the AoI using its lat-lon coordinates in the following format:  
```
  lat1  lon1
  lat2  lon2
  ...
  lat1  lon1
  NaN  NaN
```
Ensure that the AoI is a closed polygon, by keeping the first coordinate the same as the last coordinate. Separate different polygons by including a `NaN NaN` at the end. Also ensure that the coordinates of the polygon are described clockwise, and counter-clockwise for any obstacles and forbidden regions.
`coverage_path_planner.find_min(input_file, params)` runs a single objective optimizer to find the driving angle that minimizes the trajectory length for a given AoI and the specified parameters.

## Example usage
```python
from covplan import coverage_path_planner

def main():
	n_clusters=4	#number of sections
	r=2	#radius for Dubins curves
	input_file='sample_area.txt' #location of the input file containing coordinates of the field
	width = 10	#distance between tracks
	driving_angle=90	#angle wrt X-axis in degrees
	no_hd=0	#number of margins around boundary (each with distance=0.5*width) if needed, otherwise 0
	
	op=coverage_path_planner.covplan(input_file,num_hd=no_hd,width=width,theta=driving_angle,num_clusters=n_clusters,radius=r,visualize=False) # returns list of waypoint coordinates composing full trajectory for coverage
	print('The trajectory for full coverage consists of the following waypoints:',op)
	
	min=coverage_path_planner.find_min(input_file,width=width,num_hd=no_hd,num_clusters=n_clusters,radius=r,verbose=True)  # runs optimizer and returns angle corresponding to minimum path length
	# print('Angle for trajectory with minimum length:', min)

if __name__ == '__main__':
	main()
```
