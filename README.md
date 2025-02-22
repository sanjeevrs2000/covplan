# CovPlan

A Python package for coverage path planning

[![PyPI](https://img.shields.io/pypi/v/covplan?color=blue&label=pypi)](https://pypi.org/project/covplan/0.2.0/)
[![PyPi license](https://img.shields.io/pypi/l/ansicolortags.svg)](https://pypi.org/project/covplan/0.2.0/)

This repository can be used for generating guidance trajectories for complete field coverage. It can be used for operations where complete coverage of an Area of Interest (AoI) is required for various robotic applications. It is an updated Python implementation of the methodology that was originally presented in [this paper](https://journals.sagepub.com/doi/full/10.5772/56248).

If you found this useful, you can cite us:
> Ramkumar Sudha, S. K., Mishra, D., & Hameed, I. A. (2024). A coverage path planning approach for environmental monitoring using an unmanned surface vehicle. Ocean Engineering, 310, 118645. https://doi.org/10.1016/j.oceaneng.2024.118645

> Hameed IA, Bochtis D, Sørensen CA. An Optimized Field Coverage Planning Approach for Navigation of Agricultural Robots in Fields Involving Obstacle Areas. International Journal of Advanced Robotic Systems. 2013;10(5). doi:10.5772/56248


## Installation

The package can be installed using pip 

```shell
pip install covplan
```

## Getting started

To generate a guidance trajectory for full coverage, use the API `covplan.pathplan(input_file, params)`. It returns a list of coordinates that compose a path for complete coverage.
In the input file, describe the boundaries of the AoI using its lat-lon coordinates in the following format:  
```
  lat1  lon1
  lat2  lon2
  ...
  lat1  lon1
  NaN  NaN
```
Ensure that the AoI is a closed polygon, by keeping the first coordinate the same as the last coordinate. Separate different polygons by including a `NaN NaN` at the end. Also ensure that the coordinates of the polygon are described clockwise, and counter-clockwise for any obstacles and forbidden regions.
`covplan.find_min(input_file, params)` runs a single objective optimizer to find the driving angle that minimizes the trajectory length for a given AoI and the specified parameters.

### Example usage
```python
from covplan import pathplan

def main():
	n_clusters=4	#number of sections
	r=2	#radius for Dubins curves
	input_file='sample_area.txt' #location of the input file containing coordinates of the field
	width = 10	#distance between tracks
	driving_angle=90	#angle wrt X-axis in degrees
	no_hd=0	#number of margins around boundary (each with distance=0.5*width) if needed, otherwise 0
	
	op=pathplan(input_file,num_hd=no_hd,width=width,theta=driving_angle,num_clusters=n_clusters,radius=r,visualize=False) # returns list of waypoint coordinates composing full trajectory for coverage
	print('The trajectory for full coverage consists of the following waypoints:',op)
	

if __name__ == '__main__':
	main()
```

## Documentation

The full documentation can be found [here](https://covplan.readthedocs.io/).

## Contributing

Contributions are always welcome!

See [contributing.md](/contributing.md) for ways to get started.

Please adhere to this project's [code of conduct](/contributing.md#code-of-conduct)
