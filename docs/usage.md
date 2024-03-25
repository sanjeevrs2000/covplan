
# How to use it

To use the program, run `coverage_path_planner.covplan(input_file, params)`. It returns a list of coordinates that compose a path for complete coverage.
In the input file, describe the boundaries of the AoI using its lat-lon coordinates in the following format:  
```
  lat1  lon1
  lat2  lon2
  ...
  lat1  lon1
  NaN  NaN
```
The region of interest, or obstacles/forbidden regions can be described by coordinates on its boundaries which appropriately represents the region in consideration. Separate different polygons by including a `NaN NaN` at the end. Also ensure that the coordinates of the polygon are described clockwise, and counter-clockwise for any obstacles and forbidden regions.
`coverage_path_planner.find_min(input_file, params)` runs a single objective optimizer to find the driving angle that minimizes the trajectory length for a given AoI and the specified parameters.

# API Reference

***coverage_path_planner.covplan(params)***

This function returns the list of waypoint coordinates that together compose the trajectory that covers the area. Further information about what each parameter indicates can be found in the [software description](desc.md).

| Parameter    | Type   | Description                                   |
|--------------|--------|-----------------------------------------------|
| `input_file` | text file | Text file containting list of coordinates describing the boundaries.  |
| `width`        | float    | The distance between parallel tracks aka operating width in meters.         |
| `num_hd`      | int | Number of boundary margins or headland polygons          |
| `theta`      | float | The driving or heading angle of the parallel tracks in degrees ( 0<theta <=180)  |
| `num_clusters`      | int | Number of sections into which the AoI is divided.                |
| `radius`      | string | The radius of the Dubins curves to be used.                |
| `visualize`      | Boolean | `True` if the trajectory is to be visualized.|

***coverage_path_planner.find_min(params)***

This function returns the angle that minimizes the trajectory length for a chosen set of parameters.

| Parameter    | Type   | Description                                   |
|--------------|--------|-----------------------------------------------|
| `input_file` | text file | Text file containting list of coordinates describing the boundaries.  |
| `width`        | float    | The distance between parallel tracks aka operating width.         |
| `num_hd`      | int | Number of boundary margins or headland polygons          |
| `num_clusters`      | int | Number of sections into which the AoI is divided.                |
| `radius`      | string | The radius of the Dubins curves to be used.                |
| `verbose`      | Boolean | `True` if the distance and minimum angle is to be printed.|


# Example usage

Here is an example code snippet using this API

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
