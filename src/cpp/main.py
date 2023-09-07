from coverage_path_planner import cpp
from find_min import find_min

def main():
	num_clusters=4	#number of blocks
	radius=2	#radius for Dubins curves
	input_file='../../files/sample_area.txt' #location of the input file containing coordinates of the field
	width = 10	#distance between tracks
	driving_angle=90	#angle wrt X-axis in degrees
	no_headland=1	#number of headlands if needed, otherwise 0
	
	"""
	Load field from txt file and visualize track lines and save it in a file
	boundary points can be either utm(m) or latlon (degrees)
	"""
	# cpp(input_file,num_hd=no_headland,width=width,theta=driving_angle,num_clusters=num_clusters,radius=2)
	find_min(input_file)

if __name__ == '__main__':
	main()