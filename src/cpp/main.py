from coverage_path_planner import cpp

num_clusters=4	#number of blocks
radius=2	#radius for Dubins curves
input_file='../../files/mfield.txt' #location of the input file containing coordinates of the field
utm_=True	#if input coordinates are in lat-lon, set False
utmzone=32	# required if input is in utm
width = 10	#distance between tracks
driving_angle=90	#angle wrt X-axis in degrees
no_headland=0	#number of headlands if needed, otherwise 0

def main():
	
	"""
	Load field from txt file and visualize track lines and save it in a file
	boundary points can be either utm(m) or latlon (degrees)
	"""
	cpp(input_file,num_hd=no_headland,width=width,theta=driving_angle,num_clusters=4,radius=2,utm_=utm_)


if __name__ == '__main__':
	main()