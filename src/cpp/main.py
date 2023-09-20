from cpp.coverage_path_planner import cpp, find_min

def main():
	num_clusters=4	#number of blocks
	r=2	#radius for Dubins curves
	input_file='sample_area.txt' #location of the input file containing coordinates of the field
	width = 10	#distance between tracks
	driving_angle=90	#angle wrt X-axis in degrees
	no_headland=0	#number of headlands if needed, otherwise 0
	
	op=cpp(input_file,num_hd=no_headland,width=width,theta=driving_angle,num_clusters=num_clusters,radius=r,visualize=False)
	print('The trajectory for full coverage consists of the following waypoints:',op)
	min=find_min(input_file)
	print('Angle for trajectory with minimum length:', min)

if __name__ == '__main__':
	main()