from covplan import pathplan, find_min

def main():
	n_clusters=4	#number of sections
	r=2	#radius for Dubins curves
	input_file='sample_area.txt' #location of the input file containing coordinates of the field
	width = 10	#distance between tracks
	driving_angle=90	#angle wrt X-axis in degrees
	no_hd=0	#number of margins around boundary (each with distance=0.5*width) if needed, otherwise 0
	
	op=pathplan(input_file,num_hd=no_hd,width=width,theta=driving_angle,num_clusters=n_clusters,radius=r,visualize=False) # returns list of waypoint coordinates composing trajectory
	print('The trajectory for full coverage consists of the following waypoints:',op)
	
	min=find_min(input_file,width=width,num_hd=no_hd,num_clusters=n_clusters,radius=r)  # runs optimizer and returns angle corresponding to minimum path length
	print('Angle for trajectory with minimum length:', min)

if __name__ == '__main__':
	main()