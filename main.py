from numpy import *
from math import *
import numpy as np
from tsp_solver.greedy import solve_tsp
from dubins_path_planner import plan_dubins_path
from field import Field

def main():
	
	"""
	Load field from txt file and visualize track lines and save it in a file
	boundary points can be either utm(m) or latlon (degrees)
	"""

	f = Field('files/mfield.txt', 10, 0, 90, utm_=True,utmzone=32)
	f.headlandGen()
	f.trackGen()
	f.cluster(num_clusters=4)
	f.tsp_opt()
	f.trajGen(turning_radius=2)
	f.showField()

	print("done.")


if __name__ == '__main__':
	main()