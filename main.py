from numpy import *
from math import *
import numpy as np
import math, numpy.linalg, copy
import sys, string, os, traceback, time
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pylab as pl
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
import itertools
from pylab import ginput
import matplotlib, time
from tsp_solver.greedy import solve_tsp
from dubins_path_planner import plan_dubins_path
from field import Field

def main():

	
	''' load field polygons from a file and plot it'''
	
	f = Field('mfield.txt', 10, 0, 90)
	f.headlandGen()
	f.trackGen()
	f.cluster()
	f.tsp_opt()
	f.trajGen(6)
	f.showField()

	print("done.")


if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		print('>> Exiting...')
	# except Exception as err:
	# 	print(traceback.format_exc(err))
