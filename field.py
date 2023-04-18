from numpy import *
from math import *
import numpy as np
import math, numpy.linalg, copy
import sys, string, os, traceback, time
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pylab as pl
from sklearn.cluster import KMeans
from pylab import ginput
import matplotlib, time
from tsp_solver.greedy import solve_tsp
from dubins_path_planner import plan_dubins_path
from lines import computeAngle, pointOnLine, intersectLines, intersectPointOnLine
from lines import getPoly, numPoly, tellme,rmCrossovers, rmGabs

class Field:
	def __init__(self, filename, opw, nhp, theta):
		"""Create an infinite field from a list of data points.
		Accepts a file name where field polygons are stored."""
		#self.filename = filename
		self.opw = opw
		self.nhp = nhp
		self.theta = math.fmod( theta * pi/180+ 2 * pi, 2 * pi)
		self.data = []  # data file containing field polygons
		self.tracks = [] # field tracks
		self.hd = []     #headland paths
		self.labels = [] # labels of track clusters of Kmeans
		self.n_clusters = 0 #number of clusters
		self.cluster_centers = [] # cluster centers of Kmeans
		self.robotIntiPosition = [] #robot initial position
		self.path = []  # optimal path to followed by the robot
		self.traj = []  # robot trajectory using Dubins curves

		print("file Name: ", filename)
		for line in open(filename, 'r'):
			self.data.append(tuple(line.strip().split()))
			#print data, type(data)
			#print data[0], data[0][0], data[0][:], len(data), data[len(data)-1]
		print(self.data)
		self.data = np.array(self.data,dtype=np.double)
		# print self.data[0,:]

	def showField(self):
		#showField: plots field compements
		tracks_lower,tracks_upper = [], []
		for i in range(0, len(self.tracks)):
			tracks_lower.append((self.tracks[i][0,0] , self.tracks[i][0,1]))
			tracks_upper.append((self.tracks[i][1,0] , self.tracks[i][1,1]))
		tracks_lower = np.asarray(tracks_lower)
		tracks_upper = np.asarray(tracks_upper)
		
		# plt.figure(1)
		# plt.clf()
		# # plt.hold(True)
		# plt.axis('equal')
		# plt.axis('off')

		#plot field's polygons
		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			if i == 1:
				plt.plot(poly[:,0] , poly[:,1] , 'b-')
			else:
				plt.plot(poly[:,0] , poly[:,1] , 'r-')

		#plot robot's initial position
		if self.robotIntiPosition.any():
			plt.fill( self.robotIntiPosition[:,0], self.robotIntiPosition[:,1], 'r', lw=2 )

		#plot field's tracks
		labels = self.labels
		n_clusters = len(np.unique(labels))
		colors = iter(cm.rainbow(np.linspace(0,1,n_clusters)))
		#colors = itertools.cycle(["r", "b", "g", "k", "m", "c"])
		
		for i, c in zip(list(range(n_clusters)), colors):
			my_members = labels == i
			#print "len of my_members", labels[labels==i]
			plt.plot([tracks_lower[my_members][:,0], tracks_upper[my_members][:,0]],\
					 [tracks_lower[my_members][:,1], tracks_upper[my_members][:,1]], c=c)

		'''
		for i in range(0, len(self.tracks)):
			plt.plot(self.tracks[i][:,0] , self.tracks[i][:,1] ,'b-',\
				self.tracks[i][0,0] , self.tracks[i][0,1] ,'g.',\
				self.tracks[i][1,0] , self.tracks[i][1,1] ,'r.')
		'''
		for i in range(0, len(self.tracks)):
			plt.plot(self.tracks[i][0,0] , self.tracks[i][0,1] ,'g.',\
					 self.tracks[i][1,0] , self.tracks[i][1,1] ,'r.')

		#plot field's headland paths
		print("len(self.hd):", len(self.hd))
		for i in range(0, len(self.hd)):
			if (i+1)%(self.nhp+1):
				plt.plot(self.hd[i][:,0] , self.hd[i][:,1] ,'g-')
			else:
				plt.plot(self.hd[i][:,0] , self.hd[i][:,1] ,'g-.')
		
		plt.show()
		#plt.savefig("first.jpg")
		#time.sleep(.5)
		plt.close()
		
	def trackGen(self):
		#generate tracks parallel to driving direction and at operating width
		os.system('clear')
		#load main polygon
		poly = getPoly(self.data, 1)  # later on we will work on headland paths
		#print "poly:", poly, poly[0,:], type(poly)
		
		# find minimum bounding box (MBB)
		xmin = np.min(poly[:,0])
		xmax = np.max(poly[:,0])
		ymin = np.min(poly[:,1])
		ymax = np.max(poly[:,1])

		#diagonal's length of the MBB
		dmax = np.linalg.norm([xmin-xmax, ymin-ymax])
		dmax1 = math.sqrt((xmin-xmax)**2 + (ymin-ymax)**2)
		#print "dmax:", dmax, dmax1, xmin-xmax

		# find center point of the MBB
		m = [(xmin+xmax)/2, (ymin+ymax)/2]     # you can use: np.mean([xmin, xmax])
		#print xmin, xmax, ymin, ymax, m, m[0], m[1], dmax

		#you can use
		print("Driving Angle: ", self.theta)
		Xline = [m[0], m[1], 1, np.tan(self.theta)]
		Xpline = [m[0], m[1], 1, np.tan(self.theta+pi/2)]
		p = pointOnLine(Xpline, [-dmax/2])
		Xpline = [p[0,0], p[0,1], 1, np.tan(self.theta+pi/2)]

		base_points = pointOnLine(Xpline, np.arange(0, dmax, self.opw))
		#print base_points

		#create tracks
		#print len(base_points)
		trackLines = np.concatenate((base_points, np.tile(Xline[2:4], (len(base_points), 1))), axis=1)
  
		c = 1
		#trk = {}
		for i in range(0, len(trackLines)):#range(0, len(trackLines))
			p1 = trackLines[i,0:2]
			p2 = trackLines[i,0:2]+dmax*trackLines[i,2:4]
			line1 = (p1[0], p1[1], trackLines[i,2], trackLines[i,3])
			tmp = []
			for j in range(1, len(self.hd)+1):
				if not (j%(self.nhp+1)):
					hdPoly = self.hd[j-1]
					#print "from trackGen poly: ", type(hdPoly), hdPoly
					for k in range(0, len(hdPoly)-1):
						p3 = hdPoly[k,:]
						p4 = hdPoly[k+1,:]
						line2 = (p3[0], p3[1], p4[0]-p3[0], p4[1]-p3[1])
						ip = intersectLines(line1, line2)
						#plt.hold(True)
						if ip != None:
							tmp.append(ip)
			if tmp:
				#print "tmp is not empty", tmp, type(tmp)
				tmp = array(tmp)
				tmp_x = np.abs(tmp[0,0]-tmp[-1,0])
				tmp_y = np.abs(tmp[0,1]-tmp[-1,1])

				if tmp_x < tmp_y:
					index = np.argsort(tmp[:,1], axis=-1, kind='mergesort')
					tmp = tmp[index, :]
				else:
					index = np.argsort(tmp[:,0], axis=-1, kind='mergesort')
					tmp = tmp[index, :]
				
				for l in range(0,len(tmp)-1):
					if (l%2) == 0:
						#name = 'id%s' % c
						#trk[name] = tuple(tmp[l:l+2,:])
						self.tracks.append(tmp[l:l+2,:])
						#c = c + 1

	def headlandGen(self):
		# Generates headland paths by expansion or contraction factor. in this function; polygon points are 
		# moved outward or inward by an arthogonal distnce d to the side: inwards if polygon points are in clock-wise 
		# direction and vice-versa (inward/outward polygon offseting)
		#plt.hold(True)
		#plt.axis('equal')

		#print self.nhp
		#load field polygons
		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			# check order: if ord < 0 it is anti-clockwise
			ord = 0
			for j in range(0, len(poly)-1):
				ord += (poly[j+1,0]-poly[j,0])*(poly[j+1,1]+poly[j,1])

			#print "ord is: ", np.sign(ord)

			if (ord < 0) and (i == 1):
				# outside polygon must be ordered in clockwise direction
				#print "outside poly is reversed"
				poly = poly[::-1,:]
			if (ord >= 0) and (i != 1) :
				# inside polygons must be ordered in anti-clockwise direction
				#print "inside poly is reversed"
				poly = poly[::-1,:]

			#generate n paths: (n+1) paths will be generated at distance d
			#        dis = [d/2+d*[0:n-1] d*n];
			if self.opw != 0:
				dist = [self.opw/2 + self.opw*r for r in range(0,self.nhp)]
				#dist = [dist, self.opw*self.nhp]
				dist.append(self.opw*self.nhp)
				#print "dist: ", dist

			#generate headland paths
			x = poly[0:-1,0]  #poly[0:-1,0]
			y = poly[0:-1,1]  #poly[0:-1,1]
			#plt.plot(x,y,'k-')

			#print len(poly), len(x)
			#xx = x[1::]
			#print "len: ", len(x), len(xx), len(y)
			dx2 = np.hstack((x[1:], x[0]))-x
			dy2 = np.hstack((y[1:], y[0]))-y
			dx1 = np.hstack((dx2[-1], dx2[0:-1]))
			dy1 = np.hstack((dy2[-1], dy2[0:-1]))
			s1 = np.sqrt(dx1**2+dy1**2)
			s2 = np.sqrt(dx2**2+dy2**2)
			den = s1*s2+dx1*dx2+dy1*dy2

			for j in range(0, len(dist)):
				#print j, dist[j]
				X = x + dist[j]*(dy1*s2+dy2*s1)/den
				Y = y - dist[j]*(dx1*s2+dx2*s1)/den
				#close the polygon
				X = np.hstack((X,X[0]))
				Y = np.hstack((Y,Y[0]))
				#combine
				tmp = np.vstack((X,Y)).T
				# remove crossovers
				if i == 1:
					#remove crossovers
					tmp = rmCrossovers(rmCrossovers(rmCrossovers(rmCrossovers(rmCrossovers(tmp, 6), 5), 4), 3), 2)
					#remove gabs
					tmp = rmGabs(rmGabs(rmGabs(rmGabs(tmp, self.opw, 2), self.opw, 3), self.opw, 4), self.opw, 5)
					#print "from headlandGen: poly", poly, type(poly)
		
				self.hd.append(tmp)
				

		#plt.show()

	def cluster(self):
		#classify 2D data set into a set of clusters using kmeans clustering
		# we use lower point of each track
		np.random.seed(42)
		data = []
		for i in range(0, len(self.tracks)):
			data.append((self.tracks[i][0,0] , self.tracks[i][0,1]))
		data = np.asarray(data)

		#print type(self.tracks), type(data), data, data[:,0]
		k_means = KMeans()
		k_means.fit(data)
		# array of indexes corresponding to classes around centroids, in the order of your dataset
		self.labels = k_means.labels_
		self.cluster_centers = k_means.cluster_centers_

		k_means_labels = k_means.labels_
		k_means_cluster_centers = k_means.cluster_centers_
		k_means_labels_unique = np.unique(k_means_labels)
		n_clusters = len(k_means_cluster_centers)

		self.n_clusters = n_clusters

		print("from clustering: ", len(k_means_labels_unique), n_clusters, k_means_labels, k_means_labels_unique)

		x_min, x_max = data[:, 0].min() - 1, data[:, 0].max() + 1
		y_min, y_max = data[:, 1].min() - 1, data[:, 1].max() + 1

		colors = iter(cm.rainbow(np.linspace(0,1,n_clusters)))

		# plt.figure(1)
		# plt.clf()
		# plt.plot(data[:, 0], data[:, 1], 'k.', markersize=4)
		# plt.scatter(k_means_cluster_centers[:, 0], k_means_cluster_centers[:, 1],
		#    marker='x', s=169, linewidths=3,
		#    color='b', zorder=8)
		# plt.xlim(x_min, x_max)
		# plt.ylim(y_min, y_max)
		# plt.xticks(())
		# plt.yticks(())
		# plt.show()

		fig = plt.figure()#figsize=(8, 3)
		for k, col in zip(list(range(n_clusters)), colors):
			my_members = k_means_labels == k
			cluster_center = k_means_cluster_centers[k]
		# 	plt.plot(data[my_members, 0], data[my_members, 1], 'w', markerfacecolor=col, marker='.')
		# 	plt.plot(cluster_center[0], cluster_center[1], 'o', markerfacecolor=col,markeredgecolor='k', markersize=6)
		# 	plt.title('KMeans')
		# 	plt.xlabel('X-axis')
		# 	plt.ylabel('Y-axis')
		# 	plt.xticks(())
		# 	plt.yticks(())
		# 	plt.text(cluster_center[0], cluster_center[1],  'C%d' % (k))
		# plt.grid()
		# plt.show()
	
	def getRobotIntialPoition(self):
		#get robot initial location (it should be two points to get heading angle)
		plt.figure(1)
		plt.clf()
		#plot field's polygons
		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			if i == 1:
				plt.plot(poly[:,0] , poly[:,1] , 'b-')
			else:
				plt.plot(poly[:,0] , poly[:,1] , 'r-')
		#get two points
		tellme('You will define a robot intial poition and heading angle (by selecting two points),\n click to begin')

		pl.waitforbuttonpress()
		happy = False
		while not happy:
			pts = []
			while len(pts) < 2:
				tellme('Select 2 points with mouse')
				pts = np.asarray( plt.ginput(2,timeout=-1) )
				if len(pts) < 2:
					tellme('Too few points, starting over')
					time.sleep(1) # Wait a second
				plt.fill( pts[:,0], pts[:,1], 'r', lw=2 )
				tellme('Happy? Key click for yes, mouse click for no')
				happy = pl.waitforbuttonpress()


		self.robotIntiPosition = pts
		#print "position: ", type(self.robotIntiPosition), self.robotIntiPosition
		time.sleep(1)
		plt.close(1)

	def tsp_opt(self):
		#in this function order the sequence of clusters as a TSP
		#Prepare the square symmetric distance matrix for n_cluster nodes:
		#  Distance from 0 to 1 is 1.0
		#                1 to 2 is 3.0
		#                0 to 2 is 2.0
		#D = [[ 0, 1.0, 2.0],
		#     [ 1.0, 0, 3.0],
		#     [ 2.0, 3.0, 0]]
		np.random.seed(42)
		#get robot initial position
		self.getRobotIntialPoition()

		#nodes locations using cluster centers and robot position at nodes number (n_cluster+1)
		nodes = np.vstack((self.cluster_centers, self.robotIntiPosition[0,:]))

		#print "robot intial position", self.robotIntiPosition
		#print "self.cluster_centers: ", self.cluster_centers
		#print "nodes: ", nodes

		# plt.plot(nodes[:,0], nodes[:,1], 'k*')
		# plt.show()

		n_nodes = len(nodes)
		dist = np.zeros((n_nodes, n_nodes))

		#print "dist: ", dist
		for i in range(n_nodes):
			for j in range(i+1, n_nodes):
				#print "i,j:",i,j
				tmp = nodes[i,:]-nodes[j,:]
				sum_squared = numpy.dot(tmp.T , tmp)
				dist[i,j] = sqrt(sum_squared)
		#copy upper trianle into lower traingle
		for i in range(n_nodes):
			for j in range(i, n_nodes):
				dist[j][i] = dist[i][j]

		#print "dist: \n", dist.astype(int)
		
		path = solve_tsp( dist )
		#shift path to start each time from robot intiail poition (nodes 0)
		ind = path.index(n_nodes-1)
		#print path, ind
		path = np.roll(path, -ind)
		#close the path: return to robot intial poition again (nodes 0)
		path = np.hstack((path, n_nodes-1))
		self.path = path
		# find total path distance
		d_total = 0.0
		for i in range(n_nodes):
			j = i + 1
			d_total+= dist[path[i]][path[j]]
		print("from opt: optimal path & total distance: ", path, d_total)

	def trajGen(self, turning_radius):
		#generate robot trajectory to connect blocks of tracks in the optimal order
		#the robot will start and end at at its intial position defined by two points in order to find the robots' heading angle
		#robot and trajectory parametes
		#turning_radius = 2.0
		step_size = 0.01
		plt.figure(1)
		plt.clf()
		# plt.hold(True)
		plt.axis('equal')
		plt.axis('off')

		#field tracks
		tracks_lower,tracks_upper = [], []
		for i in range(0, len(self.tracks)):
			tracks_lower.append((self.tracks[i][0,0] , self.tracks[i][0,1]))
			tracks_upper.append((self.tracks[i][1,0] , self.tracks[i][1,1]))
		tracks_lower = np.asarray(tracks_lower)
		tracks_upper = np.asarray(tracks_upper)

		
		#extract field's blocks and tracks in each block
		labels = self.labels
		n_clusters = len(np.unique(labels))
		colors = iter(cm.rainbow(np.linspace(0,1,n_clusters)))
		#colors = itertools.cycle(["r", "b", "g", "k", "m", "c"])

		#print "reduced path: ", self.path, self.path[1:-1]
		#Enter the field
		p1 = self.robotIntiPosition[0]
		p2 = self.robotIntiPosition[1]
		theta0 = computeAngle(p1, p2)


		for i in range(len(self.robotIntiPosition)):
			self.traj.append((self.robotIntiPosition[i][0], self.robotIntiPosition[i][1], 0))

		#print "traj: ", type(self.traj), self.traj, self.traj[-1], self.traj[-1][0], self.traj[-1][0:2]

		for i in self.path[1:-1]:
			# extract track i in optimal path
			my_members = labels == i
			start = tracks_lower[my_members]
			end = tracks_upper[my_members]
			#print i, len(my_members),range(len(start))

			#theta0 = computeAngle(p1, p2)


			for j in range(len(start)):
				if j%2:
					p3 = start[j,:]
					p4 = end[j,:]
				else:
					p4 = start[j,:]
					p3 = end[j,:]

				#print "p1 & p2: ", p1 ,p2
				
				theta1 = computeAngle(p3, p4)

				q0 = (p2[0], p2[1], theta0)
				q1 = (p3[0], p3[1], theta1)

				# qs, _ = dubins.path_sample(q0, q1, turning_radius, step_size)
				p_x,p_y,p_yaw,mode,_=plan_dubins_path(s_x=q0[0], s_y=q0[1], s_yaw=q0[2], g_x=q1[0], g_y=q1[1], g_yaw=q1[2], curvature=1/turning_radius)

				# plt.plot([p[0] for p in qs] , [p[1] for p in qs] , '-r')
				plt.plot(p_x , p_y , '-r')

				self.traj.append((p1[0], p1[1], 1))
				for k in range(len(p_x)):
					self.traj.append((p_x[k], p_y[k], 0))
				self.traj.append((p4[0], p4[1], 1))

				#update each step
				(p1,p2,theta0) = (p3,p4,theta1)

		#Exit the field
		p3 = self.robotIntiPosition[1]
		p4 = self.robotIntiPosition[0]
		theta1 = computeAngle(p3, p4)

		q0 = (self.traj[-1][0], self.traj[-1][1], theta0)
		q1 = (p1[0], p1[1], theta1)

		# qs, _ = dubins.path_sample(q0, q1, turning_radius, step_size)

		plt.plot(p_x, p_y, '-r')

		for k in range(len(p_x)):
			self.traj.append((p_x[k], p_y[k], 0))
			self.traj.append((p1[0], p1[1], 0))
			self.traj.append((p2[0], p2[1], 0))

		for i, c in zip(list(range(n_clusters)), colors):
			my_members = labels == i
			#print "len of my_members", labels[labels==i]
			plt.plot([tracks_lower[my_members][:,0], tracks_upper[my_members][:,0]],\
					 [tracks_lower[my_members][:,1], tracks_upper[my_members][:,1]], c=c)

		#plot field's polygons
		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			if i == 1:
				plt.plot(poly[:,0] , poly[:,1] , 'b-')
			else:
				plt.plot(poly[:,0] , poly[:,1] , 'r-')
	
		plt.show()

