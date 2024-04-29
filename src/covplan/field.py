from numpy import *
from math import *
import numpy as np
import math, numpy.linalg
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from sklearn.cluster import KMeans
from python_tsp.exact import solve_tsp_dynamic_programming
from covplan.dubins_path_planner import plan_dubins_path
from covplan.lines import computeAngle, pointOnLine, intersectLines, intersectPointOnLine
from covplan.lines import getPoly, numPoly
# from LLcoordinates import LLtoUTM, UTMtoLL
import folium
import utm

class Field:

	"""
	Creates an object of the Area of interest(AoI); does the required preprocessing and further operations for generating a trajectory for coverage
	"""

	def __init__(self, filename, opw, nhp, theta):
		
		"""
		args:
		filename - location of the input textfile that describes the region of interest
		opw - the distance between the parallel tracks, otherwise known as the operating width
		nhp - number of headlands of boundary margins to be created within the AoI
		theta - the angle that the parallel tracks make with the longitudinal axis
		"""
		self.opw = opw
		self.nhp = nhp
		self.theta = math.fmod( theta * pi/180+ 2 * pi, 2 * pi)
		self.data = []  # data file containing field polygons
		self.tracks = [] # field tracks
		self.hd = []     #headland paths
		self.labels = [] # labels of track clusters of Kmeans
		self.num_clusters = 0 #number of clusters
		self.cluster_centers = [] # cluster centers of Kmeans
		self.robotinitpos = [] #robot initial position
		self.path = []  # optimal path to followed by the robot
		self.traj = []  # robot trajectory using Dubins curves
		self.utm_l=None
		self.utmzone=None

		for line in open(filename, 'r'):
			self.data.append(tuple(line.strip().split()))

		self.data = np.array(self.data,dtype=np.double)		

		for i in range(len(self.data)):
			if np.isnan(self.data[i][0]):
				continue
			# z,ux,uy=LLtoUTM(23,self.data[i][0], self.data[i][1])
			ux,uy,z,l=utm.from_latlon(self.data[i][0],self.data[i][1])
			self.utmzone=z
			self.utm_l=l
			self.data[i]=[ux,uy]


	def showField(self):
		"""Visualizes the desired path on a map"""

		tracks_lower,tracks_upper = [], []
		for i in range(0, len(self.tracks)):
			tracks_lower.append((self.tracks[i][0,0] , self.tracks[i][0,1]))
			tracks_upper.append((self.tracks[i][1,0] , self.tracks[i][1,1]))
		tracks_lower = np.asarray(tracks_lower)
		tracks_upper = np.asarray(tracks_upper)
		
		#plot field's polygons
		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			if i == 1:
				plt.plot(poly[:,0] , poly[:,1] , 'b-')
			else:
				plt.plot(poly[:,0] , poly[:,1] , 'r-')
		
		# if self.robotinitpos.any():
		# 	plt.fill( self.robotinitpos[:,0], self.robotinitpos[:,1], 'r', lw=2 )
		
		labels = self.labels
		n_clusters = len(np.unique(labels))
		colors = iter(cm.rainbow(np.linspace(0,1,n_clusters)))
				
		for i, c in zip(list(range(n_clusters)), colors):
			my_members = labels == i
			plt.plot([tracks_lower[my_members][:,0], tracks_upper[my_members][:,0]],\
					 [tracks_lower[my_members][:,1], tracks_upper[my_members][:,1]], c=c)

		for i in range(0, len(self.tracks)):
			plt.plot(self.tracks[i][0,0] , self.tracks[i][0,1] ,'g.',\
					 self.tracks[i][1,0] , self.tracks[i][1,1] ,'r.')

		#plot field's headland paths
		# print("len(self.hd):", len(self.hd))
		for i in range(0, len(self.hd)):
			if (i+1)%(self.nhp+1):
				plt.plot(self.hd[i][:,0] , self.hd[i][:,1] ,'g-')
			else:
				plt.plot(self.hd[i][:,0] , self.hd[i][:,1] ,'g-.')
		
		plt.show()

		map=folium.Map(location=self.latlon[0],zoom_start=20)
		
		for i in range(len(self.hd)):
			self.hd_ll=[]
			for j in range(len(self.hd[i])):
				# lat,lon=UTMtoLL(23,self.hd[i][j][1], self.hd[i][j][0])
				lat,lon=utm.to_latlon(self.hd[i][j][0],self.hd[i][j][1],self.utmzone,self.utm_l)
				self.hd_ll.append([lat,lon])
			
			folium.PolyLine(self.hd_ll,color='red').add_to(map)

		folium.Polygon(self.latlon,color='blue').add_to(map)
		map.show_in_browser()

	def trackGen(self):
		"""
		Generates parallel tracks for the AoI with the given theta and width, that together compose the overall tracjectory
		"""

		poly = getPoly(self.data, 1) 
	
		# find minimum bounding box (MBB)
		xmin = np.min(poly[:,0])
		xmax = np.max(poly[:,0])
		ymin = np.min(poly[:,1])
		ymax = np.max(poly[:,1])

		#diagonal's length of the MBB
		dmax = np.linalg.norm([xmin-xmax, ymin-ymax])
		dmax1 = math.sqrt((xmin-xmax)**2 + (ymin-ymax)**2)

		# find center point of the MBB
		m = [(xmin+xmax)/2, (ymin+ymax)/2]

		# print("Driving Angle: ", self.theta)
		Xline = [m[0], m[1], 1, np.tan(self.theta)]
		Xpline = [m[0], m[1], 1, np.tan(self.theta+pi/2)]
		p = pointOnLine(Xpline, [-dmax/2])
		Xpline = [p[0,0], p[0,1], 1, np.tan(self.theta+pi/2)]

		base_points = pointOnLine(Xpline, np.arange(0, dmax, self.opw))
		trackLines = np.concatenate((base_points, np.tile(Xline[2:4], (len(base_points), 1))), axis=1)
  
		c = 1
		for i in range(0, len(trackLines)):
			p1 = trackLines[i,0:2]
			p2 = trackLines[i,0:2]+dmax*trackLines[i,2:4]
			line1 = (p1[0], p1[1], trackLines[i,2], trackLines[i,3])
			tmp = []
			for j in range(1, len(self.hd)+1):
				if not (j%(self.nhp+1)):
					hdPoly = self.hd[j-1]
					for k in range(0, len(hdPoly)-1):
						p3 = hdPoly[k,:]
						p4 = hdPoly[k+1,:]
						line2 = (p3[0], p3[1], p4[0]-p3[0], p4[1]-p3[1])
						ip = intersectLines(line1, line2)
						#plt.hold(True)
						if ip != None:
							tmp.append(ip)
			if tmp:
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
					
						self.tracks.append(tmp[l:l+2,:])
						#c = c + 1
		self.track_len=0
		for tr in self.tracks:
			self.track_len+=np.sqrt((tr[0,0]-tr[1,0])**2 + (tr[0,1]-tr[1,1])**2)



	def headlandGen(self):
		"""
		Generates boundary margins, otherwise known as headlands
		each headland has a distance of width/2 in between them
		"""

		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			# check order: if ord < 0 it is anti-clockwise
			ord = 0
			for j in range(0, len(poly)-1):
				ord += (poly[j+1,0]-poly[j,0])*(poly[j+1,1]+poly[j,1])

			if (ord < 0) and (i == 1):
				poly = poly[::-1,:]
			if (ord >= 0) and (i != 1) :
				poly = poly[::-1,:]

			if self.opw != 0:
				dist = [self.opw/2 + self.opw*r for r in range(0,self.nhp)]
				dist.append(self.opw*self.nhp)

			#generate headland paths
			x = poly[0:-1,0]  #poly[0:-1,0]
			y = poly[0:-1,1]  #poly[0:-1,1]
			dx2 = np.hstack((x[1:], x[0]))-x
			dy2 = np.hstack((y[1:], y[0]))-y
			dx1 = np.hstack((dx2[-1], dx2[0:-1]))
			dy1 = np.hstack((dy2[-1], dy2[0:-1]))
			s1 = np.sqrt(dx1**2+dy1**2)
			s2 = np.sqrt(dx2**2+dy2**2)
			den = s1*s2+dx1*dx2+dy1*dy2

			for j in range(0, len(dist)):
				X = x + dist[j]*(dy1*s2+dy2*s1)/den
				Y = y - dist[j]*(dx1*s2+dx2*s1)/den
				#close the polygon
				X = np.hstack((X,X[0]))
				Y = np.hstack((Y,Y[0]))
				#combine
				tmp = np.vstack((X,Y)).T		
				self.hd.append(tmp)
				

	def cluster(self,num_clusters=4):
		"""
		Divides the region of interest into sections/clusters by using K-means clustering
		
		args:
		num_clusters - Number of sections into which the region is divided into
		"""
		self.num_clusters=num_clusters
		np.random.seed(0)
		data = []
		for i in range(0, len(self.tracks)):
			data.append((self.tracks[i][0,0] , self.tracks[i][0,1]))
		data = np.asarray(data)

		k_means = KMeans(n_clusters=self.num_clusters,n_init=10)
		k_means.fit(data)
		self.labels = k_means.labels_
		self.cluster_centers = k_means.cluster_centers_

		k_means_labels = k_means.labels_
		k_means_cluster_centers = k_means.cluster_centers_
		k_means_labels_unique = np.unique(k_means_labels)
		n_clusters = len(k_means_cluster_centers)

		self.n_clusters = n_clusters

		# print("from clustering: ", len(k_means_labels_unique), n_clusters, k_means_labels, k_means_labels_unique)

		x_min, x_max = data[:, 0].min() - 1, data[:, 0].max() + 1
		y_min, y_max = data[:, 1].min() - 1, data[:, 1].max() + 1

		colors = iter(cm.rainbow(np.linspace(0,1,n_clusters)))

		# fig = plt.figure()
		for k, col in zip(list(range(n_clusters)), colors):
			my_members = k_means_labels == k
			cluster_center = k_means_cluster_centers[k]
	
	# def getRobotIntialPoition(self):
	# 	#get robot initial location (it should be two points to get heading angle)
	# 	plt.figure()
	# 	plt.clf()
	# 	#plot field's polygons
	# 	for i in  range(1, int(numPoly(self.data))+1):
	# 		poly = getPoly(self.data, i)
	# 		if i == 1:
	# 			plt.plot(poly[:,0] , poly[:,1] , 'b-')
	# 		else:
	# 			plt.plot(poly[:,0] , poly[:,1] , 'r-')
	# 	#get two points
	# 	tellme('You will define a robot intial poition and heading angle (by selecting two points),\n click to begin')

	# 	pl.waitforbuttonpress()
	# 	happy = False
	# 	while not happy:
	# 		pts = []
	# 		while len(pts) < 2:
	# 			tellme('Select 2 points with mouse')
	# 			pts = np.asarray( plt.ginput(2,timeout=-1) )
	# 			if len(pts) < 2:
	# 				tellme('Too few points, starting over')
	# 				# time.sleep(1) # Wait a second
	# 			plt.fill( pts[:,0], pts[:,1], 'r', lw=2 )
	# 			tellme('Happy? Key click for yes, mouse click for no')
	# 			happy = pl.waitforbuttonpress()


	# 	self.robotinitpos = pts
	# 	plt.close(1)

	def tsp_opt(self):
		"""
		Finds optimal order of visits to the clusters/sections using a TSP solver
		
		"""
		# self.getRobotIntialPoition()
		self.robotinitpos=self.data[0]
		#nodes locations using cluster centers and robot position at nodes number (n_cluster+1)
		nodes = np.vstack((self.robotinitpos,self.cluster_centers))

		
		n_nodes = len(nodes)
		dist = np.zeros((n_nodes, n_nodes))
		
		for i in range(n_nodes):
			for j in range(i+1, n_nodes):
				tmp = nodes[i,:]-nodes[j,:]
				sum_squared = numpy.dot(tmp.T , tmp)
				dist[i,j] = sqrt(sum_squared)
		
		for i in range(n_nodes):
			for j in range(i, n_nodes):
				dist[j][i] = dist[i][j]

		dist[:,0]=0 # solve as open tsp
		path,_=solve_tsp_dynamic_programming(dist)
		# path = solve_tsp( dist )
		self.path=path
		self.nodes=nodes				


	def trajGen(self, turning_radius=1.5):
		"""
		Generates complete trajectory for coverage
		Uses Dubins curves to connect waypoints for generating a smooth trajectory

		arg:
		turning_radius - radius of the Dubins curves
		"""

		step_size = 1

		self.turn_dist=0
		tracks_lower,tracks_upper = [], []
		for i in range(0, len(self.tracks)):
			tracks_lower.append((self.tracks[i][0,0] , self.tracks[i][0,1]))
			tracks_upper.append((self.tracks[i][1,0] , self.tracks[i][1,1]))
		tracks_lower = np.asarray(tracks_lower)
		tracks_upper = np.asarray(tracks_upper)

		labels = self.labels
		n_clusters = len(np.unique(labels))
		colors = iter(cm.rainbow(np.linspace(0,1,n_clusters)))

		p1 = self.robotinitpos
		p2 = self.robotinitpos
		theta0=0
		# theta0 = computeAngle(p1, p2)

		self.traj.append((self.robotinitpos[0], self.robotinitpos[1], 0))

		for i in self.path[1:]:
			# extract track i in optimal path
			my_members = labels == i-1
			start = tracks_lower[my_members]
			end = tracks_upper[my_members]

			track_1=(start[0]+end[0])/2
			track_n=(start[-1]+end[-1])/2
		
			dist_1=np.sqrt( (self.traj[-1][0]-track_1[0])**2 + (self.traj[-1][1]-track_1[1])**2 )
			dist_n=np.sqrt( (self.traj[-1][0]-track_n[0])**2 + (self.traj[-1][1]-track_n[1])**2 )
			
			if dist_n < dist_1:   # to select closer end of section first, select order based on this
				start=start[::-1]
				end=end[::-1]


			for j in range(len(start)):
				if j%2:
					p3 = start[j,:]
					p4 = end[j,:]
				else:
					p4 = start[j,:]
					p3 = end[j,:]

				theta1 = computeAngle(p3, p4)

				q0 = (p2[0], p2[1], theta0)
				q1 = (p3[0], p3[1], theta1)

				p_x,p_y,p_yaw,mode,_=plan_dubins_path(s_x=q0[0], s_y=q0[1], s_yaw=q0[2], g_x=q1[0], g_y=q1[1], g_yaw=q1[2], curvature=1/turning_radius,step_size=step_size)

				# if j==0:
				# 	plt.plot(p_x , p_y , 'y--')
				# else:
				# 	plt.plot(p_x , p_y , '-b')

				for k in range(len(p_x)):
					self.traj.append((p_x[k], p_y[k], 0))

				(p1,p2,theta0) = (p3,p4,theta1)

				self.turn_dist+=np.sum(np.sqrt((p_x[1:]-p_x[:-1])**2 + (p_x[1:]-p_x[:-1])**2))

		# q0 = (self.traj[-1][0], self.traj[-1][1], theta0)
		# q1 = (p1[0], p1[1], theta1)

		# for k in range(len(p_x)):
		# 	self.traj.append((p_x[k], p_y[k], 0))
		# 	self.traj.append((p1[0], p1[1], 0))
		# 	self.traj.append((p2[0], p2[1], 0))

		# for i, c in zip(list(range(n_clusters)), colors):
		# 	my_members = labels == i

		# 	plt.plot([tracks_lower[my_members][:,0], tracks_upper[my_members][:,0]],\
		# 			 [tracks_lower[my_members][:,1], tracks_upper[my_members][:,1]],'-b') #c=c)

		# for i in  range(1, int(numPoly(self.data))+1):
		# 	poly = getPoly(self.data, i)
		# 	if i == 1:
		# 		plt.plot(poly[:,0] , poly[:,1] , 'r-')
		# 	else:
		# 		plt.plot(poly[:,0] , poly[:,1] , 'r-')
	
		# plt.show()
		# print('Total distance = {}; Track length = {}'.format(self.track_len+self.turn_dist, self.track_len))

		self.hd_ll,self.latlon=[],[]
		
		for i in range(len(self.hd)):
			self.hd_ll=[]
			for j in range(len(self.hd[i])):
				lat,lon=utm.to_latlon(self.hd[i][j][0],self.hd[i][j][1],self.utmzone,self.utm_l)
				self.hd_ll.append([lat,lon])
			

		for i in range(len(self.traj)):
			latpos,lonpos=utm.to_latlon(self.traj[i][0],self.traj[i][1],self.utmzone,self.utm_l) 
			self.latlon.append([latpos,lonpos])