from numpy import *
from math import *
import numpy as np
import math, numpy.linalg
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pylab as pl
from sklearn.cluster import KMeans
from tsp_solver.greedy import solve_tsp
from python_tsp.exact import solve_tsp_dynamic_programming
from dubins_path_planner import plan_dubins_path
from lines import computeAngle, pointOnLine, intersectLines, intersectPointOnLine
from lines import getPoly, numPoly, tellme,rmCrossovers, rmGabs
from convert_coordinates import LLtoUTM, UTMtoLL
import folium
import utm

class Field:
	def __init__(self, filename, opw, nhp, theta, utm_=False, utmzone=None):
		self.opw = opw
		self.nhp = nhp
		self.theta = math.fmod( theta * pi/180+ 2 * pi, 2 * pi)
		self.data = []  # data file containing field polygons
		self.tracks = [] # field tracks
		self.hd = []     #headland paths
		self.labels = [] # labels of track clusters of Kmeans
		self.num_clusters = 0 #number of clusters
		self.cluster_centers = [] # cluster centers of Kmeans
		self.robotIntiPosition = [] #robot initial position
		self.path = []  # optimal path to followed by the robot
		self.traj = []  # robot trajectory using Dubins curves
		self.utm=utm_
		self.utm_l=None
		self.utmzone=utmzone

		print("file Name: ", filename)
		for line in open(filename, 'r'):
			self.data.append(tuple(line.strip().split()))

		self.data = np.array(self.data,dtype=np.double)		
		# print(self.data)
		
		if (not self.utm):
			for i in range(len(self.data)):
				if np.isnan(self.data[i][0]):
					continue
				# z,ux,uy=LLtoUTM(23,self.data[i][0], self.data[i][1])
				ux,uy,z,l=utm.from_latlon(self.data[i][0],self.data[i][1])
				self.utmzone=z
				self.utm_l=l
				self.data[i]=[ux,uy]
		

	def showField(self):

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
		
		if self.robotIntiPosition.any():
			plt.fill( self.robotIntiPosition[:,0], self.robotIntiPosition[:,1], 'r', lw=2 )
		
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
		print("len(self.hd):", len(self.hd))
		for i in range(0, len(self.hd)):
			if (i+1)%(self.nhp+1):
				plt.plot(self.hd[i][:,0] , self.hd[i][:,1] ,'g-')
			else:
				plt.plot(self.hd[i][:,0] , self.hd[i][:,1] ,'g-.')
		
		plt.show()

		hd_ll,latlon=[],[]


		map=folium.Map(zoom_start=20)
		for i in range(len(self.hd)):
			hd_ll=[]
			for j in range(len(self.hd[i])):
				# lat,lon=UTMtoLL(23,self.hd[i][j][1], self.hd[i][j][0])
				lat,lon=utm.to_latlon(self.hd[i][j][0],self.hd[i][j][1],self.utmzone,self.utm_l,northern=True)
				hd_ll.append([lat,lon])
			
			folium.PolyLine(hd_ll,color='red').add_to(map)

		for i in range(len(self.traj)):
			# f.writelines((str(self.tracks[i][0])))
			# latpos,lonpos=UTMtoLL(23,self.traj[i][1],self.traj[i][0])
			latpos,lonpos=utm.to_latlon(self.traj[i][0],self.traj[i][1],self.utmzone,self.utm_l,northern=True) 
			latlon.append([latpos,lonpos])

		folium.Polygon(latlon,color='blue').add_to(map)
		map.show_in_browser()

	def trackGen(self):
		poly = getPoly(self.data, 1)  # later on we will work on headland paths
	
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
		m = [(xmin+xmax)/2, (ymin+ymax)/2]

		print("Driving Angle: ", self.theta)
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

		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			# check order: if ord < 0 it is anti-clockwise
			ord = 0
			for j in range(0, len(poly)-1):
				ord += (poly[j+1,0]-poly[j,0])*(poly[j+1,1]+poly[j,1])

			#print "ord is: ", np.sign(ord)

			if (ord < 0) and (i == 1):
				poly = poly[::-1,:]
			if (ord >= 0) and (i != 1) :
				poly = poly[::-1,:]

			if self.opw != 0:
				dist = [self.opw/2 + self.opw*r for r in range(0,self.nhp)]
				#dist = [dist, self.opw*self.nhp]
				dist.append(self.opw*self.nhp)
				#print "dist: ", dist

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
				#print j, dist[j]
				X = x + dist[j]*(dy1*s2+dy2*s1)/den
				Y = y - dist[j]*(dx1*s2+dx2*s1)/den
				#close the polygon
				X = np.hstack((X,X[0]))
				Y = np.hstack((Y,Y[0]))
				#combine
				tmp = np.vstack((X,Y)).T
				if i == 1:
					tmp = rmCrossovers(rmCrossovers(rmCrossovers(rmCrossovers(rmCrossovers(tmp, 6), 5), 4), 3), 2)
					tmp = rmGabs(rmGabs(rmGabs(rmGabs(tmp, self.opw, 2), self.opw, 3), self.opw, 4), self.opw, 5)
		
				self.hd.append(tmp)
				

	def cluster(self,num_clusters=4):

		self.num_clusters=num_clusters
		np.random.seed(42)
		data = []
		for i in range(0, len(self.tracks)):
			data.append((self.tracks[i][0,0] , self.tracks[i][0,1]))
		data = np.asarray(data)

		k_means = KMeans(n_clusters=self.num_clusters)
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

		fig = plt.figure()
		for k, col in zip(list(range(n_clusters)), colors):
			my_members = k_means_labels == k
			cluster_center = k_means_cluster_centers[k]
	
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
					# time.sleep(1) # Wait a second
				plt.fill( pts[:,0], pts[:,1], 'r', lw=2 )
				tellme('Happy? Key click for yes, mouse click for no')
				happy = pl.waitforbuttonpress()


		self.robotIntiPosition = pts
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
		self.getRobotIntialPoition()

		#nodes locations using cluster centers and robot position at nodes number (n_cluster+1)
		nodes = np.vstack((self.cluster_centers, self.robotIntiPosition[0,:]))

		
		n_nodes = len(nodes)
		dist = np.zeros((n_nodes, n_nodes))
		
		for i in range(n_nodes):
			for j in range(i+1, n_nodes):
				#print "i,j:",i,j
				tmp = nodes[i,:]-nodes[j,:]
				sum_squared = numpy.dot(tmp.T , tmp)
				dist[i,j] = sqrt(sum_squared)
		
		for i in range(n_nodes):
			for j in range(i, n_nodes):
				dist[j][i] = dist[i][j]

		path,_=solve_tsp_dynamic_programming(dist)
		# path = solve_tsp( dist )
		
		ind = path.index(n_nodes-1)
		path = np.roll(path, -ind)
		path = np.hstack((path, n_nodes-1))
		self.path = path
				

	def trajGen(self, turning_radius=1.5):
		
		step_size = 1
		plt.figure()
		plt.axis('equal')
		plt.axis('off')

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

		p1 = self.robotIntiPosition[0]
		p2 = self.robotIntiPosition[1]
		theta0 = computeAngle(p1, p2)


		for i in range(len(self.robotIntiPosition)):
			self.traj.append((self.robotIntiPosition[i][0], self.robotIntiPosition[i][1], 0))

		for i in self.path[1:]:
			# extract track i in optimal path
			my_members = labels == i
			start = tracks_lower[my_members]
			end = tracks_upper[my_members]

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

				plt.plot(p_x , p_y , '-r')

				self.traj.append((p1[0], p1[1], 1))
				for k in range(len(p_x)):
					self.traj.append((p_x[k], p_y[k], 0))
				self.traj.append((p4[0], p4[1], 1))

				(p1,p2,theta0) = (p3,p4,theta1)

				self.turn_dist+=np.sum(np.sqrt((p_x[1:]-p_x[:-1])**2 + (p_x[1:]-p_x[:-1])**2))

		p3 = self.robotIntiPosition[1]
		p4 = self.robotIntiPosition[0]
		theta1 = computeAngle(p3, p4)

		q0 = (self.traj[-1][0], self.traj[-1][1], theta0)
		q1 = (p1[0], p1[1], theta1)

		plt.plot(p_x, p_y, '-r')

		for k in range(len(p_x)):
			self.traj.append((p_x[k], p_y[k], 0))
			self.traj.append((p1[0], p1[1], 0))
			self.traj.append((p2[0], p2[1], 0))

		for i, c in zip(list(range(n_clusters)), colors):
			my_members = labels == i

			plt.plot([tracks_lower[my_members][:,0], tracks_upper[my_members][:,0]],\
					 [tracks_lower[my_members][:,1], tracks_upper[my_members][:,1]], c=c)

		for i in  range(1, int(numPoly(self.data))+1):
			poly = getPoly(self.data, i)
			if i == 1:
				plt.plot(poly[:,0] , poly[:,1] , 'b-')
			else:
				plt.plot(poly[:,0] , poly[:,1] , 'r-')
	
		# plt.show()

