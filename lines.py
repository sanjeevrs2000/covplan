from numpy import *
from math import *
import numpy as np
import math, numpy.linalg, copy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
import itertools
from pylab import ginput


def createLine(p1, p2):
	'''
	CREATELINE create a line from two points (p1 and p2) and return it as p1 (a point belonging to the line), 
	and dx, dy (direction vector of the line) where dx=p1(:,1)-p2(:,1) and dy=p1(:,2)-p2(:,2).

	Line is represented in a parametric form : [x0 y0 dx dy]
	  x = x0 + t*dx
	  y = y0 + t*dy;

	 the line is represented by first one is a point belonging to the linethey are x0, y0 (point belongng to line) and 
	 dx, dy (direction vector of the line).

	l = CREATELINE(p1, p2) return the line going through the two given points.
	'''
	#line = np.array( [p1[0,0], p1[0,1], p2[0,0]-p1[0,0], p2[0,1]-p1[0,1] ])
	#p1 = np.array(p1)
	#p2 = np.array(p2)
	line = []#np.zeros([p1.shape[0],4])
	if p1.size == 2:
		line = [p1[0], p1[1], p2[0]-p1[0], p2[1]-p1[1]]
	else:
		for i in range(p1.size/2):
			line.append([p1[i,0], p1[i,1], p2[i,0]-p1[i,0], p2[i,1]-p1[i,1]])
	#line = np.array([p1,np.subtract(p2,p1)])
	return line

def pointOnLine(line, d):
	'''
	POINTONLINE create a point on a line at a given distance from line origin
	
	P = POINTONLINE(LINE, D) create the point located on the line LINE, and
	located at the distance D from origin of line.
	LINE has the form [x0 y0 dx dy].
	LINE and D should have the same number N of rows. The result will have
	N rows ans 2 column (x and y positions).
	'''

	angle = lineAngle(line)
	#point = np.zeros([len(d), 2])
	point = []
	for i in range(len(d)):
		#point[i,:] = [line[0]+d[i]*math.cos(angle), line[1]+d[i]*math.sin(angle)];
		point.append([line[0]+d[i]*math.cos(angle), line[1]+d[i]*math.sin(angle)])
	return point

def pointOnLine(line, d):
	'''
	POINTONLINE create a point on a line at a given distance from line origin
	
	P = POINTONLINE(LINE, D) create the point located on the line LINE, and
	located at the distance D from origin of line.
	LINE has the form [x0 y0 dx dy].
	LINE and D should have the same number N of rows. The result will have
	N rows ans 2 column (x and y positions).
	'''

	angle = lineAngle(line)
	#point = np.zeros([len(d), 2])
	point = []
	for i in range(len(d)):
		#point[i,:] = [line[0]+d[i]*math.cos(angle), line[1]+d[i]*math.sin(angle)];
		point.append([line[0]+d[i]*math.cos(angle), line[1]+d[i]*math.sin(angle)])
	return np.array(point, dtype=np.double)

def lineAngle(line):
	'''
	LINEANGLE return angle between lines
	a = LINEANGLE(line) return the angle between horizontal, right-axis
	and the given line. Angle is given in radians, between 0 and 2*pi,
	in counter-clockwise direction.
	see createLine for more details on line representation.
	'''
	# one line
	#theta = math.fmod(math.atan2(line[3], line[2]) + 2*pi, 2*pi);
	#print line
	theta = math.fmod( math.atan2(line[3],line[2]) + 2 * math.pi, 2 * math.pi)
	return theta

def numPoly(data):
	index = np.nonzero(np.isnan(data)) #find(np.isnan(data)==True)
	n = len(index[0])/2
	return n 

def getPoly(data, n):
	# this function returns polygon n stored in data
	index = np.nonzero(np.isnan(data)) #find(np.isnan(data)==True)
	nop = len(index[0])/2

	if n == 1:
		poly = data[0:index[0][0],:]
	elif n <= nop and n > 0:
		poly = data[index[0][2*n-3]+1:index[0][2*n-2],:]
	else:
		print("Oops!  the data file has only %d polygons and there is no polygon with number %d!" %(nop,n))
		poly = []

	return poly
def intersectLines(line1, line2):
	#INTERSECTLINES return all intersection points of N lines in 2D
	x1 =  line1[0]
	y1 =  line1[1]
	dx1 = line1[2]
	dy1 = line1[3]

	x2 =  line2[0]
	y2 =  line2[1]
	dx2 = line2[2]
	dy2 = line2[3]

	#indices of parallel lines
	par = abs(dx1*dy2-dx2*dy1)<1e-14
	#print "par: ", par

	#indices of colinear lines
	col = abs((x2-x1)*dy1-(y2-y1)*dx1)<1e-14 and par
	#print "col: ", col

	#compute intersection points

	x0 = ((y2-y1)*dx1*dx2 + x1*dy1*dx2 - x2*dy2*dx1) / (dx2*dy1-dx1*dy2)
	y0 = ((x2-x1)*dy1*dy2 + y1*dx1*dy2 - y2*dx2*dy1) / (dx1*dy2-dx2*dy1)
	point = ((x0, y0),)
	#print "point: ", point, point[0]

	#check if the point is located on line2
	p3 = (x2, y2)
	p4 = (x2 + dx2, y2 + dy2)
	
	for p in point:
		if p is not None:
			
			temp =  p3[0] if p3[0]>p4[0] else p4[0]
			if p[0] > (temp + 1e-14):
				continue

			temp =  p3[0] if p3[0]<p4[0] else p4[0]
			if p[0] < (temp - 1e-14):
				continue

			temp =  p3[1] if p3[1]>p4[1] else p4[1]
			if p[1] > (temp + 1e-14):
				continue

			temp =  p3[1] if p3[1]<p4[1] else p4[1]
			if p[1] < (temp - 1e-14):
				continue

			return p
		else:
			return None

def rmCrossovers(poly, N):
	# remove any crossovers up to N=5
	# generate xPolygon
	if sum(poly[-1, :] == poly[0,:]) != 2:
		xpoly = np.vstack((poly, poly[0:,:]))
	else:
		xpoly = np.vstack((poly, poly[1:,:]))

	i = 1
	cross = False
	while (not cross) and (i+N <= len(poly)):
		p1 = xpoly[i,:]
		p2 = xpoly[i+1,:]
		p3 = xpoly[i+N,:]
		p4 = xpoly[i+N+1,:]
		line1 = (p1[0], p1[1], p2[0]-p1[0], p2[1]-p1[1])
		line2 = (p3[0], p3[1], p4[0]-p3[0], p4[1]-p3[1])
		ip = intersectLines(line1, line2)
		ip = ((ip),)
	
		#if pointOnLine2(p, p1, p2) && pointOnLine2(p, p3, p4)
		for p in ip:
			if p is not None:
				p = np.asarray(p)
				#check if ip is located on line1(p1,p2)
				temp =  p1[0] if p1[0]>p2[0] else p2[0]
				if p[0] > (temp + 1e-14):
					continue

				temp =  p1[0] if p1[0]<p2[0] else p2[0]
				if p[0] < (temp - 1e-14):
					continue

				temp =  p1[1] if p1[1]>p2[1] else p2[1]
				if p[1] > (temp + 1e-14):
					continue

				temp =  p1[1] if p1[1]<p2[1] else p2[1]
				if p[1] < (temp - 1e-14):
					continue

				#check if ip is located on line2(p3,p4)
				temp =  p3[0] if p3[0]>p4[0] else p4[0]
				if p[0] > (temp + 1e-14):
					continue

				temp =  p3[0] if p3[0]<p4[0] else p4[0]
				if p[0] < (temp - 1e-14):
					continue

				temp =  p3[1] if p3[1]>p4[1] else p4[1]
				if p[1] > (temp + 1e-14):
					continue

				temp =  p3[1] if p3[1]<p4[1] else p4[1]
				if p[1] < (temp - 1e-14):
					continue

				#if there is an intersection point
				poly_new = np.delete(poly, list(range(i+1,i+N)), axis = 0)  #range(i+1,i+N-1)
				poly = poly_new
				poly = np.vstack((poly[0:i,:], p, poly[i+N:,:]))
				poly = rmCrossovers(poly,N)
				cross = not cross
		i += 1

	if sum(poly[-1, :] == poly[0,:]) != 2:
		poly = np.vstack((poly, poly[0,:]))
	return poly

def rmGabs(poly, width, N):
	#remove any crossovers up to N=5
	# generate xPolygon
	if sum(poly[-1, :] == poly[0,:]) != 2:
		xpoly = np.vstack((poly, poly[0:,:]))
	else:
		xpoly = np.vstack((poly, poly[1:,:]))

	i = 1
	cross = False
	while (not cross) and (i <= len(poly)):
		p1 = xpoly[i,:]
		p2 = xpoly[i+1,:]
		p3 = xpoly[i+N,:]
		p4 = xpoly[i+N+1,:]
		line1 = (p1[0], p1[1], p2[0]-p1[0], p2[1]-p1[1])
		line2 = (p3[0], p3[1], p4[0]-p3[0], p4[1]-p3[1])
		ip = intersectLines(line1, line2)
		ip = ((ip),)
		for p in ip:
			if p is not None:
				p = np.asarray(p)
				gab1 = numpy.linalg.norm(np.subtract(p1,p)) + numpy.linalg.norm(np.subtract(p2,p)) - numpy.linalg.norm(np.subtract(p1,p2))
				gab2 = numpy.linalg.norm(np.subtract(p3,p)) + numpy.linalg.norm(np.subtract(p4,p)) - numpy.linalg.norm(np.subtract(p3,p4))

				if (intersectPointOnLine(p, p1, p2) and gab1 > width) | \
				   (intersectPointOnLine(p, p3, p4) and gab2 > width):
					#print "a gab is discovered: ", gab1, gab2
					#if there is a gab
					poly_new = np.delete(poly, list(range(i+1,i+N)), axis = 0)  #range(i+1,i+N-1)
					poly = poly_new
					poly = np.vstack((poly[0:i,:], p, poly[i+N:,:]))
					poly = rmGabs(poly, width, N)
					cross = not cross
		i += 1
	if sum(poly[-1, :] == poly[0,:]) != 2:
		poly = np.vstack((poly, poly[0,:]))
	return poly

def intersectPointOnLine(p, p1, p2):
	#this function checks if an intersection point p is located on a line connecting points p1 and p2
	temp =  p1[0] if p1[0]>p2[0] else p2[0]
	if p[0] > (temp + 1e-14):
		return False
	temp =  p1[0] if p1[0]<p2[0] else p2[0]
	if p[0] < (temp - 1e-14):
		return False
	temp =  p1[1] if p1[1]>p2[1] else p2[1]
	if p[1] > (temp + 1e-14):
		return False
	temp =  p1[1] if p1[1]<p2[1] else p2[1]
	if p[1] < (temp - 1e-14):
		return False

	return True

def tellme(s):
	print(s)
	plt.title(s,fontsize=12)
	plt.draw()

def computeAngle(p1, p2):
	# Compute x/y distance
	(x1, y1) = (p1[0], p1[1])
	(x2, y2) = (p2[0], p2[1])
	(dx, dy) = (x2-x1, y2-y1)
	# Compute the angle
	#angle = math.atan2(float(dy),float(dx))
	#angle = math.fmod( math.atan2(float(dy),float(dx)) + 2 * math.pi, 2 * math.pi);
	angle = math.fmod( math.atan2(dy,dx) + 2 * math.pi, 2 * math.pi)


	# The angle is in radians (-pi/2 to +pi/2).  If you want degrees, you need the following line
	#angle *= 180/math.pi
	# Now you have an angle from -90 to +90.  But if the player is below the turret,
	# you want to flip it
	#if dy < 0:
	#    angle += 180
	return angle
