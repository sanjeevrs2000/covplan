#!/usr/bin/python
from __future__ import division
from pygame import Rect
from numpy import *
from math import *
import numpy as np
import math, numpy.linalg, copy
import sys, string, os, traceback, time
import matplotlib.pyplot as plt
import pylab as pl


angular_unit = 1.0

def use_degrees():
    global angular_unit
    angular_unit = 180.0/math.pi

def use_radians():
    global angular_unit
    angular_unit = 1.0

def dot(x,y):
    return inner(x,y)

def abs2(x):
    return sum(x**2)

def normalized(x):
    return x/math.sqrt(abs2(x))

def orthogonalized_to(x,d):
    """Return a copy of x orthogonalized to d != 0"""
    d = normalized(d)
    return x - dot(x,d)*d

def dual(v):
    """Return two unit vectors orthogonal to v"""
    if abs2(v) > 1e-20:
        if v[0] < 0.7:
            n1 = normalized(orthogonalized_to(array([1,0,0],'d'),v))
        else:
            n1 = normalized(orthogonalized_to(array([0,1,0],'d'),v))
        n2 = cross(v,n1)
        return [n1,n2]
    else:
        return [array([1,0,0],'d'),array([0,1,0],'d')]

def qmul(q1,q2):
    """Take q1 and q2 to be quaternions, and multiply them accordingly"""
    v1 = q1[1:]
    v2 = q2[1:]
    x = q1[0]*v2 + q2[0]*v1 + cross(v1,v2)
    return array([q1[0]*q2[0]-dot(v1,v2),x[0],x[1],x[2]])

def qconj(q):
    qc = -q
    qc[0] *= -1
    return qc

def qrotate(q,v):
    qv = array([0,v[0],v[1],v[2]])
    return qmul(q,qmul(qv,qconj(q)))[1:]

def qrotor(axis,angle):
    axis = math.sin(angle/2)*normalized(axis)
    return array([math.cos(angle/2), axis[0], axis[1], axis[2]])

def pointOnLine(line, d):
	'''
	POINTONLINE create a point on a line at a given distance from line origin
	
	P = POINTONLINE(LINE, D) create the point located on the line LINE, and
	located at the distance D from origin of line.
	LINE has the form [x0 y0 dx dy].
	LINE and D should have the same number N of rows. The result will have
	N rows ans 2 column (x and y positions).
	'''

	angle = lineAngle(line);
	#point = np.zeros([len(d), 2])
	point = []
	for i in range(len(d)):
		#point[i,:] = [line[0]+d[i]*math.cos(angle), line[1]+d[i]*math.sin(angle)];
		point.append([line[0]+d[i]*math.cos(angle), line[1]+d[i]*math.sin(angle)]);
	return np.array(point, np.float)

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
	theta = math.fmod( math.atan2(line[3],line[2]) + 2 * math.pi, 2 * math.pi);
	return theta;

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
        print "Oops!  the data file has only %d polygons and there is no polygon with number %d!" %(nop,n)
        poly = []

    return poly

def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

class Point:
    def __init__(self, *xy):
        """Create a point from a list of 2 coordinates or 2 individual
        coordinates."""
        if len(xy) == 2:
            self.r = array(xy,'d')
        elif len(xy) == 1 and len(xy[0]) == 2:
            self.r = array(xy[0],'d')
        else:
            raise TypeError("Invalid arguments to Point()")
    def moved(self, m):
        return m.on_point(self)
    def distance_to(self, obj):
        if isinstance(obj,Point):
            return math.sqrt(abs2(self.r - obj.r))
        else:
            raise TypeError("Invalid type in Point.distance_to()")
    def __str__(self):
        return "(%g, %g)" % tuple(map(float,self.r))
    def __repr__(self):
        return "Point(%g, %g)" % tuple(map(float,self.r))
    def coordinates(self):
        return map(float,self.r)
    def projected_on(self, obj):
        if isinstance(obj,Line):
            return Point(obj.r + dot(obj.t,self.r - obj.r)*obj.t)
        elif isinstance(obj,Plane):
            dr = self.r - obj.r
            return Point(obj.r + orthogonalized_to(dr,obj.n))
        else:
            raise TypeError("Cannot project point onto object")
    def distance_to(self, obj):
        if isinstance(obj,Point):
            return math.sqrt(abs2(self.r - obj.r))
        else:
            p = self.projected_on(obj)
            return self.distance_to(p)
    def midpoint_to(self,obj):
        """Return a point in the middle of the shortest line connecting this and obj."""
        if isinstance(obj,Point):
            return Point(0.5*(self.r + obj.r))
        else:
            return obj.midpoint_to(self)

class Line:
    def __init__(self, *points):
        """Create an infinite line from at least two points.
        Accepts either two points or a list of points. If more than
        two points are given the line will be a least square fit of
        the point set, in which case the (sign of the) direction is
        undefined."""
        if len(points) == 1:
            points = points[0]
        if len(points) == 2:
            self.r = array(points[0].r)
            self.r2 = array(points[1].r)
            self.t = normalized(self.r2 - self.r)
        else:
            raise RuntimeError("Too few arguments to Line()")
    def points(self):
        """Return two points defining the line"""
        return [Point(self.r),Point(self.r2)]
    def moved(self, m):
        p = self.points()
        return Line(m.on_point(p[0]),m.on_point(p[1]))
    def projected_on(self, plane):
        p = self.points()
        return Line(p[0].projected_on(plane),
                    p[1].projected_on(plane))
    def distance_to(self, obj):
        if isinstance(obj,Point):
            return obj.distance_to(self)
        elif isinstance(obj,Line):
            d = obj.r - self.r
            n = cross(self.t,obj.t)
            if abs2(n) < 1e-16: # parallel lines
                return math.sqrt(abs2(cross(d,self.t)))
            else:
                return abs(dot(d,n))/math.sqrt(abs2(n))
        else:
            # Line-plane distance is only non-zero for exactly parallel objects
            # Because of numerical errors this is unlikely to happen, so always fail.
            raise RuntimeError("Will not calculate line-plane distance")
    def angle_to(self, obj):
        if isinstance(obj,Line):
            return angular_unit*math.acos(min(1,abs(dot(self.t,obj.t))))
        elif isinstance(obj,Plane):
            return angular_unit*(math.pi/2 - math.acos(min(1,abs(dot(self.t,obj.n)))))
        else:
            raise RuntimeError("Cannot calculate angle to object of this type")
    def midpoint_to(self,obj):
        """Return a point in the middle of the shortest line connecting this and obj."""
        if isinstance(obj,Point):
            return obj.midpoint_to(obj.projected_on(self))
        elif isinstance(obj,Line):
            d = obj.r - self.r
            t1t2 = dot(self.t,obj.t)
            if abs(abs(t1t2)-1) < 1e-12: #parallel case                
                d = orthogonalized_to(d,self.t)
                return Point(self.r + 0.5*d)
            else:
                t1d = dot(d,self.t)
                t2d = dot(d,obj.t)
                s = (t1t2*t2d - t1d)/(t1t2**2-1)
                u = (t1t2*t1d - t2d)/(t1t2**2-1)
                return Point(0.5*(obj.r + u*obj.t  + self.r + s*self.t))                
        else:
            return obj.midpoint_to(self)
    def __repr__(self):
        p = self.points()
        return "Line(%s, %s)" % (repr(p[0]),repr(p[1]))
    def __str__(self):
        return repr(self)
    def dual(self):
        """Return a plane such that plane.normal() == self"""
        d = dual(self.t)
        return Plane(Point(self.r),Point(self.r + d[0]), Point(self.r + d[1]))
 
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


        print "file Name: ", filename
        for line in open(filename, 'r'):
        	self.data.append(tuple(line.strip().split()))
        #print data, type(data)
        #print data[0], data[0][0], data[0][:], len(data), data[len(data)-1]
        self.data = np.array(self.data, np.float)
        #print self.data[0,:]

    def showField(self):
        #plt.figure()
        plt.hold(True)
        plt.axis('equal')
        plt.axis('off')

        #plot field's polygons
        for i in  range(1, int(numPoly(self.data))+1):
    		poly = getPoly(self.data, i)
    		if i == 1:
    			plt.plot(poly[:,0] , poly[:,1] , 'b-')
    		else:
    			plt.plot(poly[:,0] , poly[:,1] , 'r-')

        #plot field's tracks
        for i in range(0, len(self.tracks)):
            plt.plot(self.tracks[i][:,0] , self.tracks[i][:,1] ,'b-',\
                self.tracks[i][0,0] , self.tracks[i][0,1] ,'g.',\
                self.tracks[i][1,0] , self.tracks[i][1,1] ,'r.')

        #plot field's headland paths
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
    	print "Driving Angle: ", self.theta
    	Xline = [m[0], m[1], 1, np.tan(self.theta)]
    	Xpline = [m[0], m[1], 1, np.tan(self.theta+pi/2)]
    	p = pointOnLine(Xpline, [-dmax/2])
    	Xpline = [p[0,0], p[0,1], 1, np.tan(self.theta+pi/2)]

    	base_points = pointOnLine(Xpline, pl.frange(0, dmax, self.opw))
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
                    poly = self.hd[j-1]
                    print "poly: ", type(poly), poly
                    for k in range(0, len(poly)-1):
                        p3 = poly[k,:]
                        p4 = poly[k+1,:]
                        line2 = (p3[0], p3[1], p4[0]-p3[0], p4[1]-p3[1])
                        ip = intersectLines(line1, line2)
                        plt.hold(True)
                        if ip != None:
                            tmp.append(ip)
            '''    
            for j in range(1, int(numPoly(self.data))+1):
                poly = getPoly(self.data, j)
                for k in range(0, len(poly)-1):
                    p3 = poly[k,:]
                    p4 = poly[k+1,:]
                    line2 = (p3[0], p3[1], p4[0]-p3[0], p4[1]-p3[1])
                    ip = intersectLines(line1, line2)
                    plt.hold(True)
                    if ip != None:
                        tmp.append(ip)
            '''
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

                #print tmp[:,0]
                #plt.plot(tmp[:,0],tmp[:,1],'k.', tmp[:,0],tmp[:,1],'k-')
            #else:
            #    print "tmp is empt

    def headlandGen(self):
        # Generates headland paths by expansion or contraction factor. in this function; polygon points are 
        # moved outward or inward by an arthogonal distnce d to the side: inwards if polygon points are in clock-wise 
        # direction and vice-versa (inward/outward polygon offseting)
        plt.hold(True)
        #plt.axis('equal')

        print self.nhp
        #load field polygons
        for i in  range(1, int(numPoly(self.data))+1):
            poly = getPoly(self.data, i)
            # check order: if ord < 0 it is anti-clockwise
            ord = 0
            for j in range(0, len(poly)-1):
                ord += (poly[j+1,0]-poly[j,0])*(poly[j+1,1]+poly[j,1])

            print "ord is: ", np.sign(ord)

            if ord < 0 and i == 1:
                # outside polygon must be ordered in clockwise direction
                print "outside poly is reversed"
                poly = poly[::-1,:]
            if ord >= 0 and i != 1:
                # inside polygons must be ordered in anti-clockwise direction
                print "inside poly is reversed"
                poly = poly[::-1,:]

            #generate n paths: (n+1) paths will be generated at distance d
            #        dis = [d/2+d*[0:n-1] d*n];
            if self.opw != 0:
                dist = [self.opw/2 + self.opw*r for r in range(0,self.nhp)]
                #dist = [dist, self.opw*self.nhp]
                dist.append(self.opw*self.nhp)
                print "dist: ", dist

            #generate headland paths
            x = poly[0:-1,0]  #poly[0:-1,0]
            y = poly[0:-1,1]  #poly[0:-1,1]
            plt.plot(x,y,'k-')

            print len(poly), len(x)
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
                print j, dist[j]
                X = x + dist[j]*(dy1*s2+dy2*s1)/den
                Y = y - dist[j]*(dx1*s2+dx2*s1)/den
                #close the polygon
                X = np.hstack((X,X[0]))
                Y = np.hstack((Y,Y[0]))
                #combine
                tmp = np.vstack((X,Y)).T
                self.hd.append(tmp)
                plt.plot(tmp[:,0],tmp[:,1],'g-')

        plt.show()            

        
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

def closest_point(arr, x, y):
    print "function", arr, x, y
    print "arr[:, 0] - x  = ", arr[:, 0], arr[:, 0] - x, arr[:, 1] - y
    dist = (arr[:, 0] - x)**2 + (arr[:, 1] - y)**2
    index = np.argsort(dist,axis=-1, kind='mergesort')
    print dist, index
    return arr[index,:]

def main():

    ''' load field polygons from a file and plot it'''
    f = Field('mfield.txt', 10, 2, 45)
    #print f.opw
    f.headlandGen()
    f.trackGen()
    f.showField()



    '''
    data = []
    for line in open('mfield.txt', 'r'):
        data.append(tuple(line.strip().split()))
    print data, type(data)
    print data[0], data[0][0], data[0][:], len(data), data[len(data)-1]
    data = np.array(data, np.float)
    print data, data[0,:]
    '''

    ### check 
    #p1 = Point(1,1)
    #p2 = Point(1,0)
    #print p1.coordinates()
    #checkf(p1.distance_to(p2),sqrt(2))

    #L = Line(p1,p2)
    #print L.points()

    #L1 = line([0,1], [2,3])
    #L2 = line([2,3], [0,4])

    #R = intersection(L1, L2)
    #if R:
    	#print "Intersection detected:", R
    #else:
    	#print "No single intersection point detected"

    # find intersection between field tracks and field polygons
    p1 = [0,0]
    p2 = [5,5]
    p3 = [0,5]
    p4 = [5,0]
    line1 = (p1[0], p1[1], p2[0]-p1[0], p2[1]-p1[1])
    line2 = (p3[0], p3[1], p4[0]-p3[0], p4[1]-p3[1])
    print line1, line2
    ip = intersectLines(line1, line2)
    print ip

    print "done."


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print '>> Exiting...'
    except Exception, err:
        print traceback.format_exc(err)