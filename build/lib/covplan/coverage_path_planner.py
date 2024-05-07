from covplan.field import Field
from scipy import optimize

def pathplan(input_file,width=10,num_hd=0,theta=0,num_clusters=3,radius=2,visualize=True):
    
    """Generates the required path for coverage with given parameters
    
    args:
    input_file - location of input file with the AoI coordinates
    width - the width or distance between the parallel tracks
    num_hd - number of headland polygons or boundary margins around the AoI
    theta - the angle of the parallel tracks wrt the X-axis 
    num_clusters - number of sections into which the AoI is divided
    radius - radius of the Dubins curves used to connect waypoints
    
    returns - list with the latlon coordinates describing the path
    """

    f = Field(input_file, width, num_hd, theta)
    f.headlandGen()
    f.trackGen()
    f.cluster(num_clusters)
    f.tsp_opt()
    f.trajGen(radius)
    print('Total distance = {}; Track length = {}'.format(f.track_len+f.turn_dist, f.track_len))

    if visualize==True:
        f.showField()

    return f.latlon

def find_min(input_file,width=10,num_hd=0,num_clusters=4,radius=2,verbose=False):

    """Finds the angle that minimizes the overall path length for given parameters
    
    args:
    input_file - location of input file with the AoI coordinates
    width - the width or distance between the parallel trakcs
    num_hd - number of headland polygons or boundary margins around the AoI
    num_clusters - number of sections into which the AoI is divided
    radius - radius of the Dubins curves used to connect waypoints

    returns - angle that corresponds to minimum path length
    """

    def get_dist(angle):
        f = Field(input_file, width, num_hd, angle)
        # f = Field('mfield.txt', 10, 0, 90)
        f.headlandGen()
        f.trackGen()
        f.cluster(num_clusters)
        f.tsp_opt()
        f.trajGen(radius)    
        return f.turn_dist+f.track_len
    

    min=optimize.dual_annealing(get_dist,maxiter=100,args=(), bounds=[(0,180)])
    
    if verbose:
        print('The minimum path length is: ',min.fun)
        print('The angle for the path with minimum length is: ', min.x)
    
    return min.x


