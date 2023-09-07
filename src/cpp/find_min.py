from scipy import optimize
from field import Field

def find_min(input_file,width=10,num_hd=0,num_clusters=4,radius=2):

    def get_dist(angle):
        f = Field(input_file, width, num_hd, angle)
        # f = Field('mfield.txt', 10, 0, 90)
        f.headlandGen()
        f.trackGen()
        f.cluster(num_clusters)
        f.tsp_opt()
        f.trajGen(radius)    
        return f.turn_dist+f.track_len
    
    min=optimize.dual_annealing(get_dist,maxiter=100,args=(), bounds=[(0,90)])
    print(min.values())


