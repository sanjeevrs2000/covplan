from scipy import optimize
from field import Field
from lines import *
import matplotlib.pyplot as plt

def find_opt_angle(angle):
    
    f = Field('../../files/mfield.txt', 10, 0, angle, utm_=True)
    # f = Field('mfield.txt', 10, 0, 90)
    f.headlandGen()
    f.trackGen()
    f.cluster(1)
    f.tsp_opt()
    f.trajGen()    
    return f.turn_dist+f.track_len

# find_opt_angle(135)
# min=optimize.minimize(find_opt_angle,x0=0, args=(), bounds=[(0,90)], method='BFGS')
min=optimize.dual_annealing(find_opt_angle,maxiter=100,args=(), bounds=[(0,90)])
print(min)
print(min.values())
# plt.plot(min.x,min.fun)
