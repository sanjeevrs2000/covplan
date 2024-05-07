import unittest
import numpy as np
from covplan import field
from covplan import coverage_path_planner

class Test_PathPlan(unittest.TestCase):

    def test_ob(self):
        f=field.Field(filename='sample_area.txt',opw=10,nhp=1,theta=0)
        self.assertIsInstance(f,field.Field)

        f.headlandGen()
        f.trackGen()
        f.cluster()
        f.tsp_opt()
        f.trajGen(turning_radius=2)

        self.assertGreater(f.turn_dist,0)    


    def test_boundary(self):
        f=field.Field(filename='sample_area.txt',opw=10,nhp=1,theta=0)
        n_hd=len(f.data)
        self.assertGreater(n_hd,0)

    def test_plandist(self):
        f=field.Field(filename='sample_area.txt',opw=10,nhp=1,theta=0)
        f.headlandGen()
        f.trackGen()
        f.cluster()
        f.tsp_opt()
        f.trajGen(turning_radius=2)

        self.assertGreater(f.turn_dist,0)    



if __name__ == "__main__":
    unittest.main()