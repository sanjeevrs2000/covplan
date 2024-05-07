import unittest
import numpy
from covplan import pathplan, find_min

class Test_PathPlan(unittest.TestCase):

    def test_plan(self):
        plan=pathplan(input_file='sample_area.txt',width=10,theta=0,visualize=False)
        self.assertGreater(len(plan),0)

    def test_minangle(self):
        theta=find_min(input_file='sample_area.txt')
        self.assertGreaterEqual(theta,0)
        self.assertGreaterEqual(180,theta)


if __name__ == "__main__":
    unittest.main()