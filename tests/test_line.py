import unittest
import numpy as np
from covplan import lines

class Test_Lines(unittest.TestCase):

    def test_pointonline(self):
        line=np.array([0,0,1,0])
        p=lines.pointOnLine(line,[5])
        s=(p[0][0]==5) and (p[0][1]==0)
        self.assertTrue(s)
    
    def test_lineangle(self):
        line=[0,0,1,1]
        t=lines.lineAngle(line)
        self.assertAlmostEqual(t,np.pi/4)

    def test_intersectlines(self):
        line1=[-1,0,1,0]
        line2=[-1,-1,2,2]
        p=lines.intersectLines(line1,line2)
        s = (p[0]==0 and p[1]==0)
        self.assertIsNotNone(p)
        self.assertTrue(s)

if __name__ == "__main__":
    unittest.main()