import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/..")

import unittest 
from PathPlanning.VisibilityRoadMap import visibility_road_map as m

print(__file__)


class Test(unittest.TestCase):

    def test1(self):
        m.show_animation = True
        m.main()


if __name__ == '__main__':
    unittest.main()

