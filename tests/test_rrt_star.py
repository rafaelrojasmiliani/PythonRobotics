import os
import sys
from unittest import TestCase


try:
    import PathPlanning.RRTStar.rrt_star as m
except ImportError:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = True
        m.search_until_max_iter = False
        import random; random.seed(20)
        m.main()

    def test_no_obstacle(self):
        obstacle_list = []

        # Set Initial parameters
        rrt_star = m.RRTStar(start=[0, 0],
                             goal=[1, 1],
                             rand_area=[-2, 15],
                             obstacle_list=obstacle_list, max_iter=20)
        path = rrt_star.planning(animation=True, search_until_max_iter=False)
        assert path is not None

if __name__ == '__main__':  # pragma: no cover
    
    test = Test()
    test.test1()
    #test.test_no_obstacle()
