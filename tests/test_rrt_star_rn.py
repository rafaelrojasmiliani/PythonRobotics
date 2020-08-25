import os
import sys
from unittest import TestCase

import functools
import traceback
import pdb
import numpy as np



def debug_on(*exceptions):
    ''' Decorator for entering in debug mode after exceptions '''
    if not exceptions:
        exceptions = (Exception, )

    def decorator(f):
        @functools.wraps(f)
        def wrapper(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            except exceptions:
                info = sys.exc_info()
                traceback.print_exception(*info)
                pdb.post_mortem(info[2])
                sys.exit(1)

        return wrapper

    return decorator

try:
    import PathPlanning.RRTStar.rrt_star_rn as m
except ImportError:
    raise


print(__file__)


class Test(TestCase):
    @debug_on()
    def test1(self):
        m.show_animation = True
        m.search_until_max_iter = False
        import random; random.seed(20)
        m.main()

    @debug_on()
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
    np.seterr(all='raise') 
    test = Test()
    test.test1()
    #test.test_no_obstacle()

