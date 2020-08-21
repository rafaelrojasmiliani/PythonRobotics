import os
import sys
import random
from unittest import TestCase
import unittest
import functools
import traceback
import pdb



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



sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
try:
    from PathPlanning.RRT import rrt_rn as m
except ImportError:
    raise


print(__file__)

random.seed(12345)


class Test(TestCase):
    @debug_on()
    def test1(self):
        m.show_animation = True
        m.main(gx=6.8, gy=8.0)

#    def test2(self):
#        m1.show_animation = True
#        m1.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()

