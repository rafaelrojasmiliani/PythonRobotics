"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)
Improvement of the origiginal code in order to implement RTT in R^n using low discrepancy sequences

Authors: 
    First implmentation AtsushiSakai(@Atsushi_twi) 
    Sobol and multidimensional vectors implentation Rafael A. Rojas

"""

import math
import random

from .sobol import sobol_quasirand

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, _coordinates):
            ''' Paramters:
                _coordinates: numpy array
                coordinates of the node '''
            _coordinates = np.array(_coordinates, dtype=np.float)
            self.coordinates_ = _coordinates.copy()
            self.path_ = []
            self.parent = None

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5, max_iter=500):
        """
        Setting Parameter

        start: numpy array, 
            Coordinates of the Start position
        goal:  nympy array
            Coordinats of the goal position
        obstacle_list: List of lists. 
            This is a list representing single circular obstacles.
            Each obstacle is represented by the list [x,size], 
        randArea:Random Sampling Area [min,max]

        """
        start = np.array(start, dtype=np.float)
        goal = np.array(goal, dtype=np.float)
        self.ambient_space_dim_ = start.shape[0]
        assert start.ndim == 1 and start.ndim == 1
        assert start.shape[0] == goal.shape[0]
        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

        self.sobol_inter_ = 0

    def planning(self, _animation=True):
        """
        rrt path planning

        _animation: flag for _animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if _animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1]) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if _animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        ''' Generates a sequence of points that joints the straight line
        between from_node to to_node. These points are stored in
        new_node.path. The node from_node is assigned as the parent of
        to_node.
        
        Returns
        -------
            A copy of to_node with the member path_[x, y] with the sequence
            of points that joints from_node to to_node'''


        new_node = self.Node([None, None])
        delta_vector = to_node.coordinates_ - from_node.coordinates_
        d = np.linalg.norm(delta_vector)
        delta_vector = delta_vector / d

        new_node.path_ = [from_node.coordinates_.copy()]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        path_node_coordinates = from_node.coordinates_.copy()

        for _ in range(n_expand):
            path_node_coordinates += self.path_resolution * delta_vector
            new_node.path_.append(path_node_coordinates.copy())

        d = np.linalg.norm(to_node.coordinates_ - path_node_coordinates)
        if d <= self.path_resolution:
            new_node.path_.append(to_node.coordinates_.copy())

        new_node.parent = from_node
        new_node.coordinates_ = path_node_coordinates

        return new_node

    def generate_final_course(self, goal_ind):
        ''' Travel backwards inside the tree until the first node (the one
        without father is reached '''
        path = [self.end.coordinates_]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.coordinates_)
            node = node.parent
        path.append(node.coordinates_)

        return path

    def calc_dist_to_goal(self, _node):
        dv = self.end.coordinates_ - _node.coordinates_
        return np.linalg.norm(dv)

    def get_random_node(self):
        ''' Generate the new random point in the research area'''

        if random.randint(0, 100) > self.goal_sample_rate:
            rand_coordinates, n = sobol_quasirand(self.ambient_space_dim_, self.sobol_inter_)
            rand_coordinates = self.min_rand + rand_coordinates*(self.max_rand - self.min_rand)
            self.sobol_inter_ = n
            rnd = self.Node(rand_coordinates)
        else:  # goal point sampling
            rnd = self.Node(self.end.coordinates_)
        return rnd

    def draw_graph(self, rnd=None):
        if self.ambient_space_dim_ != 2:
            print('Cannot plot multidimensioal arrays')
            return
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.coordinates_[0], rnd.coordinates_[1], "^k")
        for node in self.node_list:
            if node.parent:
                path = np.array(node.path_)
                plt.plot(path[:, 0], path[:, 1], "-g")

        for (o_coordinates, size) in self.obstacle_list:
            self.plot_circle(o_coordinates, size)

        plt.plot(self.start.coordinates_[0], self.start.coordinates_[1], "xr")
        plt.plot(self.end.coordinates_[0], self.end.coordinates_[1], "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(o_coordinates, size, color="-b"):  # pragma: no cover
        x = o_coordinates[0]
        y = o_coordinates[1]
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        ''' returns the index of the node in node_list which is nearest to
        rnd_node '''
        dlist = [np.linalg.norm(node.coordinates_ - rnd_node.coordinates_) for node in node_list]

        minind = np.argmin(dlist)

        return minind

    def check_collision(self, node, obstacleList):
        ''' Check collision in node.path_[x,y].  Check for the intersection the
        points in node.path and the spheres represented by obstacleList
            This method could be a function. However, in order to gain flexibility, 
            in the implmeentation of this libarry, this method may be overrrided in order to implement 
            new collision checkers
        '''

        if node is None:
            return False

        for (o_coordinates, size) in obstacleList:

            d_list = [np.linalg.norm(coordinates - o_coordinates) for coordinates in node.path_]

            if min(d_list) <= size:
                return False  # collision

        return True  # safe


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====

    rand_area = [-10, 10]

    size = rand_area[1] - rand_area[0]

    obsNum = np.random.randint(20, 60)
    print(obsNum)
    obstacleList = [ ((np.random.rand(2)-0.5)*20, np.random.rand()) for _ in range(0 , obsNum)]
    # [x, y, radius]
    # Set Initial parameters
    start = np.array([0, 0])
    goal = np.array([gx, gy])
    rrt = RRT(start,
              goal,
              rand_area=rand_area,
              obstacle_list=obstacleList)
    path = rrt.planning(_animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()

