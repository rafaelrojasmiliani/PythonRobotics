"""

Path planning Sample Code with Optimized Randomized Rapidly-Exploring Random Trees (RRT*)
Improvement of the origiginal code in order to implement RTT in R^n using low discrepancy sequences

Authors:
    Original implementation: AtsushiSakai(@Atsushi_twi)
    Sobol and multidimensional vectors implentation: Rafael A. Rojas

"""

import math
import os
import sys
import copy
import matplotlib.pyplot as plt
import numpy as np

from ..RRT.rrt_rn import RRT

show_animation = True
search_until_max_iter = True


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, _coordinates):
            super().__init__(_coordinates)
            self.cost = 0.0

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=5.0,
                 path_resolution=0.1,
                 goal_sample_rate=20,
                 max_iter=500,
                 connect_circle_dist=50.0):
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter)
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal)

    def planning(self, _animation=True, _search_until_max_iter=True):
        """
        rrt star path planning

        animation: flag for animation on or off
        search_until_max_iter: search until max iteration for path improving or not
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            print(rnd.coordinates_, '----- rnd')
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            print(new_node.coordinates_, '----- new_node')

            if self.check_collision(new_node, self.obstacle_list):
                new_node = self.choose_parent_and_rewire(new_node)
                if new_node:
                    self.node_list.append(new_node)


            if _animation:
                self.draw_graph(rnd)

            if new_node and self.calc_dist_to_goal(new_node) <= self.expand_dis:
                final_node = self.steer(new_node, self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

        print("reached max iteration")

#        last_index = self.search_best_goal_node()
#        if last_index is not None:
#            return self.generate_final_course(last_index)

        return None

    def choose_parent_and_rewire(self, new_node):
        '''
        Computes the chapest point to new_node contained in the list
        near_inds and set such a node as the partent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not colitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        '''
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)

        near_nodes_list = []
        for node in self.node_list:
            d = np.linalg.norm(new_node.coordinates_ - node.coordinates_) 
            if d < r:
                near_nodes_list.append(node)
            if d < self.path_resolution:
                # Discart the new node if it is nearest than the path_resolution.
                return None

        if not near_nodes_list:
            return new_node

        # search nearest cost in near_inds
        accessible_near_nodes_list = []
        new_node_alternative_list = []
        for near_node in near_nodes_list:
            new_node_alternative = self.steer(near_node, new_node)
            if self.check_collision(new_node_alternative, self.obstacle_list):
                accessible_near_nodes_list.append(near_node)
                new_node_alternative_list.append(new_node_alternative)

        if not accessible_near_nodes_list:
            return new_node

        new_node = min(
            new_node_alternative_list, key=lambda node: node.cost)

        for near_node in accessible_near_nodes_list:
            auxiliar_node = self.steer(new_node, near_node)

            if auxiliar_node.cost < near_node.cost:
                near_node.parent = new_node
                near_node.cost = auxiliar_node.cost
                new_node.path_ = auxiliar_node.path_
                self.propagate_cost_to_leaves(new_node)

        return new_node

    def calc_new_cost(self, from_node, to_node):
        d = np.linalg.norm(from_node.coordinates_ - to_node.coordinates_)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def steer(self, from_node, to_node, extend_length=float("inf")):
        ''' Comptes the local plann to arrive ti to_node and update the cost to
        arrive to to_node from from_node '''
        new_node = RRT.steer(self, from_node, to_node, extend_length)
        new_node.cost = self.calc_new_cost(from_node, new_node)
        return new_node


def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacle_list = [
        (np.array([5, 5]), 1),
        (np.array([3, 6]), 2),
        (np.array([3, 8]), 2),
        (np.array([3, 10]), 2),
        (np.array([7, 5]), 2),
        (np.array([9, 5]), 2),
        (np.array([8, 10]), 1),
        (np.array([6, 12]), 1),
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list)
    path = rrt_star.planning(
        _animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')


#            plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
