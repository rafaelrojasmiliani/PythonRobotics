from .rrt_rn import RRT
import numpy as np
import copy


class cBiRRT(RRT):
    class cNodeList(list):
        def __contains__(self, _node):
            for n in self:
                if n is _node:
                    return True

            return False

        def remove(self, _node):
            for i, n in enumerate(self):
                if n is _node:
                    self.pop(i)

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 attempt_to_connect_dist=5):

        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate)

        self.other_rrt_ = copy.copy(self)
        self.other_rrt_.start = self.Node(goal)
        self.other_rrt_.end = self.Node(start)
        self.max_iter_ = max_iter

        self.attempt_to_connect_dist_ = attempt_to_connect_dist

        self.obstacle_list_ = obstacle_list

        self.start_rrt_leaf_list_ = self.cNodeList()
        self.goal_rrt_leaf_list_ = self.cNodeList()

    def planning(self):

        self.node_list = [self.start]
        self.other_rrt_.node_list = [self.end]
        for _ in range(self.max_iter_):

            node_from_start = self.planning_step()
            node_from_goal = self.other_rrt_.planning_step()

            if node_from_goal:
                self.goal_rrt_leaf_list_.remove(node_from_goal.parent)
                self.goal_rrt_leaf_list_.append(node_from_goal)
            if node_from_start:
                self.start_rrt_leaf_list_.remove(node_from_start.parent)
                self.start_rrt_leaf_list_.append(node_from_start)

            for node_s in self.start_rrt_leaf_list_:
                for node_g in self.goal_rrt_leaf_list_:
                    p1 = node_s.coordinates_
                    p2 = node_g.coordinates_
                    dist = np.linalg.norm(p1 - p2)
                    if dist < self.attempt_to_connect_dist_:
                        new_node = self.steer(node_s, node_g, 1.1 * dist)

                        if self.check_collision(new_node, self.obstacle_list_):
                            path_to_goal = []
                            path_to_start = []

                            node = node_g
                            while node is not None:
                                path_to_goal.append(node.coordinates_)
                                node = node.parent

                            node = node_s
                            while node is not None:
                                path_to_start.append(node.coordinates_)
                                node = node.parent

                            path_to_start.reverse()
                            return path_to_start + path_to_goal
