import numpy as np
import math
import random

class Node:
    """
    Class used to represent path points
    """
    def __init__(self,pt,parent = None):
        self.point = pt;
        self.parent = parent;
        seld.path_from_parent = [];

class RRT:
    """
    Class that continually updates a map representaion and runs RRT on it
    """

    def __init__(self, map, config_radius, k, q, bounds, towards_goal_prob=0.1, min_goal_dist = 2):
        '''
        RRT on a world initialization

        :param map: unedited world representation (2d array)
        :param start_point: start coordinates (tuple)
        :param config_radius: radius to draw around objects in config space
        :param k: number of points to sample before giving up
        :param q: max distance between nodes
        :param bounds: bounds of the world, tuples of min max values for each dim
        :param towards_goal_prob: probability that new sampled point will point towards the goal point
        :param min_goal_dist: success criterea for being close to the goal;
        '''
        self.map = map;
        self.config_space = []; #configuration space of the robot
        self.explored_space = [] #map that keeps track of space that has been "explored"
        self.path = [] #List of Nodes has not been run
        self.config_radius = config_radius;
        self.k = k; #how many nodes to sample in each run_RRT call
        self.delta_q = q; # max distance between nodes
        self.towards_goal_prob = towards_goal_prob;
        # need to initialize RRT list
        self.root = Node(start_point, parent=None)
        self.path.append(self.root)


    # def update_RRT(self,from_node, to_node):
    #     '''
    #     Runs an initial RRT to a goal point.
    #     Checks for collisions along path, and rebuilds path if ecessary
    #
    #     :param from_node: current Node that the robot is travelling from
    #     :param to_node: target waypoint of type Node that the robot is travelling to
    #     :returns nothing
    #     '''
    #
    #     node_to_delete = check_collisions_along_path(self.path)
    #     if node_to_delete is not None:
    #         #delete part of path and rebuild RRT
    #         delete_branch(node_to_delete);
    #         run_RRT(from_node, to_node);

    def run_RRT(self, start_point, goal_point):
        '''
        runs_RRT on self.config_space
        :paramm start_point: tuple of coordinates to start RRT at
        :param goal_point: tuple of target coordintaes
        :returns: a list of tuples representing a path
        '''
        #begin list
        node_list = [];
        node_list.append(Node(start_point, parent=None))

        for i in range(self.k):
            if(random.random()<self.towards_goal_prob):
                new_point = goal_point
            else:
                new_point = get_random_valid_vertex(state_bounds,self.config_space)

            nearest_node = get_nearest_node(node_list, newCorrdinates)
            path = steer(nearest_node.point, new_point)

            if not path_is_valid(path, self.config_space):
                continue

            # create new node
            new_node = Node(path[len(path),1:2], Parent = nearest_node)
            node_list.append(new_node)

            # check to see if node is close enough to goal
            if np.linalg.norm(new_node.point-goal_point)<min_goal_dist:
                break

        return get_waypoints(node_list)

    def get_waypoints(nodes):
        return [node.point for node in nodes]

    def steer(from_point, to_point, samples = 10):
        # get vector that points from 'from_point' to 'to_point'
        pointingVec = np.array(to_point)-np.array(from_point)
        length =

        # normalize vector to delta_q if length is greater than delta_q
        if length > delta_q:
            pointingVec = delta_q*pointingVec/np.linalg.norm(pointingVec)


        path = np.empty([10, len(pointingVec)], np.float64)
        to_point = from_point+pointingVec

        # get n-dimansional path using linspace
        for i in range(len(pointingVec)):
            path[:,i] = np.linspace(from_point[i],to_point[i],samples)

        return path

    get_random_valid_vertex(bounds,config_space):
        point = ()

    def path_is_valid(points, config_space):
        '''
        checks if any segment of the path is colliding with the config space

        :param points: list of coordinates(tuples)
        :param config_space: 2d array representing config space
        :returns: true if path is valid
        '''
        for point in points:
            x,y = point;
            if config_space[x,y] > 0.9:
                return False

    def update_config_space(self,new_map):
        '''
        Takes in new points that have not been included in the configuration space
        from new_map, and updates self.config_space and self.map which tracks the points that have already been accounted for

        :param: new_map: a 2D array contatinng a world representation
        :returns nothing
        '''




    # def make_new_root(self,new_root):
    #     '''
    #     deletes nodes that are before the new_root Node and makes new_root the self.root
    #
    #     :param new_parent: the current Node that will become the parent of RRT
    #     :return: nothing
    #     '''
    #     if new_root == self.root:
    #         return
    #
    #     to_remove = [self.root]
    #     while to_remove:
    #         for node, i in enumerate(to_remove):
    #             if node == new_parent:
    #                 continue
    #             removed_node = self.path.pop(self.path.index(node))
    #             [to_remove.append(child) for child in removed_node.children]
    #     self.root = new_parent

    # def check_collisions_along_path(path):
    #     '''
    #     Runs all segments of a path through check collsions
    #
    #     :param path: list of Nodes representing the path
    #     '''
    #     for node, i in enumerate(path):
