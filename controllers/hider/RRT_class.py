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
        self.path_from_parent = [];

class RRT:
    """
    Class that continually updates a map representaion and runs RRT on it
    """

    def __init__(self, config_radius, k, q, map_size, towards_goal_prob=0.1, min_goal_dist = 2):
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
        self.map = np.zeros((map_size,map_size))
        self.config_space = np.zeros((map_size,map_size)) #configuration space of the robot
        self.explored_space = np.zeros((map_size,map_size)) #map that keeps track of space that has been "explored"
        self.path = [] #List of Nodes has not been run
        self.config_radius = config_radius;
        self.k = k; #how many nodes to sample in each run_RRT call
        self.delta_q = q; # max distance between nodes
        self.state_bounds = [(0,map_size),(0,map_size)]
        self.towards_goal_prob = towards_goal_prob;
        self.min_goal_dist = min_goal_dist
        # need to initialize RRT list



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

    def run_RRT(self, start_point, goal_point, map):
        '''
        runs_RRT on self.config_space
        :paramm start_point: tuple of coordinates to start RRT at
        :param goal_point: tuple of target coordintaes
        :returns: a list of tuples representing a path
        '''

        #begin list
        node_list = [];
        start_point_int = [int(val) for val in start_point]
        node_list.append(Node(np.array(start_point_int), parent=None))

        self.update_config_space(map)

        for i in range(self.k):
            # print('here in RRT')
            # print('nodes',[node.point for node in node_list])
            if(random.random()<self.towards_goal_prob):
                new_point = [int(num) for num in np.array(goal_point)]
            else:
                new_point = np.array(self.get_random_valid_vertex(self.state_bounds,self.config_space))
            # print('new point', new_point)
            nearest_node = self.get_nearest_node(node_list, new_point)
            # print('nearest_node point', nearest_node.point)

            # print('steering from ', nearest_node.point, ' to ', new_point)
            path = self.steer(nearest_node.point, new_point)

            if not self.path_is_valid(path, self.config_space):
                continue

            # create new node
            # print('path',path)

            new_node_point = np.array([int(num) for num in path[len(path)-1,:]])
            # print('after steer', new_node_point)
            if not self.point_is_valid(new_node_point,self.config_space): continue
            new_node = Node(new_node_point, parent = nearest_node)
            node_list.append(new_node)
            # print('new node point',new_node.point)
            # check to see if node is close enough to goal

            if np.linalg.norm(np.array(new_node.point)-np.array(goal_point))<self.min_goal_dist:
                break

        return node_list

    def get_config_space(self):
        return self.config_space

    def get_nearest_node(self, nodes, point):


        # for i,node in enumerate(nodes):
        #     distances[i] = np.linalg.norm(node.point-point)
        distances = [np.linalg.norm(node.point-point) for node in nodes]

        return nodes[np.argmin(distances)]

    def get_waypoints(self,nodes):
        cur_node = nodes[len(nodes)-1]
        path = []
        path.append(cur_node)
        while cur_node.parent is not None:
            path.append(cur_node.parent)
            cur_node = cur_node.parent
        #extract the actual points
        waypoints = [node.point for node in path]

        return waypoints[::-1] ##this revereses the order of the list

    def steer(self,from_point, to_point, samples = 10):
        # get vector that points from 'from_point' to 'to_point'
        pointingVec = np.array(to_point)-np.array(from_point)
        mag = np.linalg.norm(pointingVec)
        # print('mag',mag)
        # normalize vector to delta_q if length is greater than delta_q
        if mag > self.delta_q:
            pointingVec = self.delta_q*pointingVec/mag
        # print('pointingVec',pointingVec)

        path = np.empty([10, len(pointingVec)], np.float64)
        to_point = from_point+pointingVec

        # get n-dimansional path using linspace
        # print('frpm to point',from_point,to_point)
        for i in range(len(pointingVec)):
            path[:,i] = np.linspace(from_point[i],to_point[i],samples)

        return path

    def get_random_valid_vertex(self,bounds,config_space):
        # vertex = []
        # for i,limit in enumerate(bounds):
        #     vertex[i] = [random.random()*(limit[1]-limit[0])+limit[0]]\
        point = np.zeros(len(bounds))
        for j in range(500):
            for i,limits in enumerate(bounds):
                point[i] = int(random.random()*(limits[1]-limits[0])+limits[0])
            # print('get_random_valid_vertex point',point)
            if self.point_is_valid(point, config_space): break

        if j==499:
            raise Exception("get_random_valid_vertex did not find a valid point after 500 iterations")

        return point

    def point_is_valid(self, point, config_space):
        # print('point',point)
        # need to convert point from meters to pixels
        # print('config', config_space)
        x = int(point[0])
        y = int(point[1])
        if config_space[x][y] > 0.9:
            print('point not valid (config test)', x, y)
            return False
        if x<self.state_bounds[0][0] or x>self.state_bounds[0][1] or y<self.state_bounds[1][0] or y>self.state_bounds[1][1]:
            # print('invalid point',x,y)
            print('point not valid (bounds)', x, y)
            return False
        # print('valid point',x,y)
        return True

    def check_for_collsiions_2points(self,point1, point2, sampling_points = 10):
        #return true if there is a collision iminent
        point1 = np.array(point1)
        point2 = np.array(point2)
        pointingVec = point2-point1
        path = np.empty([sampling_points, len(pointingVec)], np.float64)
        for i in range(len(pointingVec)):
            path[:,i] = np.linspace(point1[i],point2[i],sampling_points)

        for point in path:
            x = int(point[0])
            y = int(point[1])

            if self.config_space[x][y] > 0.9:
                return True

        return False

    def path_is_valid(self, points, config_space):
        '''
        checks if any segment of the path is colliding with the config space

        :param points: list of coordinates(tuples)
        :param config_space: 2d array representing config space
        :returns: true if path is valid
        '''
        for point in points:
            if not self.point_is_valid(point,config_space):
                # print('invalid path',points)
                return False
        # print('valid path',points)
        return True

    def update_config_space(self,new_map):
        '''
        Takes in new points that have not been included in the configuration space
        from new_map, and updates self.config_space and self.map which tracks the points that have already been accounted for

        :param: new_map: a 2D array contatinng a world representation
        :returns nothing
        '''
        threshHold = .8 #slightly arbitrary at this point
        radius = self.config_radius;
        new_map = np.array(new_map)

        locations_to_update = new_map-self.map;

        rows = locations_to_update.shape[0]
        cols = locations_to_update.shape[1]
        for i in range(rows):
            for j in range(cols):
                if locations_to_update[i][j] > threshHold:
                    left = j-radius
                    right = j+radius
                    lower = i+radius
                    upper = i-radius

                    if left<0:
                        left = 0
                    if right>cols-1:
                        right = cols-1
                    if lower > rows-1:
                        lower = rows-1
                    if upper < 0:
                        upper = 0
                    for k in range(upper,lower):
                        for n in range(left,right):
                            self.config_space[k][n] = 1



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

    def check_collisions_along_path(self,nodes,sampling_points=10):
        '''
        Runs all segments of a path through check collsions

        :param path: list of Nodes representing the path
        '''
        cur_node = nodes[len(nodes)-1]
        while cur_node.parent is not None:
            if self.check_for_collsiions_2points(cur_node.point,cur_node.parent.point,sampling_points=sampling_points):
                return True
            cur_node = cur_node.parent

        return False
