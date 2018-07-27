import random
import math
import copy
import numpy as np
import dubins_path
import matplotlib.pyplot as plt
import time

plot_flag = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10.0, max_iter=500):
        
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.goalSampleRate = goalSampleRate
        self.max_iter = max_iter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
       

        self.nodeList = [self.start]
        for i in range(self.max_iter):
            rand = self.get_random_point()
            near_index = self.get_nearest_list_index(self.nodeList, rand)

            new_node = self.steer(rand, near_index)
            #  print(new_node.cost)

            if self.collision_check(new_node, self.obstacleList):
                near_indices = self.find_near_nodes(new_node)
                new_node = self.select_parent(new_node, near_indices)
                self.nodeList.append(new_node)
                self.rewire(new_node, near_indices)

            if animation and i % 5 == 0:
                self.plot_trees(rand=rand)
                continue

        # generate course path. Saving this variable is useful in tracing the path
        last_index = self.get_best_last_index()
        #  print(last_index)

        if last_index is None:
            return None

        path = self.gen_final_course(last_index)
        #print(path)
        return path

    def select_parent(self, new_node, near_indices):
        if len(near_indices) == 0:
            return new_node

        dlist = []
        for i in near_indices:
            tNode = self.steer(new_node, i)
            if self.collision_check(tNode, self.obstacleList):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        min_cost = min(dlist)
        min_index = near_indices[dlist.index(min_cost)]

        if min_cost == float("inf"):
            print("min_cost is inf")
            return new_node

        new_node = self.steer(new_node, min_index)

        return new_node

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def steer(self, rand, near_index):
        curvature = 1.0
        nearest_node = self.nodeList[near_index]
        px, py, pyaw, mode, clen = dubins_path.dubins_path_planning(
            nearest_node.x, nearest_node.y, nearest_node.yaw, rand.x, rand.y, rand.yaw, curvature)

        new_node = copy.deepcopy(nearest_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += clen
        new_node.parent = near_index

        return new_node

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rand = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rand = [self.end.x, self.end.y, self.end.yaw]

        node = Node(rand[0], rand[1], rand[2])

        return node

    def get_best_last_index(self):
        #  print("get_best_last_index")

        YAWTH = math.radians(1.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)

        if len(fgoalinds) == 0:
            return None

        min_cost = min([self.nodeList[i].cost for i in fgoalinds])
        for i in fgoalinds:
            if self.nodeList[i].cost == min_cost:
                return i

        return None

    def gen_final_course(self, goalind):

        path = [[self.end.x, self.end.y]]
        print(len(path))
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, new_node):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - new_node.x) ** 2 +
                 (node.y - new_node.y) ** 2 +
                 (node.yaw - new_node.yaw) ** 2
                 for node in self.nodeList]
        near_indices = [dlist.index(i) for i in dlist if i <= r ** 2]
        return near_indices

    def rewire(self, new_node, near_indices):

        nnode = len(self.nodeList)

        for i in near_indices:
            near_node = self.nodeList[i]
            tNode = self.steer(near_node, nnode - 1)

            obstacleOK = self.collision_check(tNode, self.obstacleList)
            imporveCost = near_node.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def plot_trees(self, rand=None):
        u"""
        Draw Graph
        """
        plt.clf()
        # plt.close() will close the figure window entirely, 
        # where plt.clf() will just clear the figure - you can still paint another plot onto it.


        if rand is not None:
            plt.plot(rand.x, rand.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-b")
                

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        dubins_path.marker(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path.marker(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([-80, 80, -80, 80])
        plt.grid(True)
        plt.pause(0.01)

        #  plt.show()
        #  input()

    def get_nearest_list_index(self, nodeList, rand):
        dlist = [(node.x - rand.x) ** 2 +
                 (node.y - rand.y) ** 2 +
                 (node.yaw - rand.yaw) ** 2 for node in nodeList]
        min_index = dlist.index(min(dlist))

        return min_index

    def collision_check(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            for (ix, iy) in zip(node.path_x, node.path_y):
                dx = ox - ix
                dy = oy - iy
                d = dx * dx + dy * dy
                if d <= size**2:
                    return False  # collision

        return True  # safe


class Node():
    

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None


