import random
import math
import copy
import numpy as np
import dubins_path
import matplotlib.pyplot as plt
import time
import pure_pursuit
plot_flag = True
target_speed = 7.0
import unicycle_model


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, max_iter=75):
        
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
                #self.plot_trees(rand=rand)
                continue

        # generate course path. Saving this variable is useful in tracing the path
        last_index = self.get_best_last_indexs()
        #  print(last_index)

        if last_index is None:
            return None

        path_indexs = self.get_best_last_indexs()
        
        #return path



        flag, x, y, yaw, v, t, a, d = self.search_best_feasible_path(
            path_indexs)

        return flag, x, y, yaw, v, t, a, d

    def search_best_feasible_path(self, path_indexs):

        print("Start search feasible path")

        best_time = float("inf")

        fx = None

        # pure pursuit tracking
        for ind in path_indexs:
            path = self.gen_final_course(ind)

            flag, x, y, yaw, v, t, a, d = self.check_tracking_path_is_feasible(
                path)

            if flag and best_time >= t[-1]:
                print("feasible path is found")
                best_time = t[-1]
                fx, fy, fyaw, fv, ft, fa, fd = x, y, yaw, v, t, a, d

        print("best time is")
        print(best_time)

        if fx:
            fx.append(self.end.x)
            fy.append(self.end.y)
            fyaw.append(self.end.yaw)
            return True, fx, fy, fyaw, fv, ft, fa, fd
        else:
            return False, None, None, None, None, None, None, None

    def calc_tracking_path(self, path):
        path = np.matrix(path[::-1])
        ds = 0.2
        for i in range(10):
            lx = path[-1, 0]
            ly = path[-1, 1]
            lyaw = path[-1, 2]
            move_yaw = math.atan2(path[-2, 1] - ly, path[-2, 0] - lx)
            

            lstate = np.matrix(
                [lx + ds * math.cos(lyaw), ly + ds * math.sin(lyaw), lyaw])
            #  print(lstate)

            path = np.vstack((path, lstate))

        return path

    def check_tracking_path_is_feasible(self, path):
        #  print("check_tracking_path_is_feasible")
        cx = np.array(path[:, 0])
        cy = np.array(path[:, 1])
        cyaw = np.array(path[:, 2])

        goal = [cx[-1], cy[-1], cyaw[-1]]

        cx, cy, cyaw = pure_pursuit.extend_path(cx, cy, cyaw)

        speed_profile = pure_pursuit.calc_speed_profile(
            cx, cy, cyaw, target_speed)

        t, x, y, yaw, v, a, d, find_goal = pure_pursuit.closed_loop_prediction(
            cx, cy, cyaw, speed_profile, goal)
        yaw = [self.pi_2_pi(iyaw) for iyaw in yaw]

        if not find_goal:
            print("cannot reach goal")

        if abs(yaw[-1] - goal[2]) >= math.pi / 4.0:
            print("final angle is bad")
            find_goal = False

        travel = sum([abs(iv) * unicycle_model.dt for iv in v])
        #  print(travel)
        origin_travel = sum([math.sqrt(dx ** 2 + dy ** 2)
                             for (dx, dy) in zip(np.diff(cx), np.diff(cy))])
        #  print(origin_travel)

        if (travel / origin_travel) >= 5.0:
            print("path is too long")
            find_goal = False

        if not self.CollisionCheckWithXY(x, y, self.obstacleList):
            print("This path is collision")
            find_goal = False

        return find_goal, x, y, yaw, v, t, a, d



    def CollisionCheckWithXY(self, x, y, obstacleList):

        for (ox, oy, size) in obstacleList:
            for (ix, iy) in zip(x, y):
                dx = ox - ix
                dy = oy - iy
                d = dx * dx + dy * dy
                if d <= 7:
                    return False  # collision

        return True  # safe

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

    def get_best_last_indexs(self):
        #  print("get_best_last_index")

        YAWTH = math.radians(1.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)
        print("OK XY TH num is")
        print(len(goalinds))

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)
        print("OK YAW TH num is")
        print(len(fgoalinds))

        return fgoalinds

    


    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path_x = reversed(node.path_x)
            path_y = reversed(node.path_y)
            path_yaw = reversed(node.path_yaw)
            for (ix, iy, iyaw) in zip(path_x, path_y, path_yaw):
                path.append([ix, iy, iyaw])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])

        path = np.matrix(path[::-1])
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

        plt.axis([-40, 40, -40, 40])
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
                if d <= 7:
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


def main():
    start_time = time.time()
    print("Finding path using rrt star with dubins planning")
    
    # [x,y,size(radius)]
    obstacleList = [
        (0, 20, 1.0),
        (20, 0, 1.0),
        (0, -20, 1.0),
        (-20, 0.0, 1.0),
        (20, 20, 1.0),
        (-20, -20, 1.0),
        (20, -20, 1.0),
        (-20, 20.0, 1.0),
        # (0, 10, 1.0),
        # (10, 0, 1.0),
        # (0, -10, 1.0),
        # (-10, 0.0, 1.0),
        # (10, 10, 1.0),
        # (-10, -10, 1.0),
        # (10, -10, 1.0),
        # (-10, 10.0, 1.0),
        #  (20, 10, 1.0),
        # (10, 20, 1.0),
        # (0, -10, 1.0),
        # (-10, 20.0, 1.0),
        # (10, -20, 1.0),
        (-10, -20, 1.0)  
        

        
    ]  

    # Set Initial parameters
    start = [0.0, 0.0, math.radians(0.0)]
    goal = [0.0, 30.0, math.radians(90.0)]

    #rrt = RRT(start, goal, randArea=[-2.0, 15.0], obstacleList=obstacleList)
    #path = rrt.Planning(animation=False)



    rrt = RRT(start, goal, randArea=[-15.0, 15.0], obstacleList=obstacleList)
    flag, x, y, yaw, v, t, a, d = rrt.Planning(animation=plot_flag)

    if not flag:
        print("cannot find feasible path")

    #  flg, ax = plt.subplots(1)
    # Draw final path
    if plot_flag:
        rrt.plot_trees()
        plt.plot(x, y, '-r')
        plt.grid(True)
        plt.pause(0.001)

        flg, ax = plt.subplots(1)
        plt.plot(t, [math.degrees(iyaw) for iyaw in yaw[:-1]], '-r')
        plt.xlabel("time[s]")
        plt.ylabel("Yaw[deg]")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], '-b')

        plt.xlabel("time[s]")
        plt.ylabel("velocity[km/h]")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, a, '-g')
        plt.xlabel("time[s]")
        plt.ylabel("accel[m/ss]")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [math.degrees(td) for td in d], '-k')
        plt.xlabel("time[s]")
        plt.ylabel("Steering angle[deg]")
        plt.grid(True)

        plt.show()


    end_time = time.time()
    print("Time Taken:", end_time - start_time)

    # Draw final path
    if plot_flag:
        rrt.plot_trees()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)

        plt.show()


if __name__ == '__main__':
    main()