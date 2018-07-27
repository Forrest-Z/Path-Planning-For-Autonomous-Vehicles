import search
import time
import math
import matplotlib.pyplot as plt


plot_flag = True

def main():
    start_time = time.time()
    print("Finding path using rrt star with dubins planning")
    
    # [x,y,size(radius)]
   
    obstacleList = [
      
        # (0, 20, 1.0),
        # (20, 0, 1.0),
        # (0, -20, 1.0),
        # (-20, 0.0, 1.0),
        # (20, 20, 1.0),
        # (-20, -20, 1.0),
        # (20, -20, 1.0),
        # (-20, 20.0, 1.0),
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
        # (-10, -20, 1.0),     
        ]  
    #obstacleList = []
    # Set Initial parameters
    start = [0.0, 0.0, math.radians(0.0)]
    goal = [100.0, 0.0, math.radians(0.0)]
        
      
    #obstacleList = []
    # Set Initial parameters
    # start = [0.0, 0.0, math.radians(90.0)]
    # goal = [0.0, 17.0, math.radians(-90.0)]

    rrt = search.RRT(start, goal, randArea=[-40.0, 40.0], obstacleList=obstacleList)
    path = rrt.Planning(animation=True)
    end_time = time.time()
    print("Time Taken:", end_time - start_time)

    # Draw final path
    if plot_flag:
        rrt.plot_trees()
        print(len(path))
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)

        plt.show()


if __name__ == '__main__':
    main()
