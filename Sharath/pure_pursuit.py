import numpy as np
import math
import matplotlib.pyplot as plt
import unicycle_model
import rrt_star

Kp = 1.0  # speed propotional gain
Lf = 1.5  # look-ahead distance
T = 100.0  # max simulation time
goal_dis = 0.5
stop_speed = 0.5

#  animation = True
animation = False


def PIDControl(target, current):
    a = Kp * (target - current)

    if a > unicycle_model.accel_max:
        a = unicycle_model.accel_max
    elif a < -unicycle_model.accel_max:
        a = -unicycle_model.accel_max

    return a


def pure_pursuit_control(state, cx, cy, p_index):

    index, distance = calc_target_index(state, cx, cy)

    if p_index >= index:
        index = p_index

    #  print(p_index, index)
    if index < len(cx):
        tx = cx[index]
        ty = cy[index]
    else:
        tx = cx[-1]
        ty = cy[-1]
        index = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v <= 0.0:  # back
        alpha = math.pi - alpha

    delta = math.atan2(2.0 * unicycle_model.L * math.sin(alpha) / Lf, 1.0)

    if delta > unicycle_model.steer_max:
        delta = unicycle_model.steer_max
    elif delta < - unicycle_model.steer_max:
        delta = -unicycle_model.steer_max

    return delta, index, distance


def calc_target_index(state, cx, cy):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    distance = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    minimum_distance = min(distance)

    index = distance.index(minimum_distance)

    L = 0.0

    while Lf > L and (index + 1) < len(cx):
        dx = cx[index + 1] - cx[index]
        dy = cx[index + 1] - cx[index]
        L += math.sqrt(dx ** 2 + dy ** 2)
        index += 1

    #  print(minimum_distance)
    return index, minimum_distance


def closed_loop_prediction(cx, cy, cyaw, speed_profile, goal):

    state = unicycle_model.State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    #  lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    a = [0.0]
    d = [0.0]
    target_index, minimum_distance = calc_target_index(state, cx, cy)
    find_goal = False

    maximum_distance = 0.5

    while T >= time:
        di, target_index, distance = pure_pursuit_control(state, cx, cy, target_index)

        target_speed = speed_profile[target_index]
        target_speed = target_speed * \
            (maximum_distance - min(distance, maximum_distance - 0.1)) / maximum_distance

        ai = PIDControl(target_speed, state.v)
        state = unicycle_model.update(state, ai, di)

        if abs(state.v) <= stop_speed and target_index <= len(cx) - 2:
            target_index += 1

        time = time + unicycle_model.dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            find_goal = True
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        a.append(ai)
        d.append(di)

        if target_index % 1 == 0 and animation:
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_index], cy[target_index], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed:" + str(round(state.v, 2)) +
                      "target_index:" + str(target_index))
            plt.pause(0.0001)

    else:
        print("Time out!!")

    return t, x, y, yaw, v, a, d, find_goal


def set_stop_point(target_speed, cx, cy, cyaw):
    speed_profile = [target_speed] * len(cx)
    forward = True

    d = []

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        d.append(math.sqrt(dx ** 2.0 + dy ** 2.0))
        iyaw = cyaw[i]
        move_direction = math.atan2(dy, dx)
        is_back = abs(move_direction - iyaw) >= math.pi / 2.0

        if dx == 0.0 and dy == 0.0:
            continue

        if is_back:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if is_back and forward:
            speed_profile[i] = 0.0
            forward = False
            #  plt.plot(cx[i], cy[i], "xb")
            #  print(iyaw, move_direction, dx, dy)
        elif not is_back and not forward:
            speed_profile[i] = 0.0
            forward = True
            #  plt.plot(cx[i], cy[i], "xb")
            #  print(iyaw, move_direction, dx, dy)
    speed_profile[0] = 0.0
    if is_back:
        speed_profile[-1] = -stop_speed
    else:
        speed_profile[-1] = stop_speed

    d.append(d[-1])

    return speed_profile, d


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile, d = set_stop_point(target_speed, cx, cy, cyaw)

    if animation:
        plt.plot(speed_profile, "xb")

    return speed_profile


def extend_path(cx, cy, cyaw):

    dl = 0.1
    dl_list = [dl] * (int(Lf / dl) + 1)

    move_direction = math.atan2(cy[-1] - cy[-3], cx[-1] - cx[-3])
    is_back = abs(move_direction - cyaw[-1]) >= math.pi / 2.0

    for idl in dl_list:
        if is_back:
            idl *= -1
        cx = np.append(cx, cx[-1] + idl * math.cos(cyaw[-1]))
        cy = np.append(cy, cy[-1] + idl * math.sin(cyaw[-1]))
        cyaw = np.append(cyaw, cyaw[-1])

    return cx, cy, cyaw


def main():
    #  target course
    import numpy as np
    cx = np.arange(0, 50, 0.1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 5.0 / 3.6

    T = 15.0  # max simulation time

    state = unicycle_model.State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)
    #  state = unicycle_model.State(x=-1.0, y=-5.0, yaw=0.0, v=-30.0 / 3.6)
    #  state = unicycle_model.State(x=10.0, y=5.0, yaw=0.0, v=-30.0 / 3.6)
    #  state = unicycle_model.State(
    #  x=3.0, y=5.0, yaw=math.radians(-40.0), v=-10.0 / 3.6)
    #  state = unicycle_model.State(
    #  x=3.0, y=5.0, yaw=math.radians(40.0), v=50.0 / 3.6)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_index = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_index:
        ai = PIDControl(target_speed, state.v)
        di, target_index = pure_pursuit_control(state, cx, cy, target_index)
        state = unicycle_model.update(state, ai, di)

        time = time + unicycle_model.dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        #  plt.cla()
        #  plt.plot(cx, cy, ".r", label="course")
        #  plt.plot(x, y, "-b", label="trajectory")
        #  plt.plot(cx[target_index], cy[target_index], "xg", label="target")
        #  plt.axis("equal")
        #  plt.grid(True)
        #  plt.pause(0.1)
        #  input()

    flg, ax = plt.subplots(1)
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)

    flg, ax = plt.subplots(1)
    plt.plot(t, [iv * 3.6 for iv in v], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)
    plt.show()


def main2():
    import pandas as pd
    data = pd.read_csv("rrt_course.csv")
    cx = np.array(data["x"])
    cy = np.array(data["y"])
    cyaw = np.array(data["yaw"])

    target_speed = 10.0 / 3.6

    goal = [cx[-1], cy[-1]]

    cx, cy, cyaw = extend_path(cx, cy, cyaw)

    speed_profile = calc_speed_profile(cx, cy, cyaw, target_speed)

    t, x, y, yaw, v, a, d, flag = closed_loop_prediction(
        cx, cy, cyaw, speed_profile, goal)

    flg, ax = plt.subplots(1)
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.plot(goal[0], goal[1], "xg", label="goal")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)

    flg, ax = plt.subplots(1)
    plt.plot(t, [iv * 3.6 for iv in v], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
