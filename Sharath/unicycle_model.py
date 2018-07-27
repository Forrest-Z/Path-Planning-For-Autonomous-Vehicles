import math

dt = 0.1  # [s]
L = 2.0  # [m]
steer_max = math.radians(40.0)
curvature_max = math.tan(steer_max) / L
curvature_max = 1.0 / curvature_max + 1.0

accel_max = 5.0


class State:

    def __init__(self, x=0.0, y=0.0, yaw=90.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.yaw = pi_2_pi(state.yaw)
    state.v = state.v + a * dt

    return state


def pi_2_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi
