import math
import numpy as np
from .make_race import new_race
from .map_utils import plot_poly
from .map_utils import ray_cast
from rrt_methods.rrt_utils import Map


def make_rect_region(min_x, max_x, max_y, min_y):
    b_l = np.array([min_x, min_y])
    b_r = np.array([max_x, min_y])
    t_l = np.array([min_x, max_y])
    t_r = np.array([max_x, max_y])
    return [b_l, b_r, t_r, t_l]


# Hacky obstacle maker (rectangle)
def obstacle_to_path(p0_r, p0_l, p1_r, p1_l):
    obstacle = []
    dir_r = p1_r - p0_r
    dir_l = p1_l - p0_l
    obstacle.append(.1 * dir_r + p0_r)
    obstacle.append(.1 * dir_l + p0_l)
    return obstacle


# makes rectangle with 'foci' p0, p1
def expand_obstacle(p0, p1, r):
    t_l = np.array([p0[0], p0[1] + r])
    b_l = np.array([p0[0], p0[1] - r])
    b_r = np.array([p1[0], p1[1] - r])
    t_r = np.array([p1[0], p1[1] + r])

    return [t_l, b_l, b_r, t_r]


def race_map(racetrack=None, n=20):
    if not racetrack:
        racetrack = new_race()
    midpoints = racetrack.generate_race_course_midpath(n)
    blue_cones, yellow_cones = racetrack.generate_left_and_right_cones()

    plot_poly(blue_cones, c="#e13c41")
    plot_poly(yellow_cones, c="#e13c41")

    start_pos = ((blue_cones[-1] - yellow_cones[-1]) / 2) + yellow_cones[-1]
    end_pos = ((blue_cones[-2] - yellow_cones[-2]) / 2) + yellow_cones[-2]

    obs1 = obstacle_to_path(
            blue_cones[-1],
            yellow_cones[-1],
            blue_cones[-2],
            yellow_cones[-2]
            )

    obs2 = obstacle_to_path(
            blue_cones[-2],
            yellow_cones[-2],
            blue_cones[-1],
            yellow_cones[-1]
            )

    obstacles = [blue_cones, obs1, obs2]
    map = Map(region=yellow_cones, obstacles=obstacles)
    map.add_path([start_pos, end_pos])

    return map


def square_obs_map(n, size):
    start_pos = np.array([10, 10])
    end_pos = np.array([790, 790])
    min_x, max_x, max_y, min_y = -200, 1000, 1000, -200
    b_l = np.array([min_x, min_y])
    b_r = np.array([max_x, min_y])
    t_l = np.array([min_x, max_y])
    t_r = np.array([max_x, max_y])
    region = [b_l, b_r, t_r, t_l]

    obstacles = []
    while n > 0:
        x_rand = max_x * np.random.random_sample()
        y_rand = max_y * np.random.random_sample()
        b_l = np.array([x_rand - size, y_rand - size])
        b_r = np.array([x_rand + size, y_rand - size])
        t_l = np.array([x_rand - size, y_rand + size])
        t_r = np.array([x_rand + size, y_rand + size])
        o = [b_l, b_r, t_r, t_l]
        if not ray_cast(o, start_pos) and not ray_cast(o, end_pos):
            obstacles.append(o)
            n -= 1

    plot_poly(region, c="#e13c41")
    for o in obstacles:
        plot_poly(o, c="#e13c41")

    map = Map(region=region, obstacles=obstacles)
    map.add_path([start_pos, end_pos])

    return map


# Grid is the number of divisions of our maze we want from
# the starting square
def make_maze(grid):
    min_x, max_x, max_y, min_y = 0, 800, 800, 0
    d = max(max_x, max_y)
    box_length = math.floor(d // grid)
    mid = box_length / 2
    start_pos = np.array([mid, mid])
    end_pos = np.array([d - mid, d - mid])
    obstacles = []

    region = make_rect_region(min_x, max_x, max_y, min_y)
    plot_poly(region, c="#e13c41")
    for i in range(grid - 1):
        obs_pos = []
        new_v = 0
        while True:
            new_v = np.random.randint(new_v + 1, high=new_v + 5)
            obs_pos.append(new_v)
            if new_v >= grid - 1:
                break
        obstacles.append(obs_pos)

    return_obstacles = []
    for i in range(len(obstacles)):
        for j in range(len(obstacles[i]) - 1):
            c0 = obstacles[i][j]
            c1 = c0 + 1
            p0 = np.array([c0 * box_length, (i + 1) * box_length])
            p1 = np.array([c1 * box_length, (i + 1) * box_length])
            rect_obs = expand_obstacle(p0, p1, 10)
            plot_poly(rect_obs, c="#e13c41")
            return_obstacles.append(rect_obs)

    map = Map(region=region, obstacles=return_obstacles)
    map.add_path([start_pos, end_pos])

    return map





















