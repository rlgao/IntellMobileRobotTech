#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This file is like rrt_star_2d.py
import numpy as np

# from src.rrt.rrt import RRT
from src.rrt.rrt_star import RRTStar

from src.search_space.search_space import SearchSpace
# from src.utilities.plotting import Plot


class Use_RRTStar:
    def __init__(self, plan_ox, plan_oy, plan_grid_size, plan_robot_radius):
        print('===  Using RRTStar  ===')

        self.plan_ox = plan_ox  # list of x of obstacles
        self.plan_oy = plan_oy  # list of y of obstacles

        self.plan_grid_size    = plan_grid_size     # 0.3
        self.plan_robot_radius = plan_robot_radius  # 0.8


    def plan(self, plan_sx, plan_sy, plan_gx, plan_gy):
        # dimensions of Search Space
        # X_dimensions = np.array([(0, 100), (0, 100)])
        X_dimensions = np.array([(-10.0, 10.0), (-10.0, 10.0)])

        Obstacles = []
        #######################
        ob_size = 0.2
        #######################
        
        for i in range(len(self.plan_ox)):
            Obstacles.append((self.plan_ox[i] - ob_size, self.plan_oy[i] - ob_size,
                              self.plan_ox[i] + ob_size, self.plan_oy[i] + ob_size))

        Obstacles = np.array(Obstacles)


        # create search space
        X = SearchSpace(X_dimensions, Obstacles)

        x_init = (plan_sx, plan_sy)  # starting location
        x_goal = (plan_gx, plan_gy)  # goal location

        # length of tree edges
        # Q = np.array([(8, 4)])
        # Q[0]: steer distance, step size, edge lengths
        step_size = 1  #0.7  #######################
        Q = np.array([(step_size, 4)])


        # max number of samples to take before timing out
        # max_samples = 1024  
        max_samples = 2048

        # resolution of points to sample along edge when checking for collisions
        r = self.plan_grid_size  #1  # length of smallest edge to check for intersection with obstacles

        # probability of checking for a connection to goal
        prc = 0.1

        # optional, number of nearby branches to rewire
        rewire_count = 32

        # ===================================================
        # create rrt_search
        rrt_star = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
        path = rrt_star.rrt_star_search()
        # ===================================================


        print('Global path points:')
        for path_point in path:
            print(path_point)



        plan_rx = []
        plan_ry = []
        for path_point in path:
            plan_rx.append(path_point[0])
            plan_ry.append(path_point[1])

        return plan_rx, plan_ry

