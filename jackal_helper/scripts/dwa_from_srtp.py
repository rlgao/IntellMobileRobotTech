#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from enum import Enum

import numpy as np

from sklearn import preprocessing  # for minmax_scale

class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """
    def __init__(self):
        """ # robot parameter

        # -----------------------------------
        # self.max_speed = 0.7  # [m/s]
        # self.max_speed = 0.4  # [m/s]
        # self.min_speed = -self.max_speed  # [m/s]
        self.max_speed = 0.5  #2.0
        self.min_speed = 0.0

        # self.max_yawrate = 200.0 * math.pi / 180.0  # [rad/s]
        self.max_yawrate = 60.0 * math.pi / 180.0  # [rad/s]
        # self.max_yawrate = math.pi / 3 * 2  # [rad/s]
        # -----------------------------------

        # self.max_accel = 0.4  # [m/ss]
        self.max_accel = 0.7  # [m/ss]
        # self.max_dyawrate = 200.0 * math.pi / 180.0  # [rad/ss]
        self.max_dyawrate = math.pi / 3 * 5  # [rad/ss]

        self.dt = 0.1  # [s] Time tick for motion prediction
        # self.v_reso = self.max_accel * self.dt / 10.0  # [m/s]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 0.1  # [rad/s]
        # self.predict_time = 2  # [s]
        self.predict_time = 1.0  # [s]
        
        # -----------------------------------------
        self.to_goal_cost_gain = 0.6  #0.4  #0.6  #1.0
        self.speed_cost_gain = 0.7  #0.8  #0.7  #0.1
        self.obstacle_cost_gain = 0.3  #1.0
        self.ob_gain = self.obstacle_cost_gain  ######
        # -----------------------------------------

        self.robot_type = RobotType.rectangle """

        # robot parameter
        self.max_speed = 0.7   # [m/s]
        self.min_speed = -0.7  # [m/s]
        self.max_yawrate = 200.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.4  # [m/ss]
        # self.max_dyawrate = 200.0 * math.pi / 180.0  # [rad/ss]
        # self.max_dyawrate =  2 * 200.0 * math.pi / 180.0  # [rad/ss]
        self.max_dyawrate = 3 * 200.0 * math.pi / 180.0  # [rad/ss]


        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]

        self.to_goal_cost_gain = 0.8  #0.6  #0.4  #0.6  #1.0
        self.speed_cost_gain = 0.2  #0.8  #0.7  #0.1
        self.obstacle_cost_gain = 0.3  #0.2  #1.0

        self.robot_type = RobotType.rectangle


        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.6  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.6  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    '''
    motion model
    ''' 
    # x: x, y, theta, v, w
    # x:x[0], y:x[1], theta:x[2]
    # u: v, w
    # v:u[0], w:u[1]

    # x'
    x[0] = x[0] - u[0] / u[1] * math.sin(x[2]) \
         + u[0] / u[1] * math.sin(x[2] + u[1] * dt)

    # y'
    x[1] = x[1] + u[0] / u[1] * math.cos(x[2]) \
         - u[0] / u[1] * math.cos(x[2] + u[1] * dt)   

    x[2] += u[1] * dt  # theta'

    x[3] = u[0]  # v'
    x[4] = u[1]  # w'

    return x


def calc_dynamic_window(x, config):
    '''
    get dynamic_window based on current state x
    '''
    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    # print('dw:')
    # print(dw)

    return dw


def calc_trajectory(xinit, v, w, config):
    '''
    get trajectory based on current state x, and given input v, w
    '''
    x = np.array(xinit)
    traj = np.array(x)

    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, w], config.dt)
        traj = np.vstack((traj, x))  ############
        time += config.dt

    return traj


def calc_best_input(x, u, dw, config, goal, ob):

    xinit = x[:]

    min_cost = 10000.0
    min_u = u
    # min_u[0] = 0.0

    best_traj = np.array([x])

    traj_list = []
    cost_list = []
    u_list = []

    # evalucate all trajectories with sampled inputs in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for w in np.arange(dw[2], dw[3], config.yawrate_reso):

            # print('sample v = ' + str(v))
            # print('sample w = ' + str(w))

            traj = calc_trajectory(xinit, v, w, config)

            # print('sample traj = ')
            # print(traj)


            ########## calc costs ##########
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            # print('to_goal_cost = ' + str(to_goal_cost))

            # speed_cost = config.speed_cost_gain * \
            #     (config.max_speed - traj[-1, 3])
            speed_cost = config.max_speed - traj[-1, 3]
            # print('speed_cost = ' + str(speed_cost))

            ob_cost = calc_obstacle_cost(traj, ob, config)
            # print('ob_cost = ' + str(ob_cost))
            
            final_cost = to_goal_cost + speed_cost + ob_cost
            #print (final_cost)

            if not np.isinf(final_cost):
                traj_list.append(traj)
                cost_list.append([to_goal_cost, speed_cost, ob_cost])
                # print([to_goal_cost, speed_cost, ob_cost])
                u_list.append([v, w])

            # search minimum trajectory
            # if min_cost >= final_cost:
            #     min_cost = final_cost
            #     min_u = [v, y]
            #     best_traj = traj

    # normalize cost
    if len(cost_list) > 0:
        cost_list = np.array(cost_list)
        cost_var = np.var(cost_list, axis=0)  # variance

        for j in range(len(cost_var)):
            if cost_var[j] != 0.0:
                cost_list[:,j] = preprocessing.minmax_scale(cost_list[:,j])

        # cost_list[:, 0] /= sum(cost_list[:, 0])
        # cost_list[:, 1] /= sum(cost_list[:, 1])
        # cost_list[:, 2] /= sum(cost_list[:, 2])
        cost_sum = cost_list[:, 0] * config.to_goal_cost_gain \
                 + cost_list[:, 1] * config.speed_cost_gain   \
                 + cost_list[:, 2] * config.obstacle_cost_gain  #config.ob_gain
        cost_sum = cost_sum.tolist()

        # print('cost_sum =')
        # print(cost_sum)


        best_index = cost_sum.index(min(cost_sum))
        min_u = u_list[best_index]
        best_traj = traj_list[best_index]

        # print('best_index = ' + str(best_index))
        # print('min_u = ' + str(min_u))
        # print('best_traj = ' + str(best_traj))


    return min_u, best_traj



def calc_obstacle_cost(traj, ob, config):
    '''
    calc obstacle cost, inf: collistion, 0:free
    '''

    # def filter_ob(ob_list, x, active_distance):
    #     def is_in_active_distance(ob):
    #         dis = np.linalg.norm(np.array(ob) - x[:2]) - config.obstacle_radius
    #         return dis <= active_distance

    #     filtered_ob_list = list(filter(is_in_active_distance, ob_list))
    #     return np.array(filtered_ob_list)
    # ob = filter_ob(ob.tolist(), traj[0], 5.0)


    skip_n = 1
    minr = float("inf")

    if ob is None:
        return 0
    if len(ob) is 0:
        return 0

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy
            r = math.sqrt(dx**2 + dy**2)

            # -------------------------------
            # if r <= config.robot_radius + config.obstacle_radius:
            #     return float("Inf")  # collision
            # if r <= config.robot_length:
            #     return float("Inf")  # collision
            # -------------------------------

            if minr >= r:
                minr = r


    # print('minr = ' + str(minr))

    # return 1.0 / abs(minr - config.robot_radius - config.obstacle_radius)  # OK
    # return 1.0 / abs(minr - config.robot_length)  # OK
    # return 1.0 / abs(minr)  # OK
    return 2.0 - abs(minr)  # OK


def calc_to_goal_cost(traj, goal, config):
    '''
    calc to goal cost, heading
    ''' 

    def normalize_angle(angle):
        norm_angle = angle % (2 * math.pi)
        if norm_angle > math.pi:
            norm_angle -= 2 * math.pi
        return norm_angle

    # goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
    # traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
    # dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
    # error = dot_product / (goal_magnitude * traj_magnitude)
    # error_angle = math.acos(error)
    # cost = config.to_goal_cost_gain * error_angle
    # mid_point = int(round(config.predict_time/config.dt/2))

    # mid_point = 4
    mid_point = int(config.predict_time / config.dt / 2)
    
    goal_line_angle = math.atan2(goal[1] - traj[mid_point, 1], goal[0]-traj[mid_point, 0])
    traj_angle = traj[mid_point, 2]  # theta
    error_angle = abs(normalize_angle(goal_line_angle-traj_angle))

    # cost = config.to_goal_cost_gain * error_angle
    cost = error_angle  ######

    return cost




class DWA:
    def __init__(self, config):
        self.config = config
        pass


    def plan(self, x, goal, ob):
        """
        Dynamic Window Approach control
        """
        ##TODO

        # trajectory = None
        # vx = 0.5  #0.5
        # vw = -0.25  #1.0
        # u = [vx, vw]


        config = self.config

        dw = calc_dynamic_window(x, config)

        u = [x[3], x[4]]
        u, trajectory = calc_best_input(x, u, dw, config, goal, ob)

        # if abs(u[1]) > 1.0:
        #     u[0] = 0.0

        return u, trajectory
