#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random
import os
import sys
import math

NUM_ACTIONS = 6
GO_FORWARD = 0
GO_BACKWARD = 1
TURN_RIGHT = 2
TURN_LEFT = 3
KICK = 4
STOP = 5

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
Z = 2
TH = 3
ACTIVE = 4
TOUCH = 5
BALL_POSSESSION = 6

G = 9.81

def wrap_to_pi(theta):
    while (theta > math.pi):
        theta -= 2 * math.pi
    while (theta < -math.pi):
        theta += 2 * math.pi
    return theta

def set_wheel_velocity(max_linear_velocity, left_wheel, right_wheel):
    ratio_l = 1
    ratio_r = 1

    if (left_wheel > max_linear_velocity or right_wheel > max_linear_velocity):
        diff = max(left_wheel, right_wheel) - max_linear_velocity
        left_wheel -= diff
        right_wheel -= diff
    if (left_wheel < -max_linear_velocity or right_wheel < -max_linear_velocity):
        diff = min(left_wheel, right_wheel) + max_linear_velocity
        left_wheel -= diff
        right_wheel -= diff

    return left_wheel, right_wheel

def predict_ball(cur_ball, previous_ball, prediction_step = 1):
    dx = cur_ball[X] - previous_ball[X]
    dy = cur_ball[Y] - previous_ball[Y]
    dz = cur_ball[Z] - previous_ball[Z]
    predicted_ball = [cur_ball[X] + prediction_step*dx, cur_ball[Y] + prediction_step*dy, max(0.05, cur_ball[Z] + prediction_step*dz -(G*0.05*prediction_step*0.05*prediction_step)/2)]
    return predicted_ball

def turn_to(robot_id, x, y, my_robot, max_linear_velocity):
    ka = 0.2
    tot = math.pi/360

    dx = x - my_robot[X]
    dy = y - my_robot[Y]
    desired_th = math.atan2(dy, dx)
    d_th = wrap_to_pi(desired_th - my_robot[TH])
    
    if (abs(d_th) < tot):
        ka = 0
    
    left_wheel, right_wheel = set_wheel_velocity(max_linear_velocity,
                                                        -ka*d_th,
                                                        ka*d_th)

    return [left_wheel, right_wheel, 0, 0, 0 , 0]

def defend_ball(robot_id, my_robot, cur_ball, prev_ball):
    if robot_id != 0:
        return None

    # GK takes 250ms to perform defense move
    predicted_ball_gk = predict_ball(cur_ball, prev_ball, 5)

    if predicted_ball_gk[X] < my_robot[X] + 0.1:
        # right part of the goal
        if -0.65 < predicted_ball_gk[Y] < -0.07:
            # top part of the goal
            if (predicted_ball_gk[Z] > 0.25):
                return [0, 0, 0, 0, 7, 0]
            else:
                return [0, 0, 0, 0, 6, 0]
        # center part of the goal
        if -0.07 < predicted_ball_gk[Y] < 0.07:
            # top part of the goal
            if (predicted_ball_gk[Z] > 0.25):
                return [0, 0, 0, 0, 8, 0]
            else:
                return [0, 0, 0, 0, 3, 0]
        # left part of the goal
        if 0.07 < predicted_ball_gk[Y] < 0.65:
            # top part of the goal
            if (predicted_ball_gk[Z] > 0.25):
                return [0, 0, 0, 0, 9, 0]
            else:
                return [0, 0, 0, 0, 10, 0]
    else:
        return None

def get_action(index, max):
    ACTIONS = [[1*max, 1*max, 0, 0, 0, 1],
                [-1*max, -1*max, 0, 0, 0, 1],
                [0.2*max, -0.2*max, 0, 0, 0, 1],
                [-0.2*max, 0.2*max, 0, 0, 0, 1],
                [0, 0, 10, 5, 0, 1],
                [0, 0, 0, 0, 0, 1]]
    return ACTIONS[index]