#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random
import os
import sys
import math

from scipy.optimize import fsolve
import warnings
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

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

def distance(x1, x2, y1, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def degree2radian(deg):
    return deg * math.pi / 180

def radian2degree(rad):
    return rad * 180 / math.pi

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

    return [left_wheel, right_wheel, 0, 0, 0 , 1]

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

def go_to(robot_id, cur_posture, x, y, max_linear_velocity):
    sign = 1
    kd = 5
    ka = 0.4

    dx = x - cur_posture[robot_id][X]
    dy = y - cur_posture[robot_id][Y]
    d_e = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    desired_th = math.atan2(dy, dx)

    d_th = wrap_to_pi(desired_th - cur_posture[robot_id][TH])
    
    if (d_th > degree2radian(90)):
        d_th -= math.pi
        sign = -1
    elif (d_th < degree2radian(-90)):
        d_th += math.pi
        sign = -1

    left_wheel, right_wheel = set_wheel_velocity(max_linear_velocity[robot_id],
                sign * (kd * d_e - ka * d_th), 
                sign * (kd * d_e + ka * d_th))

    return [left_wheel, right_wheel, 0, 0, 0, 1]

def pass_to(robot_id, cur_posture, x, y, max_linear_velocity, prev_posture):
    
    dist = distance(cur_posture[robot_id][X], x, cur_posture[robot_id][Y], y)
    mult_fs = 0.7
    kick_speed = (7 + 1.2 * (dist / 5.07))*mult_fs
    kick_angle = 0

    direction = math.atan2(y - cur_posture[robot_id][Y], x - cur_posture[robot_id][X]) * 4 / math.pi
    if direction > 4:
        direction -= 8

    if abs(cur_posture[robot_id][TH] - math.pi / 4 * direction) > math.pi:
        if 0 <= abs(2 * math.pi + cur_posture[robot_id][TH] - math.pi / 4 * direction) <= math.pi:
            cur_posture[robot_id][TH] += 2 * math.pi
        else:
            cur_posture[robot_id][TH] -= 2 * math.pi

    if cur_posture[robot_id][TH] > math.pi / 4 * direction:
        if cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction > 5:
            w = min(1, (cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * max_linear_velocity[robot_id] / max_linear_velocity[0]))
            if distance(cur_posture[robot_id][X], prev_posture[robot_id][X], cur_posture[robot_id][Y], prev_posture[robot_id][Y]) < 0.01: # corner case
                return [w/2, -w/2, 0, 0, 0, 1]
            return [0.4 + w/2, 0.4 - w/2, 0, 0, 0, 1]
        else:
            return [1, 1, kick_speed, kick_angle, 0, 1]
    else:
        if cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction < -5:
            w = min(1, -(cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * max_linear_velocity[robot_id] / max_linear_velocity[0]))
            if distance(cur_posture[robot_id][X], prev_posture[robot_id][X], cur_posture[robot_id][Y], prev_posture[robot_id][Y]) < 0.01:
                return [-w/2, w/2, 0, 0, 0, 1]
            return [0.4 - w/2, 0.4 + w/2, 0, 0, 0, 1]
        else:
            return [1, 1, kick_speed, kick_angle, 0, 1]

def cross_to(robot_id, cur_posture, cur_ball, x, y, z, max_linear_velocity):
    
    dist = distance(cur_posture[robot_id][X], x, cur_posture[robot_id][Y], y)
    max_cross_angle = 40
    damping = 0.2
    g = 9.81
    mult_fs = 0.7
    max_kick_speed = 10*mult_fs
    mult_angle = 4
    max_kick_angle = 10*mult_angle

    if damping == 0:
        try:
            theta = math.pi*max_cross_angle/180
            v0 = math.sqrt((g * dist * dist) / (2 * (math.cos(theta) ** 2) * (dist * math.tan(theta) - z)))
            while v0 > max_kick_speed:
                theta -= math.pi / 180
                v0 = math.sqrt((self.g * dist * dist) / (2 * (math.cos(theta) ** 2) * (dist * math.tan(theta) - z)))
        except ValueError as e:
            #helper.printConsole(e)
            return None
    else:
        try:
            theta = math.pi*max_cross_angle/180
            while True:
                relative_height_for_time = lambda t: (-g * t / damping) + g * (1 - math.exp(-damping*t)) / (damping**2) + dist * math.tan(theta) - (z - cur_ball[Z])
                t = float(fsolve(relative_height_for_time, 2))
                vx0 = dist * damping / (1 - math.exp(-damping * t))
                vy0 = vx0 * math.tan(theta)
                v0 = math.sqrt(vx0 ** 2 + vy0 ** 2)
                if v0 > max_kick_speed:
                    theta -= math.pi / 180
                    if theta < 0:
                        return None
                    continue
                break
        except ValueError as e:
            #helper.printConsole(e)
            return None

    kick_speed = v0 / mult_fs
    kick_angle = theta * (180 / math.pi) / mult_angle

    direction = math.atan2(y - cur_posture[robot_id][Y], x - cur_posture[robot_id][X]) * 4 / math.pi
    if direction > 4:
        direction -= 8

    if abs(cur_posture[robot_id][TH] - math.pi / 4 * direction) > math.pi:
        if 0 <= abs(2 * math.pi + cur_posture[robot_id][TH] - math.pi / 4 * direction) <= math.pi:
            cur_posture[robot_id][TH] += 2 * math.pi
        else:
            cur_posture[robot_id][TH] -= 2 * math.pi

    if cur_posture[robot_id][TH] > math.pi / 4 * direction:
        if cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction > 5:
            w = min(1, (cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * max_linear_velocity[robot_id] / max_linear_velocity[0]))
            return [w/2, -w/2, 0, 0, 0, 1]
        else:
            if kick_angle > 10:
                return [-1, -1, 0, 0, 0, 1]
            elif kick_speed > 10:
                return [1, 1, 0, 0, 0, 1]
            else:
                return [1, 1, kick_speed, kick_angle, 0, 1]
    else:
        if cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction < -5:
            w = min(1, -(cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * max_linear_velocity[robot_id] / max_linear_velocity[0]))
            return [-w/2, w/2, 0, 0, 0, 1]
        else:
            if kick_speed > 10:
                return [1, 1, 0, 0, 0, 1]
            elif kick_angle > 10:
                return [-1, -1, 0, 0, 0, 1]
            else:
                return [1, 1, kick_speed, kick_angle, 0, 1]

def shoot_to(robot_id, cur_posture, x, y, max_linear_velocity, prev_posture, kick_speed=10, kick_angle=5):

    direction = math.atan2(y - cur_posture[robot_id][Y], x - cur_posture[robot_id][X]) * 4 / math.pi

    if direction > 4:
        direction -= 8

    if abs(cur_posture[robot_id][TH] - math.pi / 4 * direction) > math.pi:
        if 0 <= abs(2 * math.pi + cur_posture[robot_id][TH] - math.pi / 4 * direction) <= math.pi:
            cur_posture[robot_id][TH] += 2 * math.pi
        else:
            cur_posture[robot_id][TH] -= 2 * math.pi

    if cur_posture[robot_id][TH] > math.pi / 4 * direction:
        if cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction < 15:
            return [1, 1, kick_speed, kick_angle, 0, 1]
        else:
            w = min(1, (cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * max_linear_velocity[robot_id] / max_linear_velocity[0]))
            if distance(cur_posture[robot_id][X], prev_posture[robot_id][X], cur_posture[robot_id][Y], prev_posture[robot_id][Y]) < 0.01: # corner case
                return [w/2, -w/2, 0, 0, 0, 1]
            return [0.75 + w/2, 0.75 - w/2, 0, 0, 0, 1]
    else:
        if cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction > -15:
            return [1, 1, kick_speed, kick_angle, 0, 1]
        else:
            w = min(1, -(cur_posture[robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * max_linear_velocity[robot_id] / max_linear_velocity[0]))
            if distance(cur_posture[robot_id][X], prev_posture[robot_id][X], cur_posture[robot_id][Y], prev_posture[robot_id][Y]) < 0.01:
                return [-w/2, w/2, 0, 0, 0, 1]
            return [0.75 - w/2, 0.75 + w/2, 0, 0, 0, 1]

def get_action(index, max):
    ACTIONS = [[1*max, 1*max, 0, 0, 0, 1],
                [-1*max, -1*max, 0, 0, 0, 1],
                [0.1*max, -0.1*max, 0, 0, 0, 1],
                [-0.1*max, 0.1*max, 0, 0, 0, 1],
                [0, 0, 10, 5, 0, 1],
                [0, 0, 0, 0, 0, 1]]
    return ACTIONS[index]