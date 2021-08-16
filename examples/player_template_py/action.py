#!/usr/bin/env python3

# Author(s): Taeyoung Kim, Chansol Hong, Luiz Felipe Vecchietti
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    from participant import Game, Frame
except ImportError as err:
    print('player_template: \'participant\' module cannot be imported:', err)
    raise

import math
import helper

from scipy.optimize import fsolve
import warnings
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

#coordinates
MY_TEAM = Frame.MY_TEAM
OP_TEAM = Frame.OP_TEAM
BALL = Frame.BALL
X = Frame.X
Y = Frame.Y
Z = Frame.Z
TH = Frame.TH
ACTIVE = Frame.ACTIVE
TOUCH = Frame.TOUCH
BALL_POSSESSION = Frame.BALL_POSSESSION

# pass/cross matrix
PASS_MATRIX = [[0,0,0],[0,0,0],[0,0,0]]
CROSS_MATRIX = [[0,0,0],[0,0,0],[0,0,0]]

class ActionControl:

    def __init__(self, robot_id, max_linear_velocity):
        self.max_linear_velocity = max_linear_velocity
        self.max_jump_velocity = 8.5
        self.g = 9.81 # gravity
        self.h = 0.421 # height of robot
        self.cross_height = 0.85
        self.damping = 0.2 # linear damping
        self.mult_fs = 0.7 
        self.max_kick_speed = 10*self.mult_fs # 7.5 m/s
        self.mult_angle = 5
        self.max_kick_angle = 10*self.mult_angle # 50 degrees

        self.robot_id = robot_id
        self.cur_posture = []
        self.prev_posture = []
        self.cur_posture_opp = []
        self.prev_posture_opp = []
        self.cur_ball = []
        self.prev_ball = []

        self.pass_count = 0
        self.cross_count = 0

    def update_state(self, cur_posture, prev_posture, cur_ball, prev_ball):
        self.cur_posture = cur_posture
        self.prev_posture = prev_posture
        self.cur_ball = cur_ball
        self.prev_ball = prev_ball
        self.update_counter()
        #self.reset_matrix()

    def update_counter(self):
        if self.pass_count >= 1:
            self.pass_count += 1
        if self.cross_count >= 1:
            self.cross_count += 1

    def reset_matrix(self):
        if self.pass_count > 20:
            self.pass_count = 0
            PASS_MATRIX = [[0,0,0],[0,0,0],[0,0,0]]
        if self.cross_count > 20:
            self.cross_count = 0
            CROSS_MATRIX = [[0,0,0],[0,0,0],[0,0,0]]

    def manual_control(self, left_wheel, right_wheel, kick_speed, kick_angle, jump_speed, dribble_mode):
        return [left_wheel, right_wheel, kick_speed, kick_angle, jump_speed, dribble_mode]

    def jump(self, jump_speed = 10):
        return [0, 0, 0, 0, jump_speed, 0]

    def go_to_jump(self, x, y, jump_control):
        sign = 1

        if self.cur_posture[BALL_POSSESSION]:
            kd = 0
            ka = 0.3
        # if kick_control < 0:
        #     kd = 0
        #     ka = 0.3
        # else:
        #     kd = 5
        #     ka = 0.3
        else:
            kd = 5
            ka = 0.3        

        dx = x - self.cur_posture[X]
        dy = y - self.cur_posture[Y]
        d_e = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        desired_th = math.atan2(dy, dx)

        d_th = helper.wrap_to_pi(desired_th - self.cur_posture[TH])
    
        if not self.cur_posture[BALL_POSSESSION]:
            if (d_th > helper.degree2radian(90)):
                d_th -= math.pi
                sign = -1
            elif (d_th < helper.degree2radian(-90)):
                d_th += math.pi
                sign = -1

        jump_speed = 0
        if jump_control < 0 : 
            jump_speed = 0
        else : 
            jump_speed = min(10, 10*jump_control)

        left_wheel, right_wheel = helper.set_wheel_velocity(self.max_linear_velocity,
                sign * (kd * d_e - ka * d_th), 
                sign * (kd * d_e + ka * d_th))
    
        #return [left_wheel, right_wheel, 0, 0, jump_speed, 0]
        if jump_speed > 0 :
            return [0, 0, 0, 0, jump_speed, 0]
        else : 
            return [left_wheel, right_wheel, 0, 0, 0, 0]

    def go_to(self, x, y):
        sign = 1
        # kd = 7 if ((self.robot_id == 1) or (self.robot_id == 2)) else 5
        kd = 5
        # ka = 0.3
        ka = 0.4

        # tod = 0.005 # tolerance of distance
        # tot = math.pi/360 # tolerance of theta

        dx = x - self.cur_posture[self.robot_id][X]
        dy = y - self.cur_posture[self.robot_id][Y]
        d_e = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        desired_th = math.atan2(dy, dx)

        d_th = helper.wrap_to_pi(desired_th - self.cur_posture[self.robot_id][TH])
        
        if not self.cur_posture[self.robot_id][BALL_POSSESSION]:
            if (d_th > helper.degree2radian(90)):
                d_th -= math.pi
                sign = -1
            elif (d_th < helper.degree2radian(-90)):
                d_th += math.pi
                sign = -1

        # if (d_e < tod):
        #     kd = 0
        # if (abs(d_th) < tot):
        #     ka = 0

        # if self.go_fast():
        #     kd *= 5

        left_wheel, right_wheel = helper.set_wheel_velocity(self.max_linear_velocity[self.robot_id],
                    sign * (kd * d_e - ka * d_th), 
                    sign * (kd * d_e + ka * d_th))

        return [left_wheel, right_wheel, 0, 0, 0, 0]

    def go_fast(self):
        distance2ball = helper.distance(self.cur_ball[X], self.cur_posture[self.robot_id][X],
                                    self.cur_ball[Y], self.cur_posture[self.robot_id][Y])
        d_bg = helper.distance(self.cur_ball[X], 3.9,
                                    self.cur_ball[Y], 0)
        d_rg = helper.distance(3.9, self.cur_posture[self.robot_id][X],
                                    0, self.cur_posture[self.robot_id][Y])
        
        if (distance2ball < 0.25 and d_rg > d_bg):
            if (self.cur_ball[X] > 3.7 and abs(self.cur_ball[Y]) > 0.5 and abs(self.cur_posture[self.robot_id][TH]) < 30 * math.pi/180):
                return False
            else:
                return True
        else:
            return False

    def turn_to(self, x, y):
        ka = 0.2
        tot = math.pi/360

        dx = x - self.cur_posture[self.robot_id][X]
        dy = y - self.cur_posture[self.robot_id][Y]
        desired_th = math.atan2(dy, dx)
        d_th = helper.wrap_to_pi(desired_th - self.cur_posture[self.robot_id][TH])
        
        if (abs(d_th) < tot):
            ka = 0
        
        left_wheel, right_wheel = helper.set_wheel_velocity(self.max_linear_velocity[self.robot_id],
                                                                -ka*d_th,
                                                                ka*d_th)

        return [left_wheel, right_wheel, 0, 0, 0 , 0]

    def defend_ball(self):
        if self.robot_id != 0:
            return None

        # GK takes 250ms to perform defense move
        predicted_ball_gk = helper.predict_ball(self.cur_ball, self.prev_ball, 5)

        if predicted_ball_gk[X] < self.cur_posture[self.robot_id][X] + 0.1:
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

    def is_gk_save_possible(self):
        return self.defend_ball() != None

    def pass_to(self, x, y, pass_matrix):
        
        dist = helper.distance(self.cur_posture[self.robot_id][X], x, self.cur_posture[self.robot_id][Y], y)
        kick_speed = (7 + 1.5 * (dist / 5.07))*self.mult_fs
        kick_angle = 0

        direction = math.atan2(y - self.cur_posture[self.robot_id][Y], x - self.cur_posture[self.robot_id][X]) * 4 / math.pi
        if direction > 4:
            direction -= 8

        if abs(self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) > math.pi:
            if 0 <= abs(2 * math.pi + self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) <= math.pi:
                self.cur_posture[self.robot_id][TH] += 2 * math.pi
            else:
                self.cur_posture[self.robot_id][TH] -= 2 * math.pi

        if self.cur_posture[self.robot_id][TH] > math.pi / 4 * direction:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction > 5:
                w = min(1, (self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                if helper.distance(self.cur_posture[self.robot_id][X], self.prev_posture[self.robot_id][X], self.cur_posture[self.robot_id][Y], self.prev_posture[self.robot_id][Y]) < 0.01: # corner case
                    return [w/2, -w/2, 0, 0, 0, 0]
                return [0.4 + w/2, 0.4 - w/2, 0, 0, 0, 0]
            else:
                target_robot_id = helper.find_closest_robot([x,y,0], self.cur_posture, 3)
                if pass_matrix[self.robot_id][target_robot_id] == 0:
                    pass_matrix[self.robot_id][target_robot_id] = 1
                return [1, 1, kick_speed, kick_angle, 0, 0]
        else:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction < -5:
                w = min(1, -(self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                if helper.distance(self.cur_posture[self.robot_id][X], self.prev_posture[self.robot_id][X], self.cur_posture[self.robot_id][Y], self.prev_posture[self.robot_id][Y]) < 0.01:
                    return [-w/2, w/2, 0, 0, 0, 0]
                return [0.4 - w/2, 0.4 + w/2, 0, 0, 0, 0]
            else:
                target_robot_id = helper.find_closest_robot([x,y,0], self.cur_posture, 3)
                if pass_matrix[self.robot_id][target_robot_id] == 0:
                    pass_matrix[self.robot_id][target_robot_id] = 1
                return [1, 1, kick_speed, kick_angle, 0, 0]

    def pass_to_robot(self, target_robot_id, pass_matrix):
        
        x = self.cur_posture[target_robot_id][X]
        y = self.cur_posture[target_robot_id][Y]
        
        dist = helper.distance(self.cur_posture[self.robot_id][X], x, self.cur_posture[self.robot_id][Y], y)
        kick_speed = (7 + 1.5 * (dist / 5.07))*self.mult_fs
        kick_angle = 0

        direction = math.atan2(y - self.cur_posture[self.robot_id][Y], x - self.cur_posture[self.robot_id][X]) * 4 / math.pi
        if direction > 4:
            direction -= 8

        if abs(self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) > math.pi:
            if 0 <= abs(2 * math.pi + self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) <= math.pi:
                self.cur_posture[self.robot_id][TH] += 2 * math.pi
            else:
                self.cur_posture[self.robot_id][TH] -= 2 * math.pi

        if self.cur_posture[self.robot_id][TH] > math.pi / 4 * direction:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction > 5:
                w = min(1, (self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                if helper.distance(self.cur_posture[self.robot_id][X], self.prev_posture[self.robot_id][X], self.cur_posture[self.robot_id][Y], self.prev_posture[self.robot_id][Y]) < 0.01: # corner case
                    return [w/2, -w/2, 0, 0, 0, 0]
                return [0.4 + w/2, 0.4 - w/2, 0, 0, 0, 0]
            else:
                if pass_matrix[self.robot_id][target_robot_id] == 0:
                    pass_matrix[self.robot_id][target_robot_id] = 1
                return [1, 1, kick_speed, kick_angle, 0, 0]
        else:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction < -5:
                w = min(1, -(self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                if helper.distance(self.cur_posture[self.robot_id][X], self.prev_posture[self.robot_id][X], self.cur_posture[self.robot_id][Y], self.prev_posture[self.robot_id][Y]) < 0.01:
                    return [-w/2, w/2, 0, 0, 0, 0]
                return [0.4 - w/2, 0.4 + w/2, 0, 0, 0, 0]
            else:
                if pass_matrix[self.robot_id][target_robot_id] == 0:
                    pass_matrix[self.robot_id][target_robot_id] = 1
                return [1, 1, kick_speed, kick_angle, 0, 0]

    def is_receiving_a_pass(self, pass_matrix):
        receiving = False 
        for i in range(3):
            if pass_matrix[i][self.robot_id]:
                receiving = True
        return receiving

    def cross_to(self, x, y, z, cross_matrix):
        
        dist = helper.distance(self.cur_posture[self.robot_id][X], x, self.cur_posture[self.robot_id][Y], y)
        max_cross_angle = 40

        if self.damping == 0:
            try:
                theta = math.pi*max_cross_angle/180
                v0 = math.sqrt((self.g * dist * dist) / (2 * (math.cos(theta) ** 2) * (dist * math.tan(theta) - z)))
                while v0 > self.max_kick_speed:
                    theta -= math.pi / 180
                    v0 = math.sqrt((self.g * dist * dist) / (2 * (math.cos(theta) ** 2) * (dist * math.tan(theta) - z)))
            except ValueError as e:
                #helper.printConsole(e)
                return None
        else:
            try:
                theta = math.pi*max_cross_angle/180
                while True:
                    relative_height_for_time = lambda t: (-self.g * t / self.damping) + self.g * (1 - math.exp(-self.damping*t)) / (self.damping**2) + dist * math.tan(theta) - (z - self.cur_ball[Z])
                    t = float(fsolve(relative_height_for_time, 2))
                    vx0 = dist * self.damping / (1 - math.exp(-self.damping * t))
                    vy0 = vx0 * math.tan(theta)
                    v0 = math.sqrt(vx0 ** 2 + vy0 ** 2)
                    if v0 > self.max_kick_speed:
                        theta -= math.pi / 180
                        if theta < 0:
                            return None
                        continue
                    break
            except ValueError as e:
                #helper.printConsole(e)
                return None

        kick_speed = v0 / self.mult_fs
        kick_angle = theta * (180 / math.pi) / self.mult_angle

        direction = math.atan2(y - self.cur_posture[self.robot_id][Y], x - self.cur_posture[self.robot_id][X]) * 4 / math.pi
        if direction > 4:
            direction -= 8

        if abs(self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) > math.pi:
            if 0 <= abs(2 * math.pi + self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) and abs(2 * math.pi + self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) <= math.pi:
                self.cur_posture[self.robot_id][TH] += 2 * math.pi
            else:
                self.cur_posture[self.robot_id][TH] -= 2 * math.pi

        if self.cur_posture[self.robot_id][TH] > math.pi / 4 * direction:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction > 1:
                w = min(1, (self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                return [w/2, -w/2, 0, 0, 0, 0]
            else:
                if kick_angle > 10:
                    return [-1, -1, 0, 0, 0, 0]
                elif kick_speed > 10:
                    return [1, 1, 0, 0, 0, 0]
                else:
                    target_robot_id = helper.find_closest_robot([x,y,0], self.cur_posture, 3)
                    if cross_matrix[self.robot_id][target_robot_id] == 0:
                        cross_matrix[self.robot_id][target_robot_id] = 1
                    return [1, 1, kick_speed, kick_angle, 0, 0]
        else:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction < -1:
                w = min(1, -(self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                return [-w/2, w/2   , 0, 0, 0, 0]
            else:
                if kick_speed > 10:
                    return [1, 1, 0, 0, 0, 0]
                elif kick_angle > 10:
                    return [-1, -1, 0, 0, 0, 0]
                else:
                    target_robot_id = helper.find_closest_robot([x,y,0], self.cur_posture, 3)
                    if cross_matrix[self.robot_id][target_robot_id] == 0:
                        cross_matrix[self.robot_id][target_robot_id] = 1
                    return [1, 1, kick_speed, kick_angle, 0, 0]

    def is_cross_to_possible(self, x, y, z, cross_matrix):
        return self.cross_to(x, y, z, cross_matrix) != None

    def cross_to_robot(self, target_robot_id, cross_matrix):
        
        x = self.cur_posture[target_robot_id][X]
        y = self.cur_posture[target_robot_id][Y]
        th = self.cur_posture[target_robot_id][TH]
        d_range = 0.35
        z = 0.85

        x = x + d_range*math.cos(th)
        y = y + d_range*math.sin(th)

        return self.cross_to(x, y, z, cross_matrix)

    def is_cross_to_robot_possible(self, target_robot_id):
        return self.cross_to_robot(target_robot_id) != None

    def is_receiving_a_cross(self, cross_matrix):
        receiving = False 
        for i in range(3):
            if cross_matrix[i][self.robot_id]:
                receiving = True
        return receiving

    def shoot_to(self, x, y, kick_speed=10, kick_angle=4):

        direction = math.atan2(y - self.cur_posture[self.robot_id][Y], x - self.cur_posture[self.robot_id][X]) * 4 / math.pi

        if direction > 4:
            direction -= 8

        if abs(self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) > math.pi:
            if 0 <= abs(2 * math.pi + self.cur_posture[self.robot_id][TH] - math.pi / 4 * direction) <= math.pi:
                self.cur_posture[self.robot_id][TH] += 2 * math.pi
            else:
                self.cur_posture[self.robot_id][TH] -= 2 * math.pi

        if self.cur_posture[self.robot_id][TH] > math.pi / 4 * direction:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction < 15:
                return [1, 1, kick_speed, kick_angle, 0, 0]
            else:
                w = min(1, (self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                if helper.distance(self.cur_posture[self.robot_id][X], self.prev_posture[self.robot_id][X], self.cur_posture[self.robot_id][Y], self.prev_posture[self.robot_id][Y]) < 0.01: # corner case
                    return [w/2, -w/2, 0, 0, 0, 1]
                return [0.75 + w/2, 0.75 - w/2, 0, 0, 0, 0]
        else:
            if self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction > -15:
                return [1, 1, kick_speed, kick_angle, 0, 0]
            else:
                w = min(1, -(self.cur_posture[self.robot_id][TH] * 180 / math.pi - 45 * direction) / (70 * self.max_linear_velocity[self.robot_id] / self.max_linear_velocity[0]))
                if helper.distance(self.cur_posture[self.robot_id][X], self.prev_posture[self.robot_id][X], self.cur_posture[self.robot_id][Y], self.prev_posture[self.robot_id][Y]) < 0.01:
                    return [-w/2, w/2, 0, 0, 0, 1]
                return [0.75 - w/2, 0.75 + w/2, 0, 0, 0, 0]

    def header_old(self):

        if self.cur_ball[Z] <= 0.3:
            return self.STOP()

        ball_velocity = helper.predict_ball_velocity(self.cur_ball, self.prev_ball, 0.05)
        c = self.damping

        if c == 0:
            try:
                time = (ball_velocity[Z] + math.sqrt(ball_velocity[Z] ** 2 - 2 * self.g * (self.h - 0.032 - cur_ball[Z]))) / self.g
                ball_predicted_x = self.cur_ball[X] + ball_velocity[X] * time
                ball_predicted_y = self.cur_ball[Y] + ball_velocity[Y] * time
            except:
                return [0, 0, 0, 0, 0, 0]
        else:
            try:
                relative_height_for_time = lambda t: (- self.g * t / c) + (self.g + c * ball_velocity[Z]) * (1 - math.exp(- c * t)) / (c ** 2) - (self.cross_height - 0.032 - self.cur_ball[Z])
                # relative_height_for_time = lambda t: (-self.g * t / self.damping) + self.g * (1 - math.exp(-self.damping*t)) / (self.damping**2) + dist * math.tan(theta) - (z - self.cur_ball[Z])
                time = float(fsolve(relative_height_for_time, 2))
                ball_predicted_x = self.cur_ball[X] + ball_velocity[X] * (1 - math.exp(- c * time)) / c
                ball_predicted_y = self.cur_ball[Y] + ball_velocity[Y] * (1 - math.exp(- c * time)) / c
            except Exception as e:
                helper.printConsole("exception occured")
                helper.printConsole(e)
                return [0, 0, 0, 0, 0, 0]

        theta_robot2ball = math.atan2(ball_predicted_y - self.cur_posture[self.robot_id][Y], ball_predicted_x - self.cur_posture[self.robot_id][X])
        theta_bv = math.atan2(ball_velocity[Y], ball_velocity[X])
        theta_ball2goal = math.atan2(0 - ball_predicted_y, 5.07 - ball_predicted_x)
        theta = (2 * theta_ball2goal - theta_bv) % (2 * math.pi)
        # need to consider exception
        a = ball_predicted_x - 0.064375 * math.cos(theta_robot2ball)
        b = ball_predicted_y - 0.064375 * math.sin(theta_robot2ball)

        theta = math.atan2(ball_predicted_y - self.cur_posture[self.robot_id][Y], ball_predicted_x - self.cur_posture[self.robot_id][X])
        target_x = ball_predicted_x - 0.064375 * math.cos(theta)
        target_y = ball_predicted_y - 0.064375 * math.sin(theta)
        theta = math.atan2(target_y - self.cur_posture[self.robot_id][Y], target_x - self.cur_posture[self.robot_id][X])
        distance = helper.distance(self.cur_posture[self.robot_id][X], target_x, self.cur_posture[self.robot_id][Y], target_y)
        velocity = distance / time

        # direction = theta
        theta_ball2goal = math.atan2(0 - target_y, 5.07 - target_x)
        direction = theta_ball2goal

        th_deg = helper.radian2degree(self.cur_posture[self.robot_id][TH])
        direction_deg = helper.radian2degree(direction)
        
        if time < 0.25:
            return [0, 0, 0, 0, 8.5, 0]
        else:
            if distance > 0.5:
                return self.go_to(target_x, target_y)
            else:
                return self.turn_to(5.07, 0)

    def header(self, ball_predicted, predicted_time):

        if self.cur_ball[Z] <= 0.3:
            return self.STOP()

        ball_velocity = helper.predict_ball_velocity(self.cur_ball, self.prev_ball, 0.05)
        c = self.damping

        time = predicted_time * 0.05
        if not time : 
            return [0,0,0,0,0,0]
        ball_predicted_x = ball_predicted[X]
        ball_predicted_y = ball_predicted[Y]

        theta_robot2ball = math.atan2(ball_predicted_y - self.cur_posture[self.robot_id][Y], ball_predicted_x - self.cur_posture[self.robot_id][X])
        theta_bv = math.atan2(ball_velocity[Y], ball_velocity[X])
        theta_ball2goal = math.atan2(0 - ball_predicted_y, 5.07 - ball_predicted_x)
        theta = (2 * theta_ball2goal - theta_bv) % (2 * math.pi)
        # need to consider exception
        a = ball_predicted_x - 0.064375 * math.cos(theta_robot2ball)
        b = ball_predicted_y - 0.064375 * math.sin(theta_robot2ball)

        theta = math.atan2(ball_predicted_y - self.cur_posture[self.robot_id][Y], ball_predicted_x - self.cur_posture[self.robot_id][X])
        target_x = ball_predicted_x - 0.064375 * math.cos(theta)
        target_y = ball_predicted_y - 0.064375 * math.sin(theta)
        theta = math.atan2(target_y - self.cur_posture[self.robot_id][Y], target_x - self.cur_posture[self.robot_id][X])
        distance = helper.distance(self.cur_posture[self.robot_id][X], target_x, self.cur_posture[self.robot_id][Y], target_y)
        velocity = distance / time

        # direction = theta
        theta_ball2goal = math.atan2(0 - target_y, 5.07 - target_x)
        direction = theta_ball2goal

        th_deg = helper.radian2degree(self.cur_posture[self.robot_id][TH])
        direction_deg = helper.radian2degree(direction)

        if time < 0.25:
            return [0, 0, 0, 0, 8.5, 0]
        else:
            if distance > 0.5:
                return self.go_to(target_x, target_y)
            else:
                return self.turn_to(5.07, 0)

    def GO_FORWARD(self):
        return [1*self.max_linear_velocity[self.robot_id], 1*self.max_linear_velocity[self.robot_id], 0, 0, 0, 0]

    def GO_BACKWARD(self):
        return [-1*self.max_linear_velocity[self.robot_id], -1*self.max_linear_velocity[self.robot_id], 0, 0, 0, 0]

    def TURN_RIGHT(self):
        return [0.05*self.max_linear_velocity[self.robot_id], -0.05*self.max_linear_velocity[self.robot_id], 0, 0, 0, 0]

    def TURN_LEFT(self):
        return [-0.05*self.max_linear_velocity[self.robot_id], 0.05*self.max_linear_velocity[self.robot_id], 0, 0, 0, 0]

    def KICK(self):
        return [0, 0, 10, 5, 0, 0]

    def KICK_CONTROL(self, kick_speed=10, kick_height=5):
        return [0, 0, kick_speed, kick_height, 0, 0]

    def STOP(self):
        return [0, 0, 0, 0, 0, 0]

    def SLIDE(self):
        return [0, 0, 0, 0, -10, 0]

    def MOVE_RIGHT(self):
        return [0, 0, 0, 0, 0, 1]

    def MOVE_LEFT(self):
        return [0, 0, 0, 0, 0, -1]