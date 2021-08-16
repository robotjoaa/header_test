#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    from participant import Participant, Game, Frame
except ImportError as err:
    print('player_random-walk: \'participant\' module cannot be imported:', err)
    raise

import math
import action

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

class Player(Participant):
    def init(self, info):
        self.number_of_robots = info['number_of_robots']
        self.max_linear_velocity = info['max_linear_velocity']
        self.field = info['field']
        self.goal = info['goal']
        self.frame = Frame()
        self.gk_index = 0
        self.f1_index = 1
        self.f2_index = 2
        self.i = 0
        self.passed = False

    def update(self, frame):

        self.frame = frame
        cur_posture = self.frame.coordinates[MY_TEAM]
        my_robot_gk = self.frame.coordinates[MY_TEAM][self.gk_index]
        my_robot_f1 = self.frame.coordinates[MY_TEAM][self.f1_index]
        my_robot_f2 = self.frame.coordinates[MY_TEAM][self.f2_index]
        ball = self.frame.coordinates[BALL]
        target = self.frame.target
        if self.i == 0:
            self.prev_ball = ball
            self.prev_posture = cur_posture

        ## GK
        speeds_gk = action.get_action(action.STOP, self.max_linear_velocity[self.gk_index])

        ## F1       

        x = -1.0
        y = -1.0
        if not self.passed:
            if my_robot_f1[BALL_POSSESSION]:
                speeds_f1 = action.pass_to(self.f1_index, cur_posture, my_robot_f2[X] + 0.2, my_robot_f2[Y], self.max_linear_velocity, self.prev_posture)
                if speeds_f1 == None:
                    speeds_f1 = action.go_to(self.f1_index, cur_posture, ball[X], ball[Y], self.max_linear_velocity)
                if speeds_f1[2] > 0.5:
                    self.passed = True
            else:
                speeds_f1 = action.go_to(self.f1_index, cur_posture, ball[X], ball[Y], self.max_linear_velocity)
        else:
            speeds_f1 = action.get_action(action.STOP, self.max_linear_velocity[self.f1_index])

        ## F2
        x = 2.6
        y = 0
        speeds_f2 = action.get_action(action.STOP, self.max_linear_velocity[self.f2_index])
        if my_robot_f2[BALL_POSSESSION]:
            if action.distance(my_robot_f2[X], x, my_robot_f2[Y], y) < 0.1:
                speeds_f2 = action.shoot_to(self.f2_index, cur_posture, self.field[X]/2, self.goal[Y]/2 - 0.2, self.max_linear_velocity, self.prev_posture)
            else:
                speeds_f2 = action.go_to(self.f2_index, cur_posture, x, y, self.max_linear_velocity)
        else:
            if ball[Y] < -1:
                speeds_f2 = action.go_to(self.f2_index, cur_posture, x, y, self.max_linear_velocity)
            else:
                speeds_f2 = action.go_to(self.f2_index, cur_posture, ball[X], ball[Y], self.max_linear_velocity)

        speeds = speeds_gk + speeds_f1 + speeds_f2
        self.set_speeds(speeds)

        self.prev_posture = cur_posture
        self.prev_ball = ball
        self.i += 1

if __name__ == '__main__':
    player = Player()
    player.run()
