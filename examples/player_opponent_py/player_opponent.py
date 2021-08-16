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
        self.max_linear_velocity = info['max_linear_velocity'][0]
        self.field = info['field']
        self.frame = Frame()
        self.gk_index = 0
        self.f1_index = 1
        self.f2_index = 2
        self.i = 0

    def update(self, frame):

        self.frame = frame
        my_robot_gk = self.frame.coordinates[MY_TEAM][self.gk_index]
        my_robot_f1 = self.frame.coordinates[MY_TEAM][self.f1_index]
        my_robot_f2 = self.frame.coordinates[MY_TEAM][self.f2_index]
        ball = self.frame.coordinates[BALL]
        if self.i == 0:
            self.prev_ball = ball

        ## GK
        speeds_gk = action.defend_ball(self.gk_index, my_robot_gk, ball, self.prev_ball)
        if speeds_gk == None:
            speeds_gk = action.turn_to(self.gk_index, 0, 0, my_robot_gk, self.max_linear_velocity)

        ## F1
        if ball[X] > -0.5:
            dx = ball[X] - my_robot_f1[X]
            dy = ball[Y] - my_robot_f1[Y]
            desired_th = math.atan2(dy, dx)
            if my_robot_f1[TH] > desired_th + 0.2:
                index = action.TURN_RIGHT
            elif my_robot_f1[TH] < desired_th - 0.2:
                index = action.TURN_LEFT
            else:
                index = action.GO_FORWARD
        else:
            index = action.STOP
         
        speeds_f1 = action.get_action(index, self.max_linear_velocity)
        
        ## F2
        if ball[X] < 0.5:
            dx = ball[X] - my_robot_f2[X]
            dy = ball[Y] - my_robot_f2[Y]
            desired_th = math.atan2(dy, dx)
            if my_robot_f2[TH] > desired_th + 0.2:
                index = action.TURN_RIGHT
            elif my_robot_f2[TH] < desired_th - 0.2:
                index = action.TURN_LEFT
            else:
                index = action.GO_FORWARD
        else:
            index = action.STOP
         
        speeds_f2 = action.get_action(index, self.max_linear_velocity)

        speeds = speeds_gk + speeds_f1 + speeds_f2
        self.set_speeds(speeds)

        self.prev_ball = ball
        self.i += 1

if __name__ == '__main__':
    player = Player()
    player.run()
