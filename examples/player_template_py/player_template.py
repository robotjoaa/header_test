#!/usr/bin/env python3

# Author(s): Taeyoung Kim, Chansol Hong, Luiz Felipe Vecchietti
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random
import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    from participant import Participant, Game, Frame
except ImportError as err:
    print('player_rulebasedB: \'participant\' module cannot be imported:', err)
    raise

import json
import math
import numpy as np
import action
import helper


from helper import Logger
from parameters import Parameters
from players import TeamManager, Goalkeeper, Forward_1, Forward_2

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
        self.field = info['field']
        self.max_linear_velocity = info['max_linear_velocity']
        self.robot_size = info['robot_size'][0]
        self.goal = info['goal']
        self.penalty_area = info['penalty_area']
        self.goal_area = info['goal_area']
        self.number_of_robots = info['number_of_robots']

        self.params = Parameters()
        #self.frame = Frame()
        self.i = 1 
        self.state_size = info['state_size']
        self.action_size = info['action_size']

        self.number_of_agents = 1
        
        #self._frame = 0 
        self.speeds = [0 for _ in range(18)]
        self.cur_posture = []
        self.cur_posture_opp = []
        self.cur_ball = []
        self.previous_ball = []
        self.previous_posture = []
        self.previous_posture_opp = []
        self.predicted_ball = []
        self.idx = 0
        self.idx_opp = 0
        self.previous_frame = Frame()
        self.defense_angle = 0
        self.attack_angle = 0
        self.gk_index = 0
        self.f1_index = 1
        self.f2_index = 2
        self.MANAGER = TeamManager(self.number_of_robots)
        self.GK = Goalkeeper(self.field, self.goal, self.penalty_area,
                                self.goal_area, self.robot_size,
                                self.max_linear_velocity, self.MANAGER, self.gk_index)
        
        self.F1 = Forward_1(self.field, self.goal, self.penalty_area,
                                self.goal_area, self.robot_size,
                                self.max_linear_velocity, self.MANAGER, self.params, self.state_size, self.action_size, self.f1_index)
        
        self.F2 = Forward_2(self.field, self.goal, self.penalty_area,
                                self.goal_area, self.robot_size,
                                self.max_linear_velocity, self.MANAGER, self.f2_index)
        helper.printConsole("Initializing variables...")
        

    def get_coord(self, received_frame):
        self.cur_ball = received_frame.coordinates[BALL]
        self.cur_posture = received_frame.coordinates[MY_TEAM]
        self.cur_posture_opp = received_frame.coordinates[OP_TEAM]
    

    def update(self, frame):
        if (self.i == 1):
            self.previous_frame = frame
            self.get_coord(frame)
            self.previous_ball = self.cur_ball
            self.state_size = frame.state_size

        self.get_coord(frame)

        self.predicted_ball = helper.predict_ball(self.cur_ball, self.previous_ball)
        self.idx = helper.find_closest_robot(self.cur_ball, self.cur_posture, self.number_of_robots)
        self.idx_opp = helper.find_closest_robot(self.cur_ball, self.cur_posture_opp, self.number_of_robots)
        self.defense_angle = helper.get_defense_kick_angle(self.predicted_ball, self.field, self.cur_ball)
        self.attack_angle = helper.get_attack_kick_angle(self.predicted_ball, self.field)

##############################################################################
      
        #(update the robots wheels)
        # Robot Functions
        ## update team matrix
       
        #helper.printConsole(str([i[TOUCH] for i in self.cur_posture]))
        #helper.printConsole(str([i[TOUCH] for i in self.cur_posture_opp]))
        
        self.MANAGER.update_matrix(self.cur_posture, self.previous_posture, self.cur_posture_opp, self.previous_posture_opp)
    
        speeds_gk = self.GK.move(self.gk_index, 
                                    self.idx, self.idx_opp, 
                                    self.defense_angle, self.attack_angle,
                                    self.cur_posture, self.cur_posture_opp,
                                    self.previous_posture, self.previous_posture_opp,
                                    self.previous_ball, self.cur_ball, self.predicted_ball)
        '''
        speeds_f1 = self.F1.move(self.f1_index, 
                                    self.idx, self.idx_opp, 
                                    self.defense_angle, self.attack_angle,
                                    self.cur_posture, self.cur_posture_opp,
                                    self.previous_posture, self.previous_posture_opp,
                                    self.previous_ball, self.cur_ball, self.predicted_ball)
        '''
        speeds_f1 = self.F1.move(self.f1_index, 
                                    self.idx, self.idx_opp, 
                                    self.defense_angle, self.attack_angle,
                                    self.cur_posture, self.cur_posture_opp,
                                    self.previous_posture, self.previous_posture_opp,
                                    self.previous_ball, self.cur_ball, self.predicted_ball, frame)

        speeds_f2 = self.F2.move(self.f2_index, 
                                    self.idx, self.idx_opp, 
                                    self.defense_angle, self.attack_angle,
                                    self.cur_posture, self.cur_posture_opp,
                                    self.previous_posture, self.previous_posture_opp,
                                    self.previous_ball, self.cur_ball, self.predicted_ball)
        
        self.speeds = speeds_gk + speeds_f1 + speeds_f2
        #helper.printConsole(self.speeds)
        self.set_speeds(self.speeds)
##############################################################################

        self.previous_frame = frame
        self.previous_ball = self.cur_ball
        self.previous_posture = self.cur_posture
        self.previous_posture_opp = self.cur_posture_opp

        self.i += 1

if __name__ == '__main__':
    player = Player()
    player.run()