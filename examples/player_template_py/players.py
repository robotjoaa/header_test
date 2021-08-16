# Author(s): Taeyoung Kim, Chansol Hong, Luiz Felipe Vecchietti
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    from participant import Game, Frame
except ImportError as err:
    print('player_rulebasedB: \'participant\' module cannot be imported:', err)
    raise

import math
from ddpg import DDPG
import helper
from action import ActionControl
import numpy as np

from helper import Logger
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

class TeamManager : 
    def __init__(self, robot_num = 1) : 
        self.robot_num = robot_num
        self.pass_matrix = [[0]*self.robot_num for i in range(self.robot_num)]
        self.cross_matrix = [[0]*self.robot_num for i in range(self.robot_num)]
        # self.pass_count = 0
        # self.cross_count = 0
        # dummy action control
        # self.action = ActionControl(-1, -1)
        # self.strategy
    
    '''
    def update_counter(self):
        if self.pass_count >= 1:
            self.pass_count += 1
        if self.cross_count >= 1:
            self.cross_count += 1
    '''
    def reset_matrix(self, pass_reset, cross_reset) : 
        #if self.pass_count > 20:
        if pass_reset : 
            #self.pass_count = 0
            self.pass_matrix = [[0]*self.robot_num for i in range(self.robot_num)]
        #if self.cross_count > 20:
        if cross_reset : 
            #self.cross_count = 0
            self.cross_matrix = [[0]*self.robot_num for i in range(self.robot_num)]

    def update_matrix(self, cur_posture, prev_posture, cur_posture_opp, prev_posture_opp) : 
        # reset when someone newly get the ball
        # prev_posture, prev_posture_opp all false
        #helper.printConsole("update_matrix")
        #helper.printConsole(self.cross_matrix)
        prev_touch = [i[TOUCH] for i in prev_posture]
        prev_touch_opp =  [i[TOUCH] for i in prev_posture_opp]
        cur_touch = [i[TOUCH] for i in cur_posture]
        cur_touch_opp =  [i[TOUCH] for i in cur_posture_opp]
        '''
        helper.printConsole("----------")
        helper.printConsole(prev_touch)
        helper.printConsole(prev_touch_opp)
        helper.printConsole("----------")
        '''
        if (helper.all_false(prev_touch) and helper.all_false(prev_touch_opp)) and \
            (not helper.all_false(cur_touch) or not helper.all_false(cur_touch_opp)) :
            self.reset_matrix(True, True)
            #helper.printConsole("reset_matrix")
            

        
class Goalkeeper:
    def __init__(self, field, goal, penalty_area, goal_area, robot_size, max_linear_velocity, robot_id = 0, team_manager = None):
        self.field = field
        self.goal = goal
        self.penalty_area = penalty_area
        self.goal_area = goal_area
        self.robot_size = robot_size
        self.max_linear_velocity = max_linear_velocity
        self.action = ActionControl(robot_id, max_linear_velocity)
        self.flag = 0
        self.gk_index = 0
        self.f1_index = 1
        self.f2_index = 2
        self.team_manager = team_manager

    def move(self, robot_id, idx, idx_opp, defense_angle, attack_angle, cur_posture, cur_posture_opp, prev_posture, prev_posture_opp, prev_ball, cur_ball, predicted_ball, target=[0,0]):
        robot_to_target = helper.relative_distance(target[X], cur_posture[robot_id][X], target[Y], cur_posture[robot_id][Y])
        robot_to_ball = helper.relative_distance(cur_ball[X], cur_posture[robot_id][X], cur_ball[Y], cur_posture[robot_id][Y])
        robot_to_goal = helper.relative_distance(self.field[X]/2, cur_posture[robot_id][X], 0, cur_posture[robot_id][Y])        
        self.action.update_state(cur_posture, prev_posture, cur_ball, prev_ball)
        speeds = self.action.STOP()

        return speeds

class Forward_1:

    def __init__(self, field, goal, penalty_area, goal_area, robot_size, max_linear_velocity, team_manager, params, state_size, action_size, robot_id = 0):
        self.field = field
        self.goal = goal
        self.penalty_area = penalty_area
        self.goal_area = goal_area
        self.robot_size = robot_size
        self.max_linear_velocity = max_linear_velocity
        self.action_control = ActionControl(robot_id, max_linear_velocity)
        self.flag = 0
        self.gk_index = 0
        self.f1_index = 1
        self.f2_index = 2
        self.team_manager = team_manager
        self.frame = Frame()
        self.state_size = state_size
        self.action_size = action_size
        self.state = []
        self.action = np.zeros(self.action_size)
        self.reward = 0 
        self.previous_frame = Frame()
        self.previous_action = np.zeros(self.action_size)
        self.previous_state = []
        self.previous_reward = 0

        self.load = params.play
        self.play = params.play
        self.frame_skip = params.frame_skip

        self.action_type = params.action_type
        self.state_type = params.state_type
        self.reward_type = params.reward_type
        if (params.algorithm == 'ddpg'):
            self.trainer = DDPG(self.state_size, self.action_size, self.load, self.play)

        self.total_rewards = 0
        self.t = 0
        self.plot_reward = Logger()
        self.save_png_interval = 2500
        self.i = 1
        self.jump_count = 0 
    
    def move(self, robot_id, idx, idx_opp, defense_angle, attack_angle, cur_posture, cur_posture_opp, prev_posture, prev_posture_opp, prev_ball, cur_ball, predicted_ball,frame,target=[0,0]):
        '''
        robot_to_target = helper.relative_distance(target[X], cur_posture[robot_id][X], target[Y], cur_posture[robot_id][Y])
        robot_to_ball = helper.relative_distance(cur_ball[X], cur_posture[robot_id][X], cur_ball[Y], cur_posture[robot_id][Y])
        robot_to_goal = helper.relative_distance(self.field[X]/2, cur_posture[robot_id][X], 0, cur_posture[robot_id][Y])        
        '''
        
        self.action_control.update_state(cur_posture, prev_posture, cur_ball, prev_ball)
        
        self.frame = frame
        self.state = self.frame.state
        if (self.reward_type == 'reward_continuous'):
            self.reward = self.frame.reward_continuous
        elif (self.reward_type == 'reward_binary'):
            self.reward = self.frame.reward_binary
            if self.jump_count != 1: 
                self.reward -= 1
        elif (self.reward_type == 'reward_sparse'):
            self.reward = self.frame.reward_sparse
        else:
            self.reward = self.frame.reward_binary

        speeds = self.action_control.STOP()
        
        if not self.play:
            if self.i % self.frame_skip == 1:
                self.action = self.trainer.select_action(self.state)
            else:
                self.action = self.previous_action
        else:
            self.action = self.trainer.select_action(self.state)
        
        cur_posture = frame.coordinates[MY_TEAM]

        if not self.play:
        
            if self.i == 1:
                self.trainer.store_experience(self.state, self.state, self.previous_action, self.previous_reward)
            else:
                self.trainer.store_experience(self.previous_state, self.state, self.previous_action, self.previous_reward)
            
            # logging training agent's reward and plot graph
            self.t += 1
            self.total_rewards += self.reward
            if self.i % self.save_png_interval == 0:
                mean_total_reward = self.total_rewards/self.t
                self.plot_reward.update(self.i, mean_total_reward)
                self.plot_reward.plot('REWARD-GRAPH')
                # reset episode timesteps and total reward 
                self.t = 0
                self.total_rewards = 0

            # Training script: called every timestep  
            self.trainer.update()
                
            # save checkpoint
            self.trainer.save_checkpoint(self.i)
        
        if self.action[2] > 0 and self.team_manager.cross_matrix[2][1] > 0 :
            speeds = self.action_control.jump(8.5)
            self.jump_count += 1
        else:
            speeds = self.action_control.go_to(self.action[0]*self.field[0]/2, self.action[1]*self.field[1]/2)
        
        '''
        if cur_ball[Z] > 0.3: 
            predicted_time = helper.predict_header_time(cur_ball, prev_ball)
            ball_predicted = helper.predict_ball(cur_ball, prev_ball, predicted_time)
            speeds = self.action.header(ball_predicted, predicted_time) 
        else:
            if helper.distance(4.5, cur_posture[robot_id][X], 0.5, cur_posture[robot_id][Y]) < 0.1:
                speeds = self.action.turn_to(self.field[X]/2, 0)
            else : 
                speeds = self.action.go_to(4.5, 0.5)
        '''
        self.previous_state = self.state
        self.previous_action = self.action
        self.previous_reward = self.reward
        self.i += 1
        return speeds

class Forward_2:

    def __init__(self, field, goal, penalty_area, goal_area, robot_size, max_linear_velocity, team_manager, robot_id = 0):
        self.field = field
        self.goal = goal
        self.penalty_area = penalty_area
        self.goal_area = goal_area
        self.robot_size = robot_size
        self.max_linear_velocity = max_linear_velocity
        self.action = ActionControl(robot_id, max_linear_velocity)
        self.flag = 0
        self.gk_index = 0
        self.f1_index = 1
        self.f2_index = 2
        self.team_manager = team_manager

    def move(self, robot_id, idx, idx_opp, defense_angle, attack_angle, cur_posture, cur_posture_opp, prev_posture, prev_posture_opp, prev_ball, cur_ball, predicted_ball, target=[0,0]):

        robot_to_target = helper.relative_distance(target[X], cur_posture[robot_id][X], target[Y], cur_posture[robot_id][Y])
        robot_to_ball = helper.relative_distance(cur_ball[X], cur_posture[robot_id][X], cur_ball[Y], cur_posture[robot_id][Y])
        robot_to_goal = helper.relative_distance(self.field[X]/2, cur_posture[robot_id][X], 0, cur_posture[robot_id][Y])        
        self.action.update_state(cur_posture, prev_posture, cur_ball, prev_ball)
        #speeds = self.action.STOP()

        if cur_posture[robot_id][BALL_POSSESSION]:
            if helper.robot_is_opp_penalty(cur_posture[self.f1_index], self.field, self.penalty_area):
                cross_target = helper.cross_target(cur_posture[self.f1_index])
                if self.action.is_cross_to_possible(cross_target[X], cross_target[Y], cross_target[Z], self.team_manager.cross_matrix):
                    speeds = self.action.cross_to(cross_target[X], cross_target[Y], cross_target[Z], self.team_manager.cross_matrix)
                    #helper.printConsole("crossed")
                else:
                    speeds = self.action.go_to(cur_posture[self.f1_index][X], cur_posture[self.f1_index][Y])
            else:
                speeds = self.action.STOP()
        else:
            speeds = self.action.go_to(cur_ball[X], cur_ball[Y])

        return speeds