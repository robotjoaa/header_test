#!/usr/bin/env python3

import errno
import json
import math
import os
import random
import select
import socket
import string
import subprocess
import sys
import time
import collections
import numpy as np
from numpy.lib.polynomial import _polyder_dispatcher
sys.path.append(os.path.join(os.path.dirname(__file__), '../submodules'))

from controller import Supervisor

from player import Game

import logger
import constants

def get_distance(x1, y1, x2, y2):
    dist =  math.sqrt( \
                        math.pow(x1 - x2, 2) + \
                        math.pow(y1 - y2, 2))
    return dist

def random_string(length):
    """Generate a random string with the combination of lowercase and uppercase letters."""
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(length))

def get_key(rpc):
    """The key is the first argument of the RPC."""
    first = rpc.find('"') + 1
    return rpc[first:rpc.find('"', first)]

def get_robot_name(self, color, id):
    name = self.constants.DEF_ROBOT_PREFIX
    if color == self.constants.TEAM_RED:
        name += 'R'
    elif color == self.constants.TEAM_BLUE:
        name += 'B'
    else:
        sys.stderr.write("Error: get_robot_name: Invalid team color.\n")
    name += str(id)
    return name

def get_role_name(self, role):
    if role == self.constants.TEAM_RED:
        return 'team red'
    if role == self.constants.TEAM_BLUE:
        return 'team blue'
    sys.stderr.write("Error: get_role_name: Invalid role.\n")
    return ''

def next_available_port(server_port, max_instances):
    port = server_port
    while True:
        if port > server_port + max_instances:
            print('Maximum number of instances reached')
            return port
        found = False
        # test if the port is available
        testSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            testSocket.bind(('0.0.0.0', port))
            found = True
        except socket.error as e:
            found = False
        finally:
            testSocket.close()
            if found:
                return port
            port += 1

class TcpServer:
    def __init__(self, host, server_port, max_instances):
        self.port = next_available_port(server_port, max_instances)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.setblocking(False)
        self.server.bind((host, self.port))
        self.server.listen(5)
        self.connections = [self.server]
        self.unprocessedData = {}
        self.unprocessedData[self.server.fileno()] = ''

    def get_port(self):
        return self.port

    def send_to_all(self, message):  # send message to all clients
        for client in self.connections:
            if client == self.server:
                continue
            self.send(client, message)

    def send(self, client, message):  # send message to a single client
        if client.fileno() == -1:  # was closed
            return
        try:
            client.sendall(message.encode())
        except ConnectionAbortedError:
            self.connections.remove(client)

    def spin(self, game_supervisor):  # handle asynchronous requests from clients
        def cleanup(s):
            print('Cleanup')
            if s in self.connections:
                self.connections.remove(s)
            s.close()
        while True:
            readable, writable, exceptional = select.select(self.connections, [], self.connections, 0)
            if not readable and not writable and not exceptional:
                return
            for s in readable:
                if s is self.server:
                    connection, client_address = s.accept()
                    connection.setblocking(False)
                    self.connections.append(connection)
                    self.unprocessedData[connection.fileno()] = ''
                    print('Accepted ', client_address)
                else:
                    success = True
                    data = ''
                    try:
                        while True:
                            d = s.recv(4096)
                            if not d:
                                break
                            data += d.decode()
                    except socket.error as e:
                        if e.args[0] == errno.EWOULDBLOCK:
                            success = True
                        else:
                            if e.args[0] != 10053:  # WSAECONNABORTED
                                print('Error caught: ', e.args[0])
                            success = False
                    if data and success:
                        self.unprocessedData[s.fileno()] = \
                            game_supervisor.callback(s, self.unprocessedData[s.fileno()] + data)
                    else:
                        print('Closing')
                        cleanup(s)
            for s in exceptional:
                print('Exceptional')
                cleanup(s)

class GameSupervisor(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.constants = constants
        self.basicTimeStep = int(self.getBasicTimeStep())
        self.timeStep = self.constants.PERIOD_MS
        self.waitReady = 0

        self.speeds_buffer = [[0 for _ in range(30)],[0 for _ in range(30)]]
        #self.pre_pos = [[[0 for _ in range(5)] for _ in range(self.constants.NUMBER_OF_ROBOTS)] for _ in range(2)]
        self.pre_ball = [0, 0, 0]
        self.reset_speeds = [False, False]

        # kick
        self.kick_count = [[0,0,0,0,0],[0,0,0,0,0]]
        self.kick_trig = [[0,0,0,0,0],[0,0,0,0,0]]
        self.kick_buffer = [[[0,0],[0,0],[0,0],[0,0],[0,0]],[[0,0],[0,0],[0,0],[0,0],[0,0]]]
        # jump
        self.jump_count = [[0,0,0,0,0],[0,0,0,0,0]]
        self.jump_trig = [[0,0,0,0,0],[0,0,0,0,0]]
        # arm and leg
        self.arm_count = [[0,0,0,0,0],[0,0,0,0,0]]
        self.p_count = [[1,1,1,1,1],[1,1,1,1,1]]
        self.arm_speed = [[0,0,0,0,0],[0,0,0,0,0]]
        self.gk_grab = [0,0]
        self.gk_side_count = [0,0]
        self.gk_side_trig = [0,0]
        self.gk_leg_rot = [[0,0],[0,0]]
        # ball_possession
        self.ball_possession = [[False] * self.constants.NUMBER_OF_ROBOTS, [False] * self.constants.NUMBER_OF_ROBOTS]
        # spotlight
        self.spotlight = self.getFromDef("DEF_BALLSPOTLIGHT")
        # dribble
        self.dribbler = None

        self.receiver = self.getDevice(self.constants.NAME_RECV)
        self.receiver.enable(self.constants.RECV_PERIOD_MS)

        self.viewpointNode = self.getFromDef(self.constants.DEF_AUDVIEW)
        ori = [0 - 0.27, (-1 + 0.08), (0 - 0.27), 1.57+0.08]
        pos = [-3.5, (2+2.7) * 1, 0]
        self.viewpointNode.getField('orientation').setSFRotation(ori)
        self.viewpointNode.getField('position').setSFVec3f(pos)
        # DEF_GRASS is not visible to cam a and cam b, optional
        grass = self.getFromDef(self.constants.DEF_GRASS)
        # BALLSHAPE is visible only to viewpoint, ORANGESHAPE is to cam_a and cam_b, mandatory
        ball = self.getFromDef(self.constants.DEF_BALLSHAPE)
        # Stadium is visible only to viewpoint, optional
        stadium = self.getFromDef(self.constants.DEF_STADIUM)
        # Wall is visible only to robots
        wall = self.getFromDef(self.constants.DEF_WALL)
        if wall:
            wall.setVisibility(self.viewpointNode, False)
        # VisualWall is visible only to viewpoint, optional
        visual_wall = self.getFromDef(self.constants.DEF_VISWALL)
        # patches'
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                robot = self.getFromDef(get_robot_name(self, team, id))
                patches = robot.getField('patches')
                number = patches.getMFNode(0)

        self.first_touch = True

    ## basic ##
    def step(self, timeStep, runTimer=False):
        for i in range(0, timeStep, self.basicTimeStep):
            if Supervisor.step(self, self.basicTimeStep) == -1:
                return -1
            if runTimer:
                self.time += self.basicTimeStep
            self.update_label()

    def get_role(self, rpc):
        key = get_key(rpc)
        for role in self.constants.ROLES:
            if role in self.role_info and 'key' in self.role_info[role] and self.role_info[role]['key'] == key:
                return role
        sys.stderr.write("Error: get_role: invalid key.\n")
        return -1

    def callback(self, client, message):
        unprocessed = ''
        if not message.startswith('aiwc.'):
            print('Error, AIWC RPC messages should start with "aiwc.".')
            return unprocessed

        # Handle concatenated messages
        data = message.split('aiwc.')
        for command in data:
            if not command:
                continue
            if not command.endswith(')'):
                unprocessed += 'aiwc.' + command
                continue
            role = self.get_role(command)
            self.role_client[role] = client
            if command.startswith('get_info('):
                print('Server receive aiwc.get_info from ' + get_role_name(self, role))
                if role == self.constants.TEAM_RED:
                    self.tcp_server.send(client, json.dumps(self.role_info[self.constants.TEAM_RED]))
                elif role == self.constants.TEAM_BLUE:
                    self.tcp_server.send(client, json.dumps(self.role_info[self.constants.TEAM_BLUE]))
                # else:
                #     self.tcp_server.send(client, json.dumps(self.role_info[self.constants.TEAM_RED]))
            elif command.startswith('ready('):
                self.ready[role] = True
                print('Server receive aiwc.ready from ' + get_role_name(self, role))
            elif command.startswith('set_speeds('):
                start = command.find('",') + 2
                end = command.find(')', start)
                speeds = command[start:end]
                speeds = [float(i) for i in speeds.split(',')]
                if self.reset_speeds[role]:
                    speeds = [0 for _ in range(30)]
                    self.reset_speeds[role] = False
                self.speeds_buffer[role] = speeds
                self.set_speeds(role, speeds)
            else:
                print('Server received unknown message', message)
        return unprocessed

    def get_state(self, team, id):
        ball = self.ball_position
        pos = self.get_robot_posture(team, id)
        #target = [self.target[0], -self.target[2], self.target[1]]
        dx = ball[0] - pos[0]
        dy = ball[1] - pos[1]
        dz = ball[2] - (pos[2] + 0.42)
        desired_th = math.atan2(dy, dx)
        if team == self.constants.TEAM_RED:
            th = pos[3]
        else:
            th = pos[3] + self.constants.PI if pos[3] < 0 else pos[3] - self.constants.PI
        d_th = desired_th - th
        # distance from ball to opponent goal
        # dx_target = target[0] - ball[0]
        # dy_target = target[1] - ball[1]
        # d_th_robot2target = math.atan2(target[1] - pos[0], target[1] - pos[1]) - th
        # distance from ball to opponent goal
        dx_ball = self.constants.FIELD_LENGTH/2 - ball[0]
        dy_ball = 0 - ball[1]
        dz_ball = ball[2]
        d_th_robot2goal = math.atan2(self.constants.FIELD_LENGTH/2 - pos[0], 0 - pos[1]) - th
        '''
        return [round(dx/self.constants.FIELD_LENGTH, 2), 
                round(dy/self.constants.FIELD_WIDTH, 2), 
                round(d_th/self.constants.PI, 2),
                round(dx_target/self.constants.FIELD_LENGTH, 2), 
                round(dy_target/self.constants.FIELD_WIDTH, 2), 
                round(d_th_robot2target/self.constants.PI, 2),
                round(dx_ball/self.constants.FIELD_LENGTH, 2), 
                round(dy_ball/self.constants.FIELD_WIDTH, 2), 
                round(d_th_robot2goal/self.constants.PI, 2)]
        '''
        return [round(dx/self.constants.FIELD_LENGTH, 2), 
                round(dy/self.constants.FIELD_WIDTH, 2), 
                round(dz,2),
                round(d_th/self.constants.PI, 2),
                round(dx_ball/self.constants.FIELD_LENGTH, 2), 
                round(dy_ball/self.constants.FIELD_WIDTH, 2), 
                round(dz_ball,2),
                round(d_th_robot2goal/self.constants.PI, 2)]


    def get_reward(self, team, id, type='continuous'):
        # distance robot 2 ball
        ball = self.ball_position
        pos = self.get_robot_posture(team, id)
        #target = [self.target[0], -self.target[2], self.target[1]]
        #dx = target[0] - pos[0]
        #dy = target[1] - pos[1]
        #distance = math.sqrt(dx*dx + dy*dy)
        # distance ball 2 goal
        dx_ball = self.constants.FIELD_LENGTH/2 - ball[0]
        #dy_ball = 0 - ball[1]
        #distance_ball = math.sqrt(dx_ball*dx_ball + dy_ball*dy_ball)
        distance_ball = dx_ball

        dx_ball_head = pos[0] - ball[0]
        dy_ball_head = pos[1] - ball[1]
        dz_ball_head = (pos[2]+0.42) - ball[2]
        distance_ball_head = math.sqrt(dx_ball_head*dx_ball_head + dy_ball_head*dy_ball_head + dz_ball_head*dz_ball_head)

        if team == self.constants.TEAM_RED and id == 1 :
            self.log_head.update_iter(distance_ball_head, pos[2])

        reward = 0
        if type == 'continuous':
            #reward = math.exp(-1*distance) - 1
            reward += 2*math.exp(-1*distance_ball) - 1
            if ball[0] > self.constants.FIELD_LENGTH / 2 and abs(ball[1]) < self.constants.GOAL_WIDTH / 2 and abs(ball[2]) < 0.5:
                reward += 10
        elif type == 'binary':
            # if distance between ball and head decreased
            if distance_ball_head < 0.3 :
                reward = 0
            else:
                reward = -1

            
            dx_ball = self.constants.FIELD_LENGTH/2 - self.pre_ball[0]
            #dy_ball = 0 - self.pre_ball[1]
            #pre_distance_ball = math.sqrt(dx_ball*dx_ball + dy_ball*dy_ball)
            pre_distance_ball = dx_ball
            
            # if robot in penalty area
            if (self.constants.FIELD_LENGTH/2  - self.constants.PENALTY_AREA_DEPTH <= pos[0] and pos[0] <= self.constants.FIELD_LENGTH/2 and
            -self.constants.PENALTY_AREA_WIDTH/2 <= pos[1] and pos[1] <= self.constants.PENALTY_AREA_WIDTH/2) and self.first_touch: 
                reward -= 1
            else :
                reward += 0
            

            # if ball had hit the head 
            if not self.first_touch :
                if pre_distance_ball > distance_ball :
                    reward += 0.5    
                else : 
                    reward += 0
            else:
                reward += -1

            #if self.robot[self.constants.TEAM_RED][1]['touch'] == True and self.get_robot_posture(self.constants.TEAM_RED,1)[2] > 0.1 and self.first_touch: 
            #    self.first_touch = False
            if self.robot[self.constants.TEAM_RED][1]['touch'] == True and self.first_touch and pos[2] > 0.2: 
                self.first_touch = False
                reward += 10
            
            if ball[0] > self.constants.FIELD_LENGTH / 2 and abs(ball[1]) < self.constants.GOAL_WIDTH / 2 and abs(ball[2]) < 0.5 and not self.first_touch:
                reward += 10
            '''
                if self.get_robot_posture(self.constants.TEAM_RED,1)[2] > 0.1 : 
                    reward += 10
                else :    
                    reward += 2
                
            else : 
                reward -= 1 
            '''
            '''
            # if only jumped once (reward on player)

            # reward if f2 crossed (reward on player)

            # should not be close to f2
            f2_pos = self.get_robot_posture(team, 2)
            dx_f2 = pos[0] - f2_pos[0]
            dy_f2 = pos[1] - f2_pos[1]
            dist_f2 = math.sqrt(dx_f2*dx_f2+dy_f2*dy_f2)
            if dist_f2 < self.constants.PENALTY_AREA_WIDTH/2 : 
                reward -= 3

            # should not be in goal area 
            if (self.constants.FIELD_LENGTH/2  - self.constants.GOAL_AREA_DEPTH <= pos[0] and pos[0] <= self.constants.FIELD_LENGTH/2 and
            -self.constants.GOAL_AREA_WIDTH/2 <= pos[1] and pos[1] <= self.constants.GOAL_AREA_WIDTH/2) : 
                reward -= 2
            else :
                reward += 0

            if pre_distance_ball - distance_ball > 0.01:
                reward += 0
            else:
                reward += -1

            
            '''
        elif type == 'sparse':
            if ball[0] > self.constants.FIELD_LENGTH / 2 and abs(ball[1]) < self.constants.GOAL_WIDTH / 2 and abs(ball[2]) < 0.5:
                reward = 0
            else:
                reward = -1
        return reward
    
    def publish_current_frame(self, reset_reason=None):
        frame_team_red = self.generate_frame(self.constants.TEAM_RED, reset_reason)  # frame also sent to commentator and reporter
        frame_team_blue = self.generate_frame(self.constants.TEAM_BLUE, reset_reason)
        for role in self.constants.ROLES:
            if role in self.role_client:
                frame = frame_team_blue if role == self.constants.TEAM_BLUE else frame_team_red
                self.tcp_server.send(self.role_client[role], json.dumps(frame))

    def generate_frame(self, team, reset_reason=None):
        opponent = self.constants.TEAM_BLUE if team == self.constants.TEAM_RED else self.constants.TEAM_RED
        frame = {}
        frame['time'] = self.getTime()

        frame['coordinates'] = [None] * 3
        #target = [self.target[0], -self.target[2], self.target[1]]
        #frame['target'] = target
        # target = [self.target[0], -self.target[2], self.target[1]]
        # frame['target'] = target
        for t in self.constants.TEAMS:
            frame['coordinates'][t] = [None] * self.constants.NUMBER_OF_ROBOTS
            c = team if t == self.constants.TEAM_RED else opponent
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                frame['coordinates'][t][id] = [None] * 7
                pos = self.get_robot_posture(c, id)
                frame['coordinates'][t][id][0] = pos[0] if team == self.constants.TEAM_RED else -pos[0]
                frame['coordinates'][t][id][1] = pos[1] if team == self.constants.TEAM_RED else -pos[1]
                frame['coordinates'][t][id][2] = pos[2]
                if team == self.constants.TEAM_RED:
                    frame['coordinates'][t][id][3] = pos[3]
                else:
                    frame['coordinates'][t][id][3] = pos[3] + self.constants.PI if pos[3] < 0 else pos[3] - self.constants.PI
                frame['coordinates'][t][id][4] = self.robot[c][id]['active']
                frame['coordinates'][t][id][5] = self.robot[c][id]['touch']
                frame['coordinates'][t][id][6] = self.robot[c][id]['ball_possession']
        frame['coordinates'][2] = [None] * 3
        frame['coordinates'][2][0] = self.ball_position[0] if team == self.constants.TEAM_RED else -self.ball_position[0]
        frame['coordinates'][2][1] = self.ball_position[1] if team == self.constants.TEAM_RED else -self.ball_position[1]
        frame['coordinates'][2][2] = self.ball_position[2]
        frame['state'] = [None] * 3
        
        frame['reward_continuous'] = [None] * 1
        frame['reward_binary'] = [None] * 1
        frame['reward_sparse'] = [None] * 1
        # filling the state
        if team == self.constants.TEAM_RED : 
            frame['state'] = self.get_state(team, 0)
            frame['state_size'] = len(frame['state'])
            frame['reward_continuous'] = self.get_reward(team, 1, 'continuous')
            frame['reward_binary'] = self.get_reward(team, 1, 'binary')
            frame['reward_sparse'] = self.get_reward(team, 1, 'sparse')
        #self.pre_pos[team][0] = self.get_robot_posture(team, 0)
        self.pre_ball = self.ball_position
        frame['reset_reason'] = reset_reason
        return frame

    def robot_in_field(self, team, id):
        robot_pos = self.get_robot_posture(team, id)
        x = robot_pos[0]
        y = robot_pos[1]
        if abs(y) < self.constants.GOAL_WIDTH / 2:
            if abs(x) > self.constants.FIELD_LENGTH / 2 + self.constants.GOAL_DEPTH:
                return False
            else:
                return True
        if abs(x) > self.constants.FIELD_LENGTH / 2:
            return False
        else:
            return True

    def ball_in_field(self):
        pos = self.get_ball_position()
        # checking with absolute values is sufficient since the field is symmetrical
        abs_x = abs(pos[0])
        abs_y = abs(pos[1])
        abs_z = abs(pos[2])

        if (abs_x > self.constants.FIELD_LENGTH / 2) and \
           (abs_y < self.constants.GOAL_WIDTH / 2) and \
           (abs_z >= 0.5):
            return False
        if (abs_x > self.constants.FIELD_LENGTH / 2 + self.constants.WALL_THICKNESS) and \
           (abs_y > self.constants.GOAL_WIDTH / 2 + self.constants.WALL_THICKNESS):
            return False
        if abs_y > self.constants.FIELD_WIDTH / 2 + self.constants.WALL_THICKNESS:
            return False
        # check triangular region at the corner
        cs_x = self.constants.FIELD_LENGTH / 2 - self.constants.CORNER_LENGTH
        cs_y = self.constants.FIELD_WIDTH / 2 + self.constants.WALL_THICKNESS
        ce_x = self.constants.FIELD_LENGTH / 2 + self.constants.WALL_THICKNESS
        ce_y = self.constants.FIELD_WIDTH / 2 - self.constants.CORNER_LENGTH
        if cs_x < abs_x and abs_x < ce_x:
            border_y = ce_y + (abs_x - ce_x) * (ce_y - cs_y) / (ce_x - cs_x)
            if abs_y > border_y:
                return False
        return True

    ## get informations ##
    def get_robot_touch_ball(self):
        rc = [[False] * self.constants.NUMBER_OF_ROBOTS, [False] * self.constants.NUMBER_OF_ROBOTS]
        while self.receiver.getQueueLength() > 0:
            message = self.receiver.getData()
            for team in self.constants.TEAMS:
                for id in range(self.constants.NUMBER_OF_ROBOTS):
                    cond = self.speeds_buffer[team][6*id+2] > 0
                    if message[2 * id + team] == 1:
                        rc[team][id] = True
            self.receiver.nextPacket()
        return rc

    def flush_touch_ball(self):
        while self.receiver.getQueueLength() > 0:
            self.receiver.nextPacket()

    def get_robot_posture(self, team, id):
        position = self.robot[team][id]['node'].getPosition()
        orientation = self.robot[team][id]['node'].getOrientation()
        f = 1
        x = position[0]
        y = -position[2]
        z = position[1]
        th = (0) + math.atan2(orientation[2], orientation[8]) + self.constants.PI / 2
        # Squeeze the orientation range to [-PI, PI]
        while th > self.constants.PI:
            th -= 2 * self.constants.PI
        while th < -self.constants.PI:
            th += 2 * self.constants.PI
        stand = orientation[4] > 0.8
        return [f * x, f * y, z, th, stand]

    def get_ball_position(self):
        f = 1
        position = self.ball.getPosition()
        x = position[0]
        y = -position[2]
        z = position[1]
        return [f * x, f * y, z]

    def reset_ball(self, x, z):
        f = 1
        self.ball.getField('translation').setSFVec3f([f * x, 1.5 * self.constants.BALL_RADIUS, -f * z])
        self.ball.getField('rotation').setSFRotation([0, 1, 0, 0])
        self.ball.resetPhysics()

    def reset_robot(self, team, id, x, y, z, th):
        robot = self.getFromDef(get_robot_name(self, team, id))
        f = 1
        translation = [f * x, y, f * -z]
        rotation = [0, 1, 0, th + (0)]

        al = robot.getField('axleLength').getSFFloat()
        h = robot.getField('height').getSFFloat()
        wr = robot.getField('wheelRadius').getSFFloat()

        lwTranslation = [-al / 2, (-h + 2 * wr) / 2, 0]
        rwTranslation = [al / 2, (-h + 2 * wr) / 2, 0]
        wheelRotation = [1, 0, 0, self.constants.PI / 2]

        robot.getField('translation').setSFVec3f(translation)
        robot.getField('rotation').setSFRotation(rotation)
        robot.getField('lwTranslation').setSFVec3f(lwTranslation)
        robot.getField('lwRotation').setSFRotation(wheelRotation)
        robot.getField('rwTranslation').setSFVec3f(rwTranslation)
        robot.getField('rwRotation').setSFRotation(wheelRotation)
        self.relocate_all(team, id)

        robot.resetPhysics()
        self.robot[team][id]['active'] = True
        self.robot[team][id]['touch'] = False
        self.robot[team][id]['ball_possession'] = False
        self.robot[team][id]['fall_time'] = self.time
        self.robot[team][id]['sentout_time'] = 0
        self.stop_robots()

    def reset(self, red_formation, blue_formation):
        # reset the ball
        if red_formation == self.constants.FORMATION_DEFAULT or red_formation == self.constants.FORMATION_KICKOFF:
            self.reset_ball(self.constants.BALL_POSTURE[self.constants.BALL_DEFAULT][0],
                            self.constants.BALL_POSTURE[self.constants.BALL_DEFAULT][1])

        # reset the robots
        for team in self.constants.TEAMS:
            if team == self.constants.TEAM_RED:
                s = 1
                a = 0
                formation = red_formation
            else:
                s = -1
                a = self.constants.PI
                formation = blue_formation
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.reset_robot(team, id,
                                 self.constants.ROBOT_FORMATION[formation][id][0] * s,
                                 0.09 / 2,
                                 self.constants.ROBOT_FORMATION[formation][id][1] * s,
                                 self.constants.ROBOT_FORMATION[formation][id][2] + a - self.constants.PI / 2)

        # reset recent touch
        self.recent_touch = [[False] * self.constants.NUMBER_OF_ROBOTS, [False] * self.constants.NUMBER_OF_ROBOTS]
        self.deadlock_time = self.time
        # flush touch packet
        self.flush_touch_ball()
        # reset arm and leg
        self.relocate_all_every()
        self.touch_flag_corner = False
        self.touch_flag_penalty = False

    def lock_all_robots(self, locked):
        for t in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.robot[t][id]['active'] = not locked
        if locked:
            self.relocate_all_every()

    def stop_robots(self):
        self.dribbler = None
        for t in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.arm_count[t][id] = 0
                self.arm_speed[t][id] = 0
            self.set_speeds(t, [0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0])

    ## label ##
    def update_label(self):
        seconds = self.time / 1000.0
        minutes = seconds // 60
        seconds -= minutes * 60
        self.setLabel(0, '%d:%05.2f' % (minutes, seconds), 0.01, 0.01, 0.11, 0xe6b800, 0, 'Impact')
            
    def set_wheel_velocity(self, max_linear_velocity, left_wheel, right_wheel):
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

    ## speed ##
    def set_speeds(self, team, speeds):
        letter = 'R' if team == self.constants.TEAM_RED else 'B'
        def_robot_prefix = self.constants.DEF_ROBOT_PREFIX + letter
        for id in range(self.constants.NUMBER_OF_ROBOTS):
            robot = self.getFromDef(def_robot_prefix + str(id))
            if self.robot[team][id]['active']:
                kick_speed = 0
                jump_speed = 0
                if id == 0:
                    self.gk_side_move(team, 0)
                    self.check_jump_GK(speeds, team, 0)
                else:
                    self.check_jump(speeds, team, id)
                max_speed = 0
                left_wheel_speed = 0
                right_wheel_speed = 0
                if self.dribbler != None and self.dribbler[0] == team and self.dribbler[1] == id:
                    max_speed = 0.8*self.constants.MAX_LINEAR_VELOCITY[id]
                    left_wheel_speed, right_wheel_speed = self.set_wheel_velocity(max_speed, speeds[id * 6], speeds[id * 6 + 1])
                    left_wheel_speed = left_wheel_speed / self.constants.WHEEL_RADIUS[id]
                    right_wheel_speed = right_wheel_speed / self.constants.WHEEL_RADIUS[id]
                else:
                    max_speed = self.constants.MAX_LINEAR_VELOCITY[id]
                    left_wheel_speed, right_wheel_speed = self.set_wheel_velocity(max_speed, speeds[id * 6], speeds[id * 6 + 1])
                    left_wheel_speed = left_wheel_speed / self.constants.WHEEL_RADIUS[id]
                    right_wheel_speed = right_wheel_speed / self.constants.WHEEL_RADIUS[id]
                if self.robot[team][id]['node'].getOrientation()[4] < 0.9 \
                    or 0 < self.kick_count[team][id] <= 6:
                    left_wheel_speed, right_wheel_speed = 0, 0
                arm_leg_speed = [0,0,0,0] if self.jump_trig[team][0] != 0 else self.move_arm_leg(speeds, team, id)
                robot.getField('customData').setSFString(
                    "%f %f %f %f %f %f %f %f" % (left_wheel_speed, right_wheel_speed,
                                        kick_speed, jump_speed, arm_leg_speed[0], arm_leg_speed[1], arm_leg_speed[2], arm_leg_speed[3])
                )
            else:
                self.arm_count[team][id] = 0
                self.relocate_arm_leg(team, id)
                custom_0 = '0 0 0 0 0 0 0 0'
                robot.getField('customData').setSFString(custom_0)

    ## arm_leg##
    def move_arm_leg(self, speeds, team, id):
        if self.jump_trig[team][id] != 0:
            return 0,0,0,0
        if 0 < self.kick_count[team][id] <= 6:
            if self.kick_count[team][id] >= 5:
                self.arm_count[team][id] = 0
                self.arm_speed[team][id] = 0
                self.relocate_arm_leg(team, id)
            return 0,0,0,0
        if ((abs(speeds[id * 6]) < 0.1) ^ (abs(speeds[id * 6 + 1]) < 0.1)) or \
            ((speeds[id * 6]*speeds[id * 6 + 1] < 0) and (speeds[id * 6] + speeds[id * 6 + 1] < 0.1)):
            self.arm_count[team][id] = 0
            self.arm_speed[team][id] = 0
            self.turn_motion(team, id)
            return 0,0,0,0
        m_speed = abs(3*((speeds[id * 6])+(speeds[id * 6 + 1])))
        speed_arm_leg1 = 0
        speed_arm_leg2 = 0
        speed_arm_leg3 = 0
        speed_arm_leg4 = 0
        if (self.arm_count[team][id] % self.p_count[team][id] == 0):
            self.relocate_arm_leg(team, id)
            if (m_speed < 1):
                self.arm_count[team][id] = 0
            self.arm_count[team][id] = 0
            i_speed = int(m_speed)
            if (i_speed < 3):
                self.arm_speed[team][id] = 0
            elif (i_speed < 5):
                self.arm_speed[team][id] = i_speed
            elif (i_speed < 8):
                self.arm_speed[team][id] = 6
            elif (i_speed < 10):
                self.arm_speed[team][id] = i_speed
            else:
                self.arm_speed[team][id] = 12
            if (self.arm_speed[team][id] == 0):
                self.p_count[team][id] = 4
            else:
                self.p_count[team][id] = 72*2/self.arm_speed[team][id]

        temp = self.arm_count[team][id] % self.p_count[team][id]

        if (temp < self.p_count[team][id]/4):
            speed_arm_leg1 = 1.5*self.arm_speed[team][id]
            speed_arm_leg2 = -self.arm_speed[team][id]
            speed_arm_leg3 = -self.arm_speed[team][id]
            speed_arm_leg4 = 0
        elif (temp < self.p_count[team][id]*2/4):
            speed_arm_leg1 = -1.5*self.arm_speed[team][id]
            speed_arm_leg2 = self.arm_speed[team][id]
            speed_arm_leg3 = self.arm_speed[team][id]
            speed_arm_leg4 = 0
        elif (temp < self.p_count[team][id]*3/4):
            speed_arm_leg1 = -self.arm_speed[team][id]
            speed_arm_leg2 = 1.5*self.arm_speed[team][id]
            speed_arm_leg3 = 0
            speed_arm_leg4 = -self.arm_speed[team][id]
        else:
            speed_arm_leg1 = self.arm_speed[team][id]
            speed_arm_leg2 = -1.5*self.arm_speed[team][id]
            speed_arm_leg3 = 0
            speed_arm_leg4 = self.arm_speed[team][id]

        self.arm_count[team][id] += 1

        return speed_arm_leg1, speed_arm_leg2, speed_arm_leg3, speed_arm_leg4

    def relocate_arm_leg(self, team, id):
        self.arm_count[team][id] = 0
        robot = self.robot[team][id]['node']

        Rotation = [1, 0, 0, 0]
        Rotation2 = [0, 0, 1,-1.57]
        laTranslation = [0, 0.07, 0]
        raTranslation = [0, 0.07, 0]
        laTranslation2 = [-0.238, 0.256, 0]
        raTranslation2 = [-0.238, 0.256, 0]
        llTranslation = [0, 0.01, 0]
        rlTranslation = [0, 0.01, 0]

        robot.getField('laTranslation').setSFVec3f(laTranslation)
        robot.getField('raTranslation').setSFVec3f(raTranslation)
        robot.getField('laRotation').setSFRotation(Rotation)
        robot.getField('raRotation').setSFRotation(Rotation)
        robot.getField('laTranslation2').setSFVec3f(laTranslation2)
        robot.getField('raTranslation2').setSFVec3f(raTranslation2)
        robot.getField('laRotation2').setSFRotation(Rotation2)
        robot.getField('raRotation2').setSFRotation(Rotation2)
        robot.getField('llTranslation').setSFVec3f(llTranslation)
        robot.getField('rlTranslation').setSFVec3f(rlTranslation)
        robot.getField('llRotation').setSFRotation(Rotation)
        robot.getField('rlRotation').setSFRotation(Rotation)
        robot.getField('llTranslation2').setSFVec3f(llTranslation)
        robot.getField('rlTranslation2').setSFVec3f(rlTranslation)
        robot.getField('llRotation2').setSFRotation(Rotation)
        robot.getField('rlRotation2').setSFRotation(Rotation)

    def relocate_arm_leg_all(self):
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.relocate_arm_leg(team, id)

    def turn_motion(self, team, id):
        robot = self.robot[team][id]['node']
        lw = self.speeds_buffer[team][id * 6]
        rw = self.speeds_buffer[team][id * 6 + 1]
        is_dribbler = self.dribbler != None and self.dribbler[0] == team and self.dribbler[1] == id
        if (abs(lw) < 0.1) and (abs(rw) < 0.1) and not is_dribbler:
            return
        elif (abs(lw) < 0.1) ^ (abs(rw) < 0.1):
            if abs(lw) < 0.1:
                if rw > 0:
                    index = 0
                    if is_dribbler:
                        index = 4
                else:
                    index = 2
                    if is_dribbler:
                        index = 6
            else:
                if lw > 0:
                    index = 1
                    if is_dribbler:
                        index = 5
                else:
                    index = 3
                    if is_dribbler:
                        index = 7
        else:
            if rw > 0:
                index = 0
                if is_dribbler:
                    index = 4
            else:
                index = 1
                if is_dribbler:
                    index = 5

        ll1_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.023,-0.052],
                 [ 0.000, 0.010, 0.000],[ 0.000, 0.019, 0.053],
                 [ 0.000, 0.010, 0.000],[-0.017, 0.114,-0.128],
                 [ 0.000, 0.010, 0.000],[ 0.026, 0.111,-0.115]]
        ll2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.010, 0.000],[ 0.000, 0.025, 0.036],
                 [ 0.000, 0.010, 0.000],[ 0.000, 0.025, 0.036]]
        rl1_t = [[ 0.000, 0.023,-0.052],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.019, 0.053],[ 0.000, 0.010, 0.000],
                 [ 0.017, 0.114,-0.128],[ 0.000, 0.010, 0.000],
                 [-0.026, 0.111,-0.115],[ 0.000, 0.010, 0.000]]
        rl2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.025, 0.036],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.025, 0.036],[ 0.000, 0.010, 0.000]]
        ll1_r = [[ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00, 0.4],
                 [ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00,-0.4],
                 [ 1, 0.00, 0.00, 0.0],[ 1, 0.08,-0.03, 1.3],
                 [ 1, 0.00, 0.00, 0.0],[ 1,-0.18, 0.13, 1.3]]
        ll2_r = [[ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00, 0.0],
                 [ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00, 0.0],
                 [ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00,-0.8],
                 [ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00,-0.8]]
        rl1_r = [[ 1, 0.00, 0.00, 0.4],[ 1, 0.00, 0.00, 0.0],
                 [ 1, 0.00, 0.00,-0.4],[ 1, 0.00, 0.00, 0.0],
                 [ 1,-0.08, 0.03, 1.3],[ 1, 0.00, 0.00, 0.0],
                 [ 1, 0.18,-0.13, 1.3],[ 1, 0.00, 0.00, 0.0]]
        rl2_r = [[ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00, 0.0],
                 [ 1, 0.00, 0.00, 0.0],[ 1, 0.00, 0.00, 0.0],
                 [ 1, 0.00, 0.00,-0.8],[ 1, 0.00, 0.00, 0.0],
                 [ 1, 0.00, 0.00,-0.8],[ 1, 0.00, 0.00, 0.0]]
        robot.getField('laTranslation').setSFVec3f([0, 0.07, 0])
        robot.getField('raTranslation').setSFVec3f([0, 0.07, 0])
        robot.getField('laRotation').setSFRotation([1, 0, 0, 0])
        robot.getField('raRotation').setSFRotation([1, 0, 0, 0])
        robot.getField('llTranslation').setSFVec3f(ll1_t[index])
        robot.getField('rlTranslation').setSFVec3f(rl1_t[index])
        robot.getField('llRotation').setSFRotation(ll1_r[index])
        robot.getField('rlRotation').setSFRotation(rl1_r[index])
        robot.getField('llTranslation2').setSFVec3f(ll2_t[index])
        robot.getField('rlTranslation2').setSFVec3f(rl2_t[index])
        robot.getField('llRotation2').setSFRotation(ll2_r[index])
        robot.getField('rlRotation2').setSFRotation(rl2_r[index])

    def kick_motion(self, team, id):
        kc = self.kick_count[team][id]
        if 0 < kc <= 6:
            kick_speed = max(min(self.kick_buffer[team][id][0],10),0)
            kick_height = max(min(self.kick_buffer[team][id][1],10),0)
            robot = self.robot[team][id]['node']
            if kc == 3:
                v = self.abs2rel([0,0,-1,0,0,0],robot.getOrientation())
                robot.setVelocity(v)
            if kc < 5:
                self.ball.setVelocity([0,0,0,0,0,0])
            kc = min(kc - 1,3)
            la1_t = [[ 0.000, 0.086, 0.084],[ 0.000, 0.086, 0.084],
                     [ 0.000, 0.070, 0.000],[ 0.000, 0.088,-0.083]]
            ra1_t = [[ 0.000, 0.088,-0.083],[ 0.000, 0.070, 0.000],
                     [ 0.000, 0.086, 0.084],[ 0.000, 0.133, 0.154]]
            la1_r = [[ 1, 0, 0,-0.4],[ 1, 0, 0,-0.4],
                     [ 1, 0, 0, 0.0],[ 1, 0, 0, 0.4]]
            ra1_r = [[ 1, 0, 0, 0.4],[ 1, 0, 0, 0.0],
                     [ 1, 0, 0,-0.4],[ 1, 0, 0,-0.8]]
            if kick_speed < 5:
                ll1_t = [[ 0.000, 0.014,-0.027],[ 0.000, 0.014,-0.027],
                         [ 0.000, 0.014,-0.027],[ 0.000, 0.010, 0.000]]
                ll2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                         [ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000]]
                rl1_t = [[ 0.000, 0.019, 0.053],[ 0.000, 0.012, 0.027],
                         [ 0.000, 0.014,-0.027],[ 0.000, 0.022,-0.052]]
                rl2_t = [[ 0.000, 0.029, 0.039],[ 0.000, 0.016, 0.024],
                         [ 0.000, 0.016, 0.024],[ 0.000, 0.010, 0.000]]
                ll1_r = [[ 1, 0, 0, 0.2],[ 1, 0, 0, 0.2],
                         [ 1, 0, 0, 0.2],[ 1, 0, 0, 0.0]]
                ll2_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0],
                         [ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0]]
                rl1_r = [[ 1, 0, 0,-0.4],[ 1, 0, 0,-0.2],
                         [ 1, 0, 0, 0.2],[ 1, 0, 0, 0.4]]
                rl2_r = [[ 1, 0, 0,-0.9],[ 1, 0, 0,-0.5],
                         [ 1, 0, 0,-0.5],[ 1, 0, 0, 0.0]]
            else:
                ll1_t = [[ 0.000, 0.022,-0.052],[ 0.000, 0.022,-0.052],
                         [ 0.000, 0.022,-0.052],[ 0.000, 0.010, 0.000]]
                ll2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                         [ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000]]
                rl1_t = [[ 0.000, 0.038, 0.088],[ 0.000, 0.019, 0.053],
                         [ 0.000, 0.029,-0.064],[ 0.000, 0.088,-0.118]]
                rl2_t = [[ 0.000, 0.051, 0.049],[ 0.000, 0.029, 0.039],
                         [ 0.000, 0.016, 0.024],[ 0.000, 0.010, 0.000]]
                ll1_r = [[ 1, 0, 0, 0.4],[ 1, 0, 0, 0.4],
                         [ 1, 0, 0, 0.4],[ 1, 0, 0, 0.0]]
                ll2_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0],
                         [ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0]]
                rl1_r = [[ 1, 0, 0,-0.7],[ 1, 0, 0,-0.4],
                         [ 1, 0, 0, 0.5],[ 1, 0, 0, 1.0]]
                rl2_r = [[ 1, 0, 0,-1.4],[ 1, 0, 0,-0.9],
                         [ 1, 0, 0,-0.5],[ 1, 0, 0, 0.0]]
            l = 'l'
            r = 'r'
            f = 1
            if self.kick_trig[team][id] == 2:
                l = 'r'
                r = 'l'
                f = -1
            if (kick_height > 3 and kick_speed > 5) or kick_speed > 9:
                la1_t = [[-0.308*f, 0.202, 0.000],[-0.218*f, 0.201,-0.16],
                         [-0.173*f, 0.254,-0.204],[-0.089*f, 0.287,-0.222]]
                la2_t = [[ 0.000, 0.000, 0.000],[ 0.000, 0.000, 0.000],
                         [ 0.000, 0.000, 0.000],[ 0.000, 0.000, 0.000]]
                ra1_t = [[ 0.053*f, 0.087,-0.111],[ 0.067*f, 0.055, 0.000],
                         [ 0.060*f, 0.087, 0.117],[ 0.000*f, 0.133, 0.154]]
                ra2_t = [[-0.190, 0.101, 0.000],[-0.190, 0.101, 0.000],
                         [-0.190, 0.101, 0.000],[-0.190, 0.101, 0.000]]
                la1_r = [[ 0.0*f, 0.0, 1.0,-1.57*f],[ 0.5*f,-0.2,-0.8, 1.57*f],
                         [ 0.8*f, 0.2,-0.5, 1.57*f],[ 1.0*f, 0.2,-0.2, 1.61*f]]
                la2_r = [[ 0.0, 0.0, 1.0, 0.00],[ 0.0, 0.0, 1.0, 0.00],
                         [ 0.0, 0.0, 1.0, 0.00],[ 0.0, 0.0, 1.0, 0.00]]
                ra1_r = [[ 1.0*f, 0.13, 0.5, 0.60*f],[ 0.0*f, 0.0, 1.0, 0.30*f],
                         [ 1.0*f,-0.13,-0.5,-0.60*f],[ 1.0*f, 0.0, 0.0,-0.80*f]]
                ra2_r = [[ 0.0, 0.0, 1.0,-0.90],[ 0.0, 0.0, 1.0,-0.90],
                         [ 0.0, 0.0, 1.0,-0.90],[ 0.0, 0.0, 1.0,-0.90]]
                robot.getField(l+'aTranslation2').setSFVec3f(la2_t[kc])
                robot.getField(r+'aTranslation2').setSFVec3f(ra2_t[kc])
                robot.getField(l+'aRotation2').setSFRotation(la2_r[kc])
                robot.getField(r+'aRotation2').setSFRotation(ra2_r[kc])

            robot.getField(l+'aTranslation').setSFVec3f(la1_t[kc])
            robot.getField(r+'aTranslation').setSFVec3f(ra1_t[kc])
            robot.getField(l+'aRotation').setSFRotation(la1_r[kc])
            robot.getField(r+'aRotation').setSFRotation(ra1_r[kc])
            robot.getField(l+'lTranslation').setSFVec3f(ll1_t[kc])
            robot.getField(r+'lTranslation').setSFVec3f(rl1_t[kc])
            robot.getField(l+'lRotation').setSFRotation(ll1_r[kc])
            robot.getField(r+'lRotation').setSFRotation(rl1_r[kc])
            robot.getField(l+'lTranslation2').setSFVec3f(ll2_t[kc])
            robot.getField(r+'lTranslation2').setSFVec3f(rl2_t[kc])
            robot.getField(l+'lRotation2').setSFRotation(ll2_r[kc])
            robot.getField(r+'lRotation2').setSFRotation(rl2_r[kc])

    def gk_jump_motion(self, team, id):
        robot = self.robot[team][id]['node']
        s = int(self.jump_trig[team][id]) - 1
        la_id, ra_id, ll_id, rl_id = -1, -1, -1, -1
        la1_t = [[ 0.000, 0.070, 0.000],[ 0.000, 0.495, 0.005],
                 [ 0.000, 0.484,-0.075],[ 0.000, 0.441,-0.143],
                 [ 0.000, 0.375,-0.191],[ 0.000, 0.235,-0.206],
                 [ 0.000, 0.118,-0.132],[ 0.047, 0.480, 0.005],
                 [ 0.059, 0.366,-0.163],[ 0.050, 0.234,-0.180]]
        ra1_t = [[ 0.000, 0.070, 0.000],[ 0.000, 0.495, 0.005],
                 [ 0.000, 0.484,-0.075],[ 0.000, 0.441,-0.143],
                 [ 0.000, 0.375,-0.191],[ 0.000, 0.235,-0.206],
                 [ 0.000, 0.118,-0.132],[-0.047, 0.480, 0.005],
                 [-0.059, 0.366,-0.163],[-0.050, 0.234,-0.180]]
        la1_r = [[ 1, 0, 0, 0.00],[ 1, 0, 0, 3.14],
                 [ 1, 0, 0, 2.76],[ 1, 0, 0, 2.38],
                 [ 1, 0, 0, 2.00],[ 1, 0, 0, 1.33],
                 [ 1, 0, 0, 0.67],[ 1,-0.10, 0, 3.14],
                 [ 1,-0.13, 0.13, 2.00],[ 1,-0.15, 0.15, 1.33]]
        ra1_r = [[ 1, 0, 0, 0.00],[ 1, 0, 0, 3.14],
                 [ 1, 0, 0, 2.76],[ 1, 0, 0, 2.38],
                 [ 1, 0, 0, 2.00],[ 1, 0, 0, 1.33],
                 [ 1, 0, 0, 0.67],[ 1, 0.10, 0, 3.14],
                 [ 1, 0.13,-0.13, 2.00],[ 1, 0.15,-0.15, 1.33]]
        ll1_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.045,-0.086],
                 [ 0.000, 0.076,-0.111],[ 0.000, 0.101,-0.123]]
        ll2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.019, 0.028],
                 [ 0.000, 0.033, 0.042],[ 0.000, 0.052, 0.049]]
        rl1_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.045,-0.086],
                 [ 0.000, 0.076,-0.111],[ 0.000, 0.101,-0.123]]
        rl2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.019, 0.028],
                 [ 0.000, 0.033, 0.042],[ 0.000, 0.052, 0.049]]
        ll1_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0, 0.7],
                 [ 1, 0, 0, 1.0],[ 1, 0, 0, 1.2]]
        ll2_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0,-0.6],
                 [ 1, 0, 0,-1.0],[ 1, 0, 0,-1.4]]
        rl1_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0, 0.7],
                 [ 1, 0, 0, 1.0],[ 1, 0, 0, 1.2]]
        rl2_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0,-0.6],
                 [ 1, 0, 0,-1.0],[ 1, 0, 0,-1.4]]
        if self.jump_count[team][id] == 1:
            self.relocate_arm_leg(team, id)
        if self.jump_count[team][id] == 1:
            la_id, ra_id, ll_id, rl_id = 1, 1, 0, 0
        elif self.jump_count[team][id] == 11:
            la_id, ra_id, ll_id, rl_id = 2, 2, 1, 1
        elif self.jump_count[team][id] == 12:
            la_id, ra_id, ll_id, rl_id = 3, 3, 2, 2
        elif self.jump_count[team][id] == 13:
            la_id, ra_id, ll_id, rl_id = 4, 4, 3, 3
        elif self.jump_count[team][id] == 16:
            la_id, ra_id, ll_id, rl_id = 5, 5, 2, 2
        elif self.jump_count[team][id] == 17:
            la_id, ra_id, ll_id, rl_id = 6, 6, 1, 1
        elif self.jump_count[team][id] == 18:
            la_id, ra_id, ll_id, rl_id = 0, 0, 1, 1
        elif self.jump_count[team][id] == 19:
            la_id, ra_id, ll_id, rl_id = -1, -1, 0, 0
        elif self.jump_count[team][id] == 20:
            la_id, ra_id, ll_id, rl_id = 0, 0, 0, 0

        if self.gk_grab[team] != 0:
            if self.jump_count[team][id] < 11:
                la_id, ra_id = 7, 7
            elif 13 < self.jump_count[team][id] <= 20:
                la_id, ra_id = 8, 8
            elif self.jump_count[team][id] == 21:
                la_id, ra_id = 9, 9
            elif self.jump_count[team][id] == 22:
                la_id, ra_id = 6, 6
            elif self.jump_count[team][id] == 23:
                la_id, ra_id = 0, 0

        if la_id != -1:
            robot.getField('laTranslation').setSFVec3f(la1_t[la_id])
            robot.getField('laRotation').setSFRotation(la1_r[la_id])
        if ra_id != -1:
            robot.getField('raTranslation').setSFVec3f(ra1_t[ra_id])
            robot.getField('raRotation').setSFRotation(ra1_r[ra_id])
        if ll_id != -1:
            robot.getField('llTranslation').setSFVec3f(ll1_t[ll_id])
            robot.getField('llRotation').setSFRotation(ll1_r[ll_id])
            robot.getField('llTranslation2').setSFVec3f(ll2_t[ll_id])
            robot.getField('llRotation2').setSFRotation(ll2_r[ll_id])
        if rl_id != -1:
            robot.getField('rlTranslation').setSFVec3f(rl1_t[rl_id])
            robot.getField('rlRotation').setSFRotation(rl1_r[rl_id])
            robot.getField('rlTranslation2').setSFVec3f(rl2_t[rl_id])
            robot.getField('rlRotation2').setSFRotation(rl2_r[rl_id])

    def gk_side_move(self, team, id):
        side_speed = min(max(self.speeds_buffer[team][id * 6 + 5],-constants.MAX_LINEAR_VELOCITY[id]),constants.MAX_LINEAR_VELOCITY[id])
        robot = self.robot[team][id]['node']
        orientation = robot.getOrientation()
        ll_t = robot.getField('llTranslation').getSFVec3f()
        rl_t = robot.getField('rlTranslation').getSFVec3f()
        if self.gk_side_trig[team] == 0 and side_speed != 0 and orientation[4] > 0.8:
            self.gk_side_trig[team] = side_speed
            self.relocate_arm_leg(team, id)
            ll_t = [0,0.01,0]
            rl_t = [0,0.01,0]
        if self.gk_side_trig[team] != 0:
            gst = abs(self.gk_side_trig[team])
            tran_y = 0.032 # 0.07
            tran_z = 0.015 # 0.02
            max_angle = 3.14/12   # 3.14/7
            st = int(6/gst)
            cnt = int(self.gk_side_count[team]/st)
            if self.gk_side_trig[team] < 0:
                if cnt == 0:
                    v = [-0.7*gst,0,0,0,0,0]
                    ll_t_n = [ll_t[0] - tran_y/st, ll_t[1] - tran_z/st, 0]
                    rl_t_n = rl_t
                    ll_r_n = [0,0,-1, self.gk_leg_rot[team][0] + max_angle/st]
                    rl_r_n = [0,0,1,0]
                elif cnt == 1:
                    v = [-1*gst,0,0,0,0,0]
                    ll_t_n = [ll_t[0] + tran_y/st, ll_t[1] + tran_z/st, 0]
                    rl_t_n = [rl_t[0] + tran_y/st, rl_t[1] - tran_z/st, 0]
                    ll_r_n = [0,0,-1, self.gk_leg_rot[team][0] - max_angle/st]
                    rl_r_n = [0,0,1, self.gk_leg_rot[team][1] + max_angle/st]
                elif cnt == 2:
                    v = [-1*gst,0,0,0,0,0]
                    ll_t_n = ll_t
                    rl_t_n = [rl_t[0] - tran_y/st, rl_t[1] + tran_z/st, 0]
                    ll_r_n = [0,0,1,0]
                    rl_r_n = [0,0,1, self.gk_leg_rot[team][1] - max_angle/st]
                self.gk_leg_rot[team] = [ll_r_n[3],rl_r_n[3]]
            elif self.gk_side_trig[team] > 0:
                if cnt == 0:
                    v = [0.7*gst,0,0,0,0,0]
                    ll_t_n = ll_t
                    rl_t_n = [rl_t[0] + tran_y/st, rl_t[1] - tran_z/st, 0]
                    ll_r_n = [0,0,1,0]
                    rl_r_n = [0,0,1, self.gk_leg_rot[team][1] + max_angle/st]
                elif cnt == 1:
                    v = [1*gst,0,0,0,0,0]
                    ll_t_n = [ll_t[0] - tran_y/st, ll_t[1] - tran_z/st, 0]
                    rl_t_n = [rl_t[0] - tran_y/st, rl_t[1] + tran_z/st, 0]
                    ll_r_n = [0,0,-1, self.gk_leg_rot[team][0] + max_angle/st]
                    rl_r_n = [0,0,1, self.gk_leg_rot[team][1] - max_angle/st]
                elif cnt == 2:
                    v = [1*gst,0,0,0,0,0]
                    ll_t_n = [ll_t[0] + tran_y/st, ll_t[1] + tran_z/st, 0]
                    rl_t_n = rl_t
                    ll_r_n = [0,0,-1, self.gk_leg_rot[team][0] - max_angle/st]
                    rl_r_n = [0,0,1,0]
                self.gk_leg_rot[team] = [ll_r_n[3],rl_r_n[3]]
            self.gk_side_count[team] += 1
            if self.gk_side_count[team] == 3*st:
                self.gk_side_count[team] = 0
                self.gk_side_trig[team] = 0

            robot.getField('llTranslation').setSFVec3f(ll_t_n)
            robot.getField('rlTranslation').setSFVec3f(rl_t_n)
            robot.getField('llRotation').setSFRotation(ll_r_n)
            robot.getField('rlRotation').setSFRotation(rl_r_n)

            v = self.abs2rel(v, orientation)
            robot.setVelocity(v)

    def sliding_motion(self, team, id):
        robot = self.robot[team][id]['node']
        l, r = 'l', 'r'
        f = 1
        if (self.time/50 - self.jump_count[team][id]) % 2 == 1:
            l, r = 'r', 'l'
            f = -1
        la_id, ra_id, ll_id, rl_id = -1, -1, -1, -1
        la1_t = [[ 0.000, 0.070, 0.000],[ 0.000, 0.095, 0.103],
                 [ 0.000, 0.125, 0.146],[ 0.000, 0.166, 0.181],
                 [-0.133*f, 0.060,-0.014],[-0.265*f, 0.135,-0.039],
                 [-0.317*f, 0.302,-0.062],[ 0.000, 0.070, 0.000]]
        ra1_t = [[ 0.000, 0.070, 0.000],[ 0.000, 0.095, 0.103],
                 [ 0.000, 0.125, 0.146],[ 0.000, 0.166, 0.181],
                 [ 0.105*f, 0.058, 0.017],[ 0.211*f, 0.095, 0.034],
                 [ 0.290*f, 0.192, 0.064],[ 0.000, 0.070, 0.000]]
        la1_r = [[ 1, 0, 0, 0.00],[ 1, 0, 0,-0.50],
                 [ 1, 0, 0,-0.75],[ 1, 0, 0,-1.00],
                 [-0.1*f, 0.1, 1,-0.6*f],[-0.1*f, 0.1, 1,-1.3*f],
                 [-0.1*f, 0.1, 1,-2*f],[ 1, 0, 0, 0.00]]
        ra1_r = [[ 1, 0, 0, 0.00],[ 1, 0, 0,-0.50],
                 [ 1, 0, 0,-0.75],[ 1, 0, 0,-1.00],
                 [-0.15*f,-0.15, 1, 0.5*f],[-0.15*f,-0.15, 1, 1*f],
                 [-0.15*f,-0.15, 1, 1.57*f],[ 1, 0, 0, 0.00]]
        ll1_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.017,-0.027],[ 0.000, 0.018,-0.040],
                 [ 0.000, 0.010, 0.008],[ 0.000, 0.011, 0.018],
                 [ 0.000, 0.012, 0.027],[ 0.000, 0.010, 0.000]]
        ll2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.016, 0.024],
                 [ 0.000, 0.038, 0.044],[ 0.000, 0.067, 0.050],
                 [ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000]]
        rl1_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.010, 0.000],
                 [ 0.000, 0.014,-0.027],[ 0.000, 0.018,-0.040],
                 [ 0.005*f, 0.010,-0.006],[ 0.011*f, 0.010,-0.012],
                 [ 0.012*f, 0.010,-0.018],[ 0.000, 0.010, 0.000]]
        rl2_t = [[ 0.000, 0.010, 0.000],[ 0.000, 0.016, 0.024],
                 [ 0.000, 0.038, 0.044],[ 0.000, 0.067, 0.050],
                 [ 0.000, 0.025, 0.036],[ 0.000, 0.062, 0.050],
                 [ 0.000, 0.093, 0.037],[ 0.000, 0.010, 0.000]]
        ll1_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0],
                 [ 1, 0, 0, 0.2],[ 1, 0, 0, 0.3],
                 [ 1, 0, 0,-0.06],[ 1, 0, 0,-0.13],
                 [ 1, 0, 0,-0.2],[ 1, 0, 0, 0.0]]
        ll2_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0,-0.5],
                 [ 1, 0, 0,-1.1],[ 1, 0, 0,-1.7],
                 [ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0],
                 [ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0]]
        rl1_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0, 0.0],
                 [ 1, 0, 0, 0.2],[ 1, 0, 0, 0.3],
                 [ 0, 1, 0,-0.2*f],[ 0, 1, 0,-0.5*f],
                 [ 0, 1, 0,-0.8*f],[ 1, 0, 0, 0.0]]
        rl2_r = [[ 1, 0, 0, 0.0],[ 1, 0, 0,-0.5],
                 [ 1, 0, 0,-1.1],[ 1, 0, 0,-1.7],
                 [ 1, 0, 0,-0.8],[ 1, 0, 0,-1.6],
                 [ 1, 0, 0,-2.3],[ 1, 0, 0, 0.0]]
        if self.jump_count[team][id] == 1:
            self.relocate_arm_leg(team, id)
        if self.jump_count[team][id] == 2:
            la_id, ra_id, ll_id, rl_id = 4, 4, 4, 4
        elif self.jump_count[team][id] == 3:
            la_id, ra_id, ll_id, rl_id = 5, 5, 5, 5
        elif 5 <= self.jump_count[team][id] <= 10:
            la_id, ra_id, ll_id, rl_id = 6, 6, 6, 6
        elif self.jump_count[team][id] == 11:
            la_id, ra_id, ll_id, rl_id = 5, 5, 5, 5
        elif self.jump_count[team][id] == 12:
            la_id, ra_id, ll_id, rl_id = 4, 4, 4, 4
        elif self.jump_count[team][id] == 13:
            la_id, ra_id, ll_id, rl_id = 0, 0, 1, 1
        elif self.jump_count[team][id] == 14:
            la_id, ra_id, ll_id, rl_id = 0, 0, 2, 2
        elif self.jump_count[team][id] == 15:
            la_id, ra_id, ll_id, rl_id = 1, 1, 3, 3
        elif self.jump_count[team][id] == 16:
            la_id, ra_id, ll_id, rl_id = 2, 2, 3, 3
        elif self.jump_count[team][id] == 17:
            la_id, ra_id, ll_id, rl_id = 3, 3, 2, 2
        elif self.jump_count[team][id] == 18:
            la_id, ra_id, ll_id, rl_id = 2, 2, 2, 2
        elif self.jump_count[team][id] == 19:
            la_id, ra_id, ll_id, rl_id = 1, 1, 1, 1
        elif self.jump_count[team][id] == 20:
            la_id, ra_id, ll_id, rl_id = 0, 0, 0, 0

        if la_id != -1:
            robot.getField(l+'aTranslation').setSFVec3f(la1_t[la_id])
            robot.getField(l+'aRotation').setSFRotation(la1_r[la_id])
        robot.getField(l+'aTranslation2').setSFVec3f([0, 0, 0])
        robot.getField(l+'aRotation2').setSFRotation([1, 0, 0, 0])
        if ra_id != -1:
            robot.getField(r+'aTranslation').setSFVec3f(ra1_t[ra_id])
            robot.getField(r+'aRotation').setSFRotation(ra1_r[ra_id])
        robot.getField(r+'aTranslation2').setSFVec3f([0, 0, 0])
        robot.getField(r+'aRotation2').setSFRotation([1, 0, 0, 0])
        if ll_id != -1:
            robot.getField(l+'lTranslation').setSFVec3f(ll1_t[ll_id])
            robot.getField(l+'lRotation').setSFRotation(ll1_r[ll_id])
            robot.getField(l+'lTranslation2').setSFVec3f(ll2_t[ll_id])
            robot.getField(l+'lRotation2').setSFRotation(ll2_r[ll_id])
        if rl_id != -1:
            robot.getField(r+'lTranslation').setSFVec3f(rl1_t[rl_id])
            robot.getField(r+'lRotation').setSFRotation(rl1_r[rl_id])
            robot.getField(r+'lTranslation2').setSFVec3f(rl2_t[rl_id])
            robot.getField(r+'lRotation2').setSFRotation(rl2_r[rl_id])

    def kick(self):
        flags = [[False for _ in range(3)] for _ in range(2)]
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                if self.robot[team][id]['active']:
                    flags[team][id] = self.check_kick(team, id)
                    self.kick_motion(team, id)
        if flags[0].count(True) + flags[1].count(True) == 1:
            for team in self.constants.TEAMS:
                for id in range(self.constants.NUMBER_OF_ROBOTS):
                    if flags[team][id] == True:
                        self.robot[team][id]['touch'] = True
                        m_kick = 0.7
                        # locate the ball in front of the robot
                        if self.dribbler != None and \
                            self.dribbler[0] == team and self.dribbler[1] == id:
                            self.dribbler = None
                        robot_pos = self.get_robot_posture(team, id)
                        x = robot_pos[0]
                        y = robot_pos[1]
                        th = robot_pos[3]
                        node = self.ball

                        d_range = self.constants.ROBOT_SIZE[id]/2 + self.constants.BALL_RADIUS

                        t_x = x + d_range*math.cos(th)
                        t_y = y + d_range*math.sin(th)
                        ball_new = [t_x, t_y]

                        node.resetPhysics()
                        f = 1
                        translation = [f * ball_new[0], 0.05, -f * ball_new[1]]
                        node.getField('translation').setSFVec3f(translation)

                        # set ball velocity
                        orientation = self.robot[team][id]['node'].getOrientation()
                        kick_th = (0) + math.atan2(orientation[2], orientation[8]) + self.constants.PI / 2
                        while kick_th > self.constants.PI:
                            kick_th -= 2 * self.constants.PI
                        while kick_th < -self.constants.PI:
                            kick_th += 2 * self.constants.PI

                        kick_height = max(0,min(self.kick_buffer[team][id][1],10))
                        kick_h_angle = self.constants.PI / 2 - kick_height * 4 / 180 * self.constants.PI
                        kick_speed = m_kick * max(min(self.kick_buffer[team][id][0],10),0)

                        x = kick_speed * math.sin(kick_h_angle) * math.cos(kick_th)
                        y = kick_speed * math.sin(kick_h_angle) * math.sin(kick_th)
                        z = kick_speed * math.cos(kick_h_angle)

                        f = 1

                        bv = [f * x, z, f * -y,
                              random.randrange(-10,10), random.randrange(-10,10), random.randrange(-10,10)]
                        self.ball.setVelocity(bv)

    def check_kick(self, team, id):
        kick_speed = max(min(self.speeds_buffer[team][id * 6 + 2],10),0)
        flag = False

        if (sum(self.kick_trig[0]) + sum(self.kick_trig[1]) == 0) and (kick_speed > 0) and \
            (self.ball_in_kick_area(team, id) or \
                (self.dribbler != None and self.dribbler[0] == team and self.dribbler[1] == id)):
            self.kick_trig[team][id] = np.random.randint(2) + 1
            self.kick_buffer[team][id] = [self.speeds_buffer[team][6*id+2], self.speeds_buffer[team][6*id+3]]

        if self.kick_trig[team][id] != 0:
            self.kick_count[team][id] += 1
            if self.kick_count[team][id] == 4 and self.ball_in_kick_area(team, id):
                flag = True
            elif self.kick_count[team][id] == 7:
                self.relocate_arm_leg(team, id)
                self.kick_buffer[team][id] = [0,0]
                self.kick_count[team][id] = 0
                self.kick_trig[team][id] = 0

        return flag

    def ball_in_kick_area(self, team1, id1):
        ball_x = self.get_ball_position()[0]
        ball_y = self.get_ball_position()[1]
        ball_z = self.get_ball_position()[2]
        robot_pos = self.get_robot_posture(team1, id1)
        x = robot_pos[0]
        y = robot_pos[1]
        z = robot_pos[2]
        th = robot_pos[3]

        theta = th
        if (th > self.constants.PI):
            theta -= 2*self.constants.PI
        d_theta = abs(theta - math.atan2(ball_y-y, ball_x-x))
        if (d_theta > self.constants.PI):
            d_theta -= 2*self.constants.PI
        dist = math.sqrt((ball_y - y)*(ball_y - y)+(ball_x - x)*(ball_x - x))
        add = 0.07
        d_range = self.constants.ROBOT_SIZE[id1]/2 + self.constants.BALL_RADIUS + add

        if ((dist < d_range) and (abs(d_theta) < self.constants.PI/8) and (abs(z - ball_z) < 0.01)):
            return True
        else:
            return False

    def check_jump_GK(self, speeds, team, id):
        jump_signal = max(min(speeds[id * 6 + 4],10),-10)
        robot = self.robot[team][id]['node']
        orientation = robot.getOrientation()
        power = [0,0,0.5,0,0,
                 1,1.2,1.5,1.2,1]
        power_turn = [-15, -10.5, 0, 10.5, 15,
                      -15, -7.5, 0, 7.5, 15]
        power_re = [12.5, 12.5, 0, 12.5, 12.5,
                    12.5, 12.5, 0, 12.5, 12.5]
        sound_time = [6,10,5,10,6,
                      6,10,5,10,6]
        if ((self.jump_trig[team][id] == 0) and (jump_signal != 0) and (orientation[4] > 0.8)):
            self.jump_trig[team][id] = jump_signal
        if self.jump_trig[team][id] < 0:
            self.sliding(team, id)
            return
        if (self.jump_trig[team][id] != 0):
            s = int(self.jump_trig[team][id]) - 1
            l = -1 if s % 5 > 2 else 1
            self.jump_count[team][id] += 1
            if (self.jump_count[team][id] == 1):
                v = [0,0,0,0,0,power_turn[s]]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif (self.jump_count[team][id] == 2):
                v = [0,power[s],0,0,0,power_turn[s]]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif (self.jump_count[team][id] == 3):
                v = [0,power[s],0,0,0,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif (self.jump_count[team][id] == 6):
                if sound_time[s] == 6:
                    v = [0,0,0,0,0,0]
                else:
                    v = [0,0,0,0,0,0.5*power_turn[s]]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 9:
                if s < 5:
                    robot.resetPhysics()
            elif self.jump_count[team][id] == 10:
                if orientation[4] > 0.2:
                    self.jump_count[team][id] = 19
                else:
                    v = [0,0,0,0,-35*l,0]
                    v = self.abs2rel(v, orientation)
                    robot.setVelocity(v)
            elif 14 <= self.jump_count[team][id] <= 15:
                robot.resetPhysics()
            elif self.jump_count[team][id] == 16:
                v = [0,0,0,power_re[s]*2/4,20*l,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 17:
                v = [0,-0.6,0,power_re[s]*2/4,20*l,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 18:
                v = [0,-1,0,power_re[s]*2/4,20*l,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 19:
                r = 10 if l == 1 else -28
                v = [0,-1.05,0,power_re[s]*2/5,r,3]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 20:
                v = [0,0,0,0,0,0]
                robot.setVelocity(v)
                trans = robot.getField('translation').getSFVec3f()
                rot = robot.getField('rotation').getSFRotation()
                robot.getField('translation').setSFVec3f([trans[0],0.045,trans[2]])
                robot.getField('rotation').setSFRotation([0,1,0,rot[3]])
                robot.resetPhysics()
            elif self.jump_count[team][id] == 26:
                self.jump_count[team][id] = 0
                self.jump_trig[team][id] = 0
            self.gk_grab_ball(team)
            self.gk_jump_motion(team, id)

    def gk_grab_ball(self, team):
        robot_pos = self.get_robot_posture(team, 0)
        robot = self.robot[team][0]['node']
        x = robot_pos[0]
        y = robot_pos[1]
        z = robot_pos[2]
        th = robot_pos[3]
        ball = self.ball
        ball_x = self.get_ball_position()[0]
        ball_y = self.get_ball_position()[1]
        ball_z = self.get_ball_position()[2]
        orientation = robot.getOrientation()
        ori = [[],[],[]]
        ori[0] = orientation[0:3]
        ori[1] = orientation[3:6]
        ori[2] = orientation[6:9]
        theta = th + constants.PI/2
        if team == 1:
            theta = th - constants.PI*3/2
        orientation = np.dot(ori,[[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
        f = 1
        orientation = orientation.tolist()[0]+orientation.tolist()[1]+orientation.tolist()[2]
        dx = ball_x - x
        dy = ball_y - y
        dz = ball_z - z
        dx,dz,dy,_,_,_ = self.abs2rel([dx,dz,dy,0,0,0], orientation)
        vx_b = self.abs2rel(ball.getVelocity(), orientation)[0]

        if self.jump_trig[team][0] != 0:
            if ((-0.05 < dx < max(abs(vx_b)*0.05,0.05)) and (abs(dy) < 0.05) and (0.35 < dz < 0.45)) and self.gk_grab[team] == 0:
                self.gk_grab[team] = 1

        if self.gk_grab[team] != 0:
            self.robot[team][0]['touch'] = True
            if self.jump_count[team][0] < 23:
                t_d = [[0.01, 0, 0.45],
                       [0.06, 0, 0.41],
                       [0.07, 0, 0.38],
                       [0.08, 0, 0.34],
                       [0.09, 0, 0.32],
                       [0.09, 0, 0.32],
                       [0.10, 0, 0.32],
                       [0.10, 0, 0.32],
                       [0.10, 0, 0.32],
                       [0.12, 0, 0.32],
                       [0.13, 0, 0.32],
                       [0.14, 0, 0.24],
                       [0.14, 0, 0.20]
                ]
                if self.jump_count[team][0] < 11:
                    t = 0
                else:
                    t = min(self.jump_count[team][0] - 10, len(t_d)-1)
                if 17 <= self.jump_count[team][0] <= 20:
                    if (self.jump_trig[team][0] - 1) % 5 < 2:
                        t_d[t][1] = -0.02
                    elif (self.jump_trig[team][0] - 1) % 5 > 2:
                        t_d[t][1] = 0.02
                tran = np.dot([robot.getOrientation()[:3],robot.getOrientation()[3:6],robot.getOrientation()[6:9]],\
                                [t_d[t][1],t_d[t][2],-t_d[t][0]])+robot.getPosition()
                translation = [tran[0],tran[1],tran[2]]
                ball.getField('translation').setSFVec3f(translation)
                ball.setVelocity([0,0,0,0,0,0])
            elif self.jump_count[team][0] == 25:
                self.gk_grab[team] = 0

    def any_object_nearby(self, target_x, target_y, target_r):
        # check ball position
        pos = self.get_ball_position()
        x = pos[0]
        y = pos[1]
        dist_sq = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y)
        # the ball is within the region
        if dist_sq < target_r * target_r:
            return True
        # check robot positions
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                pos = self.get_robot_posture(team, id)
                x = pos[0]
                y = pos[1]
                dist_sq = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y)
                # a robot is within the region
                if dist_sq < target_r * target_r:
                    return True

    def return_to_field(self, team, id):
        robot = self.robot[team][id]['node']
        f = 1
        s = 1 if team == 0 else -1
        translation = [f * self.constants.ROBOT_FORMATION[self.constants.FORMATION_DEFAULT][id][0] * s,
                       0.09 / 2,
                       f * -self.constants.ROBOT_FORMATION[self.constants.FORMATION_DEFAULT][id][1] * s]
        angle = 0
        angle += self.constants.ROBOT_FORMATION[self.constants.FORMATION_DEFAULT][id][2]
        angle += 0 if team == 0 else self.constants.PI
        angle -= self.constants.PI / 2
        rotation = [0, 1, 0, angle]
        al = robot.getField('axleLength').getSFFloat()
        h = robot.getField('height').getSFFloat()
        wr = robot.getField('wheelRadius').getSFFloat()
        lwTranslation = [-al / 2, (-h + 2 * wr) / 2, 0]
        rwTranslation = [al / 2, (-h + 2 * wr) / 2, 0]
        wheelRotation = [1, 0, 0, self.constants.PI / 2]
        robot.getField('translation').setSFVec3f(translation)
        robot.getField('rotation').setSFRotation(rotation)
        robot.getField('lwTranslation').setSFVec3f(lwTranslation)
        robot.getField('lwRotation').setSFRotation(wheelRotation)
        robot.getField('rwTranslation').setSFVec3f(rwTranslation)
        robot.getField('rwRotation').setSFRotation(wheelRotation)
        self.relocate_all(team, id)
        robot.resetPhysics()

    def check_gk_grab_rule(self):
        for t in constants.TEAMS:
            if self.gk_grab[t] == 1:
                gk_pos = self.get_robot_posture(t, 0)
                for id in range(constants.NUMBER_OF_ROBOTS):
                    team = 1 if t == 0 else 0
                    pos = self.get_robot_posture(team, id)
                    sign = 1 if team == constants.TEAM_RED else -1
                    dist = get_distance(gk_pos[0], gk_pos[1], pos[0], pos[1])
                    if dist < 0.5:
                        ix = sign * constants.ROBOT_FORMATION[constants.FORMATION_DEFAULT][id][0]
                        iy = sign * constants.ROBOT_FORMATION[constants.FORMATION_DEFAULT][id][1]
                        r = 1.5 * constants.ROBOT_SIZE[id]
                        if not self.any_object_nearby(ix, iy, r):
                            self.return_to_field(team, id)

    def sliding(self, team, id):
        robot = self.robot[team][id]['node']
        orientation = robot.getOrientation()
        if (self.jump_trig[team][id] != 0):
            s = 0.15 * self.jump_trig[team][id] - 1.5
            self.jump_count[team][id] += 1
            if self.jump_count[team][id] == 1:
                v = [0,1.5,-2,0,0,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] <= 3:
                th = self.get_robot_posture(team,id)[3]
                f = 1
                v_l = [-s*math.cos(th)*f,0,s*math.sin(th)*f]
                v_a = [0,0,0,12,0,0]
                v_a = self.abs2rel(v_a, orientation)
                v = v_l + v_a[3:]
                robot.setVelocity(v)
            elif self.jump_count[team][id] <= 6:
                v = [0,s,0,0,0,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 7:
                v = [0,0,0,5,0,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 13:
                if orientation[4] > 0.2:
                    self.jump_count[team][id] = 20
            elif 14 <= self.jump_count[team][id] <= 16:
                v = [0,0,0,-3,0,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif 16 < self.jump_count[team][id] <= 18:
                v = [0,0,0,-9,0,0]
                v = self.abs2rel(v, orientation)
                robot.setVelocity(v)
            elif self.jump_count[team][id] == 20:
                v = [0,0,0,0,0,0]
                robot.setVelocity(v)
                trans = robot.getField('translation').getSFVec3f()
                rot = robot.getField('rotation').getSFRotation()
                robot.getField('translation').setSFVec3f([trans[0],0.045,trans[2]])
                robot.getField('rotation').setSFRotation([0,1,0,rot[3]])
                robot.resetPhysics()
            elif self.jump_count[team][id] == 26:
                self.jump_count[team][id] = 0
                self.jump_trig[team][id] = 0
            self.sliding_motion(team, id)

    def check_jump(self, speeds, team, id):
        jump_speed = max(min(speeds[id * 6 + 4],10),-10)
        robot = self.robot[team][id]['node']
        orientation = robot.getOrientation()
        if ((self.jump_trig[team][id] == 0) and (jump_speed != 0) and (orientation[4] > 0.8)):
            self.jump_trig[team][id] = jump_speed
        if self.jump_trig[team][id] < 0:
            self.sliding(team, id)
            return

        if (self.jump_trig[team][id] != 0):
            self.jump_count[team][id] += 1
            speed = self.jump_trig[team][id]
            if (self.jump_count[team][id] <= 2):
                v = self.robot[team][id]['node'].getVelocity()
                v[1] += 0.15*speed
                robot.setVelocity(v)
                self.robot[team][id]['node'].setVelocity(v)
            elif self.jump_count[team][id] == 20:
                self.jump_count[team][id] = 0
                self.jump_trig[team][id] = 0

    def abs2rel(self, v_a, orientation):
        ori = [[],[],[]]
        ori[0] = orientation[0:3]
        ori[1] = orientation[3:6]
        ori[2] = orientation[6:9]
        v_a[0:3] = np.dot(ori,v_a[0:3])
        v_a[3:6] = np.dot(ori,v_a[3:6])
        return v_a

    def relocate_all(self, team, id):
        self.jump_count[team][id] = 0
        self.jump_trig[team][id] = 0
        self.kick_count[team][id] = 0
        self.kick_trig[team][id] = 0
        self.kick_buffer[team][id] = [0,0]
        self.relocate_arm_leg(team, id)
        self.arm_count[team][id] = 0
        self.arm_speed[team][id] = 0
        self.reset_speeds[team] = True

    def relocate_all_every(self):
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.relocate_all(team, id)

    ## ball possession ##
    def get_ball_possession(self):
        for team in constants.TEAMS:
            for id in range(constants.NUMBER_OF_ROBOTS):
                if self.robot_in_field(team, id):
                    self.ball_possession[team][id] = self.check_ball_poss(team, id)
                    self.robot[team][id]['ball_possession'] = self.ball_possession[team][id]

    def check_ball_poss(self, team, id):
        ball_x = self.get_ball_position()[0]
        ball_y = self.get_ball_position()[1]
        ball_z = self.get_ball_position()[2]
        robot_pos = self.get_robot_posture(team, id)
        x = robot_pos[0]
        y = robot_pos[1]
        z = robot_pos[2]
        th = robot_pos[3]

        theta = th
        if (th > self.constants.PI):
            theta -= 2*self.constants.PI
        d_theta = abs(theta - math.atan2(ball_y-y, ball_x-x))
        if (d_theta > self.constants.PI):
            d_theta -= 2*self.constants.PI
        dist = math.sqrt((ball_y - y)*(ball_y - y)+(ball_x - x)*(ball_x - x))
        add = 0.1
        d_range = self.constants.ROBOT_SIZE[id]/2 + self.constants.BALL_RADIUS + add

        if ((dist < d_range) and (abs(d_theta) < self.constants.PI/4) and (abs(z - ball_z) < 0.01)):
            return True
        else:
            return False

    ## spotlight
    def ball_spotlight_stop(self):
        self.spotlight.setVelocity([0, 0, 0, 0, 0, 0])

    def ball_spotlight(self):
        p = self.ball.getPosition()
        v = self.ball.getVelocity()
        self.spotlight.getField('translation').setSFVec3f([p[0], 0.95 + (p[1]-0.05)*2/7, p[2]])
        self.spotlight.setVelocity([v[0], v[1]*2/7, v[2], 0, 0, 0])

    def target_spotlight(self, p):
        self.spotlight.getField('translation').setSFVec3f([p[0], 0.95 + (p[1]-0.05)*2/7, p[2]])

    ## dribble
    def is_kicking(self, team, id):
        return self.kick_trig[team][id] == 1

    def check_dribble(self):
        if self.dribbler == None:
            for t in self.constants.TEAMS:
                for i in range(self.constants.NUMBER_OF_ROBOTS):
                    if self.ball_in_dribble_area(t, i) and self.recent_touch[t][i]:
                        self.dribbler = [t, i, 0, 0]
                        break

        if self.dribbler == None:
            return False

        team, id = self.dribbler[0], self.dribbler[1]

        if not self.ball_in_dribble_area(team, id):
            self.dribbler = None
            return False

        if self.speeds_buffer[team][6*id] + self.speeds_buffer[team][6*id+1] < 0:
            self.dribbler = None
            return False

        if self.reset_reason != Game.NONE:
            self.dribbler = None
            return False

        # robot shouldn't be kicking or jumping
        if self.jump_trig[team][id] != 0:
            self.dribbler = None
            return False

        if self.kick_trig[team][id] != 0:
            self.dribbler = None
            return False

        # robot shouldn't fall
        if self.get_robot_posture(team, id)[4] == False:
            self.dribbler = None
            return False

        if self.robot[team][id]['active'] == False:
            self.dribbler = None
            return False

        robot_pos = self.get_robot_posture(team, id)
        if (abs(robot_pos[0]) > (self.constants.FIELD_LENGTH / 2)) or \
            (abs(robot_pos[1]) > (self.constants.FIELD_WIDTH / 2)):
            self.dribbler = None
            return False

        # other robots shuoldn't be near the ball
        ball_x, ball_y, _ = self.get_ball_position()
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                if team == self.dribbler[0] and  id == self.dribbler[1]:
                    continue
                robot_pos = self.get_robot_posture(team, id)
                dist = get_distance(robot_pos[0], robot_pos[1], ball_x, ball_y)
                if self.ball_in_dribble_area(team, id):
                    self.dribbler = None
                    return False

        return self.dribbler != None

    def ball_in_dribble_area(self, team, id):
        ball_x = self.get_ball_position()[0]
        ball_y = self.get_ball_position()[1]
        ball_z = self.get_ball_position()[2]
        robot_pos = self.get_robot_posture(team, id)
        x = robot_pos[0]
        y = robot_pos[1]
        z = robot_pos[2]
        th = robot_pos[3]
        # robot pos and ball pos received

        theta = th
        if (th > self.constants.PI):
            theta -= 2*self.constants.PI
        d_theta = abs(theta - math.atan2(ball_y-y, ball_x-x))
        if (d_theta > self.constants.PI):
            d_theta -= 2*self.constants.PI
        dist = math.sqrt((ball_y - y)*(ball_y - y)+(ball_x - x)*(ball_x - x))
        add = 0.1
        if self.dribbler != None and team == self.dribbler[0] and  id == self.dribbler[1]:
            add = 0.15
        d_range = self.constants.ROBOT_SIZE[id]/2 + self.constants.BALL_RADIUS + add

        d_z = 0.01
        if id == 0 and self.jump_trig[team][id] < 0:
            d_z = 0.03

        if ((dist < d_range) and (abs(d_theta) < self.constants.PI/4) and (abs(z - ball_z) < d_z)):
            return True
        else:
            return False

    def ball_reposition(self, team, id, theta_type):
        robot_pos = self.get_robot_posture(team, id)
        x = robot_pos[0]
        y = robot_pos[1]
        th = robot_pos[3]
        node = self.ball
        ball_x = self.get_ball_position()[0]
        ball_y = self.get_ball_position()[1]
        ball_z = self.get_ball_position()[2]
        dist = get_distance(x, y, ball_x, ball_y)
        d_range = self.constants.ROBOT_SIZE[id]/2 + self.constants.BALL_RADIUS + 0.01
        r = dist if theta_type else d_range
        t_x = x + r*math.cos(th)
        t_y = y + r*math.sin(th)
        ball_new = [t_x, t_y]
        node.resetPhysics()
        f = 1
        translation = [f * ball_new[0], ball_z, -f * ball_new[1]]
        node.getField('translation').setSFVec3f(translation)

    def set_ball_velocity(self, team, id, ball_velocity):
        robot_velocity = self.robot[team][id]['node'].getVelocity()
        robot_speed = np.linalg.norm(robot_velocity[0:3])
        node = self.ball
        orientation = self.robot[team][id]['node'].getOrientation()
        ori = [[],[],[]]
        ori[0] = orientation[0:3]
        ori[1] = orientation[3:6]
        ori[2] = orientation[6:9]
        av_x = 20
        if robot_speed < 0.0001:
            av_x = 0
        elif robot_speed < 0.01:
            av_x = 5
        av_z = 3*round(robot_velocity[4],2)
        ball_rot = np.dot(ori,[av_x,0,av_z]).tolist()
        node.setVelocity(ball_velocity[:3] + ball_rot)

    def dribble_ball(self):
        team, id, f_dribbled, _ = self.dribbler
        robot_pos = self.get_robot_posture(team, id)
        x = robot_pos[0]
        y = robot_pos[1]
        th = robot_pos[3]
        node = self.ball
        ball_x = self.get_ball_position()[0]
        ball_y = self.get_ball_position()[1]
        dist = get_distance(x, y, ball_x, ball_y) 
        speed_m = (abs(self.speeds_buffer[team][6*id])+abs(self.speeds_buffer[team][6*id+1]))/2
        max_velocity = self.constants.MAX_LINEAR_VELOCITY[id]
        robot_velocity = self.robot[team][id]['node'].getVelocity()
        robot_speed = np.linalg.norm(robot_velocity[0:3])

        if robot_speed < 0.5 * max_velocity or self.dribbler[2] == 0:
            self.dribbler[3] = 0
            self.ball_reposition(team, id, False)
            ball_velocity = [robot_velocity[0], robot_velocity[1], robot_velocity[2], 0, 0, 0]
        else:
            if self.dribbler[3] == 0:
                self.ball_reposition(team, id, False)
            else:
                self.ball_reposition(team, id, True)

            time_go,time_back = 3,6
            allow_dist = 0.18 * robot_speed / max_velocity*0.8

            if self.dribbler[3] <= time_go-1:
                ball_speed = robot_speed + (allow_dist / (time_go*0.05))
            else:
                diff = (dist - (self.constants.ROBOT_SIZE[id]/2 + self.constants.BALL_RADIUS))
                ball_speed = robot_speed - (diff / ((time_go+time_back - self.dribbler[3]) * 0.05))
            vx = ball_speed * math.cos(th)
            vy = ball_speed * math.sin(th)
            f = 1
            ball_velocity = [f * vx, 0, f * -vy,0,0,0]
            self.dribbler[3] = (self.dribbler[3] + 1) % (time_go+time_back)

        self.set_ball_velocity(team, id, ball_velocity)

        self.dribbler[2] += 1

    def dribble(self):
        if self.check_dribble():
            self.dribble_ball()

    def change_view(self):
        if self.multi_view:
            pn = self.getFromDef("DEF_AUDVIEW")
            x = self.get_ball_position()[0]
            y = self.get_ball_position()[1]
            z = self.get_ball_position()[2]
            follow = "soccer_ball"
            f = 1
            sc = 1.3
            orientation = [
                    [-1.00,  0.00,  0.00,  0.76],   # normal
            ]
            if follow != pn.getField('follow').getSFString():
                pn.getField('follow').setSFString(follow)
            cur_view = pn.getField('position').getSFVec3f()
            if self.time%15000 < 11000:
                dx = abs(cur_view[0] - f*x)
                dy = abs(cur_view[2] - (5.86 * sc - f*y))
                if dx > 2:
                    pn.getField('orientation').setSFRotation(orientation[0])
                    pn.getField('position').setSFVec3f([f * x, 5.27 * sc, cur_view[2]])
                if dy > 1.5:
                    pn.getField('orientation').setSFRotation(orientation[0])
                    pn.getField('position').setSFVec3f([cur_view[0], 5.27 * sc, 5.86 * sc - f * y])
            elif self.time%15000 == 11000:
                pn.getField('orientation').setSFRotation(orientation[0])
                pn.getField('position').setSFVec3f([f * x, 2.3 * sc, 2.7 * sc - f * y])
            elif self.time%15000 < 15000:
                dx = abs(cur_view[0] - f*x)
                dy = abs(cur_view[2] - (2.7 * sc - f*y))
                if dx > 1.2:
                    pn.getField('orientation').setSFRotation(orientation[0])
                    pn.getField('position').setSFVec3f([f * x, 2.3 * sc, cur_view[2]])
                if dy > 1:
                    pn.getField('orientation').setSFRotation(orientation[0])
                    pn.getField('position').setSFVec3f([cur_view[0], 2.3 * sc, 2.7 * sc - f * y])
            elif self.time%15000 == 0:
                pn.getField('orientation').setSFRotation(orientation[0])
                pn.getField('position').setSFVec3f([f * x, 5.27 * sc, 5.86 * sc - f * y])

    def start_view(self):
        pn = self.getFromDef("DEF_AUDVIEW")
        tran = self.robot[0][-1]['node'].getField('translation').getSFVec3f()
        s = [tran[0], 0.25, tran[2] + 1, 0]
        e = [0, 11, 9, 0.891]

        for t in range(50):
            t = min(max(t - 10, 0), 30)
            t_ori = [0,2,3,3.7,4.1,4.5,4.7,4.8,4.9,5]
            pos = [(e[0] - s[0])*t/30 + s[0],
                   (e[1] - s[1])*t/30 + s[1],
                   (e[2] - s[2])*t/30 + s[2]]
            if t < 9:
                ori = [-1.00,  0.00,  0.00,
                       (e[3] - s[3])*t_ori[t]/6 + s[3]]
            else:
                ori = [-1.00,  0.00,  0.00,
                       (e[3] - s[3])*(5 + (t-11)/19)/6 + s[3]]

            pn.getField('orientation').setSFRotation(ori)
            pn.getField('position').setSFVec3f(pos)

            if self.step(40) == -1:
                break

    def save_results(self, finished, success, results):
        time_info = time.localtime()
        timestamp = '{:04d}-{:02d}-{:02d}T{:02d}_{:02d}_{:02d}'.format(
                            # [<year>-<month>-<day>T<hour>_<minute>_<seconds>]
                            time_info[0], time_info[1], time_info[2], time_info[3], time_info[4], time_info[5]
                            )
        
        if success:
            if self.time < 3000:
                stars = 3
            elif self.time < 4000:
                stars = 2
            else:
                stars = 1
        else:
            stars = 0
        
        if finished:
            results['results']['timestamp'] = timestamp
            results['results']['finished'] = finished
            results['results']['time'] = self.time
            results['results']['success'] = success
            results['results']['stars'] = stars
        else:
            results['results']['timestamp'] = timestamp
            results['results']['finished'] = finished
            results['results']['time'] = self.time
            results['results']['success'] = finished
            results['results']['stars'] = stars
        
        with open('../../results/results.json', 'w') as outfile:
            json.dump(results, outfile)

    def save_training_log(self, training_log, log_success, log_reward, log_head):
        time_info = time.localtime()
        timestamp = '{:04d}-{:02d}-{:02d}T{:02d}_{:02d}_{:02d}'.format(
                            # [<year>-<month>-<day>T<hour>_<minute>_<seconds>]
                            time_info[0], time_info[1], time_info[2], time_info[3], time_info[4], time_info[5]
                            )
        
        training_log['training']['timestamp'] = timestamp
        training_log['training']['training'] = True
        training_log['training']['success_frame'] = log_success.logger_success.frame
        training_log['training']['success_value'] = log_success.logger_success.value
        #training_log['training']['reward_frame'] = log_reward.logger_reward.frame
        #training_log['training']['reward_value'] = log_reward.logger_reward.value
        training_log['training']['dist_frame'] = log_head.logger_dist.frame
        training_log['training']['dist_value_min_dist'] = [i[1] for i in log_head.logger_dist.value] # only minimum
        training_log['training']['dist_value_max_z'] = [i[2] for i in log_head.logger_dist.value]

        with open('../../results/training.json', 'w') as outfile:
            json.dump(training_log, outfile)



    def run(self):
        config_file = open('../../config.json')
        config = json.loads(config_file.read())
        self.game_time = self.constants.DEFAULT_GAME_TIME_MS / self.constants.PERIOD_MS * self.constants.PERIOD_MS

        results_file = open('../../results/results.json')
        results = json.loads(results_file.read())
        results_file.close()

        if config['rule']:
            if config['rule']['game_time']:
                self.game_time = config['rule']['game_time'] * 1000 / self.constants.PERIOD_MS * self.constants.PERIOD_MS
        else:
            print('"rule" section of \'config.json\' seems to be missing: using default options\n')
        print('Rules:\n')
        print('     game duration - ' + str(self.game_time / 1000) + ' seconds\n')

        # gets other options from 'config.json' (if no option is specified, default option is given)
        player_infos = []
        pn = self.getFromDef("DEF_AUDVIEW")
        pn.getField('follow').setSFString("")

        # if not repeat:
        results_file = open('../../results/results.json')
        results = json.loads(results_file.read())
        results_file.close()

        # if repeat:
        training_file = open('../../results/training.json')
        self.training_log = json.loads(training_file.read())
        training_file.close()

        self.log_success = logger.Avg_Success()
        self.log_reward = logger.Avg_Reward()
        self.log_head = logger.Avg_Distance()

        self.multi_view = False
        if config['tool']:
            if config['tool']['multi_view']:
                self.multi_view = config['tool']['multi_view']
            if config['tool']['repeat']:
                repeat = config['tool']['repeat']

        path_prefix = '../../'
        team_name = {}
        self.role_info = {}
        self.role_client = {}
        self.ready = [False] * 2  # TEAM_RED, TEAM_BLUE

        # gets the teams' information from 'config.json'
        for team in self.constants.TEAMS:
            if team == self.constants.TEAM_RED:
                tc = 'team_a'
                tc_op = 'team_b'
            else:
                tc = 'team_b'
                tc_op = 'team_a'
            # my team
            name = ''
            rating = 0  # rating is disabled
            exe = ''
            if config[tc]:
                if config[tc]['name']:
                    name = config[tc]['name']
                if config[tc]['executable']:
                    exe = config[tc]['executable']
            # opponent
            name_op = ''
            rating_op = 0  # rating is disabled
            if config[tc_op]:
                if config[tc_op]['name']:
                    name_op = config[tc_op]['name']
            player_infos.append({
                'name': name,
                'rating': rating,
                'exe': path_prefix + exe,
                'role': team
            })

            if team == self.constants.TEAM_RED:
                print('Team A:\n')
            else:
                print('Team B:\n')
            print('  team name - ' + name + '\n')
            team_name[team] = name
            print('  executable - ' + exe + '\n')

            # create information for aiwc.get_info() in advance
            info = {}
            info['field'] = [self.constants.FIELD_LENGTH, self.constants.FIELD_WIDTH]
            info['goal'] = [self.constants.GOAL_DEPTH, self.constants.GOAL_WIDTH]
            info['penalty_area'] = [self.constants.PENALTY_AREA_DEPTH, self.constants.PENALTY_AREA_WIDTH]
            info['goal_area'] = [self.constants.GOAL_AREA_DEPTH, self.constants.GOAL_AREA_WIDTH]
            info['ball_radius'] = self.constants.BALL_RADIUS
            info['ball_mass'] = self.constants.BALL_MASS
            info['robot_size'] = self.constants.ROBOT_SIZE
            info['robot_height'] = self.constants.ROBOT_HEIGHT
            info['axle_length'] = self.constants.AXLE_LENGTH
            info['robot_body_mass'] = self.constants.ROBOT_BODY_MASS
            info['wheel_radius'] = self.constants.WHEEL_RADIUS
            info['wheel_mass'] = self.constants.WHEEL_MASS
            info['max_linear_velocity'] = self.constants.MAX_LINEAR_VELOCITY
            info['max_torque'] = self.constants.MAX_TORQUE
            info['number_of_robots'] = self.constants.NUMBER_OF_ROBOTS
            info['codewords'] = self.constants.CODEWORDS
            info['game_time'] = self.game_time / 1000
            info['team_info'] = [[['name_a', name], ['rating', rating]], [['name_b', name_op], ['rating', rating_op]]]
            info['key'] = random_string(self.constants.KEY_LENGTH)
            ## should be changed
            info['state_size'] = 8
            info['action_size'] = 3
            self.role_info[team] = info

        self.tcp_server = TcpServer(self.constants.SERVER_IP, self.constants.SERVER_PORT, self.constants.MAX_INSTANCES)
        self.ball = self.getFromDef(self.constants.DEF_BALL)
        self.time = 0
        self.kickoff_time = self.time
        self.score = [0, 0]
        self.ball_ownership = self.constants.TEAM_RED  # red
        self.robot = [[0 for x in range(self.constants.NUMBER_OF_ROBOTS)] for y in range(2)]
        for t in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                node = self.getFromDef(get_robot_name(self, t, id))
                self.robot[t][id] = {}
                self.robot[t][id]['node'] = node
                self.robot[t][id]['active'] = True
                self.robot[t][id]['touch'] = False
                self.robot[t][id]['ball_possession'] = False
        self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
        self.lock_all_robots(True)
        self.robot[self.constants.TEAM_RED][0]['active'] = True
        self.robot[self.constants.TEAM_RED][1]['active'] = True
        self.robot[self.constants.TEAM_RED][2]['active'] = True
        self.robot[self.constants.TEAM_BLUE][0]['active'] = True
        self.robot[self.constants.TEAM_BLUE][1]['active'] = False
        self.robot[self.constants.TEAM_BLUE][2]['active'] = False

        # start participants
        for player_info in player_infos:
            exe = player_info['exe']
            if not os.path.exists(exe):
                print('Participant controller not found: ' + exe)
            else:
                command_line = []
                if exe.endswith('.py'):
                    os.environ['PYTHONPATH'] += os.pathsep + os.path.join(os.getcwd(), 'player_py')
                    if sys.platform == 'win32':
                        command_line.append('python')
                command_line.append(exe)
                command_line.append(self.constants.SERVER_IP)
                command_line.append(str(self.tcp_server.get_port()))
                command_line.append(self.role_info[player_info['role']]['key'])
                print(command_line)
                subprocess.Popen(command_line)
        self.started = False
        print('Waiting for player to be ready...')

        while True:

            sys.stdout.flush()
            self.tcp_server.spin(self)
            if not self.started:
                if all(self.ready):
                    print('Starting match.')
                    self.save_results(False, False, results)
                    self.started = True
                    self.ball_position = self.get_ball_position()
                    self.publish_current_frame(Game.GAME_START)
                    if self.step(self.constants.WAIT_STABLE_MS) == -1:
                        break
                    # self.start_view()
                else:
                    if self.step(self.timeStep) == -1:
                        break
                    else:
                        self.waitReady += self.timeStep
                        if (self.waitReady == self.constants.WAIT_READY_MS):
                            print('Game could not be initiated. Need two players ready.')
                            return
                    self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
                    self.lock_all_robots(True)
                    self.robot[self.constants.TEAM_RED][0]['active'] = True
                    self.robot[self.constants.TEAM_RED][1]['active'] = True
                    self.robot[self.constants.TEAM_RED][2]['active'] = True
                    self.robot[self.constants.TEAM_BLUE][0]['active'] = True
                    self.robot[self.constants.TEAM_BLUE][1]['active'] = False
                    self.robot[self.constants.TEAM_BLUE][2]['active'] = False
                continue

            self.ball_position = self.get_ball_position()
            if self.time >= self.game_time:
                if repeat : 
                    self.log_success.update(False)
                    self.save_training_log(self.training_log, self.log_success, self.log_reward, self.log_head)
                    self.setLabel(99, 'FAIL', 0.43, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.EPISODE_END)
                    self.log_head.update_epi(True)
                    print("Distance Updated")
                    self.reset_reason = Game.EPISODE_END
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    self.setLabel(99, '', 0.4, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.ball_ownership = self.constants.TEAM_RED
                    self.game_state = Game.STATE_KICKOFF
                    self.time = 0
                    self.kickoff_time = self.time
                    self.score = [0, 0]
                    self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
                    self.lock_all_robots(True)
                    self.robot[self.constants.TEAM_RED][0]['active'] = True
                    self.robot[self.constants.TEAM_RED][1]['active'] = True
                    self.robot[self.constants.TEAM_RED][2]['active'] = True
                    self.robot[self.constants.TEAM_BLUE][0]['active'] = True
                    self.robot[self.constants.TEAM_BLUE][1]['active'] = False
                    self.robot[self.constants.TEAM_BLUE][2]['active'] = False
                    if self.step(self.constants.WAIT_STABLE_MS) == -1:
                        break
                    self.publish_current_frame(Game.GAME_START)
                else :  
                    self.save_results(True, False, results)
                    self.setLabel(99, 'FAIL', 0.43, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.GAME_END)
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    break

            self.publish_current_frame()
            self.reset_reason = Game.NONE

            # update ball possession statuses of robots
            self.get_ball_possession()

            # update touch statuses of robots
            touch = self.get_robot_touch_ball()
            for team in self.constants.TEAMS:
                for id in range(self.constants.NUMBER_OF_ROBOTS):
                    self.robot[team][id]['touch'] = touch[team][id]
                    if touch[team][id]:  # if any of the robots has touched the ball at this frame, update touch status
                        self.recent_touch = touch

            # target_x = self.target[0]
            # target_y = -self.target[2]
            # robot_pos = self.get_robot_posture(self.constants.TEAM_RED, 0)
            # x = robot_pos[0]
            # y = robot_pos[1]

            # check if a goal is scored
            # if get_distance(x, y, target_x, target_y) < 0.1:
            ball_x = self.ball_position[0]
            ball_y = self.ball_position[1]
            ball_z = self.ball_position[2]
            
            if ball_x > self.constants.FIELD_LENGTH / 2 and abs(ball_y) < self.constants.GOAL_WIDTH / 2 and abs(ball_z) < 0.5 and not self.first_touch :
            #(self.robot[self.constants.TEAM_RED][1]['touch'] == True and self.get_robot_posture(self.constants.TEAM_RED,1)[2] > 0.1) :
                if repeat : 
                    self.log_success.update(True)
                    
                    self.save_training_log(self.training_log, self.log_success, self.log_reward, self.log_head)
                    '''
                    if self.log_success.get_last_success() > 0.9:
                        self.save_results(True, True, results)
                        self.setLabel(99, 'SUCCESS', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                        self.publish_current_frame(Game.GAME_END)
                        self.stop_robots()
                        if self.step(self.constants.WAIT_END_MS) == -1:
                            break
                        break
                    '''
                    self.setLabel(99, 'SUCCESS', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.EPISODE_END)
                    self.log_head.update_epi(True)
                    print("Distance Plotted")
                    self.reset_reason = Game.EPISODE_END
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    self.setLabel(99, '', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.ball_ownership = self.constants.TEAM_RED
                    self.game_state = Game.STATE_KICKOFF
                    self.time = 0
                    self.kickoff_time = self.time
                    self.score = [0, 0]
                    self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
                    self.lock_all_robots(True)
                    self.robot[self.constants.TEAM_RED][0]['active'] = True
                    self.robot[self.constants.TEAM_RED][1]['active'] = True
                    self.robot[self.constants.TEAM_RED][2]['active'] = True
                    self.robot[self.constants.TEAM_BLUE][0]['active'] = True
                    self.robot[self.constants.TEAM_BLUE][1]['active'] = False
                    self.robot[self.constants.TEAM_BLUE][2]['active'] = False
                    if self.step(self.constants.WAIT_STABLE_MS) == -1:
                        break
                    self.publish_current_frame(Game.GAME_START)
                else : 
                    self.save_results(True, True, results)
                    self.setLabel(99, 'SUCCESS', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.GAME_END)
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    break

            self.ball_spotlight()
            if not self.ball_in_field():
                self.ball_spotlight_stop()

            # self.target_spotlight(self.target)

            # self.change_view()
            self.kick()
            self.dribble()
            self.check_gk_grab_rule()
            
            if self.step(self.timeStep, runTimer=True) == -1:
                break

controller = GameSupervisor()
controller.run()