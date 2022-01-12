#!/usr/bin/env python
#This project is inspired by 'https://theconstructcore.bitbucket.io/openai_ros/'
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
from __future__ import absolute_import, division, print_function
import gym
from gym import wrappers
import rospy
import math
import numpy as np
import random
import os
import datetime
import matplotlib.pyplot as plt
import pickle
import envs.pin_box_env

class bcolors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'
  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'

def obs_to_state(obs, info, laser_data):
  # compute states
  r = obs[:2]
  p = info["goal_position"] - obs[:2]
  r_norm = np.linalg.norm(r)
  p_norm = np.linalg.norm(p)
  o_norm = np.min(laser_data)
  gama   = 0.00436940183863 * list(laser_data).index(np.min(list(laser_data)))
  alpha  = np.arctan2(obs[1], obs[0])
  alpha_dot = np.arctan2(obs[3], obs[2])
  # comput phi: angle from map's x_axis to p
  x_axis = np.array([1, 0])
  y_axis = np.array([0, 1])
  cos_phi = np.dot(p, x_axis) / (np.linalg.norm(p)*np.linalg.norm(x_axis))
  sin_phi = np.dot(p, y_axis) / (np.linalg.norm(p)*np.linalg.norm(y_axis))
  phi = np.arctan2(sin_phi, cos_phi)
  beta = phi - np.arctan2(obs[-2], obs[-3])
  if beta > math.pi:
    beta -= 2*math.pi
  elif beta < -math.pi:
    beta += 2*math.pi
  beta_dot = obs[-1]
  state = np.array([r_norm, p_norm, o_norm, alpha, alpha_dot, beta, beta_dot, gama]).astype(np.float32)

  return state

def discretize_state(state, boxes):
  # match state into box
  index = []
  for i_s, st in enumerate(state):
    for i_b, box in enumerate(boxes[i_s]):
      if st >= box[0] and st <= box[1]:
        index.append(i_b)
        break
  assert len(index) == 8

  return tuple(index)


if __name__ == "__main__":
  # init node
  rospy.init_node("pin_box_qlearn", anonymous=True, log_level=rospy.WARN)
  env_name = 'PinBox-v0'
  env = gym.make(env_name)
  rospy.loginfo("PinBox environment set")
  # parameters
  num_actions = 3
  Alpha = 0.4 # learning rate
  Gamma = 0.8 # reward discount
  num_episodes = 50000
  num_steps = 400
  # define state boxes
  box_1 = np.array([[0, 1.00], [1.00, 2.0], [2.0, 4.0], [4.0, np.inf]])
  box_2 = np.array([[0, 0.35], [0.35, 1.0], [1.0, 3.0], [3.0, np.inf]])
  box_3 = np.array([[-np.inf, 0],[0, 0.5], [0.5, 1.0], [1.0, np.inf]])
  box_4 = np.array([
    [-math.pi, -math.pi/2],
    [-math.pi/2, 0],
    [0, math.pi/2],
    [math.pi/2, math.pi]
  ])
  box_5 = np.array([
    [-np.inf, -math.pi],
    [-math.pi, -math.pi/6],
    [-math.pi/6, -math.pi/12],
    [-math.pi/12, 0],
    [0, math.pi/12],
    [math.pi/12, math.pi/6],
    [math.pi/6, math.pi],
    [math.pi, np.inf]
  ])
  box_8 = np.array([
    [-np.inf, 0],
    [0, math.pi/4],
    [math.pi/4, 5*math.pi/12],
    [5*math.pi/12, math.pi/2],
    [math.pi/2, 7*math.pi/12],
    [7*math.pi/12, 3*math.pi/4],
    [3*math.pi/4, math.pi],
    [math.pi, np.inf]
  ])
  box_6 = box_4
  box_7 = box_5
  boxes = [box_1, box_2, box_3, box_4, box_5, box_6, box_7, box_8]

  # initiate Q-table
  q_axes = []
  for b in boxes:
    q_axes.append(b.shape[0])
  q_axes.append(num_actions)
  # load q tables
  qtable_dir = "/root/ros-melodic-nvidia-docker/ros/src/gazebos_rl/pyscripts/Qtable/20220107_training"
  with open(os.path.join(qtable_dir, "Qtable_50000-50000.txt"), "rb") as pkl:
    Q = pickle.load(pkl)
  # Reset env and get first observation
  obs, info, laser_data = env.reset()
  print(
    "\nRobot init position: {}".format(obs[:2]),
    "\nGoal position: {}".format(info["goal_position"]),
    bcolors.ENDC
  )
  state = obs_to_state(obs, info, laser_data)
  state_id = discretize_state(state, boxes)
  p_0 = state[1]
  o_0 = state[2]
  episode_reward = 0
  done = False
  for step in range(num_steps):
    action_id = np.argmax(Q[state_id]) # exploit
    if action_id == 0:
      action = np.array([env.action_space.high[0], env.action_space.low[2]])
    elif action_id == 1:
      action = np.array([env.action_space.high[0], env.action_space.low[1]])
    else:
      action = np.array([env.action_space.high[0], env.action_space.high[2]])
    # Get new state and reward
    obs, reward, done, info, laser_data = env.step(action)
    next_state = obs_to_state(obs, info, laser_data)
    next_state_id = discretize_state(state, boxes)
    reward = reward + (state[1]-next_state[1])/p_0
    # Update Q table
    Q[state_id][action_id] = Q[state_id][action_id] + Alpha*(reward + Gamma*np.max(Q[next_state_id]) - Q[state_id][action_id])
    episode_reward += reward
    # update state
    state = next_state
    state_id = next_state_id
    print(
      bcolors.OKGREEN, "\nStep: {}".format(step), bcolors.ENDC,
      "\nRobot current position: {}".format(obs[:2]),
      "\nGoal: {}".format(info["goal_position"]),
      "\nAction: {}".format(action),
      "\nreward: {}".format(reward)
    )

    if done:
      print(bcolors.WARNING, "\n!!!\nGOAL\n!!!\n", bcolors.ENDC)
      break
