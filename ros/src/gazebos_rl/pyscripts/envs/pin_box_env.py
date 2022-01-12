#!/usr/bin/env python
#This project is inspired by 'https://theconstructcore.bitbucket.io/openai_ros/'
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
from __future__ import print_function
import rospy
import numpy as np
import math
import random
from std_msgs import msg
import tf
from gym import spaces
from .onclouds_robot_env import OnCloudsRobotEnv
from gym.envs.registration import register

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan

# Register the pin_box env
register(
  id='PinBox-v0',
  entry_point='envs.pin_box_env:PinBoxEnv')


class PinBoxEnv(OnCloudsRobotEnv):
  def __init__(self):
    # action limits
    self.max_linear_speed = 0.1
    self.max_angular_speed = math.pi / 3
    self.neu_angular_speed = 0.0
    # observation limits
    self.max_x = 10
    self.max_y = 10
    self.max_vx = 2
    self.max_vy = 2
    self.max_cosyaw = 1
    self.max_sinyaw = 1
    self.max_yawdot = math.pi
    # action space
    self.high_action = np.array([self.max_linear_speed, self.neu_angular_speed, self.max_angular_speed])
    self.low_action = np.array([self.max_linear_speed, self.neu_angular_speed, -self.max_angular_speed])
    self.action_space = spaces.Box(low=self.low_action, high=self.high_action)
    # observation space
    self.high_observation = np.array(
      [
        self.max_x,
        self.max_y,
        self.max_vx,
        self.max_vy,
        self.max_cosyaw,
        self.max_sinyaw,
        self.max_yawdot
      ]
    )
    self.low_observation = -self.high_observation
    self.observation_space = spaces.Box(low=self.low_observation, high=self.high_observation)
    # observation
    self.observation = np.zeros(self.observation_space.shape[0])
    self.observation[4] = self.max_cosyaw
    # info, initial position and goal position
    self.init_position = np.zeros(2)
    self.current_position = np.zeros(2)
    self.previous_position = np.zeros(2)
    self.goal_position = np.zeros(2)
    self.info = {}
    # Set model state
    self.set_robot_state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=100)
    self.set_pin_state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=100)
    #set laser scan
    self.laser_scan =rospy.Publisher("/scan", LaserScan, queue_size=100)
    # self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    self._episode_done = False
    # Here we will add any init functions prior to starting the MyRobotEnv
    super(PinBoxEnv, self).__init__()

  def _set_init(self):
    rospy.logdebug("Start initializing robot...")
    self.current_position = self.init_position
    mag = random.uniform(1.5, 2.0)
    ang = random.uniform(-math.pi, math.pi)
    x = mag * math.cos(ang)
    y = mag * math.sin(ang)
    w = random.uniform(-1.0, 1.0)
    robot_state = ModelState()
    robot_state.model_name = "oncloudsbot"
    robot_state.pose.position.x = x
    robot_state.pose.position.y = y
    robot_state.pose.position.z = 0
    robot_state.pose.orientation.x = 0
    robot_state.pose.orientation.y = 0
    robot_state.pose.orientation.z = math.sqrt(1 - w**2)
    robot_state.pose.orientation.w = w
    robot_state.reference_frame = "world"

    self.init_position = np.array([x, y])
    self.previous_position = self.init_position

    # set goal point using pole coordinate
    goal_r = random.uniform(2.0, 3.0)
    goal_theta = random.uniform(-math.pi, math.pi)#([-math.pi + math.pi/16, math.pi + math.pi/16, -math.pi/2 + math.pi/16, math.pi/2 + math.pi/16])
    goal_x = goal_r * math.cos(goal_theta)
    goal_y = goal_r * math.sin(goal_theta)
    self.goal_position = np.array([goal_x, goal_y])
    # reset goal if it too close to bot's original position
    while np.linalg.norm(self.goal_position - self.init_position) <= 0.6:
      rospy.logerr("Goal was set too close to the robot, reset the goal...")
      goal_r = random.uniform(2.0, 3.0)
      goal_theta = random.uniform(-math.pi, math.pi)
      goal_x = goal_r * math.cos(goal_theta)
      goal_y = goal_r * math.sin(goal_theta)
      self.goal_position = np.array([goal_x, goal_y])
    # set pin's model state
    pin_state = ModelState()
    pin_state.model_name = "pin"
    pin_state.pose.position.x = goal_x
    pin_state.pose.position.y = goal_y
    pin_state.pose.position.z = 0
    pin_state.reference_frame = "world"
    # publish model_state to set bot
    rate = rospy.Rate(100)
    for _ in range(10):
      self.set_robot_state_publisher.publish(robot_state)
      self.set_pin_state_publisher.publish(pin_state)
      rate.sleep()

    rospy.logwarn("Robot was initiated as {}".format(self.init_position))
    rospy.logwarn("Goal point was set @ {}".format(self.goal_position))
    # Episode cannot done
    self._episode_done = False

    # Give the system a little time to finish initialization
    rospy.logdebug("Finish initialize robot.")

    return self.init_position, self.goal_position

  def _get_laser(self):
    self.laser_data = np.array(self.laser_scan.ranges[:])
    return self.laser_data

  def _take_action(self, action):
    rospy.logdebug("oncloudsbot Base Twist Cmd>>\nlinear: {}\nangular{}".format(action[0], action[1]))
    cmd_vel = Twist()
    cmd_vel.linear.x = action[0]
    cmd_vel.angular.z = action[1]
    self._check_publishers_connection()
    rate = rospy.Rate(100)
    for _ in range(10):
      self._cmd_vel_pub.publish(cmd_vel)
      rospy.logdebug("cmd_vel: \nlinear: {}\nangular{}".format(cmd_vel.linear.x,
                                                               cmd_vel.angular.z))
      rate.sleep()

  def _get_observation(self):
    """
    Get observations from env
    Return:
      observation: [x, y, v_x, v_y, cos(yaw), sin(yaw), yaw_dot]
    """
    rospy.logdebug("Start Get Observation ==>")
    model_states = self.get_model_states()
    # update previous position
    self.previous_position = self.current_position
    rospy.logdebug("model_states: {}".format(model_states))
    x = model_states.pose[-1].position.x
    y = model_states.pose[-1].position.y
    self.current_position = np.array([x, y])
    v_x = model_states.twist[-1].linear.x
    v_y = model_states.twist[-1].linear.y
    quat = (
      model_states.pose[-1].orientation.x,
      model_states.pose[-1].orientation.y,
      model_states.pose[-1].orientation.z,
      model_states.pose[-1].orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quat)
    cos_yaw = math.cos(euler[2])
    sin_yaw = math.sin(euler[2])
    yaw_dot = model_states.twist[-1].angular.z

    self.observation = np.array([x, y, v_x, v_y, cos_yaw, sin_yaw, yaw_dot])
    rospy.logdebug("Observation ==> {}".format(self.observation))

    return self.observation

  def _post_information(self):
    self.info = {
      "init_position": self.init_position,
      "goal_position": self.goal_position,
      "current_position": self.current_position,
      "previous_position": self.previous_position
    }

    return self.info

  def _is_done(self):
    """
    Return done if self._episode_done == True
    """
    return self._episode_done

  def _compute_reward(self):
    if np.min(self.laser_data) < 0.31:
      reward = -4
      self._episode_done = True
      rospy.logwarn("OnCloudsBot HIT obstacles, task will be reset...")
    elif np.linalg.norm(self.current_position-self.goal_position) < 0.35:
      reward = 2
      self._episode_done = True
      rospy.logwarn("\n!!!\nOnCloudsBot reached destination\n!!!")
    else:
      reward = 0
      self._episode_done = False
      rospy.logwarn("OnCloudsBot is navigating by its own to goal @ {}...".format(self.goal_position))
    rospy.logdebug("Reward is computed. \nreward = {}".format(reward))

    return reward


def _check_publishers_connection(self):
  raise NotImplementedError()

def _cmd_vel_pub(self):
  raise NotImplementedError()
