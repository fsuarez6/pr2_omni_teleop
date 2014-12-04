#!/usr/bin/env python
import rospy, os
import numpy as np
# Messages
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Wrench
from omni_msgs.msg import OmniButtonEvent
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndpointState
from pr2_controllers_msgs.msg import Pr2GripperCommand
from robot_mechanism_controllers.msg import JTCartesianControllerState
# Quaternions tools
import PyKDL
# Utils
from baxter_teleop.utils import read_parameter
from math import sqrt, pi


GREY_BUTTON = 0
WHITE_BUTTON = 1


class CartCoupling(object):
  def __init__(self):
    limb = rospy.get_param('~limb', 'r')
    if limb not in ['right', 'left']:
      rospy.logerr('Unknown limb name [%s]' % limb)
      return
    # Set-up grippers interface
    self.roll_angle = 0
    self.q0 = PyKDL.Rotation.Identity()
    # Set-up publishers/subscribers
    self.ik_cmd_pub = rospy.Publisher('/%s_cart/command_pose' % limb[0], PoseStamped)
    self.slave_state_pub = rospy.Publisher('/pr2/state', EndpointState)
    self.gripper_pub = rospy.Publisher('/%s_gripper_controller/command' % limb[0], Pr2GripperCommand)
    rospy.Subscriber('/pr2/ik_command', PoseStamped, self.pose_cb)
    rospy.Subscriber('/%s_cart/state' % limb[0], JTCartesianControllerState, self.cart_state_cb)
    self.prev_buttons = [0] * 2
    self.buttons = [False] * 2
    self.buttons[WHITE_BUTTON] = True
    rospy.Subscriber('/omni/button', OmniButtonEvent, self.buttons_cb)
    rospy.Subscriber('/omni/joint_states', JointState, self.joint_states_cb)
    rospy.spin()
  
  def buttons_cb(self, msg):
    button_states = [msg.grey_button, msg.white_button]
    # Check that any button was pressed / released
    for i, previous in enumerate(self.prev_buttons):
      if (previous != button_states[i]) and button_states[i] == 1:
        self.buttons[i] = not self.buttons[i]
    # Open or close the gripper
    gripper_cmd = Pr2GripperCommand()
    gripper_cmd.max_effort = -1       #  If 'max_effort' is negative, it is ignored.
    if self.buttons[GREY_BUTTON]:
      # Close
      gripper_cmd.position = 0.0
    else:
      # Open
      gripper_cmd.position = 0.1
    self.gripper_pub.publish(gripper_cmd)
  
  def joint_states_cb(self, msg):
    idx = msg.name.index('roll')
    self.roll_angle = msg.position[idx]

  def pose_cb(self, msg):
    cmd_msg = msg
    q_roll = PyKDL.Rotation.RotX(self.roll_angle)
    q = (self.q0 * q_roll).GetQuaternion()
    cmd_msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    if not self.buttons[WHITE_BUTTON]:
      try:
        self.ik_cmd_pub.publish(cmd_msg)
      except:
        pass
  
  def cart_state_cb(self, msg):
    state_msg = EndpointState()
    state_msg.header.stamp = rospy.Time.now()
    state_msg.pose = msg.x.pose
    try:
      self.slave_state_pub.publish(state_msg)
    except rospy.exceptions.ROSException:
      pass


# Main
if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  cc = CartCoupling()
  
