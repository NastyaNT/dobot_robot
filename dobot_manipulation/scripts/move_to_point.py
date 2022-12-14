#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi 
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import radians


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "dobot_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print ("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print ("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  #To convert rpy degrees input to radians
  def rad(self, rpy):
    rpyrad = [radians(rpy[0]),radians(rpy[1]),radians(rpy[2])]
    return rpyrad
    
  def moveto_xyzrpy(self, xyzrpy):
    self.group.set_pose_target(xyzrpy,end_effector_link = self.eef_link)
    plan = self.group.go(wait=True)
    self.group.stop()
    self.group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose

def main():
  try:

    armDriver = MoveGroupPythonIntefaceTutorial()
    
    # Fixed Frame base_link
    xyz = [0.126, -0.000, 0.324]
    rpy = armDriver.rad([90.000, -0.505, 179.996])     
    xyzrpy = xyz+rpy
    armDriver.moveto_xyzrpy(xyzrpy)
    
    xyz = [0.125, 0.000, 0.255]
    rpy = armDriver.rad([90.000, -0.503, -179.994])     
    xyzrpy = xyz+rpy
    armDriver.moveto_xyzrpy(xyzrpy)
    
    xyz = [0.124, 0.000, 0.152]
    rpy = armDriver.rad([90.000, -0.498, -179.998])    
    xyzrpy = xyz+rpy
    armDriver.moveto_xyzrpy(xyzrpy)
    
    xyz = [0.124, 0.000, 0.107]
    rpy = armDriver.rad([90.000, -0.490, -179.997])    
    xyzrpy = xyz+rpy
    armDriver.moveto_xyzrpy(xyzrpy)
        
             
    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

