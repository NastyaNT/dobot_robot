#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveDobot():
    def __init__(self):
        rospy.init_node('move_dobot')
                         
        # Список суставов, входящих в группу
        arm_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        # Устанавливаем целевую конфигурацию
        arm_goal = [0, 1.0, 1.57, 0]
        
        # Подключаемся к серверу действий
        rospy.loginfo('Ожидание контроллера ...')
    
        arm_client = actionlib.SimpleActionClient('dobot_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       
        arm_client.wait_for_server()
        
        rospy.loginfo('...подключено.')    
        
        # Создание траектории
        trajectory = JointTrajectory()
        trajectory.joint_names = arm_joints
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = arm_goal
        trajectory.points[0].velocities = [0.0 for i in arm_joints]
        trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        trajectory.points[0].time_from_start = rospy.Duration.from_sec(3.0)
            
        # Отправка траектории на сервер
        rospy.loginfo('Перемещение в целевое положение...')
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration.from_sec(0.0)
    
        # Отправка цели
        arm_client.send_goal(goal)
        
        # 5 секундное ожидание окончания движения
        arm_client.wait_for_result(rospy.Duration.from_sec(5.0))
        
        rospy.loginfo('...готово.')
        
if __name__ == '__main__':
    try:
        MoveDobot()
    except rospy.ROSInterruptException:
        pass
