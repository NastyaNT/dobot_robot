#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dobot_forward_kinematics");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("dobot_arm");
    
    // Устанавливаем допустимое значение погрешности движения руки робота
    arm.setGoalJointTolerance(0.001);

    // Устанавливаем максимально допустимую скорость и ускорение
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    double targetPose[4] = {1.0, 1.0, 1.57, 1.0};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];

    arm.setJointValueTarget(joint_group_positions);
    
    
    // Выполняем планирование движения и рассчитываем траекторию движения робота, движущегося к цели. В настоящее время он только рассчитывает траекторию и не контролирует движение робота
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);
   
    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED"); 
   
    // Даем роботу-манипулятору начать движение по запланированной траектории.
    arm.execute(plan);
    arm.move();
    sleep(1);


    ros::shutdown(); 

    return 0;
}

