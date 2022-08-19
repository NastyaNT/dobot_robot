#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("dobot_arm");

    // Получаем имя терминальной ссылки
    std::string end_effector_link = arm.getEndEffectorLink();

    // Устанавливаем опорную систему координат, используемую целевой позицией
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    // Когда планирование движения не удается, разрешено перепланирование
    arm.allowReplanning(true);

    // Устанавливаем допустимую ошибку положения (единицы: метр) и отношения (единицы: радианы)
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    // Устанавливаем максимально допустимую скорость и ускорение
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);


    // Устанавливаем целевую позицию терминала робота
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.70692;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.70729;

    target_pose.position.x = 0.2593;
    target_pose.position.y = 0.0636;
    target_pose.position.z = 0.1787;

    // Устанавливаем текущее состояние манипулятора робота как начальное состояние движения
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose);

    // Выполняем планирование движения и рассчитываем траекторию движения робота, движущегося к цели. В настоящее время он только рассчитывает траекторию и не контролирует движение робота
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

    // Дать роботу-манипулятору начать движение по запланированной траектории.
    if(success)
      arm.execute(plan);
    sleep(1);

    ros::shutdown(); 

    return 0;
}

