## Сведения о системе
- Ubuntu 20
- ROS noetic
- Gazebo 11
- Moveit 1
## Работа с github через консоль https://htmlacademy.ru/blog/best/git-console
### Скачиваем к себе репозиторий, делаем изменения у себя, добавляемя изменнеия в репозитоий
1. git clone https://github.com/NastyaNT/dobot_robot.git
2. добавляем изменения в файлы
3. git add .
4. git commit -m "*комментарий*"
5. git pull
6. git push
    1. логин: NastyaNT
    2. пароль: [Как создать пароль смотри здесь](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) 
### Добавляем изменнеия из репозитория
1. git fetch
2. git merge origin
## Создание пакета для работы с Moveit
Запуск помощника MoveIt: roslaunch moveit_setup_assistant setup_assistant.launch <br/>
На стартовой странице нажимаем кнопку Create New MoveIt Configuration Package.
## Инициализация
- source /opt/ros/noetic/setup.bash
- source ~/catkin_ws/devel/setup.bash
- source ~/ws_moveit/devel/setup.bash
- catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
## Для движения модели в gazebo
1. в файл ros_controllers.launch добавляем строку: <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen"/>
2. в файл gazebo.launch добавляем: command="xacro  '$(find dobot_description)/urdf/dobot_with_probe_from.xacro'"
