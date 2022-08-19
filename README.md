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
На стартовой странице нажимаем кнопку Create New MoveIt Configuration Package. <br/>
Выбираем путь до модели робота (формат .xacro или .urdf), нажимаем кнопку Load Files. <br/>
Переходим на вкладку Self-Collisions. Генерируем матрицу столкновений по кнопке Generate Collision Matrix <br/>
Вкладка Planning Groups. Для добавления группы планирования нажимаем кнопку Add Group. Добавляем группу robot_arm:
- В поле Group Name пишем название группы robot_arm
- В Kinematic Solver выбираем kdl kinematics plugin/KDLKinematicsPlugin
Нажимем Add Joints. С левого поля в правое переносим необходимые сочленения. <br/>
Двойной щелчок по Links. Добавляем звенья. <br/>
Добавляем группу robot_hand:
- В поле Group Name пишем название группы robot_arm
- В Kinematic Solver оставляем None
Добавляем сочленения и звенья. <br/>
На вкладке End Effectors нажимаем кнопку Add End Effector.
- В поле End Effector Name пишем имя конечного эффектора hand
- В поле End Effector Group выбираем robot_hand
- В поле Parent Link выбираем link5
Переходим на вкладку Controllers. Нажимаем Auto Add Follow Joints Trajectory Controllers For Each Planning Group. Меняем Controller Type на position_controllers/JointTrajectoryController. <br/>
Переходим на вкладку Simulation. Нажимаем на кнопку Generate URDF. Копируем по кнопке Copy to Clipboard. Создаем файл в рабочей директории и вставляем код. Этот файл с описанием робота будем загружать при запуске симуляции. <br/>
На вкладке Author Information  заполняем поля. <br/>
На вкладке Configuration Files выбираем куда сохранить файлы. Нажимаем кнопку Generate Package. Выскочит ошибка, что не заданы виртуальные суставы. Просто нажимаем OK.
## Инициализация
- source /opt/ros/noetic/setup.bash
- source ~/catkin_ws/devel/setup.bash
- source ~/ws_moveit/devel/setup.bash
- catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
## Для движения модели в gazebo
1. в файл ros_controllers.launch добавляем строку: <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen"/>
2. в файл gazebo.launch добавляем: command="xacro  '$(find dobot_description)/urdf/dobot_with_probe_from.xacro'"
