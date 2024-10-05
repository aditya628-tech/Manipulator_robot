# This Project is an implementation of the Udemy course Robotics and ROS 2 - Learn by Doing! Manipulators

# Prerequisites:
1. Ubuntu 22.xx (Dual boot or VMWare)
2. Python with pip (if working with python code)
3. ROS2 installed - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
4. VS Code (Or any text editor you are comfortable with)
5. Install the following:
   ```
   sudo apt-get install terminator
   sudo apt-get install ros-humble-joint-state-publisher-gui
   sudo apt-get install xacro
   sudo apt-get install ros-humble-gazebo-ros
   sudo apt-get install ros-humble-ros2-control
   sudo apt-get install ros-humble-ros2-controllers
   sudo apt-get install ros-humble-gazebo-ros2-control
   sudo apt-get install ros-humble-moveit
   sudo apt install python3-colcon-common-extensions
   sudo apt-get install libserial-dev
   sudo apt install ros-humble-rmw-cyclonedds-cpp
   pip install pyserial
   pip install flask
   pip install flask-ask-sdk
   pip install ask-sdk
   ```
6. Next setup colcon tab completion:
   ```
   echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
   ```
# Building and Running the code
## Build command
1. Run the following command in the root of the git repository:
   ```
   colcon build
   ```
2. Source the setup file into the terminal:
   ```
   . install/setup.bash

## Run RViz to check if everything is built and running:
   ```
   ros2 launch urdf_tutorial display.launch.py model:=/home/{{User_name}}/Manipulator_robot/src/arduinobot_description/urdf/arduinobot.urdf.xacro
   ```

# Commands to run rviz
1. Publish the URDF data in ROS:
   ```
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/{{User_name}}/Manipulator_robot/src/arduinobot_description/urdf/arduinobot.urdf.xacro)"
   ```
2. Run the joint state publisher gui:
   ```
   ros2 run joint_state_publisher_gui joint_state_publisher_gui
   ```
3. Launch RViz2:
   ```
   ros2 run rviz2 rviz2
   ```
   #### Note: Always source the latest setup.bash into the terminal before executing any command

# Launch RViz from launch file
```
ros2 launch arduinobot_description display.launch.py 
```
# Launch Gazebo simulation
```
ros2 launch arduinobot_description gazebo.launch.py
```
# Launch controller launch file
```
ros2 launch arduinobot_controller controller.launch.py
```

