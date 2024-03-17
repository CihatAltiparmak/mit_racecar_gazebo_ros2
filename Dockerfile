FROM ros:humble

RUN apt update && apt install -y ros-humble-gazebo-ros ros-humble-xacro

RUN mkdir /opt/mit_racecar_simulator_ws/src -p
COPY . /opt/mit_racecar_simulator_ws/src/mit_racecar_gazebo_ros2
