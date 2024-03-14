FROM ros:humble

RUN mkdir /opt/mit_racecar_simulator_ws/src -p
COPY . /opt/mit_racecar_simulator_ws/src/mit_racecar_gazebo_ros2