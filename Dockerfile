FROM ros:humble

# RUN apt update && apt install -y ros-humble-gazebo-ros ros-humble-xacro

RUN mkdir /opt/mit_racecar_simulator_ws/src -p
COPY . /opt/mit_racecar_simulator_ws/src/mit_racecar_gazebo_ros2

RUN . /opt/ros/humble/setup.sh && \
    cd /opt/mit_racecar_simulator_ws/ && \
    apt update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/humble/setup.sh && \
    cd /opt/mit_racecar_simulator_ws/ && \
    colcon build
