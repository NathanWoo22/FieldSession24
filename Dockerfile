FROM osrf/ros:humble-desktop

RUN apt-get update
RUN apt-get install ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap
RUN apt-get install -y python3-pip 
RUN apt-get install -y build-essential 

RUN pip3 install --upgrade pip
RUN pip3 install open3d
RUN pip3 install numpy


CMD bash setup.bash