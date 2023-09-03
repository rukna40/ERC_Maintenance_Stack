FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get upgrade -y && \
	apt-get install -y lsb-core g++ git

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

RUN apt install ros-noetic-moveit -y

RUN apt install ros-noetic-ros-controllers* -y

RUN apt install python3
RUN apt-get -y install python3-pip
RUN pip install pandas
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /catkin_ws

COPY src ./src

RUN apt-get update \
  && rosdep update \
  && rosdep install --from-paths src -iy \
  && rm -rf /var/lib/apt/lists/* 

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'

# Automatically source the workspace when starting a bash session

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
