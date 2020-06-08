FROM ros:kinetic-robot

WORKDIR /root/projects/robotmaster

#option: change ROS apt source
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

#install ROS dependency
RUN apt-get update && apt-get install -y \
    python-pip libmagickwand-dev ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers \
    && rm -rf /var/lib/apt/lists/

#install ROS packages
RUN cd /root && mkdir catkin_ws && cd catkin_ws && mkdir src && cd src \
    && git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \ 
    && git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && cd /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws; catkin_make'


#install robotmaster requirements
RUN pip install "django<2" "djangorestframework<3.10" apscheduler requests Wand influxdb

#build the .bashrc for bash file
RUN /bin/bash -c "echo 'source /opt/ros/kinetic/setup.bash' >> /root/.bashrc && \
                  echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc && \
                  echo 'export TURTLEBOT3_MODEL=waffle_pi' >> /root/.bashrc && source /root/.bashrc"

#add this and below command will run without cache
ARG CACHEBAUST=1

#clone project
RUN cd /root/catkin_ws/src \
    && git clone https://github.com/wdxpz/turtlebot_master_scripts.git multirobot_nv

RUN cd /root/projects \
    && git clone https://github.com/wdxpz/robotmaster.git robotmaster

#install environments and excute command in launch.sh for auto exec the CMD
ADD launch.sh /
RUN chmod +x /launch.sh

CMD ["/launch.sh"]
