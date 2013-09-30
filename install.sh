#!/bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-fuerte-desktop-full
echo "source /opt/ros/fuerte/setup.bash" >> ~/.bashrc
. ~/.bashrc
sudo apt-get install python-rosinstall
rosws init ~/fuerte_workspace /opt/ros/fuerte
source ~/fuerte_workspace/setup.bash
rosws set fsr2013 --git https://github.com/purplenavi/FSR13Group2.git
rosws update fsr2013
rosws set amor-ros-pkg https://code.google.com/p/amor-ros-pkg/ --hg -v fuerte
rosws update amor-ros-pkg
source ~/fuerte_workspace/setup.bash
rosmake ROSARIA
