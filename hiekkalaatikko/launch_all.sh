#!/bin/bash
cd ~/fuerte_workspace/fsr2013/hiekkalaatikko/gui_plannerz
roslaunch gui_plannerz.launch &
cd ~/fuerte_workspace/fsr2013/hiekkalaatikko/point2point_navigation
bin/point2point_navigation &
cd ~/fuerte_workspace/fsr2013/hiekkalaatikko/gui_plannerz/src
#./gui_plannerz.py &
./task_plannerz.py &
