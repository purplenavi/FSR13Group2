#!/bin/bash
declare -a processes=('roslaunch' 'point2point_nav')
 
for i in "${processes[@]}"
do
    num=$(ps -a | grep -v grep | grep $i | head -n1 | awk '{print $1;}')
    kill $num
done
