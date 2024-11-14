#!/bin/bash
catkin_make

./ai_py_env/bin/activate
. ./devel/setup.bash

gnome-terminal --geometry 80x18+0+0 -- bash -c ". ./ai_py_env/bin/activate;roscore; exec bash"

gnome-terminal --geometry 80x18+0+0 -- bash -c ". ./ai_py_env/bin/activate;rosrun jetbot_mini P_Robot.py; exec bash"
sleep 1

#gnome-terminal --geometry 80x18-0+0 -- bash -c ". ./ai_py_env/bin/activate;rosrun secondary_robot S_Robot.py; exec bash"
#sleep 1

gnome-terminal --geometry 150x18-0-0 -- bash -c ". ./ai_py_env/bin/activate;rosrun jetbot_mini Simulator.py; exec bash"
sleep 3

gnome-terminal --geometry 150x18-0-0 -- bash -c ". ./ai_py_env/bin/activate;rosrun jetbot_mini CollisionDetector.py; exec bash"
sleep 2

gnome-terminal --geometry 150x18-0-0 -- bash -c ". ./ai_py_env/bin/activate;rosrun jetbot_mini api.py; exec bash"
sleep 7




rqt_graph > /dev/null 2>&1 &

