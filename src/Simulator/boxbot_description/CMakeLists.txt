cmake_minimum_required(VERSION 2.8.3)
project(boxbot_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/description.launch)
