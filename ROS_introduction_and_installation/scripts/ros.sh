#!/bin/bash

#Setup your computer to accept software from packages.ros.org.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# if you haven't already installed curl
sudo apt install curl  

#Set up your keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#up-to-date
sudo apt update

PS3="Please select an option (1, 2, or 3): "
echo "1-Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages"
echo "2-Desktop Install: Everything in ROS-Base plus tools like rqt and rviz"
echo "3-ROS-Base: (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools."
options=("Option 1" "Option 2" "Option 3")
select opt in "${options[@]}"; do
    case $opt in
        "Option 1")
            echo "You chose desktop-full"
            sudo apt install ros-noetic-desktop-full
            break
            ;;
        "Option 2")
            echo "You chose desktop"
            sudo apt install ros-noetic-desktop
            break
            ;;
        "Option 3")
            echo "You chose base"
            sudo apt install ros-noetic-ros-base
            break
            ;;
        *)
            echo "Invalid option. Please choose 1, 2, or 3."
            ;;
    esac
done

#added to your bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/noetic/setup.bash

#tool and dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

#run some core components in ROS
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
