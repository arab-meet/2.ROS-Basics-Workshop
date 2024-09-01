# ROS Installation Script

Install and configure `ROS Noetic` with a `Bash file` Follow these instructions

This Bash script automates the installation process for ROS (Robot Operating System) on your Ubuntu machine. It is specifically written for ROS `noetic`.

## Dependencies

- Ubuntu 20.04

## Install

1. download this bash file
   &rarr; [ **ros.sh**](ros.sh)

2. Open a terminal window and grant executable permissions to the script:

   ```bash
   chmod +x ros.sh
   ```

3. Run the script

   ```bash
   ./ros.sh
   ```

4. Follow the on-screen instructions.
5. Congratulations! ROS is now installed on your machine.

**Note:** The script assumes a standard `ROS Desktop Full` installation. If you have specific requirements, you may need to customize the script accordingly.

# install it manually by the following steps:

- Setup your computer to accept software
  from packages.ros.org.

      ``` bash
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >
      /etc/apt/sources.list.d/ros-latest.list'
      ```

      ``` bash
      sudo apt install curl
      ```

- Check the system is up-to-date

  ```bash
  sudo apt update
  ```

  ```bash
  sudo apt install ros-noetic-desktop-full
  ```

- Then added to your bash

  ```bash
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  ```

  ```bash
  source ~/.bashrc
  ```

  ```bash
  source /opt/ros/noetic/setup.bash
  ```

- Install tool and dependencies

  ```bash
  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  ```

- Run some core components in ROS

  ```bash
  sudo apt install python3-rosdep
  ```

  ```bash
  sudo rosdep init
  ```

  ```bash
  rosdep update
  ```

- To make sure that you have ROS run the following command

  ```bash
  printenv | grep ROS
  ```

- And the `result` should be like this:

  ```bash
  ROS_ROOT=/opt/ros/noetic/share/ros
  ROS_PACKAGE_PATH=/opt/ros/noetic/share
  ROS_MASTER_URI=http://localhost:11311
  ROS_VERSION=1
  ROSLISP_PACKAGE_DIRECTORIES=
  ROS_DISTRO=noetic
  ROS_ETC_DIR=/opt/ros/noetic/etc/ros
  ```

## [↩Back to main](../ROS_intro.md)
