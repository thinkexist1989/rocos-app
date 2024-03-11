#!/bin/bash
echo "*************************************"
echo "****ROS2 Example of Installation****"
echo "*****platform: Ubuntu 20.04 LTS******"
echo "********ROS2 Version:Foxy**********"
echo "*************************************"
# Set locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc



# 在一个终端中，获取安装文件，然后运行 ​​C++ talker：
# ros2 run demo_nodes_cpp talker
# 在另一个终端中，运行 ​​C++ listener：
#  ros2 run demo_nodes_py listener
# 应该看到talker“这是Publishing消息”和“这些消息”这listener句话。这验证了 C++ 和 Python API 是否正常工作