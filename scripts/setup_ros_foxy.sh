# Authorize ros GPG key for apt
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Add ros repo to sources
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ros
sudo apt update
sudo apt install -y ros-foxy-desktop

# Source the ros environment
source /opt/ros/foxy/setup.bash

# Install arg complete
sudo apt install python3-argcomplete
