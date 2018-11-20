sudo apt update
sudo apt upgrade
sudo apt dist-upgrade

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt install ros-melodic-desktop
apt search ros-melodic
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install git
sudo apt install python
git clone https://github.com/MarquetteRMC/Software.git

sudo apt install python-pip
sudo pip install ino
sudo apt install libncurses5-dev libncursesw5-dev
