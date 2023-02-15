# Install a dependency for the install script

sudo apt update
sudo apt -y dist-upgrade

# Install git, vim, pip
sudo apt install -y git vim python-pip gcc-multilib libboost-python1.58.0

sudo ln -s libboost_python-py27.so.1.58.0 /usr/lib/x86_64-linux-gnu/libboost_python.so

# Install pexpect for use with install script
sudo pip install pexpect

