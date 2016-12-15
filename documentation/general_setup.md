# Using the Nao and the UT Austin Villa Codebase

<a id="introduction"></a>

### Introduction

This tutorial provides instructions for installing the UT Austin Villa SPL codebase on your personal machine, factory resetting and deploying code to a robot, and general notes for robot usage and care.

<a id="personal"></a>
### Instructions for Installing the Codebase on Your Personal Linux Machine
Before you begin, consider the following checklist to make sure your machine is prepared for the installation.

#### Pre-install Checklist

1. [Ubuntu 14.04 64-bit](http://releases.ubuntu.com/trusty/ubuntu-14.04.3-desktop-amd64.iso). The code may work on other configurations with some adjustments to the setup procedure, but these aren't officially supported. The setup process will configure your machine for building 32-bit software, which can cause conflicts with other applications in Ubuntu. It is **strongly suggested** that you use a fresh Ubuntu install dedicated to RoboCup. This may be easiest to accomplish using a [virtual machine](https://www.virtualbox.org/wiki/Downloads), however a native install will run faster and more smooothly with your network card(s) and/or GPU.
2. Root (sudo) access to your installation machine.
3. A GPU that supports OpenGL 2.0 or higher.
4. An active high-speed internet connection for downloading software. Most software is downloaded from locations on the CS network, so performing the installation on the UT campus is preferable.
5. You will need to install Git to retrieve code from the repository: `sudo apt-get install git`.

#### Installation Procedure

1. Create and enter the install folder in your home directory: 

        mkdir -p ~/nao/trunk && cd ~/nao/trunk
2. Retrieve the codebase from github:

        git clone git@github.com:larg/spl.git .
        
3. Run the codebase install script: 

        ~/nao/trunk/install/install
4. The previous step will add environment variable definitions to your `.bashrc` script. To load them, close the current terminal window and open a new one. Alternatively, you can run `source ~/.bashrc`.
6. Follow the instructions for [compiling the code](#compiling) and [setting up the robot](#initial_setup). You may wish to compile everything with `$NAO_HOME/build/compile everything` to ensure that the codebase install has succeeded.


<a id="compiling"></a>
### Compiling the Code

Once the codebase is installed, compiling the code is fairly straight-forward. The simplest approach to compiling is to navigate to `$NAO_HOME/build` and run `./compile everything`. Similarly, you can copy compiled code to the robot with `./copy_robot everything <robot ip address>`. You can even combine these steps into one with `./cpcompile everything <robot ip address>`.

The compiling and copying scripts accept a number of parameters to speed up builds or improve the user experience.

#### Compile

* Use the `--debug` flag to build in debug mode.
* Use the `--fast` flag to avoid reconfiguring. Do not use this flag when adding new source files!
* Use the `--sound` flag to generate a noise when compiling has finished.
* Use the `--clean` flag to remove the build directories of the specified interfaces.

#### Copy

* Use the `--debug` flag to copy debug binaries.
* Use the `--verify` flag to verify copied files using an md5 checksum (no copy).
* Use the `--copy-verify` flag to verify copied files using an md5 checksum (with copy).
* Use the `--sound` flag to generate a noise when copying has finished.

<a id="initial_setup"></a>
### Initial Robot Setup

#### Flashing

When you get a robot from repairs or from the factory, the first thing you'll want to do is flash it. We currently use Naoqi 2.1.4.13, so any robot running another version of Naoqi should be re-flashed.

1. Download the flashing software from the [Aldebaran website](https://community.aldebaran.com/en/resources/software/language/en-gb) or from [here](http://cs.utexas.edu/~AustinVilla/software/flasher-2.1.0.19-linux64.tar.gz).
2. Download the nao system image from the [Aldebaran website](https://community.aldebaran.com/en/resources/software/language/en-gb) or from  [here](http://cs.utexas.edu/~AustinVilla/software/opennao-atom-system-image-2.1.4.13_2015-08-27.opn).
3. Extract the flasher: `tar zxvf flasher-*`
4. Insert a USB stick that can be safely erased and run the extracted `flasher` script with `sudo`.
5. Select the system image you downloaded, check "Factory reset", and click "Write".
6. Wait for the system image to be written to your USB stick, then insert the stick into your nao in the back of the head.
7. Hold the Nao's chest button until it starts to flash blue. Wait up to 30 minutes for the flashing process to complete.

#### Installing the codebase

After the robot has been flashed, you will need to install the codebase onto it. 

1. Enter the `$NAO_HOME/install` directory and ensure that you have the file `vim.tar.gz` in this directory. Normally this is downloaded during your initial setup process, but you can find the file [here](http://cs.utexas.edu/~AustinVilla/software/vim.tar.gz) as well.
2. Shut down your robot, connect it to your laptop with an ethernet cable, and restart the robot.
3. Create a manual ethernet connection: IP: 169.254.1.75, Subnet Mask: 255.255.0.0, Gateway (if required): 0.0.0.0. If you're running linux on a virtual machine, create this connection on the host operating system.
4. When your robot finishes booting (it will usually start talking or stop flashing), press its chest button and it will say its IP address.
5. Run the following script with that address and the robot's 2-digit ID (usually taped on the back of the head): `$NAO_HOME/install/setup_robot --ip <IP_ADDRESS_YOUR_ROBOT_SAID> --id <ROBOT_HEAD_ID>`.
6. Wait for the script to finish, and then the robot's server will reboot. After the reboot the setup process is complete.

<a id="wireless"></a>
### Connecting with Wireless

It is usually simplest to connect to the lab robots over the wireless network. We use the SSIDs `naonet` and `naonet5g` (no password) for communicating with the robots.

1.  Connect to the appropriate wireless network using DHCP.
2.  Use the IP address 192.168.1.XX when accessing the robot, where XX is the robot number taped on the back of the head near the ear.

<a id="ethernet"></a>
### Connecting with Ethernet

It can sometimes be advantageous to connect to the robot with an ethernet cable rather than over wireless, particularly when transferring logs, or when there are latency issues. To do so you'll need to follow these steps:

1.  Create a manual connection: IP: 11.0.1.75, Subnet Mask: 255.255.255.0, Gateway (if required): 0.0.0.0. If you're running linux on a virtual machine, create this connection on the host operating system.
2.  Connect your machine with an ethernet cable to the robot. The robot's ethernet port is on the back of its head.
3.  Use the IP address 11.0.1.XX when accessing the robot, where XX is the robot number taped on the back of the head near the ear.
