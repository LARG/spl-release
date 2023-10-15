# UT Austin Villa SPL Codebase

UT Austin's codebase for the Robocup Standard Platform League using ros2

### Installation

The following installation procedure is designed to work with Ubuntu 20.04, mostly due to the dependencies of ros2 galactic and the Ubuntu system image for the Nao robot.

First, install dependencies for installation (debootstrap, pigz, and e2fsprogs are required for generating the Ubuntu system image for the robot)

		sudo apt install git python3 wget debootstrap pigz e2fsprogs

Clone the repository (creates a directory called spl_ros)

		git clone --recurse-submodules git@github.com:larg/spl_ros.git
		cd spl_ros

Run the codebase install script:

		./scripts/villa install

Once the install script is run once, some files are generated to make typing commands easier. Sourcing the setup script will enable you to just type `villa` instaed of the full path to the script.

		source ./scripts/bash_setup.sh

You can also add this line to your .bashrc so it runs automatically whenever you open a new terminal. Note that the bash_setup.sh file tells the shell where the scripts are located. If you move codebase to a different directory, you should run scripts/villa install bash_setup to update the bash_setup file. If you have multiple versions of the codebase, only the source the setup file for the version you'd like to run.

You will need a copy of the Nao V6 firmware provided by Aldebaran (nao-2.8.5.11_ROBOCUP_ONLY_with_root.opn). Since this file is not meant to be distributed outside of Robocup, it is not automatically installed by the install script. UTexas students can get access to this file via the shared UT Box folder. Other teams should email Aldebaran directly for access to the system image. Copy it into the NaoImage folder.

		cp <path to image> tools/NaoImage/

Now, you can generate the sysroot for the Nao operating system. This step isn't always necessary, but it is recommended, since the sysroot is needed for cross compiling the code. This script can take a while to run and takes quite a bit of memory (~5G) since the file size of the generated image is large. This step only needs to be run once though.

		./scripts/villa genimage

To flash the robot, first generate the opn file that will go on the flash drive, replacing ## with the robot id number.

		./scripts villa genopn --id ##

Copy the image to the flash drive via dd, replacing /dev/... with the device name for the flash drive. (You can use the lsblk command to see the available devices. It should be something like /dev/sdb. **Note:** Be very careful to use the correct device name, otherwise there is a risk of overwriting your own data/OS partitions.)

		dd if=tools/NaoImage/image.opn of=/dev/... bs=4k conv=fsync status=progress

To flash the robot, put the flash drive in the usb port on the back of the head. Turn the robot off if it is not already. Press and hold the chest button until it turns blue. When you release, it shoudl start flashing blue. The flashing indicates the flashing process has begun. When it stops flashing, it is done. You should then be able to connect to the robot via ssh, replacing ## with the robot id number

		ssh nao@11.0.1.##
