# Tigrillo2

A ROS project to simulate and pilot the Tigrillo quadruped robot


## Requirements

This project have been tested with the following software:

 - Ubuntu 16.04
 - Python 2.7
 - ROS kinetic

## Installation

### On the Tigrillo RPI or a computer

Clone the repository:
```
git clone http://github.com/Gabs48/tigrillo2
```

Build and install the package:
```
cd tigrillo2 && catkin_make
```

Source the folder in your ROS environment:
```
echo "source CUSTOM_PATH/tigrillo2/devel/setup.bash" > ~/.bashrc
```
OR
```
echo "source CUSTOM_PATH/tigrillo2/devel/setup.zsh" > ~/.zshrc
```

### On the Tigrillo OpenCM
Download the OpenCM make tools and edit the makefile of the tigrillo project to point to it:
```
git clone http://github.com/Gabs48/OpenCM
cd tigrillo2/other/open_cm && nano Makefile
```
Then, compile and upload the project to the robot via a USB cable:
```
make
sudo make do_upload
```

## Example

### Setting up the ROS environment

On one computer in the network, run roscore:
```
roscore - p PORT_NUMBER
```

On all computer, ensure that hosname and roscore adresses are correctly set in the ROS environment by setting the following line in the .bashrc or the .zshrc:
```
export ROS_HOSTNAME=192.168.1.43 # THE LOCAL IP ADDRESS OF THE COMPUTER
export ROS_MASTER_URI=http://192.168.1.12:5555/ # THE LOCAL IP ADDRESS AND PORT NUMBER OF THE ROSCORE COMPUTER
```

### Tigrillo Robot

On the RPI from the Tigrillo robot, start the low-level nodes (in different shells if needed):
```
rosrun tigrillo_io uartd
rosrun tigrillo_io i2cd
```
Be carefull that those nodes produce a ROS log file as well as csv files with all sensors and actuators values that can use disk space and slow down the processes.

### Controller
Somewhere in the network, start one controller node, an open-loop CPG for instance:
```
rosrun tigrillo_ctrl openloo_cpg
```

You can monitor the bandwith and frequency between the networks node with:
```
rostopic list
rostopic [hz|bw] TOPIC_NAME
```
By default, the nodes are set to run at 50Hz, so it is good to ensure that such a frequency can be achieved.

### Analysis
To plot the different signal values in real-time, simply use the rqt_plot tool:
```
rosrun rqt_plot rqt_plot
```
**(not implemented yet):** For further offline analysis, a new node will be written soon!

### Simulation
Compile the gazebo plugin library:
```
cd tigrillo2/other/gz_plugin
mkdir build && cd build
cmake ..
make
```
And install it in you GAZEBO_PLUGIN_PATH folder.

**(not implemented yet):** load the simulation world which interacts with the ROS topics like the */tigrillo_io* node on the robot:
```
cd tigrillo2/data/sim
rosrun gazebo_ros gazebo worlds/default.world
```

