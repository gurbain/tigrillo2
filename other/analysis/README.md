# Tigrillo Analysis
A set of files to optimize the quadruped model and the transfer learning methods. This is purely a research tool and is configured and sometimes hardcoded for a specific platform.

## Installation
- Install ROS and Gazebo and python-rosbag
- Create a Tigrillo Model for Gazebo (generally in *$HOME/.gazebo/models*)
- Change the paths variables in optim.py and view.py
- Install python libraries for optimization and vizualization:
```
sudo pip install PyQt5
sudo pip install qdarkstyle
sudo pip install bisect
sudo pip install scipy
```

## Optimization
Start an optimization to tune the model parameters using the robot real values stored in a rosbag:
```
python optim.py (python3 may give more accurate results when creating the gazebo model)
```

## Visualization
A UI using PyQt to browse the optimization experiments and display different results, values, and simulations:
```
python view.py
```