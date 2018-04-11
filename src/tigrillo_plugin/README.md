Tigrillo Plugin Installation and Test
-------------------------------------

1. Compile the GazeboRosPackages ROS workspace with catkin_make (see neurorobotics platform documentation if help is needed)

2. Create a symlink for the test model in your ~/.gazebo/models folder:
```
cd ~/.gazebo/models
ln -s <package_path>/sdf tigrillo
```

3. Start gazebo with your GazeboRosPackages workspace sourced:
```
cd <GazeboRosPackages>
source devel/setup.bash
rosrun gazebo_ros gazebo  (--verbose for debug output)
```

4. Insert the test model from the models list. It will show a robot with 4 links and 3 joints (see model.sdf in sdf folder).

5. Vizualise the sensors on a graph using Plot juggler (sudo apt-get install ros-kinetic-plotjuggler):
```
rosrun plotjuggler PlotJuggler
```

6. Test the motor control using the python test script:
```
python test_plugin.py
```

