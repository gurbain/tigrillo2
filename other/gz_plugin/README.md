Install the Gazebo Plugin
-------------------------

## Build and install the project
We install the poject in the home gazebo folder by default:
```
# Source the GAZEBO env variables
source /usr/share/gazebo/setup.sh

# Build and install the plugin in the GAZEBO_PLUGIN_PATH folder
cd tigrillo2/other/gz_plugin && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=${GAZEBO_PLUGIN_PATH%%:*} ..
sudo make install
```


## Edit your model to add the plugin
In your model sdf file, add a line for the plugin
```
	<model>
	...

		<!-- Attach the plugin to the model -->
		<plugin name='tigrillo_plugin' filename='libtigrillo_plugin.so'></plugin>

	...
    </model>
```


## Place your model in the right folder
Generally, this means to create a folder called *YOUR_MODEL* containing the files *model.config* and *model.sdf* in the path *$HOME/.gazebo/models*