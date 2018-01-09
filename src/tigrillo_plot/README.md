Ubuntu 14.04 build (Semaphore): [![Build Status](https://semaphoreci.com/api/v1/facontidavide/plotjuggler/branches/master/shields_badge.svg)](https://semaphoreci.com/facontidavide/plotjuggler)

ROS Indigo build (Travis): [![Build Status](https://travis-ci.org/facontidavide/PlotJuggler.svg?branch=master)](https://travis-ci.org/facontidavide/PlotJuggler)


[![Join the chat at https://gitter.im/PlotJuggler/Lobby](https://badges.gitter.im/PlotJuggler/Lobby.svg)](https://gitter.im/PlotJuggler/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# PlotJuggler 1.5.0

[![Faircode](https://faircode.io/product/PlotJuggler/badge)](https://faircode.io/product/PlotJuggler?utm_source=badge&utm_medium=badgelarge&utm_campaign=PlotJuggler)


QT5 based application to display time series in plots. 

To understand what PlotJuggler can do for you, take a look to the following video [PlotJuggler on Vimeo](https://vimeo.com/214389001) 

![PlotJuggler](docs/images/PlotJuggler.gif)

# How to build

Clone the repository as usual:

      git clone https://github.com/facontidavide/PlotJuggler.git

The only binary dependency that you need installed in your system is Qt5. On Ubuntu the debians can be installed with the command:

    sudo apt-get -y install qtbase5-dev libqt5svg5-dev
    
On Fedora:

    sudo dnf install qt5-qtbase-devel qt5-qtsvg-devel
    
Then compile using cmake (qmake is NOT supported):

     mkdir build; cd build
     cmake ..
     make
     sudo make install
 
 Note: the plugins need to be installed in the same folder of the executable or in __/usr/local/lib/PlotJuggler/__.
 
# Note for ROS users
 
If you use CATKIN to build this project, the ROS related plugins will be automatically included into the compilation. Both the executable and the plugins will be created in __devel/lib/plotjuggler__ (the address is relative to the catkin workspace).

To run the application, use the command:

    rosrun plotjuggler PlotJuggler
    
Alternatively, just execute the binary __PlotJuggler__. 

A mandatory dependency is https://github.com/facontidavide/ros_type_introspection 

__IMPORTANT__: you need version 1.0.1 or later.

You can easily install it in Indigo, Jade, Kinetic or Lunar using the command:

       sudo apt-get install ros-YOUR_ROS_DISTRO-ros-type-introspection

      
