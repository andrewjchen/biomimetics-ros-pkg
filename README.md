biomimetics-ros-pkg
=========

Biomimetic Millisystems Lab ROS Repository.
This repository includes an imageproc ROS node.

See [http://www.ros.org/wiki/biomimetics-ros-pkg](http://www.ros.org/wiki/biomimetics-ros-pkg) for more information.

Installation
--------------

Check out the repository:
```
cd
git clone https://github.com/andrewjchen/biomimetics-ros-pkg.git
cd biomimetics-ros-pkg
git submodule update --init
```

Install imageproc_node to the catkin workspace:
```
cd ~/catkin_ws/src/
ln -s ~/biomimetics-ros-pkg/imageproc_node
rospack profile
```

Run:
```
rosrun imageproc_node imageproc_node.py
```


imageproc_node
--------------
_Provides a ROS interface to a serial-connected ImageProc2.5_

### Theory of operation

imageproc_node merely provides a ROS wrapper to [imageproc_py]-- it accepts ROS messages and makes the analagous calls to imageproc_py.Currently the only verison of imageproc_py that does this is [https://github.com/andrewjchen/imageproc_py/tree/imageproc_driver](https://github.com/andrewjchen/imageproc_py/tree/imageproc_driver)

imageproc_py is a python API that connects to an ImageProc via UART. It uses the "cmd" protocol to send commands such as command velocity, encoder position, or setting a parameter.

On the imageproc2.5 robot controller itself runs the biomimetics imageproc library. This library needs to implement the serial "cmd" protocol in order to talk to imageproc_py over UART. A working implementation can be found here: [https://github.com/andrewjchen/imageproc-test/tree/ajc](https://github.com/andrewjchen/imageproc-test/tree/ajc)

### Usage

imageproc_node currently only accepts ```geometry_msgs/Twist``` messages on topic ```/cmd_vel```.

License
------------
All code licensed under the 3-clause modified-BSD license.


[imageproc_py]: https://github.com/biomimetics/imageproc_py
