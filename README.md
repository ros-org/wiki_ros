# tutorial

get all ros resources at <http://wiki.ros.org/>

install ros briefly as follow:

>(arm)$ sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

>(!arm)$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

>(arm)$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

>(!arm)$ sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

>(arm)$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

>$ sudo apt-get update

>(!arm)$ sudo apt-get install ros-indigo-desktop-full

>(arm)$ sudo apt-get install ros-indigo-ros-base

>(arm)$ sudo apt-get install python-rosdep

>$ sudo rosdep init

>$ rosdep update

>$ sudo apt-get install python-rosinstall

>(arm)$ unset GTK_IM_MODULE

configure environment as follow:

>$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

>$ source ~/.bashrc

***
# stacks

* [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
 * [rosbridge_suite-develop](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/rosbridge_suite-develop.zip)
* [roslibjs](http://wiki.ros.org/roslibjs)
 * [roslibjs-develop](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/roslibjs-develop.zip)
* [ros2djs](http://wiki.ros.org/ros2djs)
 * [ros2djs-develop](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros2djs-develop.zip)
* [robot_pose_publisher](http://wiki.ros.org/robot_pose_publisher)
 * [robot_pose_publisher-develop](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/robot_pose_publisher-develop.zip)
* [nav2djs](http://wiki.ros.org/nav2djs)
 * [nav2djs-develop](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/nav2djs-develop.zip)
* [ros3djs](http://wiki.ros.org/ros3djs)
* [rms](http://wiki.ros.org/rms)

***
# robots
