# tutorial community

get all ros resources at <http://wiki.ros.org/>

***
# quick installation (for indigo)

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

***
# quick configuration (for indigo)

>$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

>$ source ~/.bashrc

>$ mkdir -p ~/workspaces/src

>$ mkdir ~/catkin_ws

>$ ln -s ~/workspaces/src ~/catkin_ws/src

>$ cd ~/catkin_ws/src

>$ catkin_init_workspace

>$ cd ~/catkin_ws/

>$ catkin_make

>$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

>$ source ~/.bashrc

>$ cd ~/catkin_ws/src

create package as:

>$ catkin_create_pkg [package_name] [depend1] [depend2] [depend3] ...

install package as: (replace underscores with dashes of the package name)

>$ sudo apt-get install ros-indigo-[stack_or_package_name]

***
# install software

>$ sudo apt-get install vim

***
# interesting stacks

* [octomap](http://wiki.ros.org/octomap) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/octomap-devel.zip)
* [slam_gmapping](http://wiki.ros.org/slam_gmapping) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/slam_gmapping-hydro-devel.zip)
* [hector_mapping](http://wiki.ros.org/hector_mapping) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/hector_slam-catkin.zip)
* [hector_navigation](http://wiki.ros.org/hector_navigation)
* [navigation](http://wiki.ros.org/navigation) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/navigation-indigo-devel.zip)
* [people](http://wiki.ros.org/people) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/people-indigo-devel.zip)
* [robot_web_tools](http://wiki.ros.org/robot_web_tools) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/robot_web_tools-develop.zip)
* [robot_pose_publisher](http://wiki.ros.org/robot_pose_publisher) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/robot_pose_publisher-develop.zip)
* [tf2_web_republisher](http://wiki.ros.org/tf2_web_republisher) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/tf2_web_republisher-develop.zip)
* [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/rosbridge_suite-develop.zip)
* [roslibjs](http://wiki.ros.org/roslibjs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/roslibjs-develop.zip) |  [eventemitter2.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/eventemitter2.min.js) |  [roslib.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/roslib.min.js)
* [ros2djs](http://wiki.ros.org/ros2djs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros2djs-develop.zip) | 
[easeljs.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/easeljs.min.js) |  [ros2d.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros2d.min.js)
* [ros3djs](http://wiki.ros.org/ros3djs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros3djs-develop.zip) | 
[three.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/three.min.js) |  [ros3d.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros3d.min.js)
* [nav2djs](http://wiki.ros.org/nav2djs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/nav2djs-develop.zip) | 
[nav2d.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/nav2d.min.js)
* [keyboardteleopjs](http://wiki.ros.org/keyboardteleopjs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/keyboardteleopjs-develop.zip) | 
[keyboardteleop.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/keyboardteleop.min.js)
* [rms](http://wiki.ros.org/rms) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/rms-develop.zip)

***
# robots
