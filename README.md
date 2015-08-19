Keggy control software
=======================

Installation (Ubuntu)
---------------------
    sudo apt-get install git

	#  ROS
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
	sudo apt-get update
	sudo apt-get install ros-jade-desktop-full
	sudo rosdep init
	rosdep update
	sudo apt-get install python-rosinstall
	sudo apt-get install ros-jade-roboteq-driver
	sudo apt-get install gpsd
	sudo usermod -a -G dialout $USER
	echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
	source /opt/ros/jade/setup.bash
	roscore&


Create ROS workspace

	mkdir -p ~/keggy_ws/src && cd ~/keggy_ws/src
	mkdir src && cd src
	git clone https://github.com/code-iai/iai_kinect2.git
	git clone https://github.com/g/roboteq.git
	cd ~/keggy_ws/src/iai_kinect2 && rosdep install -r --from-paths .
	cd ~/keggy_ws/src/roboteq && rosdep install -r --from-paths .
	cd ~/keggy_ws && catkin_make -DCMAKE_BUILD_TYPE="Release"


Create UDEV rules:
sudo vi /etc/udev/rules.d/usb-parse-devpath.pm

#!/usr/bin/perl -w

@items = split("/", $ARGV[0]);
for ($i = 0; $i < @items; $i++) {
    if ($items[$i] =~ m/^usb[0-9]+$/) {
        print $items[$i + 2] . "\n";
        last;
    }
}

sudo chmod u+x /etc/udev/rules.d/usb-parse-devpath.pm
sudo vi /etc/udev/rules.d/usb-parse-devpath.rules
Add this:
  ACTION=="add", KERNEL=="ttyUSB[0-9]*", PROGRAM="/etc/udev/rules.d/usb-parse-devpath.pm %p", SYMLINK+="usb-ports/%c"



source build/devel/setup.bash
roslaunch keggy.launch


	sudo apt-get install ros-jade-roboteq-driver

   ```
cd ~
sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev
sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so
git clone https://github.com/OpenKinect/libfreenect2
cd libfreenect2/depends
./install_ubuntu.sh
sudo dpkg -i libglfw3*_3.0.4-1_*.deb
cd ..
mkdir build
cd build
cmake ../examples/protonect/ -DENABLE_CXX11=ON
make && sudo make install
```

   ```
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ..
git clone https://github.com/g/roboteq.git
rosdep install -r --from-paths .

# Change ~ in roboteq_driver/src/driver.cpp to keggy_roboteq_driver

cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
```

   ```
source devel/setup.bash
rosrun kinect2_bridge kinect2_bridge
```


	# Other dependencies
	# note: libwebsockets-dev in ubuntu repo currently has v1.2, we need 1.4, so install from sources:
	git clone https://github.com/warmcat/libwebsockets.git
	cd libwebsockets && cmake . && make && sudo make install && sudo ldconfig
    git clone git@bitbucket.org:discofish/dfkeggy.git
    cd dfkeggy
    make

To see ROS messages (e.g. for webui topic):

	source ~/dfkeggy/build/devel/setup.bash
	rostopic echo /webui


Installation (OSX)
-------------------
	
(see ROS install instructions for OSX; install libwebsockets via homebrew)

