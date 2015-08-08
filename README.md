Keggy control software
=======================

Installation (Ubuntu)
---------------------
	#  ROS
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
	sudo apt-get update
	sudo apt-get install ros-jade-desktop-full
	sudo rosdep init
	rosdep update
	sudo apt-get install python-rosinstall
	echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc

	# Other dependencies
	# note: libwebsockets-dev in ubuntu repo currently has v1.2, we need 1.4, so install from sources:
	git clone https://github.com/warmcat/libwebsockets.git
	cd libwebsockets && cmake . && make && sudo make install && sudo ldconfig
    git clone git@bitbucket.org:azov/dfkeggy.git
    cd dfkeggy
    make


Installation (OSX)
-------------------
	
(see ROS install instructions for OSX; install libwebsockets via homebrew)
