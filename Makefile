.PHONY: all clean build run

export SHELL:=/bin/bash

SOURCES := \
	dfkeggy_webui/src/* \
	roboteq/roboteq_driver/src/* \
	nmea_navsat_driver/scripts/* \
	nmea_navsat_driver/src/libnmea_navsat_driver/*

all: run

run: build
	export KEGGY_HOME=$(shell pwd) && source build/catkin_ws/devel/setup.bash && roslaunch keggy-sim.launch

runhw: build
	export KEGGY_HOME=$(shell pwd) && source build/catkin_ws/devel/setup.bash && roslaunch keggy-hrd.launch

build: build/catkin_ws $(SOURCES)
	cd build/catkin_ws && source devel/setup.bash && catkin_make

build/devel/lib/dfkeggy_webui/dfkeggy_webui: build dfkeggy_webui/src/* 
	cd build/webui && cmake ../../dfkeggy_webui && make
	# xxd -i www/index.html > 

build/devel/lib/roboteq_driver/driver_node: build roboteq/roboteq_driver/*
	cd build/roboteq && cmake ../../roboteq/roboteq_driver && make

clean:
	rm -rf build

build/catkin_ws:
	mkdir -p build/catkin_ws/src
	cd build/catkin_ws/src && catkin_init_workspace 
	cd build/catkin_ws && catkin_make
	cd build/catkin_ws && rosdep install -r --from-paths .
	ln -s ../../../dfkeggy_webui build/catkin_ws/src/webui
	ln -s ../../../roboteq build/catkin_ws/src/roboteq
	ln -s ../../../compass build/catkin_ws/src/compass
	ln -s ../../../nmea_navsat_driver build/catkin_ws/src/nmea_navsat_driver

