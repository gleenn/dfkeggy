.PHONY: all clean install run

all: run

run: build/devel/lib/dfkeggy_webui/dfkeggy_webui build/devel/lib/roboteq_driver/driver_node
	build/devel/lib/dfkeggy_webui/dfkeggy_webui

build/devel/lib/dfkeggy_webui/dfkeggy_webui: build dfkeggy_webui/src/* 
	cd build/webui && cmake ../../dfkeggy_webui && make
	# xxd -i www/index.html > 

build/devel/lib/roboteq_driver/driver_node: build roboteq/roboteq_driver/*
	cd build/roboteq && cmake ../../roboteq/roboteq_driver && make

clean:
	rm -rf build

build:
	mkdir -p build/src
	ln -s ../../dfkeggy_webui build/src/webui
	ln -s ../../roboteq build/src/roboteq
	#cd build && catkin_init_workspace
	#source devel
	cd build && catkin_make
