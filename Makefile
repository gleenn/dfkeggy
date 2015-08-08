.PHONY: all clean install run

all: run

run: build/devel/lib/dfkeggy_webui/dfkeggy_webui 
	build/devel/lib/dfkeggy_webui/dfkeggy_webui

build/devel/lib/dfkeggy_webui/dfkeggy_webui: dfkeggy_webui/src/server.cpp 
	mkdir -p build
	cd build && cmake ../dfkeggy_webui && make
	# xxd -i www/index.html > 

clean:
	rm -rf build

rosws:
	mkdir -p build
	cd build/src/dfkeggy; ln -s ../../../src
	cd build; catkin_init_workspace

