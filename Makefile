.PHONY: all clean install run

all: run

run: build/devel/lib/dfkeggy_webui/dfkeggy_webui 
	build/devel/lib/dfkeggy_webui/dfkeggy_webui

build/devel/lib/dfkeggy_webui/dfkeggy_webui: build dfkeggy_webui/src/* 
	cd build && cmake ../dfkeggy_webui && make
	# xxd -i www/index.html > 

clean:
	rm -rf build

build:
	mkdir -p build
	cd build; catkin_init_workspace

