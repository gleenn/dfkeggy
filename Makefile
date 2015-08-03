.PHONY: all clean install run

all: bin/dfkeggy

run: all
	bin/dfkeggy

bin/dfkeggy: src/webui/server.cpp 
	mkdir -p bin
	c++ -I/usr/local/include src/webui/server.cpp -L /usr/local/lib -l websockets -o bin/dfkeggy
	# xxd -i www/index.html > 

# install: env
# 	sudo python setup.py install

clean:
	rm -rf bin
	# find . -name '*.pyc' -exec rm -f {} +
	# find . -name '*.pyo' -exec rm -f {} +
	# find . -name '*~' -exec rm -f {} +
	# rm -rf dfaprs.egg-info
	# rm -rf build
	# rm -rf env


