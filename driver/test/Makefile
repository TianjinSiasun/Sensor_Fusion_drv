all:
	g++ -o camera_test camera_test.cpp  -L../lib -lsfconfig
	g++ -o fpga_cfg_test fpga_cfg.cpp -L../lib -lsfconfig
	g++ -o lidar_test lidar_test.cpp -L../lib -lsfconfig
	g++ msop_test.cpp -o msop_test -L../lib -lsfconfig

clean:
	rm *_test -f

.PHONY:clean
