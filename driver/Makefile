obj-m += altera_drv.o

altera_drv-objs := sensor_fusion_main.o sensor_fusion_ov8865.o sensor_fusion_v4l2.o sensor_fusion_queue.o i2c_adapter_siasun.o
altera_drv-objs += rs_lidar.o lidar_module.o 
altera_drv-objs += fpga_cdev.o fpga_schedule.o 

KERN_DIR ?= /lib/modules/$(shell uname -r)/build/

FPU_CFLAGS += -mhard-float
FPU_CFLAGS += -msse -msse2
CFLAGS_fpga_schedule.o += $(FPU_CFLAGS)

all:
	make -C $(KERN_DIR) M=`pwd` modules
	g++ -fPIC -Wall -Wcast-align -Wredundant-decls -shared -I./lib lib/libsfconfig.cpp -o lib/libsfconfig.so
	cd test; make; cd ..

clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm lib/*.so -f
	cd test; make clean; cd ..

install:
	cp lib/libsfconfig.so /usr/lib/
	chmod 644 lib/libsfconfig.h
	cp lib/libsfconfig.h /usr/include/
	ldconfig

uninstall:
	rm /usr/lib/libsfconfig.so
	rm /usr/include/libsfconfig.h

.PHONY:clean install uninstall
