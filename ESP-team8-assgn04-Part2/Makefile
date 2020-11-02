CCFLAGS =	-O2 -g -Wall -fmessage-length=0
CXX = i586-poky-linux-g++
CC = i586-poky-linux-gcc
$ARCH = x86
CROSS_COMPILE = i586-poky-linux-
$PATH := $(PATH):/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux
SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
obj-m:= Spi_driver_led.o
OBJS =		DriverMain.o 
LIBS = -pthread
TARGET =	DriverMain

all:
	make ARCH=x86 CROSS_COMPILE=$(CROSS_COMPILE) -C $(SROOT)/usr/src/kernel M=$(PWD) modules
	$(CC) -O2 -g -Wall -fmessage-length=0   -c -o DriverMain.o DriverMain.c
	$(CC) -o DriverMain DriverMain.o -pthread

clean:
	rm -f *.ko
	rm -f *.o
	rm -f Module.symvers
	rm -f modules.order
	rm -f *.mod.c
	rm -rf .tmp_versions
	rm -f *.mod.c
	rm -f *.mod.o
	rm -f \.*.cmd
	rm -f Module.markers
	rm -f $(TARGET) 