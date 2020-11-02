CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 
CXX = i586-poky-linux-g++
CC = i586-poky-linux-gcc
$ARCH = x86
CROSS_COMPILE = i586-poky-linux-
$PATH := $(PATH):/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux

SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
OBJS =		ESP4.0.o

LIBS = -pthread

TARGET =	ESP4.0

$(TARGET):	$(OBJS)
	$(CC) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

#clean:
#	rm -f $(OBJS) $(TARGET)
#obj-m:= UpdatedModule.o
#OBJS =		RGBLed_1.o InputSignal.o
#LIBS = 
#TARGET =	RGBLed_1

#all:
#	make ARCH=x86 CROSS_COMPILE=$(CROSS_COMPILE) -C $(SROOT)/usr/src/kernel M=$(PWD) modules

#	$(CXX) -O2 -g -Wall -fmessage-length=0   -c -o RGBLed_1.o RGBLed_1.cpp
#	$(CXX) -O2 -g -Wall -fmessage-length=0   -c -o InputSignal.o InputSignal.cpp
#	$(CXX) -o RGBLed_1 RGBLed_1.o InputSignal.o -pthread

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
	