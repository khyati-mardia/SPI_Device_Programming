Assignment 4 - SPI Device Programming and Pulse Measurement

This project is done by Team 8 for SPI Device Programming and Pulse Measurement
Before running this project , kindly note the following points 

TEAM 8 
KHYATI MARDIA  1215346587
CHANDRIKA C.S  1215321133


Instructions to run a program on target :

Connect pins on Galileo Board as below.

	Pin 11 : MOSI : spidev, 24, 44, 72 (Led-Matrix)
	Pin 12 : MISO : Slave select: using IO12 as a GPIO Pin:15 ,42,81,81 (Led-Matrix)
	Pin 13 : CLock : spidev, 30, 46, 81 (Led-Matrix)
	Pin 2  : Trigger : GPIO Pin : 13, 34, 77 (Ultra-Sonic)
	Pin 3  : Echo : GPIO Pin : 14, 16, 76 ,64 (Ultra-Sonic)
	VCC and GND of LEd Matrix and Ultra-sonic to Galileo Board

1. File name DriverMain.c is the program file for User Space and Spi_driver_led.c is the program file in Kernel Space . 
   
3. Kindly set path as Source /opt/iot-devkit/1.7.2/environment-setup-i586-poky-linux and "make all" in the Terminal or Host.
"make all" and "make clean" are the commands for building(compiling) and deleting the built files.

4. Kindly SCP both the object files using following commands.
	i) scp DriverMain root@ip_address_of_target:
	ii) scp Spi_driver_led.ko root@ip_address_of_target:

5. Remove file name spi_dev by using command : rmmod spi_dev  

6. Insert the module type , insmod file_name.ko in target (Galileo board) by using following command :
	insmod Spi_driver_led.ko 

7. Run the program by using command : ./DriverMain

8. We are reading sequence of images mentioned in a program.

Total 10 images and 2 sequences are defined which are selected as per distance measured by Ultrasonic Sensor.

	i)  Our 10 images are {HI, Right Arrow, Right Angled Arrow, Right Curved Arrow, Right Sleeping Arrow, Left Arrow, Left Angled Arrow, Left Curved Arrow, Left Sleeping ArrowBYE, BYE}
	ii) Our First sequence is {HI, Right Arrow, Right Angled Arrow, Right Curved Arrow, Right Sleeping Arrow, BYE}
	iii)Our Second sequence is {HI, Left Arrow, Left Angled Arrow, Left Curved Arrow, Left Sleeping Arrow, BYE}
	iv) If we move object away or towards the Ultrasonics sensor, the image flips from Sequence 1 to Sequence 2 or Sequence 2 to Sequence 1.

9. Distance is calculated as below.

Test distance = (pulse width * Ultrasonic spreading velocity in air) / 2

Where,  Pulse Width = difference in system ticks , generated at start time and stop time
	1 tick = 1/400MHz(Frequency of Intel Galileo board)
	1 tick = 0.25 * 10^-8 sec
	distance = (diff in ticks)* 0.25*10^-8 * 340 (m/sec) / 2
Therefore distance = diff_in_time_ticks * 4.25/100000

10. The display patterns are controlled on LED matrix via spidev device (i.e. /dev/spi1.0)

11. Kernel logs can be seen by using "dmesg" command.

12. Make file is modified for X86 architecture by giving relevant compiler paths and also PATH_variable.
 
13. Program is properly commented and warning free.

14. The program is terminated using ctrl+c command. We have used signal handler function , that uninitialises the gpio pins we requested for.

Note : We create a device /dev/spidev1.0 in the init method , Thus thr driver and the device data are registered. 
IN the probe method, they are matched and the device is created and matched with the driver successfully
IN the open method, we initialise the gpio pins of the led matrix display
IN the ioctl method, we send the 10 images data to the kernel.
IN the write method, we write the sequence in which they need to be display. WE have used a kernel thread , which executes the display function.
IN the release method, we free the gpio pins we requested for .
In the remove method, we unload the kernel module.

