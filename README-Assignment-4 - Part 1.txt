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

1. File name ESP4.0.c is the program file for User Space. 
   
2. Kindly set path as Source /opt/iot-devkit/1.7.2/environment-setup-i586-poky-linux and "make all" in the Terminal or Host.
"make all" and "make clean" are the commands for building(compiling) and deleting the built files.

3. Kindly SCP the object file using following commands.
	scp ESP4.0 root@ip_address_of_target:

4. Run the program by using command : ./ESP4.0

5. Output observed is : 

	i) If object is within 15 cm from Ultrasonic sensor, the images (Right Angled Arrow and Right Curved Arrow or Left Angled Arrow and Left Curved Arrow) moves faster with delay of 0.2 sec.
	ii) If object is between 15 cm to 35 cm from Ultrasonic sensor, the images (Right Angled Arrow and Right Curved Arrow or Left Angled Arrow and Left Curved Arrow) moves slower than first part with delay of 0.3 sec.
	iii) If object is between 35 cm to 60 cm from Ultrasonic sensor, the images (Right Angled Arrow and Right Curved Arrow or Left Angled Arrow and Left Curved Arrow) moves even slower with delay of 0.6 sec.
	iv) If object is beyond 60 cm from Ultrasonic sensor, the images (Right Angled Arrow and Right Curved Arrow or Left Angled Arrow and Left Curved Arrow) moves slowest with delay of 1 sec.
	v) If object is moved away or towards Ultrasonic Sensor , toggling between images is observed. 
	For eg. If current images displayed are Right Angled Arrow and Right Curved Arrow, and if object is moved towards or away then images would be toggled to Left Angled Arrow and Left Curved Arrow.

6. Distance is calculated as below.

Test distance = (pulse width * Ultrasonic spreading velocity in air) / 2

Where,  Pulse Width = difference in system ticks , generated at start time and stop time
	1 tick = 1/400MHz(Frequency of Intel Galileo board)
	1 tick = 0.25 * 10^-8 sec
	distance = (diff in ticks)* 0.25*10^-8 * 340 (m/sec) / 2
Therefore distance = diff_in_time_ticks * 4.25/100000

7. The display patterns are controlled on LED matrix via spidev device (i.e. /dev/spi1.0)

8. Make file is modified for X86 architecture by giving relevant compiler paths and also PATH_variable.
 
9. Program is properly commented and warning free.

10. The program is terminated using ctrl+c command. We have used signal handler function , that uninitialises the gpio pins we requested for.

