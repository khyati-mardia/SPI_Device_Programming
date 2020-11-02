//============================================================================
// Name        : 0.cpp
// Author      : Khyati Mardia
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L
#define _BSD_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <pthread.h>
#include <inttypes.h>
#include <time.h>
#include <poll.h>
#include "rdtsc.h"
#include <signal.h>
#include "Gpio_func.c"
#define DEVICE "/dev/spidev1.0"
int flag_for_direction = 1,m=0; //For flipping the image

uint8_t led_init[]={
		0x0F,0x00,
		0x09,0x00,
		0x0A,0x04,
		0x0B,0x07
};
uint8_t tx[2];

typedef struct {
	uint8_t tx1[18];
	uint8_t tx2[18];
	uint8_t tx4[18];
	uint8_t tx5[18];
}PATTERN;

//Pattern 1 : Right Angled Arrow

uint8_t tx1[]= {

		0x0c,0x01,
		0x01,0x00,
		0x02,0x00,
		0x03,0x20,
		0x04,0x10,
		0x05,0x08,
		0x06,0x05,
		0x07,0x03,
		0x08,0x07
};

//Pattern 2 : Right Curved Arrow

uint8_t tx2[]= {

		0x0c,0x01,
		0x01,0x00,
		0x02,0x00,
		0x03,0x00,
		0x04,0x20,
		0x05,0x10,
		0x06,0x50,
		0x07,0x60,
		0x08,0x70

};

//Pattern 3 : Left Angled Arrow

uint8_t tx4[]= {

		0x0c,0x01,
		0x01,0x07,
		0x02,0x03,
		0x03,0x05,
		0x04,0x08,
		0x05,0x10,
		0x06,0x20,
		0x07,0x00,
		0x08,0x00

};

//Pattern 4 : Left Curved Arrow

uint8_t tx5[]= {

		0x0c,0x01,
		0x01,0x70,
		0x02,0x60,
		0x03,0x50,
		0x04,0x10,
		0x05,0x20,
		0x06,0x00,
		0x07,0x00,
		0x08,0x00

};


int fd_spi;
char gpio_pin[2];
uint32_t speed;
double globalDistance;
double dist_measured,dist_measured_p=0;
int ret;
void spiunexport(int pin){
	int fd_unexp;
	fd_unexp = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd_unexp < 0)
	{
		printf("\n gpio unexport failed while opening");
	}
	sprintf(gpio_pin,"%d",pin);
	write(fd_unexp,&gpio_pin,2);
}
/**
 * This method is for displaying images.
 * Based on the values set by dist_and_direction method , We toggle between the sequences .
 * 0x0F, 0x00, // Display Test Register.// No LEDs ON
 * 0x0C, 0x01, // Shutdown Register //Digit 1 : Normal Mode of operation
 * 0x09, 0x00, // Decode Mode	// No decode operation for DIgits 0-7
 * 0x0A, 0x0F, // Intensity // Display Test 0xXF , maximum intensity of LEDs on.
 * 0x0B, 0x07, // Scan Limit // Displays Digits from 0 to 7
 * 0x01, 0x06, // Digit 0 register value//
 * 0x02, 0x06, // Digit 1 register value//
 * 0x03, 0x06, // Digit 2 register value//
 * 0x04, 0x83, // Digit 3 register value//
 * 0x05, 0x7F, // Digit 4 register value//
 * 0x06, 0x24, // Digit 5 register value//
 * 0x07, 0x22, // Digit 6 register value//
 * 0x08, 0x63, // Digit 7 register value//
 */
void* display_init(void* dummy){
	int i,j = 0;
	struct spi_ioc_transfer spi_msg =
	{
			.tx_buf = (unsigned long)tx,
			.rx_buf = 0,
			.len = 2,
			.delay_usecs = 1,
			.speed_hz = 10000000,
			.bits_per_word = 8,
			.cs_change = 1,
	};
	//We initalise the display by sending the
	//values to the setup registers mentioned above.
	for(i = 0 ; i < 8 ; i+=2)
	{
		tx[0] = led_init[i];
		tx[1] = led_init[i+1];
		gpio_set_value(15,0);
		ret = ioctl(fd_spi, SPI_IOC_MESSAGE (1), &spi_msg);
		gpio_set_value(15,1);
		if(ret < 0)
			printf("ioctl error\n");
	}

	while(1) {
		//Flag is 1 for a certain direction forward.
		// and 0 for opposite direction.
		if (flag_for_direction == 1) {
			for(i = 0 ; i < 18; i+=2) {
				if (m%2 == 0) {
					tx[0] = tx1[i];
					tx[1] = tx1[i+1];
				}
				else  {
					tx[0] = tx2[i];
					tx[1] = tx2[i+1];
				}
				gpio_set_value(15 ,0);
				ret = ioctl(fd_spi, SPI_IOC_MESSAGE (1), &spi_msg);
				gpio_set_value(15 ,1);
			}
		}
		else {
			for(i = 0 ; i < 18; i+=2)
			{
				if (m%2 == 0) {
					tx[0] = tx4[i];
					tx[1] = tx4[i+1];
				}
				else  {
					tx[0] = tx5[i];
					tx[1] = tx5[i+1];
				}
				gpio_set_value(15 ,0);
				ret = ioctl(fd_spi, SPI_IOC_MESSAGE (1), &spi_msg);
				gpio_set_value(15 ,1);
			}
		}
		//Manipulating the speed based on the distance.
		if (globalDistance < 15)
			usleep(200000);
		else if (globalDistance < 35)
			usleep(300000);
		else if (globalDistance < 60)
			usleep(600000);
		else
			usleep(1000000);
		m++;
		if (m > 30000) {
			m=0;
		}
	}
}

void pinSetup() {
	//Pin 11 : MOSI : spidev, 24, 44, 72 (Led-Matrix)
	//Pin 12 : MISO : Slave select: using IO12 as a GPIO Pin:15 ,42,81,81 (Led-Matrix)
	//Pin 13 : CLock : spidev, 30, 46, 81 (Led-Matrix)
	//Pin 2  : Trigger : GPIO Pin : 13, 34, 77 (Ultra-Sonic)
	//Pin 3  : Echo : GPIO Pin : 14, 16, 76 ,64 (Ultra-Sonic)

	gpio_export(24);
	gpio_set_dir(24,1);
	gpio_set_value(24,0);

	gpio_export(44);
	gpio_set_dir(44, 1);
	gpio_set_value(44, 1);

	gpio_export(72);
	gpio_set_value(72, 0);

	gpio_export(15);
	gpio_set_dir(15, 1);

	gpio_export(42);
	gpio_set_dir(42, 1);
	gpio_set_value(42, 0);

	gpio_export(30);
	gpio_set_dir(30, 1);
	gpio_set_value(30, 0);

	gpio_export(46);
	gpio_set_dir(46, 1);
	gpio_set_value(46, 1);

	gpio_export(13);
	gpio_set_dir(13, 1);

	gpio_export(14);
	gpio_set_dir(14, 0);

	gpio_export(34);
	gpio_set_dir(34, 1);
	gpio_set_value(34, 0);

	gpio_export(77);
	gpio_set_value(77, 0);

	gpio_export(16);
	gpio_set_dir(16, 1);
	gpio_set_value(16, 1);

	gpio_export(76);
	gpio_set_value(76, 0);

	gpio_export(64);
	gpio_set_value(64, 0);
}

/* This function is for determining the direction and the speed of the display*/
void delay_and_dir(unsigned int dist ){
	dist_measured =dist;
	double dist_interval= dist_measured - dist_measured_p;
	if (dist_interval > 3 ) {
		flag_for_direction = 1;
		printf(" \n Object moving away from sensor!!!! \n");
		dist_measured_p = dist_measured;
	}
	else if (dist_interval < -3 ){
		flag_for_direction = 0;
		printf(" \n Object moving away from sensor!!!! \n");
		dist_measured_p = dist_measured;
	}

}
/**
 * We basically send trigger on PIn2
 * and listen for echo on Pin 3
 * We send a value of 1 on trigger pin.
 * We poll on the echo edge , by writing rising first.
 * We check the value of echo value pin for rising edge, take that as the start time .
 * Then, we make the edge trigger as falling  then poll on echo value pin, on seeing that
 * the event matches, we take that as the stop time.
 * The difference between start time and stop time is now the width of the pulse, which
 * is proportional to the distance of the object.
 */
void* distanceMeasure(void* dummyParam) {
	int triggerPinValueFd,echoEdgePinFd,echoValuePinFd,res, returnValue;
	struct pollfd PollEch ;
	double startTime, stopTime, time_diff =0;
	char* buff[64];
	unsigned char echoValue[2];
	//	Open the edge and value files
	//	Open the value file of gpio14
	triggerPinValueFd = open("/sys/class/gpio/gpio13/value", O_WRONLY);
	if (triggerPinValueFd < 0) {
		printf("\n FdTrig : gpio13 vale open failed");
	}
	echoValuePinFd = open("/sys/class/gpio/gpio14/value", O_RDWR);
	if (echoValuePinFd < 0) {
		printf("\n gpio14 value open failed");
	}

	//Open the edge and value files
	echoEdgePinFd = open("/sys/class/gpio/gpio14/edge", O_WRONLY);
	if (echoEdgePinFd < 0) {
		printf("\n gpio14 edge open failed");
	}

	//Prepare poll fd structure
	PollEch.fd = echoValuePinFd;
	PollEch.events = POLLPRI|POLLERR;
	PollEch.revents = 0;
	while(1) {
		lseek(echoValuePinFd, 0, SEEK_SET);
		write(echoEdgePinFd,"rising",6);
		write(triggerPinValueFd,"0",1);
		write(triggerPinValueFd,"1",1);
		usleep(12);
		write(triggerPinValueFd,"0",1);
		returnValue = poll(&PollEch,1,2000);
		if (returnValue > 0) {
			if (PollEch.revents & POLLPRI) {
				startTime = rdtsc();
				read(echoValuePinFd,echoValue,1);
				PollEch.revents=0;
			}
		}
		lseek(echoValuePinFd, 0, SEEK_SET);
		write(echoEdgePinFd,"falling",7);
		returnValue = poll(&PollEch,1,2000);
		if (returnValue > 0) {
			if (PollEch.revents & POLLPRI) {
				stopTime = rdtsc();
				read(echoValuePinFd,echoValue,1);
				PollEch.revents=0;
			}
		}
		time_diff = stopTime-startTime;
		globalDistance = time_diff*4.25/100000;
		//Test distance = (pulse width * ultrasonic spreading velocity in air) / 2
		//Referring the data sheet , we get that the distance is (pulse_width* velocity of air)/2
		//pulse width here is difference in system ticks , generated at start time and stop time.
		//1 tick = 1/400MHz(Frequency of Intel Galileo board)
		//1 tick = 0.25 * 10^-8 sec
		//distance = (diff in ticks)* 0.25*10^-8 * 340 (m/sec) / 2
		// Therefore distance = diff_in_time_ticks * 4.25/100000

		if (globalDistance < 0) {
			printf( "\n Please take the object away, to a minimum distance of 10 cms.\n");
		}
		if (globalDistance > 0 && globalDistance < 60) {
			delay_and_dir(globalDistance);
		}
		printf(" \n global  %lf in cm \n" , globalDistance);
		usleep(300000);
	}
	return 0;
}
/**
 * This method is used to unexport pins whenever we
 * press Ctrl+c / Ctrl +z keys.
 */
void handling_func_for_ctrl_c() {
	int i=0;
	uint8_t tx5[]= {

			0x0c,0x01,
			0x01,0x00,
			0x02,0x00,
			0x03,0x00,
			0x04,0x00,
			0x05,0x00,
			0x06,0x00,
			0x07,0x00,
			0x08,0x00
	};
	//close(spi_dev_matrix_fd);

	struct spi_ioc_transfer spi_msg2 =
	{
			.tx_buf = (unsigned long)tx,
			.rx_buf = 0,
			.len = 2,
			.delay_usecs = 1,
			.speed_hz = 10000000,
			.bits_per_word = 8,
			.cs_change = 1,
	};

	for(i = 0 ; i < 18; i+=2) {
		tx[0] = tx5[i];
		tx[1] = tx5[i+1];
		gpio_set_value(15 ,0);
		ioctl(fd_spi, SPI_IOC_MESSAGE (1), &spi_msg2);
		gpio_set_value(15 ,1);
	}
	gpio_set_value(15,0);
	usleep(1000);
	gpio_unexport(24);
	gpio_unexport(44);
	gpio_unexport(72);
	gpio_unexport(15);
	gpio_unexport(42);
	gpio_unexport(30);
	gpio_unexport(46);
	gpio_unexport(13);
	gpio_unexport(14);
	gpio_unexport(34);
	gpio_unexport(77);
	gpio_unexport(16);
	gpio_unexport(76);
	gpio_unexport(64);
	close(fd_spi);
	printf("\n Bye \n");
}

int main(int argc, char** argv)
{
	//This is used to handle the ctrl+c/ctrl+z keys
	signal(SIGUSR1, handling_func_for_ctrl_c);
	printf("\n Entering main function \n");
	pthread_t led_display_thread;
	pthread_t distanceMeasurement;
	fd_spi = open(DEVICE,O_RDONLY);
	if(fd_spi < 0)
		printf("open failed\n");
	pinSetup();
	int spi_ret1 = pthread_create(&distanceMeasurement,NULL,distanceMeasure,NULL);
	if (spi_ret1 != 0)
		printf("\ncould not create the thread for measuring distances \n");
	int spi_ret = pthread_create(&led_display_thread, NULL, display_init, NULL);

	if (spi_ret != 0)
		printf("\ncould not create the thread for display in spi device\n");
	pthread_join(led_display_thread, NULL);
	pthread_join(distanceMeasurement, NULL);
	close(fd_spi);
	return 0;
}


