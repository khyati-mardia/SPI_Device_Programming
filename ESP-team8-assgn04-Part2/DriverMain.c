#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L
#define _BSD_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <sched.h>
#include <unistd.h>
#include <linux/kernel.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <string.h>
#include <pthread.h>
#include <stdint.h>
#include <inttypes.h>
#include <signal.h>
#include "rdtsc.h"
#define SPI_DEVICE "/dev/spidev1.0"
#include <time.h>
#include <poll.h>
#include "Gpio_func.c"
double globalDistance; //The distance that the object is present.
double dist_measured,dist_measured_p=0;  // Current measured distance, Previous distance.
int spi_dev_matrix_fd; // the File descriptor for opening spi device.
int flag_for_direction = 1; // Flag for flipping the images .
pthread_mutex_t lock1 = PTHREAD_MUTEX_INITIALIZER;  //Mutex lock.
uint8_t animation_sequence[16];
uint8_t animation_sequence1[16] = {0,250,0,250,1,250,2,250,3,250,4,250,9,250,0,0};
//This sequence consists of Hi followed by right arrow in different directions followed by BYE.
uint8_t animation_sequence2[16] = {0,250,0,250,5,250,6,250,7,250,8,250,9,250,0,0};
//This sequence consists of Hi followed by left arrow in different directions followed by BYE.

uint8_t array_images[10][24]= {
		{    //HI
				0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x04,
				0x0B, 0x07,
				0x01,0x00,
				0x02,0x7C,
				0x03,0x10,
				0x04,0x7C,
				0x05,0x44,
				0x06,0x7C,
				0x07,0x44,
				0x08,0x00,
		},
		{    //Right Straight arrow1
				0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x04,
				0x0B, 0x07,
				0x01,0x00,
				0x02,0x00,
				0x03,0x04,
				0x04,0x02,
				0x05,0x3F,
				0x06,0x02,
				0x07,0x04,
				0x08,0x00,
		},
		//Right angled arrow
		{		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x04,
				0x0B, 0x07,
				0x01,0x00,
				0x02,0x00,
				0x03,0x20,
				0x04,0x10,
				0x05,0x08,
				0x06,0x05,
				0x07,0x03,
				0x08,0x07,
		},
		//Right curved arrow
		{ 		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x0F,
				0x0B, 0x07,
				0x01,0x00,
				0x02,0x00,
				0x03,0x00,
				0x04,0x20,
				0x05,0x10,
				0x06,0x50,
				0x07,0x60,
				0x08,0x70,
		},
		//Right sleeping arrow
		{		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x0F,
				0x0B, 0x07,
				0x01,0x00,
				0x02,0x00,
				0x03,0x00,
				0x04,0x40,
				0x05,0x40,
				0x06,0x40,
				0x07,0xE0,
				0x08,0x40,
		},
		// Left Straight arrow
		{		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x0F,
				0x0B, 0x07,
				0x01,0x00,
				0x02,0x04,
				0x03,0x02,
				0x04,0x3F,
				0x05,0x02,
				0x06,0x04,
				0x07,0x00,
				0x08,0x00,
		},
		// Left angled arrow
		{		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x0F,
				0x0B, 0x07,
				0x01,0x07,
				0x02,0x03,
				0x03,0x05,
				0x04,0x08,
				0x05,0x10,
				0x06,0x20,
				0x07,0x00,
				0x08,0x00,
		},
		//Left curved arrow
		{		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x0F,
				0x0B, 0x07,
				0x01,0x70,
				0x02,0x60,
				0x03,0x50,
				0x04,0x10,
				0x05,0x20,
				0x06,0x00,
				0x07,0x00,
				0x08,0x00,
		},
		// Left sleeping arrow
		{		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x0F,
				0x0B, 0x07,
				0x01,0x40,
				0x02,0xE0,
				0x03,0x40,
				0x04,0x40,
				0x05,0x40,
				0x06,0x00,
				0x07,0x00,
				0x08,0x00,
		},
		// Bye
		{		0x0C, 0x01,
				0x09, 0x00,
				0x0A, 0x0F,
				0x0B, 0x07,
				0x01,0x7C,
				0x02,0x54,
				0x03,0x28,
				0x04,0x0C,
				0x05,0x70,
				0x06,0x0C,
				0x07,0x7C,
				0x08,0x54,
		},
};

/* This function is for determining the direction of the direction and the speed of the display
 * If the object is moving away from sensor, we send one pattern
 * And if the object is moving towards the sensor, we send another pattern.
 * If the difference in distances is significant enough upto 3 cms, is only when we
 * record that distance as the correct one.
 */
void delay_and_dir(unsigned int dist ){
	int i=0, j=0;
	dist_measured =dist;
	double dist_interval= dist_measured - dist_measured_p;
	if (dist_interval > 3 ) {
		flag_for_direction = 1;
		for(i=0;i<16;i++)
		{
			animation_sequence[i] = animation_sequence1[i];
		}
		printf(" \n Object moving away from sensor!!!! \n");
		dist_measured_p = dist_measured;
	}
	else if (dist_interval < -3 ){
		flag_for_direction = 0;
		printf(" \n Object moving towards the sensor!!!! \n");
		for(j=0;j<16;j++)
		{
			animation_sequence[j] = animation_sequence2[j];
		}
		dist_measured_p = dist_measured;
	}
}

/**
 * This method is for displaying images.
 * Based on the values set by dist_and_direction method , we set the animation sequence.
 * This animation sequence is written to the LED matrix continuously
 * 0x0F, 0x00, // Display Test Register.// No LEDs ON
   0x0C, 0x01, // Shutdown Register //Digit 1 : Normal Mode of operation
   0x09, 0x00, // Decode Mode	// No decode operation for DIgits 0-7
   0x0A, 0x0F, // Intensity // Display Test 0xXF , maximum intensity of LEDs on.
   0x0B, 0x07, // Scan Limit // Displays Digits from 0 to 7
   0x01, 0x06, // Digit 0 register value//
   0x02, 0x06, // Digit 1 register value//
   0x03, 0x06, // Digit 2 register value//
   0x04, 0x83, // Digit 3 register value//
   0x05, 0x7F, // Digit 4 register value//
   0x06, 0x24, // Digit 5 register value//
   0x07, 0x22, // Digit 6 register value//
   0x08, 0x63, // Digit 7 register value//
 */


void* display_init(void* dummy){
	printf("\n Entering display_function \n");
	int ioctlCmdRetVal,writeCmdRetVal;
	spi_dev_matrix_fd = open(SPI_DEVICE, O_RDWR); //Opening spi device.
	printf("\n Opening the SPI Device folder \n");
	if(spi_dev_matrix_fd<0) {
		printf("opening error");
	}
	ioctlCmdRetVal=ioctl(spi_dev_matrix_fd,1,array_images); // Writing IOCTL command.
	//Sending all the data of 10 images that need to be
	printf("\n IOCTL command issuing from User space \n ");
	if(ioctlCmdRetVal<0){
		printf(" \n error in sending an ioctl message of 10 patterns \n \n ");
	}
	while(1) {
		sleep(1);
		//Writing the animation sequence to the LED.
		writeCmdRetVal=write(spi_dev_matrix_fd,(void*)animation_sequence , sizeof(animation_sequence));
		if(writeCmdRetVal<0){
			printf("error in writing Animation Sequence \n");
			break;
		}
	}

	close(spi_dev_matrix_fd);
	return 0;
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
	printf("\n Entered display functioon \n");
	int triggerPinValueFd,echoEdgePinFd,echoValuePinFd, returnValue;
	struct pollfd PollEch ;
	double startTime=0, stopTime=0, time_diff =0;
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
		printf(" \n start time %lf  \n ", startTime);
		printf(" \n stop time %lf  \n ", stopTime);
		printf(" \n global  %lf in cm \n" , globalDistance);
		usleep(400000);
		printf("\n \n \n DONE \n \n \n");
	}

	return 0;

}
/**
 * Unexporting the pins on Ctl+c / Ctrl+z command.
 */
void handling_func_for_ctrl_c() {
	close(spi_dev_matrix_fd);
	gpio_unexport(13);
	gpio_unexport(14);
	gpio_unexport(34);
	gpio_unexport(77);
	gpio_unexport(16);
	gpio_unexport(76);
	gpio_unexport(64);
	printf("\n Bye \n");
	exit(0);
}
/**
 * Initialising the Ultasonic sensor pins as the LED Matrix pins are alreaady getting initialised
 * in kernel space..
 *	IO 2 and IO3 pins are being initialised.
 */
void init_gpio_pins_display() {
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
int main()
{
	//Handling Ctrl+z function using Signal Handler.
	signal(SIGTSTP, handling_func_for_ctrl_c);
	//Distance Measurement Thread
	pthread_t distance_measurement;
	//Thread for displaying Pics
	pthread_t led_display_thread;
	printf("Entering main method \n");
	init_gpio_pins_display();
	int dist_measure_thrd = pthread_create (&distance_measurement, NULL, &distanceMeasure, NULL);
	if (dist_measure_thrd != 0) {
		printf("\ncould not create the thread for distance measurement in spi device\n");
	}
	int spi_ret = pthread_create(&led_display_thread, NULL, &display_init, NULL);
	if (spi_ret != 0)
		printf("\ncould not create the thread for display in spi device\n");

	pthread_join(distance_measurement, NULL);
	pthread_join(led_display_thread, NULL);
	return 0;
}



