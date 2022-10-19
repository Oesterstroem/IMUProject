/*******************************************************
* USB-ISS test application for Linux.                  *
*                                                      *
* Sets USB-ISS into I2C mode and gets range and light  *
* readings from am SRF08.                              *
*                                                      *
* Compiled using  gcc, tested on Ubunto 10.4 LTS.      *
*                                                      *
* By James Henderson, 2012.                            *
*******************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>

void display_version(void);		// Read and display USB-ISS module information
void set_i2c_mode(void);		// Set the USB-ISS into I2C mode, 100KHz clock
void do_range(void);			// Make the SRF08 perform a ranging
void get_range(void);			// Read the light and range data back and display to screen

int fd;
unsigned char sbuf[20];			// serial buffer for r/w
unsigned char error = 0x00;		// Byte used to indicate errors

int main(int argc, char *argv[])
{
	printf("******************************\n");
	printf("USB ISS linux test application\n");
	printf("By James Henderson, 2012\n");	
	printf("******************************\n");

	if(argc != 1)
		printf("** Incorrect input! **\n\n");
	else
	{
		struct termios defaults;							// to store innitial default port settings
		struct termios config;								// These will be our new settings
		const char *device = "/dev/ttyACM4";
		fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
		if(fd == -1) {
  			printf( "failed to open port\n" );
		} else {
			if (tcgetattr(fd, &defaults) < 0) perror("tcgetattr");  		// Grab snapshot of current settings  for port
			cfmakeraw(&config);							// make options for raw data
			if (tcsetattr(fd, TCSANOW, &config) < 0) perror("tcsetattr config");   	// Set options for port
		
	 		display_version();
			set_i2c_mode();
			if(!error) do_range();
			if(!error) get_range();
			
			if (tcsetattr(fd, TCSANOW, &defaults) < 0) perror("tcsetattr default");	// Restore port default before closing
		}
		close(fd);
	}
	return 0;
}	

void display_version(void)
{
	sbuf[0] = 0x5A; 						// USB_ISS byte
	sbuf[1] = 0x01;							// Software return byte

	if (write(fd, sbuf, 2) < 0) perror("display_version write");	// Write data to USB-ISS
	if (tcdrain(fd) < 0) perror("display_version tcdrain");
	if (read(fd, sbuf, 3) < 0) perror("display_version read");	// Read data back from USB-ISS, module ID and software version

	printf("USB-ISS Module ID: %u \n", sbuf[0]);
	printf("USB-ISS Software v: %u \n\n", sbuf[1]);
}

void set_i2c_mode(void)
{
	sbuf[0] = 0x5A;							// USB_ISS command
	sbuf[1] = 0x02;							// Set mode
	sbuf[2] = 0x40;							// Set mode to 100KHz I2C
	sbuf[3] = 0x00;							// Spare pins set to output low

	if (write(fd, sbuf, 4) < 0) perror("set_i2c_mode write");	// Write data to USB-ISS
	if (tcdrain(fd) < 0) perror("set_i2c_mode tcdrain");
	if (read(fd, sbuf, 2) < 0) perror("set_i2c_mode read");		// Read back error byte
	if(sbuf[0] != 0xFF)						// If first returned byte is not 0xFF then an error has occured
	{
		printf("**set_i2c_mode: Error setting I2C mode!**\n\n");
		error = 0xFF;						// Set error byte
	}
}

void do_range(void)
{
	sbuf[0] = 0x55;							// Primary USB-ISS command
	sbuf[1] = 0xE0;							// Address of SRF08
	sbuf[2] = 0x00;							// Command register of SRF08
	sbuf[3] = 0x01;							// Number of data bytes to write
	sbuf[4] = 0x51;							// Data byte (peform range, result in cm)

	if (write(fd, sbuf, 5) < 0) perror("do_range write");		// Write data to USB-ISS
	if (tcdrain(fd) < 0) perror("do_range tcdrain");
	if (read(fd, sbuf, 1) < 0) perror("do_range read");		// Read back error byte

	if(sbuf[0] == 0x00)						// If 0x00 returned and error has occured
	{ 
		printf("**do_range: Error writing to I2C device!**\n\n");
		error = 0xFF;						// set error byte
		return;
	}

	printf("Performing ranging\n");
	usleep(750000);							// Wait for ranging to finish
}

void get_range(void)
{
int range, light;

	sbuf[0] = 0x55;							// Primary USB-ISS command
	sbuf[1] = 0xE1;							// Address of SRF08 with R/W bit high
	sbuf[2] = 0x01;							// Internal register of SRF08 to read from
	sbuf[3] = 0x03;							// Number of bytes we wish to read

	if (write(fd, sbuf, 4) < 0) perror("do_range write");		// Write data to USB-ISS
	if (tcdrain(fd) < 0) perror("do_range tcdrain");
	if (read(fd, sbuf, 3) < 0) perror("do_range read");		// Read back error byte

	light = sbuf[0];
	printf("\nLight reading: 0x%X\n",light);
	range = (sbuf[1] << 8) + sbuf[2];				// Calculate range
	printf("Range = %ucm\n\n",range);
}



