/************************************************
*Linux example coder for USBI2C and TPA81	*
*						*
*Compiled using gcc				*
*Tested on Ubuntu 10.4 LTS			*
*						*
*Opens up a port to the USBI2C and reads the 	*
*TPA81 software version, ambiant temperature and*
* all 8 pixel temperature readings.             *
*						*
*By James Henderson, 2010.			*
************************************************/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define P_ERROR	-1						// to check for errors when dealing with the port

int openPort(void);
void closePort(int fd);
void writeData(int fd, int nbytes);
void readData(int fd, int nbytes);

struct termios options;

unsigned char sbuf[10];						// Stores data to be read and written

int main(int argc, char *argv[])
{
int fd;								// file descriptor of open port

	printf("***********************************\n");
	printf("USBI2C and TPA81 linux example code\n");
	printf("By James Henderson, 2010.          \n");
	printf("***********************************\n\n");

	fd = openPort();						// Open port to USBi2C	
	
	if(fd > P_ERROR)						// Only do the folloing if there has not been an error opening the port
	{
		
		sbuf[0] = 0x55;						// USBI2C command for single byte address read write
		sbuf[1] = 0xD1;						// Address of TPA81 with read write bit set
		sbuf[2] = 0x00;						// Register we wish to start reaing from
		sbuf[3] = 0x0A;						// Read for 10 bytes
		
		writeData(fd, 4);					// Write this data to the USBI2C
	
		usleep(15000);		

		readData(fd, 10);					// Read back the 10 bytes.

		closePort(fd);						// Close opened port
		
		printf("TPA81 v : %u\n", sbuf[0]);			// Display data to the screen
		printf("Ambiant : %u\n", sbuf[1]);
		printf("Temp 1  : %u\n", sbuf[2]);
		printf("Temp 2  : %u\n", sbuf[3]);
		printf("Temp 3  : %u\n", sbuf[4]);
		printf("Temp 4  : %u\n", sbuf[5]);
		printf("Temp 5  : %u\n", sbuf[6]);
		printf("Temp 6  : %u\n", sbuf[7]);
		printf("Temp 7  : %u\n", sbuf[8]);
		printf("Temp 8  : %u\n\n", sbuf[9]);
	}
	return 0;
}


int openPort(void)
{
int fd;										// File descriptor for the port

	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );				// Open port for read and write not making it a controlling terminal
	if (fd == P_ERROR)
	{
		perror("openPort: Unable to open /dev/ttyUSB0 - ");		// If open() returns an error
	} 
	else
	{
		fcntl(fd, F_SETFL, 0);						// Get the current options for the port
		tcgetattr(fd, &options);
		
		cfsetispeed(&options, B19200);					// Set the baud rates to 19200

		options.c_cflag |= (CLOCAL | CREAD);				// Enable the receiver and set local mode
		options.c_cflag &= ~PARENB;					// No parity bit

		options.c_cflag &= ~CSTOPB;					// Set 2 stop bits

		options.c_cflag &= ~CSIZE;					// Set the character size
		options.c_cflag |= CS8;

		tcsetattr(fd, TCSANOW, &options);				// Set the new options for the port
	}
	return (fd);
}


void closePort(int fd)
{	
	if(close(fd) == P_ERROR)						// Close the port if it returns an error then display an error report
	{	
		perror("closePort: Unable to close /dev/ttyUSB0 - ");
	}
}


void writeData(int fd, int nbytes)
{
int bytes;
	
	bytes = write(fd, sbuf, nbytes);					// Write nbytes of data from wbuf
	if(bytes == P_ERROR)							// If write returns an error (-1)
	{
		perror("writeData: Error while trying to write data - ");	
	}
	else if(bytes != nbytes)
	{
		printf("only %u bytes written out of %u requested\n", bytes, nbytes);	
	}

}


void readData(int fd, int nbytes)
{
int bytes;

	bytes = read(fd, sbuf, nbytes);						// Read nbytes of data into rbuf
	if(bytes == P_ERROR)							// If read returns an error (-1)
	{
		perror("readData: Error while trying to read data - ");
	}
	else if(bytes != nbytes)
	{
		printf("Only %u bytes read out of %u requested\n", bytes, nbytes);
	}
}

