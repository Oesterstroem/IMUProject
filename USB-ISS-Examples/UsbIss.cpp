/************************************************
*Linux exapmle code for the CMPS09 and USBI2C	*
*						*
*Compiled using gcc.				*
*Tested on Ubuntu 10.4 LTS			*
*						*
*Opens a port to the USBI2C and uses it to read *
*data from the CMPS09.				*
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
int bearing;
signed char pitch, roll;

	printf("************************************\n");
	printf("USBI2C and CMPS09 linux example code\n");
	printf("By James Henderson, 2010.          \n");
	printf("************************************\n\n");

	fd = openPort();						// Open port to USBi2C	
	
	if(fd > P_ERROR)						// Only do the folloing if there has not been an error opening the port
	{
                printf("Trying to write to I2C\n");
		sbuf[0] = 0x5a;						// USBI2C command for single byte address device
		sbuf[1] = 0x01;						// CMPS09 address with R/W bit set high
// 		sbuf[2] = 0x20;						// Register we want to read from (0 is software version)
// 		sbuf[3] = 0x01;						

		writeData(fd, 2);					// Write these 4 bytes to USBI2C
	
		readData(fd, 3);					// Read back all data

// 		bearing = ((sbuf[2] << 8) + sbuf[3]) /10;		// Calculate bearing.
// 		pitch = sbuf[4];
// 		roll = sbuf[5];		
					
		printf("Buff[0]  : %u\n", sbuf[0]);			// Display all data to screen
                printf("Buff[1]  : %u\n", sbuf[1]);
                printf("Buff[2]  : %u\n", sbuf[2]);
// 		printf("Bearing as byte : %u\n", sbuf[1]);
// 		printf("Bearing         : %u\n", bearing);
// 		printf("Pitch           : %i\n", pitch);
// 		printf("Roll            : %i\n\n", roll);

		closePort(fd);						// Cloise port	
	}
	return 0;
}


int openPort(void)
{
int fd;										// File descriptor for the port

	fd = open("/dev/ttyACM3", O_RDWR | O_NOCTTY );				// Open port for read and write not making it a controlling terminal
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
		printf("only %u bytes written out of %u requested\n\n", bytes, nbytes);	
	}

}


void readData(int fd, int nbytes)
{
int bytes;

	bytes = read(fd, sbuf, nbytes);						// Read nbytes of data into rbuf
	if(bytes == P_ERROR)							// If read returns and error (-1)
	{
		perror("readData: Error while trying to read data - ");
	}
	else if(bytes != nbytes)
	{
		printf("Only %u bytes read out of %u requested\n\n", bytes, nbytes);
	}
}

