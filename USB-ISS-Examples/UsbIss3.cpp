/************************************************
*Linux example code for USBI2C and SRF02	*
*                                               *
*Compiled using gcc				*
*Tested on ubuntu 10.4 LTS			*
*						*
*Opens a port for readinn and writing to the    *
*USBI2C performs a ranging and reads back the   *
*software version of the SRF02, the light data  *
*and the range taken.                           *
*                                               *
*As the SRF02, SRF10 and SRF235 have the same   *
*registers this example will work for them all. *
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
int range;
int ver;

	printf("***********************************\n");
	printf("USBI2C and SRF02 linux example code\n");
	printf("By James Henderson, 2010.          \n");
	printf("***********************************\n\n");

	fd = openPort();						// Open port to USBi2C	
	
	if(fd > P_ERROR)						// Only do the folloing if there has not been an error opening the port
	{
		sbuf[0] = 0x5a;						// USBI2C command for R/W 1 byte address devices
		sbuf[1] = 0x01;						// Address of SRF02
		sbuf[2] = 0x00;						// Command register of SRF02
		sbuf[3] = 0x01;						// 1 command byte to follow
		sbuf[4] = 0x51;						// Perform ranging, result in CM

		writeData(fd, 2);					// Write these 5 bytes to port

		readData(fd, 3);					// Read back the returned byte into sbuf[0] this should be non 0 for success
		if(!sbuf[0])						// If it is 0 report an error
		{
			printf("USBI2C error writing to SRF02 \n");
		}	
                else
                    printf("%x %x %x \n",sbuf[0],sbuf[1],sbuf[2]);
		usleep(750000);						// Wait for the ranging to finish

		/*sbuf[0] = 0x55;						// USBI2C command for R/W 1 byte address devices
		sbuf[1] = 0xE1;						// Address of SRF02 with R/W bit set
		sbuf[2] = 0x00;						// Register we want to start reading from (0 is software revision)
		sbuf[3] = 0x01;						// Read for 1 byte

		writeData(fd, 4);					// Send these 4 bytes to USBI2C
	
		readData(fd,1);						// Read back 1 byte, software version
	
		ver = sbuf[0];						// Store the software version

		sbuf[0] = 0x55;						// USBI2C command for R/W 1 byte address devices
		sbuf[1] = 0xE1;						// Address of SRF02 with R/W bit set
		sbuf[2] = 0x02;						// Register we want to start reading from (2 is start of range data)
		sbuf[3] = 0x02;						// Read for 2 byte

		writeData(fd, 4);					// Send these 4 bytes to USBI2C
	
		readData(fd,2);						// Read back 2 bytes for the range
	
		range = (sbuf[0] << 8) + sbuf[1];			// Calculate range	*/	

		closePort(fd);						// Close the port
	
		printf("SRF02 v: %u\nRange = %u\n\n", ver, range);
	}
	return 0;	
}


int openPort(void)
{
int fd;										// File descriptor for the port

	fd = open("/dev/ttyACM4", O_RDWR | O_NOCTTY );				// Open port for read and write not making it a controlling terminal
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
	if(bytes == P_ERROR)							// If read returns and error (-1)
	{
		perror("readData: Error while trying to read data - ");
	}
	else if(bytes != nbytes)
	{
		printf("Only %u bytes read out of %u requested\n", bytes, nbytes);
	}
}

