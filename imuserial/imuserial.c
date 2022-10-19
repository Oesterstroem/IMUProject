/** \file imuserial.c
 *  \ingroup hwmodule
 *  \brief Serial Module
 *
 * ImuSerial Module for RHD. 
 * 
 */ 
/***************************************************************************
 *                  Copyright 2015 DTU (Christian Andersen)
 *                  jca@elektro.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/***************************** Plugin version  *****************************/
#define IMUSERIALVERSION 	      "1.0"
/*********************** Version control information ***********************/
#define REVISION         "$Rev: 59 $:"
#define DATE             "$Date: 2012-01-14 09:49:21 +0100 (Sat, 14 Jan 2012) $:"
#define ID               "$Id: gps.c 59 2012-10-21 06:25:02Z jcan $"
/***************************************************************************/




#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <netdb.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <expat.h>
#include <math.h>
#include <semaphore.h>
#include <ctype.h>


#include <database.h>
#include <globalfunc.h>

#include "imuserial.h"


//Definitions
int  Dev;      ///IMU Port file pointer
char DevString[64]; ///String to hold IMU device
int  baudrate = 0;
static volatile char stop = 0;
///Database indexes
// Threads are being defined
pthread_t thread_read;
pthread_t thread_decode;
pthread_attr_t attr;
//pthread_mutex_t decodeFlag; // pthread_mutex_lock
sem_t decodeFlag;

/**
 *10 buffers for MNEA messages, each 512 bytes long */
#define MaxBufSize 512
#define MaxBufCnt 20									/*AGCO*/
char buf[MaxBufCnt][MaxBufSize];
int bufIdx = -1; /* index to completed buffer */

int iDist; // read variable
int iMode; // write variable
int imuDistmm = 0; // distance
int imuMode = 0;   // 0=fast mode 1= raw mode

//Function prototypes
int initDevice(void);
int set_serial(int fd, int baud);
void *task_read(void *);
void decodeWait(void);
void decodePost(void);

int debug = 0;

/** \brief Initialize IMU
 * 
 * \returns int status
 * Status of the server thread - negative on error.
 */
int initDevice(void) {
  // Open RS232 port
  
  Dev = open(DevString, O_RDWR);
  if (Dev<0) 
  {
    fprintf(stderr,"   IMU: Error opening %s\n",DevString);
    return -1;
  }
  //Set baudrate for IMU receiver
  if (set_serial(Dev,baudrate) == -1)  {
    fprintf(stderr,"   IMU: Can't set IMU serial port parameters\n");
    return -1;
  }
  stop = 0;
  
  //Create variables
  IMU.acl = createVariable('r',3, "IMUacl");
  IMU.mag = createVariable('r',3, "IMUmag");
  IMU.gyr = createVariable('r',3, "IMUgyr");
  //   IMU.bar = createVariable('r',2, "IMUbar");
  
  // Initialization and starting of threads
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  
  if (pthread_create(&thread_read, &attr, task_read, 0))
  {
    perror("   IMU: Can't start IMU read thread");
    return -1;
  }
  return 1;
}

/** \brief Initialize Shut down IMU rx thread
 * \returns int status
 * Status of the server thread - negative on error.
 */
int terminate(void) 
{ // stop thread
  stop = 1;
  pthread_join(thread_read,NULL);
  return 1;
}


/**
 * This is called in RHD main thread at every sample period
 * */
extern int periodic(int rhdTick)
{
  int returnValue = !stop;
  if (isUpdated('w', iMode) || rhdTick == 2)
  {
    int m = getWriteVariable(iMode, 0);
    if (m != imuMode)
    { // change to text mode and either Fast or Precise
      secureWrite(Dev, "T\n", 2);
      switch(m)
      {
	case 0: 
	  secureWrite(Dev, "P\n", 2);
	  break;
	case 1: 
	  secureWrite(Dev, "F\n", 2);
	  break;
	default:
	  break;
      }
      imuMode = m;
    }
  }
  return returnValue;
}


/** \brief IMU RX thread.
 */
void *task_read(void *not_used) {
  int i = -1;
  #define MBL 100
  char bufRx[MBL], *bufNext;
  char valStr[MBL] = {'\0'}; 
  int sensor = 0, value = 0;
  int sign = 1;
  //Recieve from IMU
  printf("   IMU: Receive thread started\n");
  bufRx[0] = '\0';
  //int misCnt = 0;
  while(!stop)
  {
    sensor++;
    bufNext = bufRx;
    *bufNext = '\0';
    while (bufNext - bufRx < MaxBufSize - 1 && !stop)
    { /* space for more data */
      i = read(Dev, bufNext, 1);
      if (i <= 0)
      { /* read error */
	stop = 1;
	fprintf(stderr,"IMU: Error reading from Serial port, shutting down\n");
	break;
      }
      
      //Parse serial data to RHD global variables
      if (isdigit(bufRx[0])){
	strcat(valStr,bufNext);
      }
      if (*bufNext == ' '){
	int mm = strtol(valStr, NULL, 10);
	
	switch(sensor)
	  {
	    case 1:
	      setVariable(IMU.acl,value, mm*sign);
	      if (debug) printf("%d ",mm);
	      break;
	    case 2:
	      setVariable(IMU.mag,value, mm*sign);
	      if (debug) printf("%d ",mm);
	      break;
	    case 3:
	      setVariable(IMU.gyr,value, mm*sign);
	      if (debug) printf("%d ",mm);
	      break;  
	  }
	value++;
	sign = 1;
	
	
	*valStr = '\0';
      }
      else if (*bufNext == '-'){
	sign = -1;
      }
      else if (*bufNext == ';'){
	if (debug) printf("; ");
	sensor++;
	value = 0;
	//*valStr = '\0';
      }
      else if (*bufNext == '\n'){
	if (debug) printf("\n");
	value = 0;
	sensor = 1;
	//*valStr = '\0';
      }
      

    }
    bufNext = bufRx;
  } //Ending IMU loop
  close(Dev);
  fprintf(stderr,"IMU: Shutdown IMU read task\n");
  pthread_exit(0);
}



/************************** XML Initialization **************************/
///Struct for shared parse data
typedef struct  {
  int depth;
  char skip;
  char enable;
  char found;
}parseInfo;

//Parsing functions
void XMLCALL xmlStartTag(void *, const char *, const char **);
void XMLCALL xmlEndTag(void *, const char *);

/** \brief Initialize the IMU HAL
 * 
 * Reads the XML file and sets up the IMU settings
 * 
 * Finally the rx threads is started and the driver 
 * is ready to read data
 * 
 * \param[in] *char filename
 * Filename of the XML file
 * 
 * \returns int status
 * Status of the initialization process. Negative on error.
 */
int initXML(char *filename) {
  
  parseInfo xmlParse; 
  char *xmlBuf = NULL;
  int xmlFilelength;
  int done = 0;
  int len;
  FILE *fp;
  
  //Print initialization message
  //Find revision number from SVN Revision
  char *i,versionString[20] = REVISION, tempString[10];
  i = strrchr(versionString,'$');
  strncpy(tempString,versionString+6,(i-versionString-6));
  tempString[(i-versionString-6)] = 0;
  printf("IMU: Initializing Serial IMU HAL %s.%s\n",IMUSERIALVERSION,tempString);
  
  
  /* Initialize Expat parser*/
  XML_Parser parser = XML_ParserCreate(NULL);
  if (! parser) {
    fprintf(stderr, "IMU: Couldn't allocate memory for XML parser\n");
    return -1;
  }
  
  //Setup element handlers
  XML_SetElementHandler(parser, xmlStartTag, xmlEndTag);
  //Setup shared data
  memset(&xmlParse,0,sizeof(parseInfo));
  XML_SetUserData(parser,&xmlParse);
  
  //Open and read the XML file
  fp = fopen(filename,"r");
  if(fp == NULL)
  {
    printf("IMU: Error reading: %s\n",filename);
    return -1;
  }
  //Get the length of the file
  fseek(fp,0,SEEK_END);
  xmlFilelength = ftell(fp); //Get position
  fseek(fp,0,SEEK_SET); //Return to start of file
  
  //Allocate text buffer
  xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
  if (xmlBuf == NULL) {
    fprintf(stderr, "   Couldn't allocate memory for XML File buffer\n");
    return -1;
  }
  memset(xmlBuf,0,xmlFilelength);
  len = fread(xmlBuf, 1, xmlFilelength, fp);
  fclose(fp);
  
  //Start parsing the XML file
  if (XML_Parse(parser, xmlBuf, len, done) == XML_STATUS_ERROR) {
    fprintf(stderr, "imuserial: XML Parse error at line %d: %s\n",
	    (int)XML_GetCurrentLineNumber(parser),
	    XML_ErrorString(XML_GetErrorCode(parser)));
    return -1;
  }
  XML_ParserFree(parser);
  free(xmlBuf);
  
  //Print error, if no XML tag found
  if (xmlParse.found <= 0) {
    printf("   Error: No imuserial XML tag found in plugins section\n");
    return -1;
  }
  
  //Start crossbow thread after init
  if (xmlParse.enable) 
    done = initDevice();
  return done;
}

// found a start tag - all info is in start tags.

void XMLCALL
xmlStartTag(void *data, const char *el, const char **attr)
{
  int i;
  parseInfo *info = (parseInfo *) data;
  info->depth++;
  
  //Check for the right 1., 2. and 3. level tags
  if (!info->skip) {
    if (((info->depth == 1) && (strcmp("rhd",el) != 0)) || 
      ((info->depth == 2) && (strcmp("plugins",el) != 0)) ||
      ((info->depth == 3) && (strcmp("imuserial",el) != 0))) 
    {
      info->skip = info->depth;
      return;
    } 
    else if (info->depth == 3) 
      info->found = 1;
  } 
  else 
    return;
  
  //Branch to parse the elements of the XML file.
  if (!strcmp("imuserial",el)) 
  { //Check for the correct depth for this tag
    for(i = 0; attr[i]; i+=2) if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0)) 
    {
      info->enable = 1; 
    }
    if (!info->enable) 
    {
      printf("   IMU: Use of IMU disabled in configuration\n"); 
      info->skip = info->depth;
    }
    for(i = 0; attr[i]; i+=2) if ((strcmp("debug",attr[i]) == 0) && ((strcmp("true",attr[i+1]) == 0) || (strcmp("1",attr[i+1]) == 0))) 
    {
      debug = 1;
    }
  } 
  else if (strcmp("serial",el) == 0) 
  {
    //Check for the correct depth for this tag
    if(info->depth != 4) 
    {
      printf("Error: Wrong depth for the %s tag\n",el);
    }
    for(i = 0; attr[i]; i+=2) if (strcmp("port",attr[i]) == 0) strncpy(DevString,attr[i+1],63); 
    for(i = 0; attr[i]; i+=2) if (strcmp("baudrate",attr[i]) == 0) baudrate = atoi(attr[i+1]); 
    printf("   IMU: Serial port %s at %d baud\n",DevString,baudrate);
  } 
  
}

// XML tag end

void XMLCALL
xmlEndTag(void *data, const char *el)
{
  parseInfo *info = (parseInfo *) data;
  info->depth--;
  
  if (info->depth < info->skip) info->skip = 0;
}

