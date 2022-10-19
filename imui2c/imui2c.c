 /** \file imui2c.c
  *  \ingroup hwmodule
  *
  * interface to usb-to-i2c module and a MD25 mototcontroller (2xdc motor control with gearing and encoder)
  *
  *******************************************************************/
 /*********************** Version control information ***********************/
 #define REVISION         "$Rev: 1 $:"
 #define DATE             "$Date: 2015-06-04 NOON +0200 $:"
 #define ID               "$Id: imui2c.c 1 2015-06-04 12:00:00Z jcan $"
 /***************************************************************************/
 
 /*******************/
 //#define GYRO 0xD6
 #define ACCEL 0x32
 #define MAGNE 0x3C
 #define BAROM 0xEE
 
 #define DEBUG 1
 
 
 #define _1_MS 1000000
 #define _1_S 1000000000
 
 
 #define NEWCONSTANT 1
 
 /* Adefruit Sensor Constants */
 #define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
 #define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
 #define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
 #define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
 #define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
 #define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
 #define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
 #define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
 #define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */
 
 
 
 
 
 
 #include <sched.h>
 #include <pthread.h>
 #include <stdio.h>
 #include <string.h>
 #include <unistd.h>
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <fcntl.h>
 #include <termios.h>
 #include <unistd.h>
 #include <stdlib.h>
 #include <errno.h>
 #include <sys/ioctl.h>
 #include <signal.h>
 #include <linux/serial.h>
 #include <sys/time.h>
 #include <sys/mman.h>
 #include <expat.h>
 #include <poll.h>
 #include <math.h>
 
 //RHD Core headers
 #include <rhd.h>
 #include <smr.h>
 #include <database.h>
 #include <globalfunc.h>
 
 #include "imui2c.h"
 
 #define LS_READ_BYTES 20
 
 ///Struct for shared parse data
 typedef struct  {
   int depth; // current XML tag level
   int skip;  // skip from this level up
   char enable;
   char found;
 } parseInfo;
 
 /* prototypes */
 /**
  * Create exchange variables */
 void createI2Cvariables();
 /**
  * initialize plugin */
 int initUsbIss(void);
 /** poll to see if a read would not block.
  * \param fd is the device number
  * \param msTimeout is the maximum wait time
  * \returns 0 is no data is available and 1 if at least 1 byte is available */
 int pollDeviceRx(int fd, int msTimeout);
 /**
  * get data from device
  * \param minCnt is the minimum number of bytes to fetch
  * \param timeoutms is the timeout used when waiting for data
  * \param maxWaitCnt is the max number of poll cycles to wait for required amount of data
  * \returns number of data received */
 int getData(int minCnt, int timeoutms, int maxWaitCnt);
 /**
  * Get data from line until timeout.
  * \param dest is where data should be stored
  * \param destCnt is length of destination buffer
  * \param timeout is the max time to wait for data
  * \returns when 2 timeoutperiods has expired with no data or
  * when data is received (within two timeout periods) and there have been no new data within one timeout period or
  * when there is no more space or just one character left in destination buffer. */
 int getDataToTimeout(unsigned char * dest, int destCnt, int timeoutms);
 /** run thread for rx task */
 void * i2c_task(void *);
 /** Printout function **/
 void rxPrint();
 /** Read from i2c at addr **/
 ssize_t readI2C(char device, char addr, int bytesToRead);
 ssize_t writeI2C(char device, char addr, char data);
 ssize_t simple_readI2C(char device, char addr, char bytesToRead);
 ssize_t simple_writeI2C(char device, char addr, char data);
 ssize_t addr_chck_I2C(char device);
 /**
  * Parsing function for start tag */
 void XMLCALL lsStartTag(void *, const char *, const char **);
 /**
  * Parsing function for end tag */
 void XMLCALL lsEndTag(void *, const char *);
 /**
  * get index to a named variable - from another plugin
  * \param type is either 'r' or 'w'
  * \param name is the name of the variable
  * \returns -1 if the variable is not found, else the index (positive or zero) of the variable. */
 int getDatabaseVariable(char type, const char * name);
 /**
  * get floating point value from two integer values */
 double getVariableDouble(char type, int index);
 /**
  * Round to an integer value - also for negative values. */
 int roundi(const float v)
 { // round to closest integer
   if (v > 0)
     return (int)(v + 0.5);
   else
     return (int)(v - 0.5);
 }
 /**
  * Reset update flag for one read variable */
 void resetUpdateRead(int id);
 /** absolute value of long integers */
 int64_t i64abs(int64_t val)
 {
   if (val < 0)
     return -val;
   else
     return val;
 }
 /**
  * Get time since passed in us since this reference time
  * \param refTime reference time
  * \returns time passed in microseconds. */
 int32_t getTimePassed(struct timeval refTime)
 {
   struct timeval tv;
   //
   gettimeofday(&tv, NULL);
   int ds = tv.tv_sec - refTime.tv_sec;
   if (ds > INT32_MAX / 1000000)
     return INT32_MAX;
   else
   {
     ds = ds * 1000000 + tv.tv_usec - refTime.tv_usec;
     return ds;
   }
 }
 
 /******** Global variables *************/
 
 /// variables related to thread execution
 struct
 { /** is receive thread running */
   int running;
   /** Trigger polling of data from units - when set to 1 */
   int startNewRxCycle;
   /** name of serial device */
   /** thread handle and attributes */
   pthread_t i2c_thread;
 } rxtask;
 
 int tick = 0;
 struct timeval tickTime;
 int keepAlive = 0;
 int debugFlag = 0;
 FILE * logfile = NULL;
 FILE * logState = NULL;
 int dev_connected[4] = {0};
 
 
 // Constants
 float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
 float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
 float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain
 
 char GYRO = 0xD6;
 
 /** catch writes to a non existing file - allowed in debug mode */
 ssize_t secure2Write(const void *buf, ssize_t txLen)
 {
   if (busif.ttyDev >= 0)
     return secureWrite(busif.ttyDev, buf, txLen);
   else if (debugFlag)
     // perform as if all is written - debug mode
     return txLen;
   else
     return 0;
 }
 
 /////////////////////////////////////////////////////////
 
 /**
  * init new requests to bus (called periodically)
  * */
 extern int periodic(int rhdTick)
 {
   //unsigned char stuff[4] = {0x23, 0x20, 0x20, 0x0F};
   int returnValue = 1;
   //int i;
   if (rhdTick == 0)
   { // connect to joystick - if any
     printf("Entered Periodic first time\n");
     
     
     //
   }
   /*
    *      unsigned char setI2cMode[11] = {0x57, 0x01, 0x31, 0xD4, 0x0F, 0x02, 0x30, 0xD5, 0x04, 0x20, 0x03};
    *    secure2Write(setI2cMode,sizeof(setI2cMode));
    *    //usleep(20000);
    *    
    *    if (tick%1000 == 0){
    *      printf("Read:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", busif.rxBuf[0], busif.rxBuf[1],busif.rxBuf[2], busif.rxBuf[3], busif.rxBuf[4], busif.rxBuf[5], busif.rxBuf[6], busif.rxBuf[7]);
 }
 */
   
   
   
   tick = rhdTick;
   
   rxtask.startNewRxCycle = 1;
   pthread_mutex_unlock(&IMU.mLock);
   return returnValue;
 }
 
 //////////////////////////////////////////////////////
 
 int terminate(void)
 {
   printf("Entered terminate\n");
   rxtask.running = 0;
   printf("stopping IMUI2C ... ");
   fflush(stdout);
   pthread_join(rxtask.i2c_thread, NULL);
   close(busif.ttyDev);
   printf("[OK]\n");
   return 0;
 }
 
 /************************** XML Initialization **************************/
 
 
 /** \brief Initialize the USB to i2c bus and its units
  * 
  * Reads the XML file and initializes plugin
  * after successfully reading the XML file.
  *
  * \param[in] *char filename
  * Filename of the XML file
  *
  * \returns int status 1 is OK and -1 is error
  * Status of the initialization process. Negative on error.
  */
 extern int initXML(char *filename)
 {
   printf("Entered initXML\n");
   int result;
   parseInfo xmlParse;
   char *xmlBuf = NULL;
   int xmlFilelength;
   int done = 0;
   int len;
   FILE *fp;
   //md25.wheelBase = 0.203;
   //md25.steerBase = 0.255;
   //Print initialization message
   //Find revision number from SVN Revision
   char * p1;
   char versionString[20] = REVISION;
   char tempString[10];
   p1 = strrchr(versionString, '$');
   strncpy(tempString, &versionString[6],(p1 - versionString - 6));
   tempString[(p1 - versionString - 6)] = '\0';
   printf("imui2c: plug-in version %s\n", tempString);
   /* Initialize Expat parser*/
   XML_Parser parser = XML_ParserCreate(NULL);
   result = parser != 0;
   if (!result)
     fprintf(stderr, "IMUI2C: Couldn't allocate memory for XML parser\n");
   if (result)
   {  //Setup element handlers
     XML_SetElementHandler(parser, lsStartTag, lsEndTag);
     //Setup shared data
     memset(&xmlParse, 0, sizeof(parseInfo));
     //
     XML_SetUserData(parser, &xmlParse);
     
     //Open and read the XML file
     fp = fopen(filename,"r");
     result = fp != NULL;
     if(!result)
       printf("IMUI2C: Error reading: %s\n",filename);
   }
   if (result)
   { //Get the length of the file
     fseek(fp,0,SEEK_END);
     xmlFilelength = ftell(fp); //Get position
     fseek(fp,0,SEEK_SET); //Return to start of file
     //Allocate text buffer for full file length
     xmlBuf = realloc(xmlBuf,xmlFilelength+10); //Allocate memory
     result = (xmlBuf != NULL);
     if (!result)
       fprintf(stderr, "   IMUI2C: Couldn't allocate memory for XML File buffer\n");
   }
   if (result)
   { // clear buffer
     memset(xmlBuf,0,xmlFilelength);
     // read full file
     len = fread(xmlBuf, 1, xmlFilelength, fp);
     fclose(fp);
     //Start parsing the XML file
     result = (XML_Parse(parser, xmlBuf, len, done) != XML_STATUS_ERROR);
     if (!result)
       fprintf(stderr, "IMUI2C: XML Parse error at line %d: %s\n",
	       (int)XML_GetCurrentLineNumber(parser),
	       XML_ErrorString(XML_GetErrorCode(parser)));
   }
   if (parser != NULL)
     XML_ParserFree(parser);
   if (xmlBuf != NULL)
     free(xmlBuf);
   if (result && xmlParse.enable)
   { // all is fine - start plugin
     result = initUsbIss();
   }
   // this is variable in another plugin,
   // and can not be set just now.
   //md25.joybuttons = -1;
   //md25.joyaxes = -1;
   //md25.joySteer = -1;
   if (result)
     return 1;
   else
     return -1;
 }
 
 //////////////////////////////////////////////////
 
 /**
  * A start tag is detected by the XML parser
  * \param data is a user defined context pointer.
  * \param el is the tag name
  * \param attr is the list of attributes in the start tag. */
 void XMLCALL lsStartTag(void *data, const char *el, const char **attr)
 
 { // a start tag is detected
   int i;
   parseInfo *info = (parseInfo *) data;
   // detect context
   info->depth++;
   if (info->depth < info->skip || info->skip == 0)
   {
     switch (info->depth)
     {
       case 1:
	 if (strcmp("rhd",el) != 0)
	   info->skip = info->depth;
	 break;
       case 2:
	 if (strcmp("plugins",el) == 0)
	   ;
	 else
	   info->skip = info->depth;
	 break;
       case 3:
	 // this one handles imui2c only
	 if (strcmp("imui2c",el) == 0)
	 { // is it enabled, the only info needed
	   for(i = 0; attr[i]; i+=2)
	     if ((strcmp("enable",attr[i]) == 0) && (strcmp("true",attr[i+1]) == 0))
	       info->enable = 1;
	     if (!info->enable)
	       printf("   IMUI2C: Use is disabled in configuration\n");
	 }
	 else
	   info->skip = info->depth;
	 break;
       case 4:
	 // devices on i2c bus
	 if (strcmp("bus",el) == 0)
	 {
	   const char * att, * val;
	   for(i = 0; attr[i]; i+=2)
	   {
	     att = attr[i];
	     val = attr[i + 1];
	     if (strcmp("dev",att) == 0)
	       strncpy(busif.serialDev, val, MxDL);
	     else if (strcmp("id", att) == 0)
	       imui2c.busid = strtol(val, NULL, 0);
	     else if (strcmp("debug", att) == 0)
	     {
	       debugFlag = strtol(val, NULL, 0);
	       if (debugFlag)
		 printf("   IMUI2C started in DEBUG mode!\n");
	     }
	   }
	   info->found = 1;
	   printf("   IMUI2C: serial device to %s, busID 0x%x\n", busif.serialDev, imui2c.busid);
	 }
	 else
	   info->skip = info->depth;
	 break;
       default: // unknown tag series
	 break;
     }
   }
 }
 
 ///////////////////////////////////////////////////////
 
 void XMLCALL lsEndTag(void *data, const char *el)
 {
   parseInfo *info = (parseInfo *) data;
   //  printf("endtag %s changed depth from %d", el, info->depth);
   info->depth--;
   if (info->depth < info->skip)
     // back to normal search for valid tags
     info->skip = 0;
   //  printf(" to %d\n", info->depth);
 }
 
 ////////////////////////////////////////////////////
 
 int openAndSetMode()
 {
   printf("Entered openAndSetMode\n");
   
   int result = 0;
   int i, n = 1, v, endTask = 0;
   // open device
   busif.ttyDev = open(busif.serialDev, O_RDWR /*| O_NONBLOCK*/);
   result = busif.ttyDev != -1;
   if (result == 0)
     fprintf(stderr,"   imui2c: Can't open device: %s\n", busif.serialDev);
   else
   { // set baudrate
     result = (set_serial(busif.ttyDev, 9600) != -1);
     if (result == 0)
       fprintf(stderr,"   imui2c: Can't set serial port parameters\n");
   }
   if (debugFlag)
     // allow no device in debug mode
     result = 1;
   busif.lostConnection = ! result;
   if (result)
   { // empty data from device
     while (n > 0)
       n = getDataToTimeout(busif.rxBuf, MxBL, 100);
   }
   if (result) // Setting up the USB 2 I2C converter!
   { // format 0x5A is internal IMUI2C command
     //        0x02 is subcommand
     //        0x60 is i2c at 100 kHz using i2c hardware
     //        0x04 sets 
     unsigned char setI2cMode[4] = { 0x5A, 0x02, 0x50, 0x00 };
     
     printf("imui2c: ISS setting i2c mode ...");
     for (i = 0; i < 5; i++)
     { // signed mode
       secure2Write(setI2cMode, sizeof(setI2cMode));
       n = getDataToTimeout(busif.rxBuf, MxBL, 150);
       result = (n > 0 && busif.rxBuf[0] != 0);
       
       //printf("Read:%x %x %x %x %x %x %x %x\n", busif.rxBuf[0], busif.rxBuf[1],busif.rxBuf[2], busif.rxBuf[3], busif.rxBuf[4],
       // busif.rxBuf[5], busif.rxBuf[6], busif.rxBuf[7]);
       
       
       if (result)
	 break;
       printf("[fail (n=%d v=%d)]", n, busif.rxBuf[0]);
     }
     if (result)
       printf("[OK]\n");
     else
       printf("stuff\n");
   }
   if (result)
   {
     unsigned char getI2cVersion[2]={ 0x5A, 0x01};
     // set modes
     printf("imui2c: ISS getting version number ...");
     for (i = 0; i < 5; i++)
     { // signed mode
       secure2Write(getI2cVersion, sizeof(getI2cVersion));
       n = getDataToTimeout(busif.rxBuf, MxBL, 150);
       result = (n >= 3);
       if (result)
       {
	 printf("IMUI2C: model %x, version %x, mode %x ", busif.rxBuf[0], busif.rxBuf[1], busif.rxBuf[2]);
	 busif.version[0] = busif.rxBuf[0];
	 busif.version[1] = busif.rxBuf[1];
	 busif.version[2] = busif.rxBuf[2];
	 /*setVariable(imui2c.varI2sVersion, 0, busif.version[0]);
	  *            setVariable(imui2c.varI2sVersion, 1, busif.version[1]);
	  *            setVariable(imui2c.varI2sVersion, 2, busif.version[2]);*/
	 break;
       }
       printf("[fail]");
     }
     if (result)
       printf("[OK (%d bytes)]\n", n);
     else
       printf("\n");
   }
   if (result)
   { // serial number of device
     unsigned char getI2cSerial[2] ={ 0x5a, 0x03};
     // request version info
     printf("imui2c: ISS getting serial number ...");
     for (i = 0; i < 5; i++)
     { // signed mode
       secure2Write(getI2cSerial, sizeof(getI2cSerial));
       n = getDataToTimeout(busif.rxBuf, MxBL, 150);
       result = n >= 8;
       if (result)
       { // convert from ascii to integer
	 v = 0;
	 for (i = 0; i < 8; i++)
	   v = v * 10 + (busif.rxBuf[i] - '0');
	 busif.serial = v;
	 break;
       }
       printf("[fail]");
     }
     if (result)
       printf("%d [OK (%d bytes)]\n", busif.serial, n);
     else
       printf("\n");
   }
   
   // Gyroscope --> TODO : Doesn't work all the time (no connection)
   char checkCount;
   char checkAddr = 0xD0;
   for (checkCount = 0; checkCount <= 10; checkCount++)
   { // signed mode
     
     addr_chck_I2C((checkAddr|checkCount));
     if (busif.rxBuf[0] != 0x00){
       printf("GYRO (0x%02x) connected; ",((checkAddr&0xFF)|checkCount));
       GYRO = checkAddr+checkCount-1;
       dev_connected[0] = 1;
       //break;
     }
     
   }
   
   if (dev_connected[0] == 0) printf("GYRO FAILED\n");
   
   /*
    * for (i = 0; i < 1; i++)
    * { // signed mode
    *   unsigned char I2C_tst_Addr[2] = {0x58, 0xD3};
    *   secure2Write(I2C_tst_Addr,sizeof(I2C_tst_Addr));
    *   n = getData(busif.rxBuf, 4, 2);
    *   result = (n > 0);
    *   if (result)
    *   {
    *     if (busif.rxBuf[0] != 0x00){
    * printf("0xD3 connected; ");
    * break;
 }
 else {
   printf("0xD3 failed; ");
   result = 0;
   //endTask = 1;
 }
 //break;
 }
 
 }
 */
   /*
    *  //Gyroscope
    *  for (i = 0; i < 5; i++)
    *  { // signed mode
    *    addr_chck_I2C(GYRO);
    *    if (busif.rxBuf[0] != 0x00){
    *      printf("GYRO connected; ");
    *      dev_connected[1] = 1;
    *      //setVariable(IMU.acc_c, 0, 1);
    *      break;
 }
 else {
   printf("0x%02x failed; ",GYRO);
   //setVariable(IMU.acc_c, 0, 0);
   result = 0;
   endTask = 1;
 }
 //break;
 
 
 }*/
   
   
   //Accelerometer
   for (i = 0; i < 5; i++)
   { // signed mode
     addr_chck_I2C(ACCEL);
     if (busif.rxBuf[0] != 0x00){
       printf("ACCELEROMETER connected; ");
       dev_connected[1] = 1;
       //setVariable(IMU.acc_c, 0, 1);
       break;
     }
     else {
       printf("0x%02x failed; ",ACCEL);
       //setVariable(IMU.acc_c, 0, 0);
       result = 0;
       endTask = 1;
     }
     //break;
     
     
   }
   
   
   
   //Compass magnetic field
   for (i = 0; i < 5; i++)
   { // signed mode
     addr_chck_I2C(MAGNE);
     if (busif.rxBuf[0] != 0x00){
       printf("MAGNETOMETER connected; ");
       dev_connected[2] = 1;
       //setVariable(IMU.mag_c, 0, 1);
       break;
     }
     else {
       printf("0x%02x failed; ",MAGNE);
       //setVariable(IMU.mag_c, 0, 0);
       result = 0;
       endTask = 1;
     }
     //break;
     
     
   }
   
   
   
   //Barometer
   for (i = 0; i < 5; i++)
   { // signed mode
     addr_chck_I2C(BAROM);
     if (busif.rxBuf[0] != 0x00){
       printf("BAROMETER connected");
       dev_connected[3] = 1;
       //setVariable(IMU.bar_c, 0, 1);
       break;
     }
     else {
       printf("0x%02x failed;",BAROM);
       //setVariable(IMU.bar_c, 0, 0);
       result = 0;
       endTask = 1;
     }
     //break;
     
     
   }
   printf("\n");
   
   if (DEBUG){
     
     readI2C(GYRO,0x0F,1);
     printf("GYRO WAI 0x%02x\n",busif.rxBuf[2]);
     
     //      Setup for Gyroscope
     //         readI2C(GYRO,0x20,1);
     //         usleep(_1_MS);
     //         char olddata = busif.rxBuf[2];
     //         printf("OD = 0x%02x\n",olddata);
     //         
     //         //CTRL_REG1 == 0x20
     //         writeI2C(GYRO,0x20, (olddata|0x08));
     //         usleep(_1_MS);
     
     
     //char CTRL_REG4_g = 0x23;
     char CTRL_REG1_g = 0x20;
     //printf("REG3 = 0x%02x & REG1 = 0x%02x",CTRL_REG3_g, CTRL_REG1_g);
     writeI2C(GYRO,CTRL_REG1_g,0x00);
     usleep(_1_MS);
     
     writeI2C(GYRO,CTRL_REG1_g,0x0F);
     usleep(_1_MS);
     
     /*
     writeI2C(GYRO,CTRL_REG4_g,0x00);
     usleep(_1_MS);
     //DONEZO MR SETUP GYRO
     */
     
     
     // Setup for Accelerometer
     // Set 50 Hz outpute rate, normal mode, enable all axis
     simple_writeI2C(ACCEL,0x20,0x47);
     usleep(_1_MS);
     
     // Set to high resolution mode, fullscale to +-2G
     simple_writeI2C(ACCEL,0x23,0x08);
     usleep(_1_MS);
     
     // Setup for Magnetometer
     // Set CTRL_2 -> Scale = +- 1.3
     simple_writeI2C(MAGNE,0x01,0x20);
     usleep(_1_MS);
     
     // Set MR_reg continuous mode.
     simple_writeI2C(MAGNE,0x02,0x00);
     usleep(_1_MS);
     
     
     
   }
   
   /*
    *  //for (i = 0; i < 5; i++)
    *    { // signed mode
    *      writeI2C(0x20,0x0F);
    *      n = getDataToTimeout(busif.rxBuf, MxBL, 500);
    *      result = (n > 0);
    *      if (result)
    *      {
    *	rxPrint();
    *        //break;
 }
 else
   printf("fail: n=%x",n);
 }
 
 { // signed mode
 writeI2C(0x23,0x20);
 n = getDataToTimeout(busif.rxBuf, MxBL, 500);
 result = (n > 0);
 if (result)
 {
 rxPrint();
 //break;
 }
 else
   printf("fail: n=%x",n);
 }
 
 */
   
   //md25.accOld = -1;
   return (result || debugFlag) && !endTask;
 }
 //////////////////////////////////////////////
 
 /**
  * Initialize the communication and start rx thread
  * \returns 1 on success (else 0) */
 int initUsbIss(void)
 { //Open first serial port
   printf("Entered initUsbIss\n");
   int result;
   //
   rxtask.running = 0;
   rxtask.startNewRxCycle = 0;
   // open and configure device
   result = openAndSetMode();
   //
   if (result == 1)
   { // start thread to handle bus
     pthread_attr_t attr;
     pthread_attr_init(&attr);
     pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
     if (pthread_create(&rxtask.i2c_thread, &attr, i2c_task, 0))
     {
       perror("   imui2c: Can't start i2c receive thread");
       result = 0;
     }
   }
   if (result == 1)
   { /****** Create database variables if all is ok **************/
     int waitCount = 0;
     createI2Cvariables();
     while (!rxtask.running)
     { // wait a bit for thread to start
       usleep(20000); //Don't return before threads are running
       if (waitCount >= 50)
       {
	 result = 0;
	 break;
       }
       waitCount++;
     }
   }
   return result;
 }
 
 ////////////////////////////////////////////////////
 
 char limitSignedChar(int value, int min, int max)
 {
   if (value < min)
     return min;
   else if (value > max)
     return max;
   else
     return value;
 }
 
 ////////////////////////////////////////////////////
 
 unsigned char limitUnsignedChar(int value, int min, int max)
 {
   if (value < min)
     return min;
   else if (value > max)
     return max;
   else
     return value;
 }
 
 ////////////////////////////////////////////////////
 
 ///RS232  Recieve thread
 void * i2c_task(void * not_used)
 { // run in thread
   printf("i2c_task\n");
   
   rxtask.startNewRxCycle = 0;
   
   //const int MaxWaitCycles = 2; // max number of timeout periods
   //const int PollTimeoutMs = 4; // timeout period
   //int state = 0;
   //int loopCnt = 0;
   
   
   
   struct timeval tm;
   //unsigned int lastEnc1, lastEnc2;
   //uint32_t e1, e2;
   //int64_t eDiff1, eDiff2;
   //
   if (0)
   {
     if (mlockall(MCL_CURRENT | MCL_FUTURE))
     {
       perror("mlockall");
       exit(-1);
     }
     
     { /* use real-time (fixed priority) scheduler
       * set priority to one less than the maximum
       */
       struct sched_param param;
       
       param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
       if (sched_setscheduler(0, SCHED_RR, &param)) {
	 perror("setscheduler");
	 pthread_exit(0);
       }
       
     }
   }
   
   if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
     fprintf(stderr, "   IMUI2C: signal: can't ignore SIGPIPE.\n");
   
   //fprintf(stderr, "   IMUI2C: rx_task running\n");
   if (debugFlag)
   {
     logfile = fopen("imui2c.log", "w");
     //logState = fopen("imui2c-state.log", "w");
   }
   if (logfile != NULL)
   {
     gettimeofday(&tm, NULL);
     fprintf(logfile, "time tick looptime-ms speedref motor1 motor2 battery enc1 enc2 curr1 curr2 speed1 speed2 acc\n");
   }
   if (logState != NULL)
   {
     gettimeofday(&tm, NULL);
     fprintf(logState, "time state logging\n");
   }
   //Mark thread as running
   rxtask.running = 1;
   busif.txBuf[0] = '\0';
   while (rxtask.running)
   { // maintain interface
     //int i, v, isOK;
     //int result; /*, deb;*/
     //
     //loopCnt++;
     /*
      *   if (logState != NULL)
      *   {
      *     if (state == 0)
      * fprintf(logState,"\n");
      *     gettimeofday(&tm, NULL);
      *     fprintf(logState, "%lu.%06lu %d %2d\n ", tm.tv_sec, tm.tv_usec, loopCnt, state);
   }*/
     if (busif.lostConnection)
     { // connection is lost - durinr read or write
       if (busif.ttyDev >= 0)
       {
	 close(busif.ttyDev);
	 busif.ttyDev = -1;
	 printf("**** imui2c: lost connection - trying to reconnect\n");
	 sleep(1);
       }
       // wait a while - for udev to detect device is back
       sleep(1);
       // try to open
       openAndSetMode();
       if (busif.lostConnection)
	 // connection is still lost, so try again
	 continue;
     }
     //pthread_mutex_lock(&IMU.mLock);
     //gettimeofday(&tickTime, NULL);
     //rxtask.startNewRxCycle = 0;
     
     
     usleep(100000);
     
     int p;
     
     
     
     
     if (DEBUG){
       /******************************************************/
       /*           COLLECTING DATA FROM GYROSCOPE           */
       /******************************************************/
       
       if(dev_connected[0] == 1){
	 int tempBuff_g[6];
	 int gyroBuf[3];
	 char DataReady_g = 0;
	 // ONLY READ DATA WHEN THE READY BIT EMPLIES THERE IS NEW DATA TO BE READ.
	 readI2C(GYRO,0x27,1);
	 DataReady_g = busif.rxBuf[2];
	 printf("DataReady_g = 0x%02x\n",DataReady_g);
	 if (DataReady_g & 0x08){
	   readI2C(GYRO,0x28,6);
	   //rxPrint();
	   for (p=0; p<=5; p++){
	     tempBuff_g[p] = busif.rxBuf[2+p];
	   }
	   
	   gyroBuf[0] = (int)(tempBuff_g[0] | (tempBuff_g[1]<<8));
	   gyroBuf[1] = (int)(tempBuff_g[2] | (tempBuff_g[3]<<8));
	   gyroBuf[2] = (int)(tempBuff_g[4] | (tempBuff_g[5]<<8));
	   
	   for (p = 0; p < 3; p++) setVariable(IMU.gyr, p, gyroBuf[p]*NEWCONSTANT); //TODO create conversion constant.
	 }
	 
       }
       
       /******************************************************/
       /*         COLLECTING DATA FROM ACCELEROMETER         */
       /******************************************************/
       if(dev_connected[1] == 1){
	 int tempBuff_a[6];
	 int accelBuf[3];
	 readI2C(ACCEL,0x28,6);
	 //rxPrint();
	 for (p=0; p<=5; p++){
	   tempBuff_a[p] = busif.rxBuf[2+p];
	 }
	 
	 accelBuf[0] = (int16_t)(tempBuff_a[0] | (tempBuff_a[1]<<8)); // x
	 accelBuf[1] = (int16_t)(tempBuff_a[2] | (tempBuff_a[3]<<8)); // y
	 accelBuf[2] = (int16_t)(tempBuff_a[4] | (tempBuff_a[5]<<8)); // z
	 
	 for (p = 0; p < 3; p++) setVariable(IMU.acc, p, accelBuf[p]*NEWCONSTANT);
       }
       /******************************************************/
       /*         COLLECTING DATA FROM MAGNETOMETER          */
       /******************************************************/
       if(dev_connected[2] == 1){
	 int tempBuff_m[6];
	 int magneBuf[3];
	 char DataReady_m = 0;
	 // ONLY READ DATA WHEN THE READY BIT EMPLIES THERE IS NEW DATA TO BE READ.
	 readI2C(MAGNE,0x09,1);
	 DataReady_m = busif.rxBuf[2];
	 
	 if (DataReady_m){
	   readI2C(MAGNE,0x03,6);
	   getDataToTimeout(busif.rxBuf, MxBL, 50);
	   
	   for (p = 0; p <= 5; p++){
	     tempBuff_m[p] = busif.rxBuf[2+p];
	     
	   }
	   
	   magneBuf[0] = (int16_t)(tempBuff_m[1] | (tempBuff_m[0]<<8)); // x
	   magneBuf[1] = (int16_t)(tempBuff_m[5] | (tempBuff_m[4]<<8)); // y
	   magneBuf[2] = (int16_t)(tempBuff_m[3] | (tempBuff_m[2]<<8)); // z
	   
	   
	   for (p = 0; p < 3; p++) setVariable(IMU.mag, p, magneBuf[p]*NEWCONSTANT);
	 }
       }
       
       
       /******************************************************/
       /*          COLLECTING DATA FROM BAROMETER            */
       /******************************************************/
       if(dev_connected[3] == 1){
	 int tempBuff_b[2];
	 int baromBuf[2];
	 
	 readI2C(BAROM,0x2E,2);
	 
	 for (p = 0; p<= 1; p++){
	   tempBuff_b[p] = busif.rxBuf[2+p];
	 }
	 baromBuf[0] = (int)(tempBuff_b[0] | (tempBuff_b[1]<<8));
	 
	 readI2C(BAROM,0x34,2);
	 for (p = 0; p<= 1; p++){
	   tempBuff_b[p] = busif.rxBuf[2+p];
	 }
	 baromBuf[1] = (int)(tempBuff_b[0] | (tempBuff_b[1]<<8));
	 
	 for (p = 0; p < 2; p++) setVariable(IMU.bar, p, baromBuf[p]*NEWCONSTANT);
       }
       
       
       
       
       
     }
     
     
     
     
     
     
     
     
   }
   printf("Ended the whileloop\n");
   rxtask.running = 0;
   fprintf(stderr,"imui2c: closing bus device\n");
   close(busif.ttyDev);
   fprintf(stderr,"imui2c: Shutting down thread\n");
   if (logfile != NULL)
     fclose(logfile);
   if (logState != NULL)
     fclose(logState);
   pthread_exit(0);
   return NULL;
 }
 
 ///////////////////////////////////////////////////////////
 
 /**
  * Create variables for the usb to i2c converter itself */
 void createI2Cvariables()
 {
   imui2c.varI2sVersion = createVariable('r',3,"imui2cVersion");
   imui2c.varI2sSerial = createVariable('r',1,"imui2cSerial");
   
   IMU.gyr = createVariable('r',3, "IMUgyr");
   IMU.acc = createVariable('r',3, "IMUacc");
   IMU.mag = createVariable('r',3, "IMUmag");
   IMU.bar = createVariable('r',2, "IMUbar");
   //IMU.temp = createVariable('r',1,"IMUtemp");
   
   
   
   
   //    setVariable(IMU.gyr_c,0,0);
   //    setVariable(IMU.acc_c,0,0);
   //    setVariable(IMU.mag_c,0,0);
   //    setVariable(IMU.bar_c,0,0);
   
 }
 
 ////////////////////////////////////////////////////////////////
 
 int pollDeviceRx(int fd, int msTimeout)
 {
   int err = 0;
   int dataAvailable = 0;
   struct pollfd pollStatus;
   int revents;
   //
   if (fd == -1)
   { // no device (debugging mode)
     usleep(50000);
   }
   else
   { // test if there is available data
     pollStatus.fd = fd;
     pollStatus.revents = 0;
     pollStatus.events = POLLIN  |  /*  0x0001  There is data to read */
     POLLPRI;   /*  0x0002  There is urgent data to read */
     /* POLLOUT 0x0004  Writing now will not block */
     //
     err = poll(&pollStatus, 1, msTimeout);
     if (err < 0)
     { // not a valid call (may be debugger interrrupted)
       perror("UServerPort::serviceClients (poll)");
     }
     else if (err > 0)
     { // at least one connection has data (or status change)
       revents = pollStatus.revents;
       if (((revents & POLLIN) != 0) ||
	 ((revents & POLLPRI) != 0))
	 dataAvailable = 1;
     }
   }
   return dataAvailable;
 }
 
 /////////////////////////////////////////////////
 
 int getDatabaseVariable(char type, const char * name)
 { // get index of a variable in the database
   symTableElement * syms;
   int symsCnt;
   int result = -1;
   int i;
   //
   syms = getSymbolTable(type);
   symsCnt = getSymtableSize(type);
   for (i = 0; i < symsCnt; i++)
   {
     if (strcmp(syms->name, name) == 0)
     {
       result = i;
       break;
     }
     syms++;
   }
   return result;
 }
 
 //////////////////////////////////////////////////
 
 double getVariableDouble(char type, int index)
 { // get type double value from 2 integer values - assuming integer part and micro decimal value.
   int v, uv;
   double result;
   if (type == 'r')
   {
     v = getReadVariable(index, 0);
     uv =   getReadVariable(index, 1);
   }
   else
   {
     v = getWriteVariable(index, 0);
     uv = getWriteVariable(index, 1);
   }
   result = v + uv * 1e-6;
   return result;
 }
 
 /////////////////////////////////////
 
 void resetUpdateRead(int id)
 {
   symTableElement * syms;
   int symsCnt;
   //
   syms = getSymbolTable('r');
   symsCnt = getSymtableSize('r');
   //
   if (id >= 0 && id < symsCnt)
     syms[id].updated = 0;
 }
 
 ////////////////////////////////////////////////
 
 int getData(int minCnt, int timeoutms, int maxWaitCnt)
 {
   int l = 0;
   int n = 0;
   unsigned char * p1 = busif.rxBuf;
   // wait for reply - up to 500ms
   while (l < maxWaitCnt)
   {
     int dataOK;
     int m;
     l++; // loop counter
     // wait for data
     dataOK = pollDeviceRx(busif.ttyDev, timeoutms);
     if (dataOK)
     { // there is data
       m = read(busif.ttyDev, p1, MxBL - n - 1);
       if (m > 0)
       {
	 n += m;
	 p1 = &busif.rxBuf[n];
       }
       else if (m < 0)
       { // poll error - device is removed?
	 printf("imui2c: Read error from device - device is lost\n");
	 busif.lostConnection = 0;
	 break;
       }
     }
     if (n >= minCnt)
       break;
   }
   //if (l > 1)
   //  printf("getData: got %d bytes in %d pools (timeout=%dms maxLoop=%d\n",
   //          n, l, timeoutms, maxWaitCnt);
   return n;
 }
 
 /////////////////////////////////////////////////
 
 int getDataToTimeout(unsigned char * dest, int destCnt, int timeoutms)
 {
   int n = 0;
   unsigned char * p1 = dest;
   int toMult = 2; // timeout multiplier on first timeout
   //
   *p1 = '\0'; // clear result (to improve debug);
   while (1)
   { // wait for data
     int dataOK;
     int m;
     //
     if (destCnt - n <= 1)
       // no more space
       break;
     // wait for data
     dataOK = pollDeviceRx(busif.ttyDev, timeoutms * toMult);
     if (dataOK)
     { // there is data, so get it
       m = read(busif.ttyDev, p1, destCnt - n - 1);
       if (m > 0)
       { // got some data - search for new-line
	 n += m;
	 p1 = &busif.rxBuf[n];
       }
       else if (m < 0)
       { // poll error - device is removed?
	 printf("imui2c: Read error from device - connection lost\n");
	 busif.lostConnection = 1;
       }
     }
     else
     { // we have a timeout - no more data
       // zero terminate
       *p1 = '\0';
       break;
     }
     toMult = 1;
   }
   return n;
 }
 
 
 void rxPrint(){
   
   if (debugFlag){
     printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", busif.rxBuf[0], busif.rxBuf[1], busif.rxBuf[2], busif.rxBuf[3],
	    busif.rxBuf[4], busif.rxBuf[5], busif.rxBuf[6], busif.rxBuf[7]);
   }
   
   
   if (busif.rxBuf[0] == 0x00){
     printf("An ERROR has occurred in the I2C transfer.\n");
     printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", busif.rxBuf[0], busif.rxBuf[1], busif.rxBuf[2], busif.rxBuf[3],
	    busif.rxBuf[4], busif.rxBuf[5], busif.rxBuf[6], busif.rxBuf[7]);
   }
   else	{
     if (busif.rxBuf[0] != 0xFF){
       printf("Something went terribly wrong in the USB to I2C stuff.\n");
       
     }
     
     // Return was a success. Case to setup the amount of data has been returned.
     switch (busif.rxBuf[1])
     {
       case 0:
	 printf("Nothing to print, no data was returned\n");
	 break;
       case 1:
	 printf("0x%02x ",busif.rxBuf[2]);
	 break;
       case 2:
	 printf("0x%02x 0x%02x\n", busif.rxBuf[2], busif.rxBuf[3]);
	 break;
       case 3:
	 printf("0x%02x 0x%02x 0x%02x\n", busif.rxBuf[2], busif.rxBuf[3],
		busif.rxBuf[4]);
	 break;
       case 4:
	 printf("0x%02x 0x%02x 0x%02x 0x%02x\n", busif.rxBuf[2], busif.rxBuf[3],
		busif.rxBuf[4], busif.rxBuf[5]);
	 break;
       case 5:
	 printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", busif.rxBuf[2], busif.rxBuf[3],
		busif.rxBuf[4], busif.rxBuf[5], busif.rxBuf[6]);
	 break;
       default: //== 6
	 printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", busif.rxBuf[2], busif.rxBuf[3],
		busif.rxBuf[4], busif.rxBuf[5], busif.rxBuf[6], busif.rxBuf[7]);
	 break;
     }
   }
 }
 
 ssize_t readI2C(char device, char addr, int bytesToRead){
   ssize_t result;
   // Write Setup start
   char I2C_write_addr = device; //Write addr for 
   char I2C_read_addr = I2C_write_addr+1; // Read addr for
  
   
   char I2C_reg_addr = addr;
   if (bytesToRead > 1)
     I2C_reg_addr = (I2C_reg_addr | 0x80);
   // Address for WOI
   //char I2C_msg = 0x00;
   
   // USB ISS sub-commands
   char I2C_start = 0x01; //Start sequence
   char I2C_restart = 0x02; //Restart sequence
   char I2C_stop = 0x03; //Stop sequence
   char I2C_nack = 0x04; //Send nack after next read
   char USB_cust_seq = 0x57; //Enables sending custom I2C sequences.
   
   char I2C_bytesToRead = 0x1F+bytesToRead;
   //char I2C_bytesToRead = 0x21;
   //char I2C_final[2] = {0x58, 0xD4};
   //unsigned char I2C_final[11] = {0x57, 0x01, 0x31, 0xD6, 0x28, 0x02, 0x30, 0xD7, 0x04, 0x21, 0x03};
   
   if (I2C_bytesToRead == 0x20){
     unsigned char I2C_final[11] = {USB_cust_seq, I2C_start, 0x31, I2C_write_addr, I2C_reg_addr, I2C_restart, 0x30, I2C_read_addr, I2C_nack, I2C_bytesToRead, I2C_stop};
     result = secure2Write(I2C_final, sizeof(I2C_final));
   }
   else{
     unsigned char I2C_final[12] = {USB_cust_seq, I2C_start, 0x31, I2C_write_addr, I2C_reg_addr, I2C_restart, 0x30, I2C_read_addr, I2C_bytesToRead-1, I2C_nack, 0x20, I2C_stop};
     result = secure2Write(I2C_final, sizeof(I2C_final));
   }
   getDataToTimeout(busif.rxBuf, MxBL, 50);
   return result;
 }
 
 
 ssize_t writeI2C(char device, char addr, char data){
   ssize_t result;
   // Write Setup start
   char I2C_write_addr = device; //Write addr for LG3D20
   char I2C_reg_addr = addr; // Address for WOI
   char I2C_data = data;
   
   // USB ISS sub-commands
   char I2C_start = 0x01; //Start sequence
   char I2C_stop = 0x03; //Stop sequence
   char USB_cust_seq = 0x57; //Enables sending custom I2C sequences.
   
   //char I2C_bytesToRead = 0x21;
   //char I2C_final[2] = {0x58, 0xD4};
   //unsigned char I2C_final[11] = {0x57, 0x01, 0x31, 0xD6, 0x28, 0x02, 0x30, 0xD7, 0x04, 0x21, 0x03};
   unsigned char I2C_final[7] = {USB_cust_seq, I2C_start, 0x32, I2C_write_addr, I2C_reg_addr, I2C_data, I2C_stop};
   result = secure2Write(I2C_final, sizeof(I2C_final));
   
   //getDataToTimeout(busif.rxBuf, MxBL, 50);
   return result;
   
 }
 
 
 ssize_t simple_readI2C(char device, char addr, char bytesToRead){
   ssize_t result;
   // Write Setup start
   char I2C_write_addr = device; //Write addr for 
   char I2C_read_addr = I2C_write_addr+1; // Read addr for
   
   char I2C_reg_addr = addr; // Address for WOI
   //char I2C_msg = 0x00;
   
   
   //char I2C_bytesToRead = 0x1F+bytesToRead;
   //char I2C_bytesToRead = 0x21;
   //char I2C_final[2] = {0x58, 0xD4};
   //unsigned char I2C_final[11] = {0x57, 0x01, 0x31, 0xD6, 0x28, 0x02, 0x30, 0xD7, 0x04, 0x21, 0x03};
   unsigned char I2C_final[4] = {0x55, I2C_read_addr, I2C_reg_addr, bytesToRead};
   result = secure2Write(I2C_final, sizeof(I2C_final));
   getDataToTimeout(busif.rxBuf, MxBL, 50);
   return result;
 }
 
 ssize_t simple_writeI2C(char device, char addr, char data){
   ssize_t result;
   // Write Setup start
   char I2C_write_addr = device; //Write addr for 
   char I2C_reg_addr = addr; // Address for WOI
   
   unsigned char I2C_final[5] = {0x55, I2C_write_addr, I2C_reg_addr, 0x01, data};
   result = secure2Write(I2C_final, sizeof(I2C_final));
   getDataToTimeout(busif.rxBuf, MxBL, 50);
   return result;
 }
 
 
 
 
 
 
 
 ssize_t addr_chck_I2C(char device){
   ssize_t result;
   unsigned char I2C_tst_Addr[2] = {0x58, device};
   result = secure2Write(I2C_tst_Addr,sizeof(I2C_tst_Addr));
   getDataToTimeout(busif.rxBuf, MxBL, 50);
   return result;
 }
 
 
 