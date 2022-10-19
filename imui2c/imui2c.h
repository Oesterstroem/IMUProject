 /** \file imui2c.h
 *  \ingroup hwmodule
 *
 *   Interface for i2c bus from USBi2c converter
 *   and connected to MD25 motor controller unit
 *
 * $Rev: 59 $
 * ¤Id$
 *
 *******************************************************************/

#ifndef USBISS_H
#define USBISS_H

extern int initXML(char *);
extern int periodic(int);
extern int terminate (void) ;

/// communication variables
struct
{ /** buffer for communication */
  #define MxBL 32
  char txBuf[MxBL];
  unsigned char rxBuf[MxBL];
  /** device file descriptor */
  int ttyDev;  //File descriptors
  #define MxDL 64
  char serialDev[MxDL];
  /// stuck flag 0=not stuck, 1 = struk
  int stuck;
  /// version number 0: ID=7, 1: firmware, 2: current mode
  int version[3];
  /// serial number
  int serial;
  /// is connection lost diurinr read or write operation
  int lostConnection;
} busif;

/// variables related to bus control
struct
{ /** variables for usb to i2c bus */
  int varI2sSerial;
  /// version info
  int varI2sVersion;
  /// i2c bus id for bus controller
  int busid;
  /// baudrate of i2c
  long baudrate;
} imui2c;




struct
{
  float gyr;
  float acc;
  float mag;
  float bar;
  
  int gyr_c;
  int acc_c;
  int mag_c;
  int bar_c;
  
  pthread_mutex_t mLock; // pthread_mutex_lock
} IMU;



#endif

