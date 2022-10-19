/** \file gps.h
 *  \ingroup hwmodule
 *  \brief Serial GPS Module
 *
 * GPS Module for RHD. 
 * 
 * Based on HAKOD by Asbjørn Mejnertsen and Anders Reeske Nielsen
 * and AuGps by Lars Mogensen and Christian Andersen
 * 
 * The module supports both standard NMEA GPS and RTK GPS through
 * serial port
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $
 */
 /***************************************************************************
 *                  Copyright 2008 Anders Billesø Beck                     *
 *                       anders.beck@get2net.dk                            *
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


#ifndef IMUSERIAL_H
  #define IMUSERIAL_H

  /** RHD calls initXML once just after the plugin is loaded
   *    used to read configuration file and set initial state
   *    and open and configure connection to device */
  extern int initXML(char *);
  /**
   * RHD calls this function every time write variables may have beed updated
   * normally every 10ms (RHD sample time)
   * \param RHDtick is RHD sample count. */
  extern int periodic(int RHDtick);
  /**
   * Called by RHD when RHD is shutting down. 
   * Should be used to close files and connections */
  extern int terminate (void) ; //No shutdown function
  
  
  struct
{
  float gyr;
  float acl;
  float mag;
  float bar;
  
}IMU;
  
  
#endif

