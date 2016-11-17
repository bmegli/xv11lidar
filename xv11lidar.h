/*
 * XV11 LIDAR communication library
 *
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#pragma once

#include <stdint.h> //uint8_t, uint16_t
#include <termios.h> //termios

#ifdef __cplusplus
extern "C" {
#endif

// xv11lidar_read return values
enum xv11lidar_status
{
	XV11LIDAR_SUCCESS= 0, 
	XV11LIDAR_TTY_ERROR = -1, //tty read fatal error, errno is set in this case
	XV11LIDAR_SYNC_ERROR = 2  //the number of incorrectly read frames (CRC failures) exceeded the set set limit 
};

// xv11lidar_reading.distance encoded error when xv11lidar_reading.invalid_data==1
// TO DO - explain other codes and give them descriptive names
enum xv11lidar_invalid_data 
{
	XV11LIDAR_CRC_FAILURE=0x66, //the frame had incorrect CRC, don't use the data
	XV11LIDAR_ERROR1=0x02,
	XV11LIDAR_ERROR2=0x03,
	XV11LIDAR_ERROR3=0x21,
	XV11LIDAR_ERROR4=0x25,
	XV11LIDAR_ERROR5=0x35,
	XV11LIDAR_ERROR6=0x50,
};

/*	For complete information on LIDAR data format see: 
 *	http://xv11hacking.wikispaces.com/LIDAR+Sensor
 *  LIDAR returns data in frames of 4 consecutive readings (angles)
*/
//single angle reading
struct xv11lidar_reading
{
	unsigned int distance : 14; //distance or error code when invalid_data flag is set
	unsigned int strength_warning : 1; //flag indicating that reported signal strength is lower then expected
	unsigned int invalid_data : 1; //flag indicating that distance could not be calculated	
	uint16_t signal_strength; //received signal strength 
} __attribute__((packed));

//single frame read from lidar
struct xv11lidar_frame
{
	uint8_t start; //fixed 0xFA can be used for synchronization
	uint8_t index; //(index-0xA0)*4 is the angle for readings[0] (+1,2,3 for consecutive readings)
	uint16_t speed; //divide by 64 to get speed in rpm
	struct xv11lidar_reading readings[4]; //readings for 4 consecutive angles
	uint16_t checksum; //if checksum is incorrect invalid_data
	
} __attribute__((packed));


//internal data used by the functions (as pointer)
struct xv11lidar;

/* Sets terminal to the appropriate mode and synchrnonizes with the device
 * 
* parameters:
* tty - the path to lidar tty
* laser_frames_per_read - configure tty to read that much frames per read, each frame has 4 degree scan
* crc_tolerance_percent - accept up to this crc_failures each revolution, range <0, 100>
* 
* returns:
* - NULL on error, currently the reason will be printed to standard error
* - otherwise internal library data pointer, pass it to other functions
* 
* preconditions:
* -lidar spinning CCW at around 300 RPM
* -lidar uart reachable at tty 
* -port set to other-uart mode
*
*/
struct xv11lidar *xv11lidar_init(const char *tty, int laser_frames_per_read, int crc_tolerance_percent);

/* Cleans up (file descriptors, memory,  tty is set back to inital configuration)
* parameters:
* lidar_data - internal library data
*  
* preconditions:
* -lidar initialized successfully with xv11lidar_init
*/
void xv11lidar_close(struct xv11lidar *lidar_data);

/* Reads from the lidar tty the number of frames configured during initialization
 * This function blocks until configured number of frames is read, read error occurs or sync is lost
 * 
 * parameters:
 * lidar_data - internal data pointer
 * frame_data - a pointer to array that is big enough to store laser_frames_per_read configured during InitLaser
 * 
 * returns:
 * xv11lidar_status.XV11LIDAR_SUCCESS (0) on success
 * xv11lidar_status.XV11LIDAR_TTY_ERROR (-1) on tty read failure, you can check errno in this case
 * xv11lidar_status.XV11LIDAR_SYNC_ERROR (-2) on laser sync failure (CRC failures exceeded per rotation limit)
 * 
 * The errors are generally fatal (unless you set very strict crc_tolerance_percent in xv11lidar_init)
 * 
 * preconditions:
 * -lidar initialized with xv11lidar_init
 */
int xv11lidar_read(struct xv11lidar *lidar_data, struct xv11lidar_frame *frame_data);

#ifdef __cplusplus
}
#endif
