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

#include "xv11lidar.h"

#include <termios.h> //struct termios, tcgetattr, tcsetattr, cfsetispeed, tcflush
#include <fcntl.h> //file open flags
#include <stdlib.h> //exit
#include <stdio.h> //fprintf
#include <unistd.h> //read, close
#include <errno.h> //errno
#include <string.h> //memcpy
#include <limits.h> //UCHAR_MAX

// this constant can be tuned
const int REQUIRED_SYNC_FRAMES=45; //the number of consecutuve frames that is required to be read correctly (CRC) for sync

// those constants should not be touched
const int FRAME_SIZE=sizeof(struct xv11lidar_frame);	
const int FRAME_CHECKSUM_OFFSET=20;
const int FRAME_INDEX_OFFSET=1;

const uint8_t FRAME_START_BYTE=0xFA;
const uint8_t FRAME_INDEX_0=0xA0;

const int FRAMES_PER_ROTATION=90;
const int READINGS_PER_FRAME=4;

struct xv11lidar
{
	int fd;
	struct termios old_io;
	int laser_frames_per_read;
	int crc_tolerance;
	int crc_failures;
	int last_frame_index;
	uint8_t *data;
};

struct xv11lidar *xv11lidar_alloc(int laser_frames_per_read, int crc_tolerance_percent);
struct xv11lidar *xv11lidar_free(struct xv11lidar *lidar_data, const char *error_message);
int synchronize_laser(struct xv11lidar *lidar_data);
int read_all(int fd, uint8_t *data,int total_read_size);
int is_frame_checksum_ok(const uint8_t data[FRAME_SIZE]);
uint16_t checksum(const uint8_t data[FRAME_SIZE]);



/*
 * Alloc internal memory for laser data and readings
 * Initialize values
 * Open the terminal and save it's original settings
 * Set terminal for raw byte input single byte at a time at 115200 speed
 * Close and open tty (this is workaround for "...too much work for IRQ..."
 * Synchronize with the laser
 */
 struct xv11lidar *xv11lidar_init(const char *tty, int laser_frames_per_read, int crc_tolerance_percent)
{	
	struct termios io;
	struct xv11lidar *lidar_data;
	
	if( (lidar_data=xv11lidar_alloc(laser_frames_per_read, crc_tolerance_percent)) == NULL)
		return NULL;

	if ((lidar_data->fd=open(tty, O_RDONLY))==-1)
		return xv11lidar_free(lidar_data, "unable to open tty");
	
	if(tcgetattr(lidar_data->fd, &lidar_data->old_io) < 0)
		return xv11lidar_free(lidar_data, "unable to get tty attributes");
			
	io.c_iflag=io.c_oflag=io.c_lflag=0;
	io.c_cflag=CS8|CREAD|CLOCAL; //8 bit characters
	
	if(laser_frames_per_read * FRAME_SIZE <= UCHAR_MAX)					
		io.c_cc[VMIN]=laser_frames_per_read * FRAME_SIZE; 
	else
		io.c_cc[VMIN]=11 * FRAME_SIZE; //11*22=242 which is the largest reasonable value <= UCHAR_MAX 	

	io.c_cc[VTIME]=0; // no timeout
	
	if(cfsetispeed(&io, B115200) < 0 || cfsetospeed(&io, B115200) < 0)
		return xv11lidar_free(lidar_data, "unable to set tty speed");

	if(tcsetattr(lidar_data->fd, TCSAFLUSH, &io) < 0)
		return xv11lidar_free(lidar_data, "unable to set tty attributes");
	
	// this is workaround for "too much work for IRQ", we close and reopoen the tty after settings and flush
	close(lidar_data->fd);	
		
	if((lidar_data->fd=open(tty, O_RDONLY))==-1)
		return xv11lidar_free(lidar_data, "unable to reopen tty");
	
	if(synchronize_laser(lidar_data) != XV11LIDAR_SUCCESS)
		return xv11lidar_free(lidar_data, "unable to synchronize with laser");
	
	return lidar_data;
}

struct xv11lidar *xv11lidar_alloc(int laser_frames_per_read, int crc_tolerance_percent)
{
	struct xv11lidar *lidar_data=(struct xv11lidar*)malloc(sizeof(struct xv11lidar));

	if(lidar_data==NULL)
	{
		fprintf(stderr, "xv11lidar: not enough memory for lidar data\n");
		return NULL;
	}
	if( (lidar_data->data=(uint8_t*)malloc(laser_frames_per_read*sizeof(struct xv11lidar_frame))) == 0)
	{
		fprintf(stderr, "xv11lidar: not enough memory for readings data\n");
		free(lidar_data);
		return NULL;
	}				

	lidar_data->crc_failures = 0;
	lidar_data->crc_tolerance = crc_tolerance_percent * 90 / 100;
	lidar_data->last_frame_index = FRAME_INDEX_0 + FRAMES_PER_ROTATION - 1;
	lidar_data->laser_frames_per_read=laser_frames_per_read;
	lidar_data->fd=-1;
	
	return lidar_data;
}

struct xv11lidar *xv11lidar_free(struct xv11lidar *lidar_data, const char *error_message)
{
	if(error_message != NULL)
		fprintf(stderr, "xv11lidar: %s\n", error_message);
	if(lidar_data->fd != -1)
		close(lidar_data->fd);
	free(lidar_data->data);
	free(lidar_data);
	return NULL; //for convenience
}

/*
 * Waits for 0xFA byte and REQUIRED_SYNC_FRAMES consecutive frames with correct checksum
 * Discards the rest bytes so that next read starts from frame with index 0 (0xA0)
 */
int synchronize_laser(struct xv11lidar *lidar_data)
{	
	int fd=lidar_data->fd, i;
	uint8_t *data=lidar_data->data;
	int  data_size=FRAME_SIZE * lidar_data->laser_frames_per_read;
	
	while(1)
	{
		if ( (read_all(fd, data, FRAME_SIZE)) != XV11LIDAR_SUCCESS)
			return XV11LIDAR_TTY_ERROR;
		
		// find frame start byte
		for(i=0;i<FRAME_SIZE;++i)
			if(data[i] == FRAME_START_BYTE)
				break;
				
		if(i == FRAME_SIZE)
			continue; 

		//get the rest of frame
		memmove(data, data+i,FRAME_SIZE-i);
			
		if(i>0)
			if(read_all(fd, data+i, i) != XV11LIDAR_SUCCESS ) 
				return XV11LIDAR_TTY_ERROR;

		//checksum k consecutive frames
		if( !is_frame_checksum_ok(data) )
			continue;

		for(i=1;i<REQUIRED_SYNC_FRAMES;++i)
		{
			if( read_all(fd, data, FRAME_SIZE) != XV11LIDAR_SUCCESS )
				return XV11LIDAR_TTY_ERROR;
			
			if(data[0] != FRAME_START_BYTE || !is_frame_checksum_ok(data) )
				break;
		}

		if( i != REQUIRED_SYNC_FRAMES )
			continue;

		//discard bytes until 0 angle frame		
		int index=*(data+FRAME_INDEX_OFFSET)-FRAME_INDEX_0;
		int bytes_to_discard=(FRAMES_PER_ROTATION-1-index) * FRAME_SIZE;
		
		while(bytes_to_discard > data_size)
			if( read_all(fd, data, data_size) != XV11LIDAR_SUCCESS )
				return XV11LIDAR_TTY_ERROR;	
			else
				bytes_to_discard-=data_size;

		if( read_all(fd, data, bytes_to_discard) != XV11LIDAR_SUCCESS )
			return XV11LIDAR_TTY_ERROR;	
						 
		return XV11LIDAR_SUCCESS;
	}
}

int read_all(int fd, uint8_t *data,int total_read_size)
{
	size_t bytes_read=0;
	int status;
	
	while( bytes_read < total_read_size )
	{   // this should never return 0 with our tty settings so treat it as error
		if( (status=read(fd,data+bytes_read,total_read_size-bytes_read)) <=0 )
		{
			perror("xv11lidar read failed: ");
			return XV11LIDAR_TTY_ERROR;
		}
			
		bytes_read+=status;
	}
	return XV11LIDAR_SUCCESS;
}

int is_frame_checksum_ok(const uint8_t data[FRAME_SIZE])
{
	uint16_t crc;
	memcpy(&crc, data+FRAME_CHECKSUM_OFFSET, sizeof(crc));
	return crc == checksum(data);
}

uint16_t checksum(const uint8_t data[FRAME_SIZE])
{
	uint32_t chk32=0;
	uint16_t word;
	int i;
	
	for(i=0;i<10;++i)
	{
		word=data[2*i] + (data[2*i+1] << 8);
		chk32 = (chk32 << 1) + word;
	}
	
	uint32_t checksum=(chk32 & 0x7FFF) + (chk32 >> 15);
	return checksum & 0x7FFF;
}

/*
 * Restore original terminal settings
 * Free the allocated buffer for laser readings
 * Close the terminal
 */
void xv11lidar_close(struct xv11lidar *lidar_data)
{	
	if(lidar_data==NULL)
		return;
	tcsetattr(lidar_data->fd, TCSAFLUSH, &lidar_data->old_io);	
	xv11lidar_free(lidar_data, NULL);
}

/*
 * Read from LIDAR until requested number of frames is read, error occurs or synchronization is lost
 * 
 */
int xv11lidar_read(struct xv11lidar *lidar_data, struct xv11lidar_frame *frame_data)
{
	const size_t total_read_size=FRAME_SIZE * lidar_data->laser_frames_per_read;
	uint8_t *data=lidar_data->data; 
	int i, j;
	int return_value = XV11LIDAR_SUCCESS;
	
	if( read_all(lidar_data->fd, data, total_read_size) != XV11LIDAR_SUCCESS )
		return XV11LIDAR_TTY_ERROR;
		
	memcpy(frame_data, data, total_read_size);
	
	for(i=0;i<lidar_data->laser_frames_per_read;++i)
	{
		lidar_data->last_frame_index = lidar_data->last_frame_index + 1;
		
		//each rotation we start index from 0 (0xA0)
		if(lidar_data->last_frame_index >= FRAMES_PER_ROTATION + FRAME_INDEX_0)		
			lidar_data->last_frame_index = FRAME_INDEX_0;
			
		if(checksum(data + i * FRAME_SIZE) != frame_data[i].checksum || frame_data[i].start != FRAME_START_BYTE)
		{
			++lidar_data->crc_failures;

			if(lidar_data->crc_failures > lidar_data->crc_tolerance)
				return_value = XV11LIDAR_SYNC_ERROR;
		
			frame_data[i].index = lidar_data->last_frame_index;
			for(j=0;j<READINGS_PER_FRAME;++j)
			{
				frame_data[i].readings[j].invalid_data=1;
				frame_data[i].readings[j].distance=XV11LIDAR_CRC_FAILURE;				
			}
	
			fprintf(stderr, " CRCFAIL ");			
		}
		
		if(frame_data[i].index == FRAME_INDEX_0)
			lidar_data->crc_failures = 0;
		if(frame_data[i].index != lidar_data->last_frame_index)
		{
			lidar_data->last_frame_index=frame_data[i].index;
			fprintf(stderr, " LIDAR_SKIPPED_SOME_FRAMES \n");				
		}
	}
	return return_value; 
}
