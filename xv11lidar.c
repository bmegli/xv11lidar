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

// those constants can be tuned
const int CRC_FAILURE_ERROR_CODE=666; //the error code encoded in distance when CRC failure happens (invalid data is also set)
const int REQUIRED_SYNC_FRAMES=45; //the number of consecutuve frames that is required to be read correctly (CRC) for sync

// those constants should not be touched
const int FRAME_SIZE=sizeof(struct laser_frame);	
const int FRAME_CHECKSUM_OFFSET=20;
const int FRAME_INDEX_OFFSET=1;

const uint8_t FRAME_START_BYTE=0xFA;
const uint8_t FRAME_INDEX_0=0xA0;

const int FRAMES_PER_ROTATION=90;
const int READINGS_PER_FRAME=4;

int SynchronizeLaser(int fd, int laser_frames_per_read);
int ReadAll(int fd, uint8_t *data,int total_read_size);
uint16_t Checksum(const uint8_t data[FRAME_SIZE]);
int IsFrameChecksumCorrect(const uint8_t data[FRAME_SIZE]);


/*
 * Initialize data values
 * Open the terminal
 * Save its original settings in lidar_data->old_io
 * Set terminal for raw byte input single byte at a time at 115200 speed
 * Synchronize with the laser
 * Alloc internal buffer for laser readings
 */
int InitLaser(struct xv11lidar_data *lidar_data, const char *tty, int laser_frames_per_read, int crc_tolerance_percent)
{
	int error;
	struct termios io;
	lidar_data->data=NULL;
	lidar_data->crc_failures = 0;
	lidar_data->crc_tolerance = crc_tolerance_percent * 90 / 100;
	lidar_data->last_frame_index = FRAME_INDEX_0 + FRAMES_PER_ROTATION - 1;

	if ((lidar_data->fd=open(tty, O_RDONLY))==-1)
		return TTY_ERROR;
	
	if(tcgetattr(lidar_data->fd, &lidar_data->old_io) < 0)
	{
		close(lidar_data->fd);
		return TTY_ERROR;		
	}
			
	io.c_iflag=io.c_oflag=io.c_lflag=0;
	io.c_cflag=CS8|CREAD|CLOCAL; //8 bit characters
	
	io.c_cc[VMIN]=1; //one input byte enough
	io.c_cc[VTIME]=0; //no timer
	
	if(cfsetispeed(&io, B115200) < 0 || cfsetospeed(&io, B115200) < 0)
	{
		close(lidar_data->fd);
		return TTY_ERROR;		
	}

	if(tcsetattr(lidar_data->fd, TCSAFLUSH, &io) < 0)
	{
		close(lidar_data->fd);
		return TTY_ERROR;
	}
				
	lidar_data->laser_frames_per_read=laser_frames_per_read;
		
	error=SynchronizeLaser(lidar_data->fd, laser_frames_per_read);
	if(error!=SUCCESS)
		close(lidar_data->fd);
	else if( (lidar_data->data=(uint8_t*)malloc(laser_frames_per_read*sizeof(struct laser_frame))) == 0)
	{
		close(lidar_data->fd);
		return MEMORY_ERROR;
	}
	return error;
}

/*
 * Restore original terminal settings
 * Free the allocated buffer for laser readings
 * Close the terminal
 */
int CloseLaser(struct xv11lidar_data *lidar_data)
{
	int error=SUCCESS;
	
	if(tcsetattr(lidar_data->fd, TCSAFLUSH, &lidar_data->old_io) < 0)
		error=TTY_ERROR;
	
	free(lidar_data->data);
	close(lidar_data->fd);
	
	return error;
}

/*
 * Read from LIDAR until requested number of frames is read or error occurs
 * 
 */
int ReadLaser(struct xv11lidar_data *lidar_data, struct laser_frame *frame_data)
{
	const size_t total_read_size=sizeof(struct laser_frame)*lidar_data->laser_frames_per_read;
	uint8_t *data=lidar_data->data; 
	int status=0;
	int i, j;
	
	if( ( status=ReadAll(lidar_data->fd, data, total_read_size) ) != SUCCESS)
		return status;
	
	memcpy(frame_data, data, total_read_size);
	
	for(i=0;i<lidar_data->laser_frames_per_read;++i)
	{
		lidar_data->last_frame_index = lidar_data->last_frame_index + 1;
		
		//each rotation we start index from 0 (0xA0)
		if(lidar_data->last_frame_index >= FRAMES_PER_ROTATION + FRAME_INDEX_0)		
			lidar_data->last_frame_index = FRAME_INDEX_0;
			
		if(Checksum(data+i*sizeof(struct laser_frame)) != frame_data[i].checksum || frame_data[i].start != FRAME_START_BYTE)
		{
			++lidar_data->crc_failures;
			if(lidar_data->crc_failures > lidar_data->crc_tolerance)
				return SYNCHRONIZATION_ERROR;
			frame_data[i].index = lidar_data->last_frame_index;
			for(j=0;j<READINGS_PER_FRAME;++j)
			{
				frame_data[i].readings[j].invalid_data=1;
				frame_data[i].readings[j].distance=CRC_FAILURE_ERROR_CODE;				
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
	return SUCCESS; 
}

/*
 * Flushes the TTY buffer so that we synchronize on new data
 * Waits for 0xFA byte and REQUIRED_SYNC_FRAMES consecutive frames with correct checksum
 * Discards the rest bytes so that next read starts from frame with index 0 (0xA0)
 * Finally sets the termios to return the data in largest possible chunks (termios.c_cc[VMIN])
 */
int SynchronizeLaser(int fd, int laser_frames_per_read)
{	
	int i, status;
	uint8_t data[FRAME_SIZE];
	
	//flush the current TTY data so that buffers are clean
	//The LIDAR may have been spinning for a while
	if(tcflush(fd, TCIOFLUSH)!=0)
		return TTY_ERROR;

	while(1)
	{
		if (read(fd,data,1)>0)
		{
			//wait for frame start
			if(data[0] == FRAME_START_BYTE)
			{
				if( (status=ReadAll(fd, data+1, FRAME_SIZE-1)) != SUCCESS )
					return TTY_ERROR;
			
				if( !IsFrameChecksumCorrect(data) )
					continue;

				for(i=1;i<REQUIRED_SYNC_FRAMES;++i)
				{
					if( (status=ReadAll(fd, data, FRAME_SIZE)) != SUCCESS )
						return TTY_ERROR;
					
					if( !IsFrameChecksumCorrect(data) )
						break;
				}
				if( i != REQUIRED_SYNC_FRAMES )
					continue;

				//discard bytes until 0 inxed frame		
				int index=*(data+FRAME_INDEX_OFFSET)-FRAME_INDEX_0;
				int bytes_to_discard=(FRAMES_PER_ROTATION-1-index) * FRAME_SIZE;
			
				for(i=0;i<bytes_to_discard;++i) 
					if (read(fd,data,1)<0)
						return TTY_ERROR;						
					
				//get the termios and set it to return data in largest possible chunks
				struct termios io;
				if(tcgetattr(fd, &io) < 0)				
					return TTY_ERROR;
							
				if(laser_frames_per_read*sizeof(struct laser_frame) <= UCHAR_MAX)					
					io.c_cc[VMIN]=laser_frames_per_read*sizeof(struct laser_frame); 
				else
					io.c_cc[VMIN]=11*sizeof(struct laser_frame); //11*22=242 which is the largest possible value <= UCHAR_MAX 	
					
				if(tcsetattr(fd, TCSANOW, &io) < 0)
					return TTY_ERROR;
		
				break;
			}	
		}
		else
			return TTY_ERROR;
	}
	return SUCCESS;
}


int ReadAll(int fd, uint8_t *data,int total_read_size)
{
	size_t bytes_read=0;
	int status;
	
	while( bytes_read < total_read_size )
	{
		if( (status=read(fd,data+bytes_read,total_read_size-bytes_read))<0 )
			return TTY_ERROR;
		else if(status==0)
			return TTY_ERROR;
			
		bytes_read+=status;
	}
	return SUCCESS;
}
uint16_t Checksum(const uint8_t data[FRAME_SIZE])
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
int IsFrameChecksumCorrect(const uint8_t data[FRAME_SIZE])
{
	uint16_t checksum;
	memcpy(&checksum, data+FRAME_CHECKSUM_OFFSET, sizeof(checksum));
	return checksum == Checksum(data);
}
