/* arch/arm/mack-rk29/include/mach/mirroring.h
 *
 * Copyright (C) 2007 Google, Inc.
 * author: chenhengming chm@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_RK29_MIRRORING_H
#define __ARCH_ARM_MACH_RK29_MIRRORING_H
#if	1//def CONFIG_FB_MIRRORING





#define MIRRORING_IOCTL_MAGIC 0x60
#define MIRRORING_START						_IOW(MIRRORING_IOCTL_MAGIC, 0x1, unsigned int)
#define MIRRORING_STOP						_IOW(MIRRORING_IOCTL_MAGIC, 0x2, unsigned int)
#define MIRRORING_SET_ROTATION				_IOW(MIRRORING_IOCTL_MAGIC, 0x3, unsigned int)
#define MIRRORING_DEVICE_OKAY				_IOW(MIRRORING_IOCTL_MAGIC, 0x4, unsigned int)
#define MIRRORING_SET_TIME					_IOW(MIRRORING_IOCTL_MAGIC, 0x5, unsigned int)
#define MIRRORING_GET_TIME					_IOW(MIRRORING_IOCTL_MAGIC, 0x6, unsigned int)
#define MIRRORING_GET_ROTATION				_IOW(MIRRORING_IOCTL_MAGIC, 0x7, unsigned int)
#define MIRRORING_SET_ADDR 					_IOW(MIRRORING_IOCTL_MAGIC, 0x8, unsigned int)
#define MIRRORING_GET_ADDR 					_IOW(MIRRORING_IOCTL_MAGIC, 0x9, unsigned int)
#define MIRRORING_SET_SIGNAL					_IOW(MIRRORING_IOCTL_MAGIC, 0xa, unsigned int)
#define MIRRORING_GET_SIGNAL					_IOW(MIRRORING_IOCTL_MAGIC, 0xb, unsigned int)
#define MIRRORING_SET_WIRELESSDB				_IOW(MIRRORING_IOCTL_MAGIC, 0xc, unsigned int)
#define MIRRORING_GET_WIRELESSDB				_IOW(MIRRORING_IOCTL_MAGIC, 0xd, unsigned int)
#define MIRRORING_GET_WIRELESS_DATALOST		_IOW(MIRRORING_IOCTL_MAGIC, 0xe, unsigned int)
#define MIRRORING_SET_FRAMERATE				_IOW(MIRRORING_IOCTL_MAGIC, 0xf, unsigned int)
#define MIRRORING_GET_CHIPINFO				_IOR(MIRRORING_IOCTL_MAGIC, 0x31, unsigned int)

#define MIRRORING_VIDEO_OPEN					_IOW(MIRRORING_IOCTL_MAGIC, 0x11, unsigned int)
#define MIRRORING_VIDEO_CLOSE				_IOW(MIRRORING_IOCTL_MAGIC, 0x12, unsigned int)
#define MIRRORING_VIDEO_GET_BUF				_IOW(MIRRORING_IOCTL_MAGIC, 0x13, unsigned int)

#define MIRRORING_AUDIO_OPEN           	      	_IOW(MIRRORING_IOCTL_MAGIC, 0x21, unsigned int)
#define MIRRORING_AUDIO_CLOSE                	_IOW(MIRRORING_IOCTL_MAGIC, 0x22, unsigned int)
#define MIRRORING_AUDIO_GET_BUF            		_IOW(MIRRORING_IOCTL_MAGIC, 0x23, unsigned int)
#define MIRRORING_AUDIO_SET_PARA                     _IOW(MIRRORING_IOCTL_MAGIC, 0x24, unsigned int)
#define	MIRRORING_AUDIO_SET_VOL			_IOW(MIRRORING_IOCTL_MAGIC, 0x25, unsigned int)
#define	MIRRORING_AUDIO_GET_VOL			_IOR(MIRRORING_IOCTL_MAGIC, 0x26, unsigned int)

#define	MIRRORING_AUDIO_SET_BUFFER_SIZE		0xa1
#define	MIRRORING_AUDIO_SET_BYTEPERFRAME		0xa2


#define MIRRORING_COUNT_ZERO				-111
#define VIDEO_ENCODER_CLOSED			-222
#define AUDIO_ENCODER_CLOSED			-222
#define ENCODER_BUFFER_FULL			-333


#define MIRRORING_IOCTL_ERROR			-1111

#endif

struct mirroring_platform_data
{
	const char* name;
	/* starting physical address of memory region */
	unsigned long start;
	/* size of memory region */
	unsigned long size;
	/* set to indicate maps of this region should be cached, if a mix of
	 * cached and uncached is desired, set this and open the device with
	 * O_SYNC to get an uncached region */
	unsigned cached;
	/* The MSM7k has bits to enable a write buffer in the bus controller*/
	unsigned buffered;
};

#endif //__ARCH_ARM_MACH_RK29_MIRRORING_H

