/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include "../../../richtek/rt-flashlight.h"
#include "kd_flashlight.h"
#include "kd_camera_typedef.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#undef DEF_RT5081_CHANNEL_CH1
#define RT5081_CHANNEL_CH1 0
#define RT5081_CHANNEL_CH2 1

static DEFINE_MUTEX(rt5081_mutex);
static DEFINE_MUTEX(rt5081_enable_mutex);
static DEFINE_MUTEX(rt5081_disable_mutex);
static int use_count = 0;

extern void rt5081_set_timeout_ms(int channel,unsigned long arg);
extern int rt5081_init(int channel);
extern int rt5081_uninit(int channel);
extern int rt5081_set_level(int channel, int level);
extern int rt5081_operate(int channel, int enable);

/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int rt5081_ioctl(unsigned int cmd, unsigned long arg)
{
#ifdef DEF_RT5081_CHANNEL_CH1
	int channel = RT5081_CHANNEL_CH1;
#else
	int channel = RT5081_CHANNEL_CH2;
#endif

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("xxxxxFLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel,(int)arg);
		rt5081_set_timeout_ms(channel,arg);
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("xxxxxxFLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)arg);
		rt5081_set_level(channel, arg);
		break;
	case FLASH_IOC_SET_ONOFF:
		pr_debug("xxxxxFLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)arg);
		rt5081_operate(channel, arg);
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)arg);
		return -ENOTTY;
	}

	return 0;
}

static int rt5081_set_driver(void)
{
	int ret = 0;

	/* init chip and set usage count */
	mutex_lock(&rt5081_mutex);
	if (!use_count)
#ifdef DEF_RT5081_CHANNEL_CH1
		ret = rt5081_init(RT5081_CHANNEL_CH1);
#else
		ret = rt5081_init(RT5081_CHANNEL_CH2);
#endif
	use_count++;
	mutex_unlock(&rt5081_mutex);

	pr_debug("Set driver: %d\n", use_count);

	return ret;
}

static int rt5081_unset_driver(void)
{
	int ret = 0;

	mutex_lock(&rt5081_mutex);
	use_count--;
	if (!use_count)
#ifdef DEF_RT5081_CHANNEL_CH1
		ret = rt5081_uninit(RT5081_CHANNEL_CH1);
#else
		ret = rt5081_uninit(RT5081_CHANNEL_CH2);
#endif
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&rt5081_mutex);

	pr_debug("UnSet driver: %d\n", use_count);

	return ret;
}

static int rt5081_open(void *pArg)
{
	pr_debug("xxxxx%s,\n",__func__);
	rt5081_set_driver();
	return 0;
}

static int rt5081_release(void *pArg)
{
	/* uninit chip and clear usage count */
	rt5081_unset_driver();

	return 0;
}


static FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
         rt5081_open,
         rt5081_release,
         rt5081_ioctl
};

MUINT32 strobeInit_main_sid2_part1(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
