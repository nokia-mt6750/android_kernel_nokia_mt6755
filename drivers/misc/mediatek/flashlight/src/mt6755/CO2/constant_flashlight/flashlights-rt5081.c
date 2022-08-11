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
#include "../../../../richtek/rt-flashlight.h"
#include "kd_flashlight.h"
#include "kd_camera_typedef.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>


//#include "mtk_charger.h"

//#include "flashlight.h"
//#include "flashlight-dt.h"

#define DEF_RT5081_CHANNEL_CH1 1

/* device tree should be defined in flashlight-dt.h */
#ifndef RT5081_DTNAME
#define RT5081_DTNAME "mediatek,flashlights_rt5081"
#endif

#define RT5081_NAME "flashlights-rt5081"

/* define channel, level */
#define RT5081_CHANNEL_NUM 2
#define RT5081_CHANNEL_CH1 0
#define RT5081_CHANNEL_CH2 1

#define RT5081_NONE (-1)
#define RT5081_DISABLE 0
#define RT5081_ENABLE 1
#define RT5081_ENABLE_TORCH 1
#define RT5081_ENABLE_FLASH 2

#define RT5081_LEVEL_NUM 11
#define RT5081_LEVEL_TORCH 2
#define RT5081_LEVEL_FLASH RT5081_LEVEL_NUM
#define RT5081_WDT_TIMEOUT 1248 /* ms */

/* define mutex, work queue and timer */
static DEFINE_MUTEX(rt5081_mutex);
static DEFINE_MUTEX(rt5081_enable_mutex);
static DEFINE_MUTEX(rt5081_disable_mutex);
static struct work_struct rt5081_work_ch1;
static struct work_struct rt5081_work_ch2;
static struct hrtimer rt5081_timer_ch1;
static struct hrtimer rt5081_timer_ch2;
static unsigned int rt5081_timeout_ms[RT5081_CHANNEL_NUM];

/* define usage count */
static int use_count = 0;

/* define RTK flashlight device */
struct flashlight_device *flashlight_dev_ch1;
struct flashlight_device *flashlight_dev_ch2;
#define RT_FLED_DEVICE_CH1  "rt-flash-led1"
#define RT_FLED_DEVICE_CH2  "rt-flash-led2"

/******************************************************************************
 * rt5081 operations
 *****************************************************************************/
/*
static const unsigned char rt5081_torch_level[RT5081_LEVEL_TORCH] = {
	0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E
};
*/
static const unsigned char rt5081_torch_level[RT5081_LEVEL_TORCH] = {
	0x04, 0x00
};
/*
static const unsigned char rt5081_strobe_level[RT5081_LEVEL_FLASH] = {
	0x80, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
	0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x20, 0x24, 0x28, 0x2C,
	0x30, 0x34, 0x38, 0x3C, 0x40, 0x44, 0x48, 0x4C, 0x50, 0x54,
	0x58, 0x5C
};
*/
static const unsigned char rt5081_strobe_level[RT5081_LEVEL_FLASH] = {
	0x00, 0x09, 0x12, 0x1B, 0x24, 0x2D, 0x36, 0x3F, 0x48, 0x51,
        0x5A
};

static int rt5081_en_ch1 = RT5081_NONE;
static int rt5081_level_ch1;
static int rt5081_en_ch2 = RT5081_NONE;
static int rt5081_level_ch2;
extern int flashlight_is_ready(struct flashlight_device *flashlight_dev);
extern int flashlight_strobe(struct flashlight_device *flashlight_dev);
extern int flashlight_set_mode(struct flashlight_device *flashlight_dev,int mode);
extern struct flashlight_device *find_flashlight_by_name(const char *name);
extern int flashlight_set_strobe_timeout(
		struct flashlight_device *flashlight_dev,
		int min_ms, int max_ms);
extern int flashlight_set_torch_brightness(
		struct flashlight_device *flashlight_dev,
		int brightness_level);
extern int flashlight_set_strobe_brightness(
		struct flashlight_device *flashlight_dev,
		int brightness_level);

static int rt5081_is_torch(int level)
{
	if (level >= RT5081_LEVEL_TORCH)
		return -1;

	return 0;
}

static int rt5081_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= RT5081_LEVEL_NUM)
		level = RT5081_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int rt5081_enable(void)
{
	enum flashlight_mode mode = FLASHLIGHT_MODE_TORCH;
	int ret = 0;
	if (!flashlight_dev_ch1)
		flashlight_dev_ch1 = find_flashlight_by_name(RT_FLED_DEVICE_CH1);
	if (!flashlight_dev_ch2)
		flashlight_dev_ch2 = find_flashlight_by_name(RT_FLED_DEVICE_CH2);

	if (!flashlight_dev_ch1 || !flashlight_dev_ch2) {
		pr_err("Failed to enable since no flashlight device.\n");
		return -1;
	}
	/* set flash mode if any channel is flash mode */
	if ((rt5081_en_ch1 == RT5081_ENABLE_FLASH)
			|| (rt5081_en_ch2 == RT5081_ENABLE_FLASH))
		mode = FLASHLIGHT_MODE_FLASH;

	/* enable channel 1 and channel 2 */
	if (rt5081_en_ch1)
	{
		ret |= flashlight_set_mode(
				flashlight_dev_ch1, mode);
		pr_debug("%s %d***%s****mode=%d\n",__func__,__LINE__,RT_FLED_DEVICE_CH1,mode);
	}
	else
		ret |= flashlight_set_mode(
				flashlight_dev_ch1, FLASHLIGHT_MODE_OFF);
	if (rt5081_en_ch2)
	{
		ret |= flashlight_set_mode(
				flashlight_dev_ch2, mode);
		pr_debug("%s %d***%s****mode=%d\n",__func__,__LINE__,RT_FLED_DEVICE_CH2,mode);
	}
	else
		ret |= flashlight_set_mode(
				flashlight_dev_ch2, FLASHLIGHT_MODE_OFF);
	if (ret < 0)
		pr_err("Failed to enable.\n");
	return ret;
}

/* flashlight disable function */
int rt5081_disable(void)
{
	int ret = 0;
	pr_debug("xxxxx%s,\n",__func__);
	if (!flashlight_dev_ch1 || !flashlight_dev_ch2) {
		pr_err("Failed to disable since no flashlight device.\n");
		return -1;
	}

	ret |= flashlight_set_mode(flashlight_dev_ch1, FLASHLIGHT_MODE_OFF);
	ret |= flashlight_set_mode(flashlight_dev_ch2, FLASHLIGHT_MODE_OFF);
	if (ret < 0)
		pr_err("Failed to disable.\n");
	return ret;
}

void rt5081_set_timeout_ms(int channel,unsigned long arg)
{
	rt5081_timeout_ms[channel] = arg;
}
/* set flashlight level */
static int rt5081_set_level_ch1(int level)
{
	level = rt5081_verify_level(level);
	rt5081_level_ch1 = level;

	if (!flashlight_dev_ch1) {
		pr_err("Failed to set ht level since no flashlight device.\n");
		return -1;
	}

	/* set brightness level */
	if (!rt5081_is_torch(level))
		flashlight_set_torch_brightness(
				flashlight_dev_ch1, rt5081_torch_level[level]);
	flashlight_set_strobe_brightness(
			flashlight_dev_ch1, rt5081_strobe_level[level]);

	return 0;
}
int rt5081_set_level_ch2(int level)
{
	level = rt5081_verify_level(level);
	rt5081_level_ch2 = level;

	if (!flashlight_dev_ch2) {
		pr_err("Failed to set lt level since no flashlight device.\n");
		return -1;
	}

	/* set brightness level */
	if (!rt5081_is_torch(level))
		flashlight_set_torch_brightness(
				flashlight_dev_ch2, rt5081_torch_level[level]);
	flashlight_set_strobe_brightness(
			flashlight_dev_ch2, rt5081_strobe_level[level]);

	return 0;
}
int rt5081_set_level(int channel, int level)
{
	if (channel == RT5081_CHANNEL_CH1)
		rt5081_set_level_ch1(level);
	else if (channel == RT5081_CHANNEL_CH2)
		rt5081_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static void rt5081_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	rt5081_disable();
}
static void rt5081_work_disable_ch2(struct work_struct *data)
{
	pr_debug("lt work queue callback\n");
	rt5081_disable();
}
static enum hrtimer_restart rt5081_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&rt5081_work_ch1);
	return HRTIMER_NORESTART;
}
static enum hrtimer_restart rt5081_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&rt5081_work_ch2);
	return HRTIMER_NORESTART;
}
int rt5081_timer_start(int channel, ktime_t ktime)
{
	if (channel == RT5081_CHANNEL_CH1)
		hrtimer_start(&rt5081_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == RT5081_CHANNEL_CH2)
		hrtimer_start(&rt5081_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int rt5081_timer_cancel(int channel)
{
	if (channel == RT5081_CHANNEL_CH1)
		hrtimer_cancel(&rt5081_timer_ch1);
	else if (channel == RT5081_CHANNEL_CH2)
		hrtimer_cancel(&rt5081_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int rt5081_init(int channel)
{
	if(channel == RT5081_CHANNEL_CH1 )
	{
		/* clear flashlight state */
		rt5081_en_ch1 = RT5081_NONE;
		INIT_WORK(&rt5081_work_ch1, rt5081_work_disable_ch1);

		/* init timer */
		hrtimer_init(&rt5081_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		rt5081_timer_ch1.function = rt5081_timer_func_ch1;
		rt5081_timeout_ms[RT5081_CHANNEL_CH1] = 600;

		flashlight_dev_ch1 = find_flashlight_by_name(RT_FLED_DEVICE_CH1);
		if (!flashlight_dev_ch1) {
			pr_err("Failed to get ht flashlight device.\n");
			return -EFAULT;
		}
		/* setup strobe mode timeout */
		if (flashlight_set_strobe_timeout(flashlight_dev_ch1, 400, 600) < 0)
			pr_err("Failed to set strobe timeout.\n");

		return 0;
	}
	else
	{
		/* clear flashlight state */
		rt5081_en_ch2 = RT5081_NONE;
		INIT_WORK(&rt5081_work_ch2, rt5081_work_disable_ch2);

		/* init timer */
		hrtimer_init(&rt5081_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		rt5081_timer_ch2.function = rt5081_timer_func_ch2;
		rt5081_timeout_ms[RT5081_CHANNEL_CH2] = 600;

		flashlight_dev_ch2 = find_flashlight_by_name(RT_FLED_DEVICE_CH2);
		if (!flashlight_dev_ch2) {
			pr_err("Failed to get lt flashlight device.\n");
			return -EFAULT;
		}
		/* setup strobe mode timeout */
		if (flashlight_set_strobe_timeout(flashlight_dev_ch2, 400, 600) < 0)
			pr_err("Failed to set strobe timeout.\n");

		return 0;
	}
}
/* flashlight uninit */
int rt5081_uninit(int channel)
{
	if(channel == 1)
	{
		/* clear flashlight state */
		rt5081_en_ch1 = RT5081_NONE;
		return rt5081_disable();
	}
	else
	{
		/* clear flashlight state */
		rt5081_en_ch2 = RT5081_NONE;
		return rt5081_disable();
	}
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
/******************************************************************************
 * Flashlight operation wrapper function
 *****************************************************************************/
int rt5081_operate(int channel, int enable)
{
	ktime_t ktime;

	/* setup enable/disable */
	if (channel == RT5081_CHANNEL_CH1) {
		rt5081_en_ch1 = enable;
		if (rt5081_en_ch1){
			if (!rt5081_is_torch(rt5081_level_ch1))
				rt5081_en_ch1 = RT5081_ENABLE_TORCH;
			else
				rt5081_en_ch1 = RT5081_ENABLE_FLASH;
		}
	} else if (channel == RT5081_CHANNEL_CH2) {
		rt5081_en_ch2 = enable;
		if (rt5081_en_ch2){
			if (!rt5081_is_torch(rt5081_level_ch2))
				rt5081_en_ch2 = RT5081_ENABLE_TORCH;
			else
				rt5081_en_ch2 = RT5081_ENABLE_FLASH;
		}
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	/* operate flashlight and setup timer */
	if ((rt5081_en_ch1 != RT5081_NONE) && (rt5081_en_ch2 != RT5081_NONE)) {
		if ((rt5081_en_ch1 == RT5081_DISABLE) &&
				(rt5081_en_ch2 == RT5081_DISABLE)) {
			rt5081_disable();
			rt5081_timer_cancel(RT5081_CHANNEL_CH1);
			rt5081_timer_cancel(RT5081_CHANNEL_CH2);
		} else if((rt5081_en_ch1 == RT5081_ENABLE_TORCH) && (rt5081_en_ch2 == RT5081_DISABLE)){
			rt5081_timer_cancel(RT5081_CHANNEL_CH1);
			rt5081_enable();
		} else if(rt5081_en_ch2 == RT5081_ENABLE_TORCH && (rt5081_en_ch1 == RT5081_DISABLE)){
			rt5081_timer_cancel(RT5081_CHANNEL_CH2);
			rt5081_enable();
		} else if((rt5081_en_ch2 == RT5081_ENABLE_TORCH) && (rt5081_en_ch1 == RT5081_ENABLE_TORCH)){
			//rt5081_en_ch2 = RT5081_DISABLE;
			rt5081_timer_cancel(RT5081_CHANNEL_CH1);
			rt5081_timer_cancel(RT5081_CHANNEL_CH2);
			rt5081_enable();
		} else {
			if (rt5081_timeout_ms[RT5081_CHANNEL_CH1]) {
				ktime = ktime_set(
						rt5081_timeout_ms[RT5081_CHANNEL_CH1] / 1000,
						(rt5081_timeout_ms[RT5081_CHANNEL_CH1] % 1000) * 1000000);
				rt5081_timer_start(RT5081_CHANNEL_CH1, ktime);
			}
			if (rt5081_timeout_ms[RT5081_CHANNEL_CH2]) {
				ktime = ktime_set(
						rt5081_timeout_ms[RT5081_CHANNEL_CH2] / 1000,
						(rt5081_timeout_ms[RT5081_CHANNEL_CH2] % 1000) * 1000000);
				rt5081_timer_start(RT5081_CHANNEL_CH2, ktime);
			}
			rt5081_enable();
		}

		/* clear flashlight state */
		rt5081_en_ch1 = RT5081_NONE;
		rt5081_en_ch2 = RT5081_NONE;
	}

	return 0;
}

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

static int rt5081_open(void *pArg)
{
	pr_debug("xxxxx%s,\n",__func__);
	rt5081_set_driver();
	return 0;
}

static int rt5081_release(void *pArg)
{
	/* uninit chip and clear usage count */
	pr_debug("xxxxx%s,\n",__func__);
	rt5081_unset_driver();

	return 0;
}

FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
         rt5081_open,
         rt5081_release,
         rt5081_ioctl
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int rt5081_probe(struct platform_device *pdev)
{
	pr_debug("Probe start.\n");

	/* clear attributes */
	use_count = 0;
	rt5081_en_ch1 = RT5081_NONE;
	rt5081_en_ch2 = RT5081_NONE;

	/* get RTK flashlight handler */
	flashlight_dev_ch1 = find_flashlight_by_name(RT_FLED_DEVICE_CH1);
	if (!flashlight_dev_ch1) {
		pr_err("Failed to get ht flashlight device.\n");
		return -EFAULT;
	}
	flashlight_dev_ch2 = find_flashlight_by_name(RT_FLED_DEVICE_CH2);
	if (!flashlight_dev_ch2) {
		pr_err("Failed to get lt flashlight device.\n");
		return -EFAULT;
	}

	/* setup strobe mode timeout */
	if (flashlight_set_strobe_timeout(flashlight_dev_ch1, 400, 600) < 0)
		pr_err("Failed to set strobe timeout.\n");
	if (flashlight_set_strobe_timeout(flashlight_dev_ch2, 400, 600) < 0)
		pr_err("Failed to set strobe timeout.\n");

	pr_debug("Probe done.\n");

	return 0;
}

static int rt5081_remove(struct platform_device *pdev)
{
	pr_debug("Remove start.\n");

	/* flush work queue */
	flush_work(&rt5081_work_ch1);
	flush_work(&rt5081_work_ch2);

	/* clear RTK flashlight device */
	flashlight_dev_ch1 = NULL;
	flashlight_dev_ch2 = NULL;
	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rt5081_of_match[] = {
	{.compatible = RT5081_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, rt5081_of_match);
#else
static struct platform_device rt5081_platform_device[] = {
	{
		.name = RT5081_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, rt5081_platform_device);
#endif

static struct platform_driver rt5081_platform_driver = {
	.probe = rt5081_probe,
	.remove = rt5081_remove,
	.driver = {
		.name = RT5081_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rt5081_of_match,
#endif
	},
};
static ssize_t  flash_proc_write(struct file *file, const char *buffer, size_t count,
		loff_t *data)
{
	int  ret = 0;
	int  val = 0;
	char regBuf[2] = {'\0'};
	u32  u4CopyBufSize = (count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	if (copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	sscanf(regBuf, "%d\n", &val);
	if(val <0 || val>8)
		pr_err("%s: val number is wrong!\n", __func__);

	if((int)val)
	{
#if 1
		if((int)val == 1)//LT Torch
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,1);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_ENABLE_TORCH);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,1);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_DISABLE);
		}
		else if((int)val == 2)//HT Torch
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,1);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_DISABLE);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,1);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_ENABLE_TORCH);
		}
		else if((int)val == 3)//Double Torch
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,1);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_ENABLE_TORCH);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,1);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_ENABLE_TORCH);
		}
		if((int)val == 4)//LT strobe
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,3);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_ENABLE_FLASH);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,1);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_DISABLE);
		}
		else if((int)val == 5)//HT strobe
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,1);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_DISABLE);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,3);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_ENABLE_FLASH);
		}
		else if((int)val == 6)//Double strobe,same as val=7/8
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,3);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_ENABLE_FLASH);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,3);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_ENABLE_FLASH);
		}
		else if((int)val == 7)//Double strobe,effect same as val=6/8
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,1);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_ENABLE_TORCH);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,3);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_ENABLE_FLASH);
		}
		else if((int)val == 8)//Double strobe,effect same as val=6/7
		{
			rt5081_init(RT5081_CHANNEL_CH1);
			rt5081_set_level(RT5081_CHANNEL_CH1,3);
			rt5081_operate(RT5081_CHANNEL_CH1,RT5081_ENABLE_FLASH);

			rt5081_init(RT5081_CHANNEL_CH2);
			rt5081_set_level(RT5081_CHANNEL_CH2,1);
			rt5081_operate(RT5081_CHANNEL_CH2,RT5081_ENABLE_TORCH);
		}
#endif
	}
	else
	{
		ret |= rt5081_disable();
	}

	if (ret)
		return -EFAULT;

	return count;
}

static ssize_t  flash_proc_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	        return 0;
}

static  struct file_operations flash_proc_fops = {
	.read = flash_proc_read,
	.write = flash_proc_write,
};

static int __init flashlight_rt5081_init(void)
{
	int ret;

	pr_debug("Init start.\n");
	proc_create("driver/flashlight", 0, NULL, &flash_proc_fops);

#ifndef CONFIG_OF
	ret = platform_device_register(&rt5081_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&rt5081_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_rt5081_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&rt5081_platform_driver);

	pr_debug("Exit done.\n");
}

/* replace module_init() since conflict in kernel init process */
late_initcall(flashlight_rt5081_init);
module_exit(flashlight_rt5081_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight RT5081 Driver");
