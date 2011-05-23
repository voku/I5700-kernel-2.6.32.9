/****************************************************************************
**
** COPYRIGHT(C)	: Samsung Electronics Co.Ltd, 2006-2015 ALL RIGHTS RESERVED
** Modified by Gabriel-LG (l.gorter@gmail.com)
** Finally cleaned and modified by Tomasz Figa <tomasz.figa at gmail.com>
**
*****************************************************************************/
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <mach/hardware.h>

#ifdef CONFIG_MACH_GT_I5700
#include <mach/gt_i5700.h>
#endif

#include <plat/gpio-cfg.h>
#include <linux/delay.h>
#include "timed_output.h"

#define GPIO_LABEL(gpio)	(#gpio)

/*********** for debug **********************************************************/
#if 0 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

#define VIBRATOR_DEF_HZ		22222	// 128 * actuator resonant frequency
#define VIBRATOR_DEF_DUTY	33	// weak vibrations (use 1 for strong)
#define VIBRATOR_MAX_TIMEOUT	5000

static int pwm_period = (NSEC_PER_SEC / VIBRATOR_DEF_HZ);
static int pwm_duty = VIBRATOR_DEF_DUTY;

static struct pwm_device *vibetonz_pwm;

static struct hrtimer timer;
static struct timed_output_dev timed_output_vt;
static DEFINE_MUTEX(vib_mutex);

static int set_vibetonz(int timeout)
{	
	int duty = timeout >> 16;

	timeout &= 0xFFFF;
	if (duty <= 0)
		duty = pwm_duty;
	if (duty > 100)
		duty = 100;

	if(!timeout) {	
		gpio_set_value(GPIO_VIB_EN, 0);
		pwm_disable(vibetonz_pwm);
	} else {
		pwm_config(vibetonz_pwm, (duty*pwm_period) / 100, pwm_period);
		pwm_enable(vibetonz_pwm);
		gpio_set_value(GPIO_VIB_EN, 1);
	}

	return timeout;
}

static enum hrtimer_restart vibetonz_timer_func(struct hrtimer *timer)
{
	set_vibetonz(0);

	return HRTIMER_NORESTART;
}

static int get_time_for_vibetonz(struct timed_output_dev *dev)
{
	int remaining;

	if (hrtimer_active(&timer)) {
		ktime_t r = hrtimer_get_remaining(&timer);
		remaining = r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else {
		remaining = 0;
	}

	return remaining;
}
static void enable_vibetonz_from_user(struct timed_output_dev *dev, int value)
{
	if(value == 0)
		return;

	mutex_lock(&vib_mutex);

	hrtimer_cancel(&timer);

	value = set_vibetonz(value);

	if (value > VIBRATOR_MAX_TIMEOUT)
		value = VIBRATOR_MAX_TIMEOUT;

	hrtimer_start(&timer, ktime_set(value / MSEC_PER_SEC,
		(value % MSEC_PER_SEC) * NSEC_PER_MSEC), HRTIMER_MODE_REL);

	mutex_unlock(&vib_mutex);
}

/* Frequency */

static ssize_t freq_store(struct device *aDevice, struct device_attribute *aAttribute, const char *aBuf, size_t aSize)
{
	if(sscanf(aBuf, "%d", &pwm_period)) {
		pwm_period = NSEC_PER_SEC / pwm_period;
		enable_vibetonz_from_user(&timed_output_vt, 1000);
	}

	return aSize;
}

static ssize_t freq_show(struct device *aDevice, struct device_attribute *aAttribute, char *aBuf)
{
	return sprintf(aBuf, "%ld\n", NSEC_PER_SEC / pwm_period);
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, freq_show, freq_store);

/* Duty */

static ssize_t duty_store(struct device *aDevice, struct device_attribute *aAttribute, const char *aBuf, size_t aSize)
{
	if(sscanf(aBuf, "%d", &pwm_duty)) {
		if (pwm_duty < 1)
			pwm_duty = 1;
		if (pwm_duty > 100)
			pwm_duty = 100;
	}

	return aSize;
}

static ssize_t duty_show(struct device *aDevice, struct device_attribute *aAttribute, char *aBuf)
{
	return sprintf(aBuf, "%d\n", pwm_duty);
}

static DEVICE_ATTR(duty, S_IRUGO | S_IWUSR, duty_show, duty_store);

/* Timed output */

static struct timed_output_dev timed_output_vt = {
	.name     = "vibrator",
	.get_time = get_time_for_vibetonz,
	.enable   = enable_vibetonz_from_user,
};

static int vibetonz_start(void)
{
	int ret = 0;

	/* hrtimer settings */
	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = vibetonz_timer_func;

	vibetonz_pwm = pwm_request(1, "vibetonz");
	if (IS_ERR(vibetonz_pwm)) {
		printk(KERN_ERR "Failed to request PWM timer 1 (%ld)\n",
							PTR_ERR(vibetonz_pwm));
		return -ENOENT;
	}

	/* pwm timer settings */
	pwm_config(vibetonz_pwm, (pwm_duty*pwm_period)/100, pwm_period);

	if (gpio_is_valid(GPIO_VIB_EN)) {
		if (gpio_request(GPIO_VIB_EN, GPIO_LABEL(GPIO_VIB_EN))) {
			printk(KERN_ERR "Failed to request GPIO_VIB_EN!\n");
			pwm_free(vibetonz_pwm);
			return -EBUSY;
		}
		pwm_enable(vibetonz_pwm);
		gpio_direction_output(GPIO_VIB_EN,1);
		mdelay(10);
		gpio_set_value(GPIO_VIB_EN, 0);
		pwm_disable(vibetonz_pwm);
	}
	s3c_gpio_setpull(GPIO_VIB_EN, S3C_GPIO_PULL_NONE);

	/* timed_output_device settings */
	ret = timed_output_dev_register(&timed_output_vt);
	if(ret) {
		printk(KERN_ERR "[VIBETONZ] timed_output_dev_register is fail \n");
		pwm_free(vibetonz_pwm);
		gpio_free(GPIO_VIB_EN);
		return -EINVAL;
	}

	ret = device_create_file(timed_output_vt.dev, &dev_attr_freq);
	if(ret) {
		printk(KERN_ERR "[VIBETONZ] failed to add freq attribute\n");
		timed_output_dev_unregister(&timed_output_vt);
		pwm_free(vibetonz_pwm);
		gpio_free(GPIO_VIB_EN);
		return -EINVAL;
	}
	
	ret = device_create_file(timed_output_vt.dev, &dev_attr_duty);
	if(ret) {
		printk(KERN_ERR "[VIBETONZ] failed to add duty attribute\n");
		timed_output_dev_unregister(&timed_output_vt);
		pwm_free(vibetonz_pwm);
		gpio_free(GPIO_VIB_EN);
		return -EINVAL;
	}
	
	return 0;
}


static void vibetonz_end(void)
{
	printk("[VIBETONZ] %s \n",__func__);
	device_remove_file(timed_output_vt.dev, &dev_attr_freq);
	timed_output_dev_unregister(&timed_output_vt);
	pwm_free(vibetonz_pwm);
	gpio_free(GPIO_VIB_EN);
}

static int __init vibetonz_init(void)
{
	
	return vibetonz_start();
}


static void __exit vibetonz_exit(void)
{
	vibetonz_end();
}

module_init(vibetonz_init);
module_exit(vibetonz_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("vibetonz control interface");
