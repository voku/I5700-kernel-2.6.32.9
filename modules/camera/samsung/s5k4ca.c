/*
 *  Copyright (C) 2004 Samsung Electronics
 *             SW.LEE <hitchcar@samsung.com>
 *            - based on Russell King : pcf8583.c
 * 	      - added  smdk24a0, smdk2440
 *            - added  poseidon (s3c24a0+wavecom)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for FIMC2.x Camera Decoder
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <mach/hardware.h>

#include <mach/gpio.h>
#include <mach/gpio-cfg.h>
//#include <plat/egpio.h>

#include "../s3c_camif.h"

#include "s5k4ca.h"

// function define
//#define CONFIG_LOAD_FILE
#define I2C_BURST_MODE //dha23 100325 카메라 기동 시간 줄이기 위해 I2C Burst mode 사용.
// Purpose of verifying I2C operaion. must be ignored later.
//#define LOCAL_CONFIG_S5K4CA_I2C_TEST

static struct i2c_driver s5k4ca_driver;
int set_af_after_capture = 0; //dha23 100518
int af_success = 0; //dha23 100609
int locked_ae_awb = 0; //dha23 100526
unsigned short saved_wb_value = 0; //dha23 100526
int cam_flash_on = 0; //dha23 100527

static void s5k4ca_sensor_gpio_init(void);
void s5k4ca_sensor_enable(void);
static void s5k4ca_sensor_disable(void);

static int s5k4ca_sensor_init(void);
static void s5k4ca_sensor_exit(void);

static int s5k4ca_sensor_change_size(struct i2c_client *client, int size);

#ifdef CONFIG_FLASH_AAT1271A
	extern int aat1271a_flash_init(void);
	extern void aat1271a_flash_exit(void);
	extern void aat1271a_falsh_camera_control(int ctrl);
	extern void aat1271a_falsh_movie_control(int ctrl);
    extern void aat1271a_falsh_torch_camera_control(int ctrl); //hjkang_DE28	
#endif

#ifdef CONFIG_LOAD_FILE
	static int s5k4ca_regs_table_write(char *name);
#endif

/* 
 * MCLK: 24MHz, PCLK: 54MHz
 * 
 * In case of PCLK 54MHz
 *
 * Preview Mode (1024 * 768)  
 * 
 * Capture Mode (2048 * 1536)
 * 
 * Camcorder Mode
 */
static camif_cis_t s5k4ca_data = {
	itu_fmt:       	CAMIF_ITU601,
	order422:      	CAMIF_CRYCBY,
	camclk:        	24000000,		
	source_x:      	1024,		
	source_y:      	768,
	win_hor_ofst:  	0,
	win_ver_ofst:  	0,
	win_hor_ofst2: 	0,
	win_ver_ofst2: 	0,
	polarity_pclk: 	0,
	polarity_vsync:	1,
	polarity_href: 	0,
	reset_type:		CAMIF_RESET,
	reset_udelay: 	5000,
};

/* #define S5K4CA_ID	0x78 */

static int previous_scene_mode = -1;
static int af_mode = 2;
static unsigned short lux_value = 0;
static unsigned short Init_Position = 0x00DF; //dha23 100313 AF구현
static unsigned short Dummy_Position = 0x00E0; //dha23 100313 AF구현

static int s5k4ca_sensor_read(struct i2c_client *client,
		unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

static int s5k4ca_sensor_write(struct i2c_client *client,unsigned short subaddr, unsigned short val)
{
	if(subaddr == 0xdddd)
	{
			if (val == 0x0010)
				msleep(10);
			else if (val == 0x0020)
				msleep(20);
			else if (val == 0x0030)
				msleep(30);
			else if (val == 0x0040)
				msleep(40);
			else if (val == 0x0050)
				msleep(50);
			else if (val == 0x0100)
				msleep(100);
			printk("delay 0x%04x, value 0x%04x\n", subaddr, val);
	}	
	else
	{					
		unsigned char buf[4];
		struct i2c_msg msg = { client->addr, 0, 4, buf };

		buf[0] = (subaddr >> 8);
		buf[1] = (subaddr & 0xFF);
		buf[2] = (val >> 8);
		buf[3] = (val & 0xFF);

		return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	}

	return 0;
}

static int s5k4ca_sensor_write_list(struct i2c_client *client, struct samsung_short_t *list,char *name)
{
	int i, ret;
	ret = 0;
	
	printk("s5k4ca_sensor_write_list( %s ) \n", name); //dha23 100310
#ifdef CONFIG_LOAD_FILE 
	s5k4ca_regs_table_write(name);	
#else
	for (i = 0; list[i].subaddr != 0xffff; i++)
	{
		if(s5k4ca_sensor_write(client, list[i].subaddr, list[i].value) < 0)
			return -1;
	}
#endif
	return ret;
}

#ifdef I2C_BURST_MODE //dha23 100325

#define BURST_MODE_SET			1
#define BURST_MODE_END			2
#define NORMAL_MODE_SET			3
#define MAX_INDEX				1000
static int s5k4ca_sensor_burst_write_list(struct i2c_client *client, struct samsung_short_t *list,char *name)
{
	__u8 temp_buf[MAX_INDEX];
	int index_overflow = 1;
	int new_addr_start = 0;
	int burst_mode = NORMAL_MODE_SET;
	unsigned short pre_subaddr = 0;
	struct i2c_msg msg = { client->addr, 0, 4, temp_buf };
	int i=0, ret=0;
	unsigned int index = 0;
	
	printk("s5k4ca_sensor_burst_write_list( %s ) \n", name); 
#ifdef CONFIG_LOAD_FILE 
	s5k4ca_regs_table_write(name);	
#else
	for (i = 0; list[i].subaddr != 0xffff; i++)
	{
		if(list[i].subaddr == 0xdddd)
		{
			if (list[i].value == 0x0010)
				msleep(10);
			else if (list[i].value == 0x0020)
				msleep(20);
			else if (list[i].value == 0x0030)
				msleep(30);
			else if (list[i].value == 0x0040)
				msleep(40);
			else if (list[i].value == 0x0050)
				msleep(50);
			else if (list[i].value == 0x0100)
				msleep(100);
			printk("delay 0x%04x, value 0x%04x\n", list[i].subaddr, list[i].value);
		}	
		else
		{					
			if( list[i].subaddr == list[i+1].subaddr )
			{
				burst_mode = BURST_MODE_SET;
				if((list[i].subaddr != pre_subaddr) || (index_overflow == 1))
				{
					new_addr_start = 1;
					index_overflow = 0;
				}
			}
			else
			{
				if(burst_mode == BURST_MODE_SET)
				{
					burst_mode = BURST_MODE_END;
					if(index_overflow == 1)
					{
						new_addr_start = 1;
						index_overflow = 0;
					}
				}
				else
				{
					burst_mode = NORMAL_MODE_SET;
				}
			}

			if((burst_mode == BURST_MODE_SET) || (burst_mode == BURST_MODE_END))
			{
				if(new_addr_start == 1)
				{
					index = 0;
					memset(temp_buf, 0x00 ,1000);
					index_overflow = 0;

					temp_buf[index] = (list[i].subaddr >> 8);
					temp_buf[++index] = (list[i].subaddr & 0xFF);

					new_addr_start = 0;
				}
				
				temp_buf[++index] = (list[i].value >> 8);
				temp_buf[++index] = (list[i].value & 0xFF);
				
				if(burst_mode == BURST_MODE_END)
				{
					msg.len = ++index;

					ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("i2c_transfer fail ! \n");
						return -1;
					}
				}
				else if( index >= MAX_INDEX-1 )
				{
					index_overflow = 1;
					msg.len = ++index;
					
					ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("I2C_transfer Fail ! \n");
						return -1;
					}
				}
				
			}
			else
			{
				memset(temp_buf, 0x00 ,4);
			
				temp_buf[0] = (list[i].subaddr >> 8);
				temp_buf[1] = (list[i].subaddr & 0xFF);
				temp_buf[2] = (list[i].value >> 8);
				temp_buf[3] = (list[i].value & 0xFF);

				msg.len = 4;
				ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
				if( ret < 0)
				{
					printk("I2C_transfer Fail ! \n");
					return -1;
				}
			}
		}
		
		pre_subaddr = list[i].subaddr;
	}
#endif
	return ret;
}

#endif
static void s5k4ca_sensor_get_id(struct i2c_client *client)
{
	unsigned short id = 0;
	
	s5k4ca_sensor_write(client, 0x002C, 0x7000);
	s5k4ca_sensor_write(client, 0x002E, 0x01FA);
	s5k4ca_sensor_read(client, 0x0F12, &id);

	printk("Sensor ID(0x%04x) is %s!\n", id, (id == 0x4CA4) ? "Valid" : "Invalid"); 
}

#ifdef CONFIG_MACH_GT_I5700
#define I2C_CAM_DIS do {	\
	s3c_gpio_cfgpin(GPIO_I2C1_SCL, S3C_GPIO_INPUT);			\
	s3c_gpio_cfgpin(GPIO_I2C1_SDA, S3C_GPIO_INPUT);			\
	s3c_gpio_setpull(GPIO_I2C1_SCL, S3C_GPIO_PULL_DOWN);		\
	s3c_gpio_setpull(GPIO_I2C1_SDA, S3C_GPIO_PULL_DOWN);		\
} while (0)

#define I2C_CAM_EN do {		\
	s3c_gpio_cfgpin(GPIO_I2C1_SCL, S3C_GPIO_SFN(GPIO_I2C1_SCL_AF));	\
	s3c_gpio_cfgpin(GPIO_I2C1_SDA, S3C_GPIO_SFN(GPIO_I2C1_SDA_AF));	\
	s3c_gpio_setpull(GPIO_I2C1_SCL, S3C_GPIO_PULL_NONE);		\
	s3c_gpio_setpull(GPIO_I2C1_SDA, S3C_GPIO_PULL_NONE);		\
} while (0)

#define AF_PWR_DIS do {} while (0)
#define AF_PWR_EN do {} while (0)

#define	MCAM_RST_DIS do {	\
	/* MCAM RST Low */	\
	if (gpio_is_valid(GPIO_MCAM_RST_N)) {	\
		gpio_direction_output(GPIO_MCAM_RST_N, GPIO_LEVEL_LOW);	\
	}	\
	s3c_gpio_setpull(GPIO_MCAM_RST_N, S3C_GPIO_PULL_NONE);	\
} while (0)

#define	MCAM_RST_EN do {	\
	gpio_direction_output(GPIO_MCAM_RST_N, GPIO_LEVEL_HIGH);	\
} while (0)

#define	VCAM_RST_DIS do { } while (0)
#define	VCAM_RST_EN do { } while (0)

#define	CAM_PWR_DIS do {	\
	/* CAM PWR Low */	\
	if (gpio_is_valid(GPIO_CAM_EN)) {	\
		gpio_direction_output(GPIO_CAM_EN, GPIO_LEVEL_LOW);	\
	}	\
	s3c_gpio_setpull(GPIO_CAM_EN, S3C_GPIO_PULL_NONE);	\
} while (0)

#define	CAM_PWR_EN do {	\
	gpio_direction_output(GPIO_CAM_EN, GPIO_LEVEL_HIGH);	\
} while (0)

#define	MCAM_STB_EN do {	\
	/* MCAM STB High */	\
	gpio_set_value(GPIO_CAM_3M_STBY_N, GPIO_LEVEL_HIGH);	\
} while (0)	

#define	MCAM_STB_DIS do {	\
	/* CAM_3M STB Low */	\
	if (gpio_is_valid(GPIO_CAM_3M_STBY_N)) {	\
		gpio_direction_output(GPIO_CAM_3M_STBY_N, GPIO_LEVEL_LOW);	\
	}	\
	s3c_gpio_setpull(GPIO_CAM_3M_STBY_N, S3C_GPIO_PULL_NONE);	\
} while (0)

#define	VCAM_STB_DIS do { } while (0)
#endif

static void s5k4ca_sensor_gpio_init(void)
{
	I2C_CAM_DIS;
	MCAM_RST_DIS;
	VCAM_RST_DIS;
	CAM_PWR_DIS;
	AF_PWR_DIS;
	MCAM_STB_DIS;
	VCAM_STB_DIS;
}

#if defined(CONFIG_LDO_LP8720)
extern void	s5k4ca_sensor_power_init(void);	
#endif

void s5k4ca_sensor_enable(void)
{
	s5k4ca_sensor_gpio_init();

	MCAM_STB_EN;

	/* > 0 ms */
	msleep(1);

	AF_PWR_EN;	

#if defined(CONFIG_LDO_LP8720)
	s5k4ca_sensor_power_init();	
#endif

	CAM_PWR_EN;

	/* > 0 ms */
	msleep(1);

	/* MCLK Set */
	clk_set_rate(cam_clock, s5k4ca_data.camclk);

	/* MCLK Enable */
	clk_enable(cam_clock);
	clk_enable(cam_hclk);
	
	msleep(1);

	MCAM_RST_EN;
	
	msleep(40);
	
	I2C_CAM_EN;
}

static void s5k4ca_sensor_disable(void)
{
	I2C_CAM_DIS;
	
	MCAM_STB_DIS;

	/* > 20 cycles */
	msleep(1);

	/* MCLK Disable */
	clk_disable(cam_clock);
	clk_disable(cam_hclk);

	/* > 0 ms */
	msleep(1);

	MCAM_RST_DIS;

	/* > 0 ms */
	msleep(1);

	AF_PWR_DIS;

	CAM_PWR_DIS;
}

static int sensor_init(struct i2c_client *client)
{
	int ret = 0;

#ifdef I2C_BURST_MODE //dha23 100325
	if(s5k4ca_sensor_burst_write_list(client,s5k4ca_init0_04,"s5k4ca_init0_04") < 0)
		return -1;
#else	
	if(s5k4ca_sensor_write_list(client,s5k4ca_init0_04,"s5k4ca_init0_04") < 0)		
		return -1;
#endif		

	msleep(10);	

	/* Check Sensor ID */
	s5k4ca_sensor_get_id(client);
#ifdef I2C_BURST_MODE //dha23 100325
	ret = s5k4ca_sensor_burst_write_list(client,s5k4ca_init1_rev04,"s5k4ca_init1_rev04");
#else
	ret = s5k4ca_sensor_write_list(client,s5k4ca_init1_rev04,"s5k4ca_init1_rev04");
#endif	
	
	return 0;
}

static int s5k4ca_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	return 0;
}

static int s5k4ca_remove(struct i2c_client *i2c)
{
	return 0;
}

static int s5k4ca_sensor_mode_set(struct i2c_client *client, int type)
{
	unsigned short light;
	int delay = 300;

	printk("Sensor Mode ");

	if (type & SENSOR_PREVIEW)
	{	
		if( locked_ae_awb )
		{
			printk("AE/AWB Unlock!!! \n");
			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);
			s5k4ca_sensor_write(client, 0x002A, 0x0578);
			s5k4ca_sensor_write(client, 0x0F12, saved_wb_value);  //AE AWE UnLock : restore saved wb value
			locked_ae_awb = 0;
		}
	
		printk("-> Preview ");
#ifdef I2C_BURST_MODE //dha23 100325		
		s5k4ca_sensor_burst_write_list(client,s5k4ca_preview_04,"s5k4ca_preview_04"); //dha23 100325 I2C burst mode 적용.
#else
		s5k4ca_sensor_write_list(client,s5k4ca_preview_04,"s5k4ca_preview_04");
#endif
	}

	else if (type & SENSOR_CAPTURE)
	{	
		printk("-> Capture ");
			
		s5k4ca_sensor_write(client, 0x002C, 0x7000);	
		s5k4ca_sensor_write(client, 0x002E, 0x12FE);
		s5k4ca_sensor_read(client, 0x0F12, &light);
		lux_value = light;
	
		if (light <= 0x20) /* Low light */
		{	

			printk("Normal Low Light\n");
			delay = 0;//dha23 100401 <--800;
			
			s5k4ca_sensor_write_list(client,s5k4ca_snapshot_low_04,"s5k4ca_snapshot_low_04");
		}
	
		else
		{
			if (light <= 0x40)
			{
				printk("Normal Middle Lignt\n");
//					delay = 600;
			}
			else
			{	
				printk("Normal Normal Light\n");
//					delay = 300;
			}
			delay = 150; //hjkang_DE26 0 -> 150 //dha23 100401 <--200;
			s5k4ca_sensor_write_list(client,s5k4ca_capture_04,"s5k4ca_capture_04");
		}
		set_af_after_capture = 1; //dha23 100518
	}
	else if (type & SENSOR_FLASH_CAP_LOW)
	{	
		printk("flash Normal Low Light Capture\n");
  		delay = 0;//dha23 100401 <--300;

		s5k4ca_sensor_write_list(client,s5k4ca_flashcapture_low_04,"s5k4ca_flashcapture_low_04");
		set_af_after_capture = 1; //dha23 100518
	}
	else if (type & SENSOR_FLASH_CAPTURE)
	{	
		printk("flash Normal Normal Light Capture\n");
		delay = 0;//dha23 100401 <--300;

		s5k4ca_sensor_write_list(client,s5k4ca_flashcapture_04,"s5k4ca_flashcapture_04");
		set_af_after_capture = 1; //dha23 100518
	}
	else if (type & SENSOR_CAMCORDER )
	{
		printk("Record\n");
		//dha23 100313 Capture후 Lens 초기 위치로 돌림.<<<
		s5k4ca_sensor_write(client, 0xFCFC, 0xD000); 
		s5k4ca_sensor_write(client, 0x0028, 0x7000); 

		s5k4ca_sensor_write(client, 0x002A, 0x030E);  
		s5k4ca_sensor_write(client, 0x0F12, Dummy_Position); //중복되는 버그를 방지하기 위해...

		s5k4ca_sensor_write(client, 0x002A, 0x030C); 
		s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual 

		msleep(150); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
		//여기까지 lens 움직임 없음
		s5k4ca_sensor_write(client, 0x002A, 0x030E);  
		s5k4ca_sensor_write(client, 0x0F12, Init_Position); // AF Searching 초기 위치.

		msleep(300);  // Lens가 초기 위치로 돌아가는 이동시간.

		//dha23 100313 Capture후 Lens 초기 위치로 돌림. >>>
		delay = 300;
		s5k4ca_sensor_write_list(client,s5k4ca_fps_15fix_04,"s5k4ca_fps_15fix_04");
	}

	if(delay != 0)	msleep(delay);

	printk("delay time(%d msec)\n", delay);

	return 0;
	
}

static int s5k4ca_sensor_change_size(struct i2c_client *client, int size)
{
	switch (size) {
		case SENSOR_XGA:
			s5k4ca_sensor_mode_set(client, SENSOR_PREVIEW);
			break;

		case SENSOR_QXGA:
			s5k4ca_sensor_mode_set(client, SENSOR_CAPTURE);
			break;		
	
		default:
			printk("Unknown Size! (Only XGA & QXGA)\n");
			break;
	}

	return 0;
}


static int s5k4ca_sensor_af_control(struct i2c_client *client, int type)
{

	int count = 50;
	unsigned short tmpVal = 0;
	int ret = 0;
	unsigned short light = 0;

	switch (type)
	{
		case 0: // Release
			printk("[CAM-SENSOR] =Focus Mode -> release\n"); 
			printk("[CAM-SENSOR] Init_Position = 0x00%x, Dummy_Position = 0x00%x\n",Init_Position,Dummy_Position); 

			s5k4ca_sensor_write(client, 0xFCFC, 0xD000); 
			s5k4ca_sensor_write(client, 0x0028, 0x7000); 

			s5k4ca_sensor_write(client, 0x002A, 0x030E);  
			s5k4ca_sensor_write(client, 0x0F12, Dummy_Position);  //중복되는 버그를 방지하기 위해...

			s5k4ca_sensor_write(client, 0x002A, 0x030C); 
			s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual 

			msleep(150); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
			//여기까지 lens 움직임 없음
			s5k4ca_sensor_write(client, 0x002A, 0x030E);  
			s5k4ca_sensor_write(client, 0x0F12, Init_Position);  // AF Searching 초기 위치.

			msleep(300);  // Lens가 초기 위치로 돌아가는 이동시간.

			// AF Off할 경우 Lens 오동작 하기 때문에 AF Off 는 하지 않는다. 
			if( locked_ae_awb )
			{
				printk("AE/AWB Unlock!!! \n");
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x0028, 0x7000);
				s5k4ca_sensor_write(client, 0x002A, 0x0578);
				s5k4ca_sensor_write(client, 0x0F12, saved_wb_value);  //AE AWE UnLock : restore saved wb value
				locked_ae_awb = 0;
			}
			af_success = 0; //dha23 100609
			break;

		case 1: // AF start
			//dha23 100518 <<<
			if( set_af_after_capture || af_success) //dha23 100609
			{
				if( af_success && locked_ae_awb )
				{
					printk("AE/AWB Unlock!!! \n");
					s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
					s5k4ca_sensor_write(client, 0x0028, 0x7000);
					s5k4ca_sensor_write(client, 0x002A, 0x0578);
					s5k4ca_sensor_write(client, 0x0F12, saved_wb_value);  //AE AWE UnLock : restore saved wb value
					locked_ae_awb = 0;
					msleep(500);
				}

				switch( af_mode )
				{
					case 2:
						printk("[CAM-SENSOR] =Focus Mode -> AUTO after Capture! \n");	
						Init_Position = 0x00DF; //dha23 100312
						Dummy_Position = 0x00E0; //dha23 100312
						
						s5k4ca_sensor_write(client, 0xFCFC, 0xD000);    
						s5k4ca_sensor_write(client, 0x0028, 0x7000); // Set Page

						s5k4ca_sensor_write(client, 0x002A, 0x030E);  
						s5k4ca_sensor_write(client, 0x0F12, Dummy_Position); //중복되는 버그를 방지하기 위해...

						s5k4ca_sensor_write(client, 0x002A, 0x030C);    
						s5k4ca_sensor_write(client, 0x0F12, 0x0000); // Set Manual Mode

						msleep(150); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
						//여기까지 lens 움직임 없음
						
						s5k4ca_sensor_write(client, 0x002A, 0x030E);  
						s5k4ca_sensor_write(client, 0x0F12, Init_Position); // 0xDF는 AF Searching 초기 위치이다.
						
						s5k4ca_sensor_write(client, 0x002A, 0x161C);    
						s5k4ca_sensor_write(client, 0x0F12, 0x82A8); // Set Normal AF 
						break;

					case 4:
						printk("[CAM-SENSOR] =Focus Mode -> Macro after Capture! \n");
						Init_Position = 0x0050; //dha23 100312
						Dummy_Position = 0x004F; //dha23 100312
						
						s5k4ca_sensor_write(client, 0xFCFC, 0xD000);    
						s5k4ca_sensor_write(client, 0x0028, 0x7000); // Set Page

						s5k4ca_sensor_write(client, 0x002A, 0x030E);  
						s5k4ca_sensor_write(client, 0x0F12, Dummy_Position);  //중복되는 버그를 방지하기 위해...

						s5k4ca_sensor_write(client, 0x002A, 0x030C);    
						s5k4ca_sensor_write(client, 0x0F12, 0x0000); // Set Manual Mode

						msleep(150); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
						//여기까지 lens 움직임 없음
						s5k4ca_sensor_write(client, 0x002A, 0x030E);  
						s5k4ca_sensor_write(client, 0x0F12, Init_Position);  // 0xDF는 AF Searching 초기 위치이다.
						
						s5k4ca_sensor_write(client, 0x002A, 0x161C);
						s5k4ca_sensor_write(client, 0x0F12, 0xA2A8);  // Set Macro AF
						break;
				}
				set_af_after_capture = 0;
			}
			//dha23 100518 >>>
			
			printk("Focus Mode -> Single\n");

			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);	
			s5k4ca_sensor_write(client, 0x002C, 0x7000);
			s5k4ca_sensor_write(client, 0x002E, 0x12FE);
			//msleep(100);
			s5k4ca_sensor_read(client, 0x0F12, &light);

			lux_value = light;

			if (light < 0xFA) /* Low light AF */ // NB 기준값 250
			{
				s5k4ca_sensor_burst_write_list(client,s5k4ca_af_low_lux_val,"s5k4ca_af_low_lux_val"); //dha23 100331
				printk("[CAM-SENSOR] =Low Light AF Single light=0x%04x\n", light);
			}
			else
			{
				s5k4ca_sensor_burst_write_list(client,s5k4ca_af_normal_lux_val,"s5k4ca_af_normal_lux_val"); //dha23 100331
				printk("[CAM-SENSOR] =Normal Light AF Single light=0x%04x\n", light);
			}

			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);
			s5k4ca_sensor_write(client, 0x002A, 0x030C);
			s5k4ca_sensor_write(client, 0x0F12, 0x0002); //AF Single 

			if( cam_flash_on )
			{
				printk("AE/AWB is not locked for flash!!! \n");
			}
			else
			{
				printk("AE/AWB Lock!!! \n");
				saved_wb_value = 0; //dha23 100527
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x002C, 0x7000);
				s5k4ca_sensor_write(client, 0x002E, 0x0578);
				s5k4ca_sensor_read(client, 0x0F12, &saved_wb_value); // save existing WB value
				printk("saved_wb_value = 0x%x \n",saved_wb_value);
				
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x0028, 0x7000);
				s5k4ca_sensor_write(client, 0x002A, 0x0578);
				s5k4ca_sensor_write(client, 0x0F12, 0x0075);  //AE AWE Lock
				locked_ae_awb = 1; //dha23 100526
			}
			
			do
			{
				if( count == 0)
					break;

				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x002C, 0x7000);    
				s5k4ca_sensor_write(client, 0x002E, 0x130E);
				if (light < 0xFA) //dha23 100520
					msleep(250);
				else
					msleep(100);
				s5k4ca_sensor_read(client, 0x0F12, &tmpVal); 

				count--;

				printk("CAM 3M AF Status Value = 0x%x \n", tmpVal); 

			}while( (tmpVal & 0x3) != 0x3 && (tmpVal & 0x3) != 0x2 );

			if(count == 0 ) // TIMEOUT
			{
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000); 
				s5k4ca_sensor_write(client, 0x0028, 0x7000); 

				s5k4ca_sensor_write(client, 0x002A, 0x030E);  
				s5k4ca_sensor_write(client, 0x0F12, Dummy_Position);  //중복되는 버그를 방지하기 위해...

				s5k4ca_sensor_write(client, 0x002A, 0x030C); 
				s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual 

				msleep(150); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
				//여기까지 lens 움직임 없음
				s5k4ca_sensor_write(client, 0x002A, 0x030E);  
				s5k4ca_sensor_write(client, 0x0F12, Init_Position);  // AF Searching 초기 위치.
				
				msleep(300);  // Lens가 초기 위치로 돌아가는 이동시간.
				
				ret = 0;
				printk("CAM 3M AF_Single Mode Fail.==> TIMEOUT \n");

			}

			if((tmpVal & 0x3) == 0x02) // AF Fail 시 
			{
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000); 
				s5k4ca_sensor_write(client, 0x0028, 0x7000); 

				s5k4ca_sensor_write(client, 0x002A, 0x163C); 
				s5k4ca_sensor_write(client, 0x0F12, Init_Position); // AF Manual 

				ret = 0;
				printk("CAM 3M AF_Single Mode Fail.==> FAIL \n");
			}

			if((tmpVal & 0x3) == 0x3)
			{
				ret = 1;
				printk("CAM 3M AF_Single Mode SUCCESS. \r\n");
				af_success = 1; //dha23 100609
			}

			break;
			
		case 2: // auto
			if( af_mode != type)
			{
				set_af_after_capture = 0;
			}
		
			if( !set_af_after_capture )
			{
				printk("[CAM-SENSOR] =Focus Mode -> AUTO\n");	
				Init_Position = 0x00DF; //dha23 100312
				Dummy_Position = 0x00E0; //dha23 100312
				
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);    
				s5k4ca_sensor_write(client, 0x0028, 0x7000); // Set Page

				s5k4ca_sensor_write(client, 0x002A, 0x030E);  
				s5k4ca_sensor_write(client, 0x0F12, Dummy_Position); //중복되는 버그를 방지하기 위해...

				s5k4ca_sensor_write(client, 0x002A, 0x030C);    
				s5k4ca_sensor_write(client, 0x0F12, 0x0000); // Set Manual Mode

				msleep(150); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
				//여기까지 lens 움직임 없음
				
				s5k4ca_sensor_write(client, 0x002A, 0x030E);  
				s5k4ca_sensor_write(client, 0x0F12, Init_Position); // 0xDF는 AF Searching 초기 위치이다.

				msleep(300); // Lens가 초기 위치로 돌아가는 이동시간.
				
				s5k4ca_sensor_write(client, 0x002A, 0x161C);    
				s5k4ca_sensor_write(client, 0x0F12, 0x82A8); // Set Normal AF 
			}
			else
			{
				printk("[CAM-SENSOR] = Focus Mode -> AUTO dosen't set! \n");	
			}
			
			af_mode = 2;
			break;
		
		case 3: // infinity
			printk("[CAM-SENSOR] =Focus Mode -> infinity\n");	
			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);
			s5k4ca_sensor_write(client, 0x002A, 0x030E);    
			s5k4ca_sensor_write(client, 0x0F12, 0x0088); // dummy lens position

			s5k4ca_sensor_write(client, 0x002A, 0x030C);
			s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual
			msleep(130);

			s5k4ca_sensor_write(client, 0x002A, 0x030E);    
			s5k4ca_sensor_write(client, 0x0F12, 0x00FF);
			msleep(50);  
			af_mode = 3;
			break;
			
		case 4: // macro
			if( af_mode != type)
			{
				set_af_after_capture = 0;
			}
		
			if( !set_af_after_capture )
			{
				printk("[CAM-SENSOR] =Focus Mode -> Macro\n");
				Init_Position = 0x0050; //dha23 100312
				Dummy_Position = 0x004F; //dha23 100312
				
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);    
				s5k4ca_sensor_write(client, 0x0028, 0x7000); // Set Page

				s5k4ca_sensor_write(client, 0x002A, 0x030E);  
				s5k4ca_sensor_write(client, 0x0F12, Dummy_Position);  //중복되는 버그를 방지하기 위해...

				s5k4ca_sensor_write(client, 0x002A, 0x030C);    
				s5k4ca_sensor_write(client, 0x0F12, 0x0000); // Set Manual Mode

				msleep(150); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
				//여기까지 lens 움직임 없음
				s5k4ca_sensor_write(client, 0x002A, 0x030E);  
				s5k4ca_sensor_write(client, 0x0F12, Init_Position);  // 0xDF는 AF Searching 초기 위치이다.

				msleep(300); // Lens가 초기 위치로 돌아가는 이동시간.
				
				s5k4ca_sensor_write(client, 0x002A, 0x161C);
				s5k4ca_sensor_write(client, 0x0F12, 0xA2A8);  // Set Macro AF
			}	
			else
			{
				printk("[CAM-SENSOR] = Focus Mode -> Macro dosen't set! \n");	
			}
			
			af_mode = 4;
			break;
			
		case 5: // fixed, same as infinity
			printk("[CAM-SENSOR] =Focus Mode -> fixed\n");	
			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);
			s5k4ca_sensor_write(client, 0x002A, 0x030E);    
			s5k4ca_sensor_write(client, 0x0F12, 0x0088); // dummy lens position

			s5k4ca_sensor_write(client, 0x002A, 0x030C);
			s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual
			msleep(130);

			s5k4ca_sensor_write(client, 0x002A, 0x030E);    
			s5k4ca_sensor_write(client, 0x0F12, 0x00FF);
			msleep(50);  
			af_mode = 5;
			break;
		default:
			break;
	}
	return ret;
}

static int s5k4ca_sensor_change_effect(struct i2c_client *client, int type)
{
	printk("Effects Mode ");

	switch (type)
	{
		case 0:
		default:
			printk("-> Mode None\n");
			s5k4ca_sensor_write_list(client,s5k4ca_effect_off_04,"s5k4ca_effect_off_04");
		break;

		case 1:
			printk("-> Mode Gray\n");
			s5k4ca_sensor_write_list(client,s5k4ca_effect_gray_04,"s5k4ca_effect_gray_04");
		break;

		case 2:
			printk("-> Mode Sepia\n");
			s5k4ca_sensor_write_list(client,s5k4ca_effect_sepia_04,"s5k4ca_effect_sepia_04");
		break;

		case 3:
			printk("-> Mode Negative\n");
			s5k4ca_sensor_write_list(client,s5k4ca_effect_negative_04,"s5k4ca_effect_negative_04");
		break;
		
		case 4:
			printk("-> Mode Aqua\n");
			s5k4ca_sensor_write_list(client,s5k4ca_effect_aqua_04,"s5k4ca_effect_aqua_04");
		break;

		case 5:
			printk("-> Mode Sketch\n");
			s5k4ca_sensor_write_list(client,s5k4ca_effect_sketch_04,"s5k4ca_effect_sketch_04");
		break;
	}

	return 0;
	
}
static int s5k4ca_sensor_set_framerate(struct i2c_client *client, int type) //hjkang_DC11
{
	printk("Set FrameRate \n");
	switch(type)
	{
		case 0:
			s5k4ca_sensor_write_list(client,s5k4ca_fps_nonfix_04,"s5k4ca_fps_nonfix_04"); //camcorder -> camera hjkang_DC18
			break;

		case 15:
			s5k4ca_sensor_write_list(client,s5k4ca_fps_15fix_04,"s5k4ca_fps_15fix_04"); //camera -> camcorder
			break;

		default:
		break;
	}
 	set_af_after_capture = 0; //dha23 100520
	return 0;
}

static int s5k4ca_sensor_change_br(struct i2c_client *client, int type)
{
	printk("Brightness Mode \n");

	switch (type)
	{
		case 0: 
		default :
			printk("-> Brightness Minus 4\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_minus4_04,"s5k4ca_br_minus4_04");
			break;

		case 1:
			printk("-> Brightness Minus 3\n");	
			s5k4ca_sensor_write_list(client,s5k4ca_br_minus3_04,"s5k4ca_br_minus3_04");
			break;
		
		case 2:
			printk("-> Brightness Minus 2\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_minus2_04,"s5k4ca_br_minus2_04");
			break;
		
		case 3:				
			printk("-> Brightness Minus 1\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_minus1_04,"s5k4ca_br_minus1_04");
			break;
		
		case 4:
			printk("-> Brightness Zero\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_zero_04,"s5k4ca_br_zero_04");
			break;

		case 5:
			printk("-> Brightness Plus 1\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_plus1_04,"s5k4ca_br_plus1_04");
			break;

		case 6:
			printk("-> Brightness Plus 2\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_plus2_04,"s5k4ca_br_plus2_04");
			break;

		case 7:
			printk("-> Brightness Plus 3\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_plus3_04,"s5k4ca_br_plus3_04");
			break;

		case 8:
			printk("-> Brightness Plus 4\n");
			s5k4ca_sensor_write_list(client,s5k4ca_br_plus4_04,"s5k4ca_br_plus4_04");
			break;
		
	}

	return 0;

}

static int s5k4ca_sensor_change_wb(struct i2c_client *client, int type)
{
	printk("White Balance Mode ");

	switch (type)
	{
		case 0:
		default :
			printk("-> WB auto mode\n");
			s5k4ca_sensor_write_list(client,s5k4ca_wb_auto_04,"s5k4ca_wb_auto_04");
		break;
		
		case 1:
			printk("-> WB Sunny mode\n");
			s5k4ca_sensor_write_list(client,s5k4ca_wb_sunny_04,"s5k4ca_wb_sunny_04");
		break;

		case 2:
			printk("-> WB Cloudy mode\n");
			s5k4ca_sensor_write_list(client,s5k4ca_wb_cloudy_04,"s5k4ca_wb_cloudy_04");
		break;

		case 3:
			printk("-> WB Flourescent mode\n");
			s5k4ca_sensor_write_list(client,s5k4ca_wb_fluorescent_04,"s5k4ca_wb_fluorescent_04");
		break;

		case 4:
			printk("-> WB Tungsten mode\n");
			s5k4ca_sensor_write_list(client,s5k4ca_wb_tungsten_04,"s5k4ca_wb_tungsten_04");
		break;
	}
	return 0;
	
}

static int s5k4ca_sensor_change_scene_mode(struct i2c_client *client, int type)
{
	printk("[CAM-SENSOR] =Scene Mode %d",type);
	if(previous_scene_mode != 0 && type != 0)
	{
		printk("-> Pre-auto-set");
		s5k4ca_sensor_write_list(client,s5k4ca_scene_mode_off_04,"s5k4ca_scene_mode_off_04");
	}

	switch (type)
	{
		case 0:
			printk("-> auto\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_mode_off_04,"s5k4ca_scene_mode_off_04");
			previous_scene_mode = type;
			break;
		case 1:
			printk("-> portrait\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_portrait_on_04,"s5k4ca_scene_portrait_on_04");
			previous_scene_mode = type;
			break;
		case 2:
			printk("-> landscape\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_landscape_on_04,"s5k4ca_scene_landscape_on_04");
			previous_scene_mode = type;
			break;
		case 3:
			printk("-> night\n");
			s5k4ca_sensor_write_list(client,s5k4ca_nightmode_on_04,"s5k4ca_nightmode_on_04");
			previous_scene_mode = type;
			break;
		case 4:
			printk("-> sunset\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_sunset_on_04,"s5k4ca_scene_sunset_on_04");
			previous_scene_mode = type;
			break;
		case 5:
			printk("-> sports\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_sports_on_04,"s5k4ca_scene_sports_on_04");
			previous_scene_mode = type;
			break;
		case 6:
			printk("-> fireworks\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_fireworks_on_04,"s5k4ca_scene_fireworks_on_04");
			previous_scene_mode = type;
			break;
		case 7:
			printk("-> candlelight\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_candlelight_on_04,"s5k4ca_scene_candlelight_on_04");
			previous_scene_mode = type;
			break;
		case 8:
			printk("-> text\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_text_on_04,"s5k4ca_scene_text_on_04");
			previous_scene_mode = type;
			break;
		case 9:
			printk("-> beach snow\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_beach_snow_on_04,"s5k4ca_scene_beach_snow_on_04");
			previous_scene_mode = type;
			break;
		case 10:
			printk("-> party indoor\n");
			s5k4ca_sensor_write_list(client,s5k4ca_scene_party_indoor_on_04,"s5k4ca_scene_party_indoor_on_04");
			previous_scene_mode = type;
			break;
		default :
			printk("-> UnKnow Scene Mode\n");
			break;
	}

	previous_scene_mode = type;
	
	return 0;
}

static int s5k4ca_sensor_user_read(struct i2c_client *client, s5k4ca_t *r_data)
{
	s5k4ca_sensor_write(client, 0x002C, r_data->page);
	s5k4ca_sensor_write(client, 0x002E, r_data->subaddr);
	return s5k4ca_sensor_read(client, 0x0F12, &(r_data->value));
}

static int s5k4ca_sensor_user_write(struct i2c_client *client, unsigned short *w_data)
{
	return s5k4ca_sensor_write(client, w_data[0], w_data[1]);
}

static int s5k4ca_sensor_exif_read(struct i2c_client *client, exif_data_t *exif_data)
{
	int ret = 0;
	
//unsigned short lux = 0;
	unsigned short extime = 0;
	unsigned short iso_val = 0;

	s5k4ca_sensor_write(client, 0xFCFC, 0xD000);	/// exposure time
	s5k4ca_sensor_write(client, 0x002C, 0x7000);
	s5k4ca_sensor_write(client, 0x002E, 0x1C3C);
//	msleep(100);
	s5k4ca_sensor_read(client, 0x0F12, &extime);

//	msleep(100);

	s5k4ca_sensor_write(client, 0xFCFC, 0xD000);	/// ISO  hjkang_DE17 [
	s5k4ca_sensor_write(client, 0x002C, 0x7000);
	s5k4ca_sensor_write(client, 0x002E, 0x1C46);
//	msleep(100);
	s5k4ca_sensor_read(client, 0x0F12, &iso_val); 


    iso_val = iso_val * 10;
	iso_val = iso_val / 255;

	if(iso_val < 10){
		exif_data->iso = 64;
		printk("[CAM-SENSOR] =%s iso_val_read(%d) < 1.0 \n",__func__,iso_val);
	}
    else if(iso_val >= 10 && iso_val < 12)
        exif_data->iso = 64;
    else if(iso_val >= 12 && iso_val < 18)
        exif_data->iso = 100;
    else if(iso_val >= 18 && iso_val < 26)
        exif_data->iso = 125;
    else if(iso_val >= 26 && iso_val < 35)
        exif_data->iso = 200;
    else if(iso_val >= 36 && iso_val < 46)
        exif_data->iso = 250;
    else if(iso_val >= 46 && iso_val < 56)
        exif_data->iso = 320;
    else if(iso_val >= 56 && iso_val < 66)
        exif_data->iso = 400;
    else if(iso_val >= 66 && iso_val < 76)
        exif_data->iso = 500;
    else if(iso_val >= 76 && iso_val < 96)
        exif_data->iso = 640;
    else if(iso_val >= 96)
        exif_data->iso = 800;
    else{ 
        exif_data->iso = 0;
        printk("[CAM-SENSOR] =%s iso_val_read unknown range (%d) \n",__func__,iso_val);
		
    }

    printk("[CAM-SENSOR] =%s iso_val_read %d \n",__func__,iso_val);    //hjkang_DE17 ]
	
	exif_data->exposureTime = extime; //hjkang_DE18 /100	
	exif_data->lux = lux_value;
	printk("[CAM-SENSOR] =%s extime=%d, lux=%d,\n",__func__,extime,lux_value);

	return ret;
}

static int s5k4ca_sensor_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	struct v4l2_control *ctrl;
	unsigned short *w_data;		/* To support user level i2c */	
	s5k4ca_short_t *r_data;
	exif_data_t *exif_data;

	int ret=0;

	switch (cmd)
	{
		case SENSOR_INIT:
			ret = sensor_init(client);
			break;

		case USER_ADD:
			break;

		case USER_EXIT:
			s5k4ca_sensor_exit();
			break;

		case SENSOR_EFFECT:
			ctrl = (struct v4l2_control *)arg;
			s5k4ca_sensor_change_effect(client, ctrl->value);
			break;

		case SENSOR_BRIGHTNESS:
			ctrl = (struct v4l2_control *)arg;
			s5k4ca_sensor_change_br(client, ctrl->value);
			break;
			
		case SENSOR_FRAMERATE: //hjkang_DC11
			ctrl = (struct v4l2_control *)arg;
			s5k4ca_sensor_set_framerate(client, ctrl->value);
			break;

		case SENSOR_WB:
			ctrl = (struct v4l2_control *)arg;
			s5k4ca_sensor_change_wb(client, ctrl->value);
			break;

		case SENSOR_SCENE_MODE:
			ctrl = (struct v4l2_control *)arg;
			s5k4ca_sensor_change_scene_mode(client, ctrl->value);
			break;

		case SENSOR_AF:
			ctrl = (struct v4l2_control *)arg;
			ret = s5k4ca_sensor_af_control(client, ctrl->value);
			break;

		case SENSOR_MODE_SET:
			ctrl = (struct v4l2_control *)arg;
			s5k4ca_sensor_mode_set(client, ctrl->value);
			break;

		case SENSOR_XGA:
			s5k4ca_sensor_change_size(client, SENSOR_XGA);	
			break;

		case SENSOR_QXGA:
			s5k4ca_sensor_change_size(client, SENSOR_QXGA);	
			break;

		case SENSOR_QSVGA:
			s5k4ca_sensor_change_size(client, SENSOR_QSVGA);
			break;

		case SENSOR_VGA:
			s5k4ca_sensor_change_size(client, SENSOR_VGA);
			break;

		case SENSOR_SVGA:
			s5k4ca_sensor_change_size(client, SENSOR_SVGA);
			break;

		case SENSOR_SXGA:
			s5k4ca_sensor_change_size(client, SENSOR_SXGA);
			break;

		case SENSOR_UXGA:
			s5k4ca_sensor_change_size(client, SENSOR_UXGA);
			break;

		case SENSOR_USER_WRITE:
			w_data = (unsigned short *)arg;
			s5k4ca_sensor_user_write(client, w_data);
			break;

		case SENSOR_USER_READ:
			r_data = (s5k4ca_short_t *)arg;
			s5k4ca_sensor_user_read(client, r_data);
			break;
	
		case SENSOR_FLASH_CAMERA:
			ctrl = (struct v4l2_control *)arg;
			cam_flash_on = ctrl->value; //dha23 100527
			
#ifdef CONFIG_FLASH_AAT1271A			
			if(ctrl->id == 1)
			    aat1271a_falsh_camera_control(ctrl->value);	
			else if(ctrl->id == 2)
			    aat1271a_falsh_movie_control(ctrl->value);	
			else if(ctrl->id == 3)
			   aat1271a_falsh_torch_camera_control(ctrl->value);
#endif			
			printk("[SENSOR_FLASH_CAMERA] ctrl->value = %d ctrl->id = %d\n", ctrl->value, ctrl->id);
			break;

		case SENSOR_EXIF_DATA:
			exif_data = (exif_data_t *)arg;
			s5k4ca_sensor_exif_read(client, exif_data);	
			break;

		default:
			break;
	}

	return ret;
}

static const struct i2c_device_id s5k4ca_i2c_id[] = {
       { "s5k4ca", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, s5k4ca_i2c_id);

static struct i2c_driver s5k4ca_driver = {
	.driver = {
		   .name = "s5k4ca",
		   .owner = THIS_MODULE,
	},
	.probe = s5k4ca_probe,
	.remove = s5k4ca_remove,
	.id_table = s5k4ca_i2c_id,
	.command = s5k4ca_sensor_command,
};

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *s5k4ca_regs_table = NULL;

static int s5k4ca_regs_table_size;

void s5k4ca_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int i;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
#if 0
	filp = filp_open("/data/camera/s5k4ca.h", O_RDONLY, 0);
#else
	filp = filp_open("/sdcard/s5k4ca.h", O_RDONLY, 0);
#endif
	if (IS_ERR(filp)) {
		printk("file open error\n");
		return;
	}
	l = filp->f_path.dentry->d_inode->i_size;	
	printk("l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
	if (dp == NULL) {
		printk("Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		printk("Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return;
	}

	filp_close(filp, current->files);
	
	set_fs(fs);

	s5k4ca_regs_table = dp;
	
	s5k4ca_regs_table_size = l;

	*((s5k4ca_regs_table + s5k4ca_regs_table_size) - 1) = '\0';

	printk("s5k4ca_regs_table 0x%08x, %ld\n", dp, l);
}

void s5k4ca_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (s5k4ca_regs_table) {
		kfree(s5k4ca_regs_table);
		s5k4ca_regs_table = NULL;
	}	
}

static int s5k4ca_regs_table_write(char *name)
{
	char *start, *end, *reg, *data;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';

	start = strstr(s5k4ca_regs_table, name);
	
	end = strstr(start, "};");

	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{ 0x");		
		if (reg)
			start = (reg + 16);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 2), 6);	
			memcpy(data_buf, (reg + 10), 6);	
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
//			printk("addr 0x%04x, value 0x%04x\n", addr, value);
			if (addr == 0xdddd)
			{
/*				if (value == 0x0010)
					mdelay(10);
				else if (value == 0x0020)
					mdelay(20);
				else if (value == 0x0030)
					mdelay(30);
				else if (value == 0x0040)
					mdelay(40);
				else if (value == 0x0050)
					mdelay(50);
				else if (value == 0x0100)
					mdelay(100);*/
				mdelay(value);
				printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else
				s5k4ca_sensor_write(s5k4ca_data.sensor, addr, value);
		}
	}

	return 0;
}
#endif

static int s5k4ca_sensor_init(void)
{
	int ret;

	set_af_after_capture = 0; //dha23 100527
	locked_ae_awb = 0; //dha23 100527
	saved_wb_value = 0; //dha23 100527
	cam_flash_on = 0; //dha23 100527

	printk("[CAM_SENSOR] s5k4ca_sensor_init\n"); //dha23 100315
	
#ifdef CONFIG_LOAD_FILE
	s5k4ca_regs_table_init();
#endif

//	s5k4ca_sensor_enable();
	
	s3c_camif_open_sensor(&s5k4ca_data);

	if (s5k4ca_data.sensor == NULL)
		if ((ret = i2c_add_driver(&s5k4ca_driver)))
			return ret;

	if (s5k4ca_data.sensor == NULL) {
		i2c_del_driver(&s5k4ca_driver);	
		return -ENODEV;
	}

	s3c_camif_register_sensor(&s5k4ca_data);
	
	return 0;
}

static void s5k4ca_sensor_exit(void)
{
	printk("[CAM_SENSOR] s5k4ca_sensor_exit\n"); //dha23 100315

	s5k4ca_sensor_disable();

#ifdef CONFIG_LOAD_FILE
	s5k4ca_regs_table_exit();
#endif
	
	if (s5k4ca_data.sensor != NULL)
		s3c_camif_unregister_sensor(&s5k4ca_data);
}

static struct v4l2_input s5k4ca_input = {
	.index		= 0,
	.name		= "Camera Input (S5K4CA)",
	.type		= V4L2_INPUT_TYPE_CAMERA,
	.audioset	= 1,
	.tuner		= 0,
	.std		= V4L2_STD_PAL_BG | V4L2_STD_NTSC_M,
	.status		= 0,
};

static struct v4l2_input_handler s5k4ca_input_handler = {
	s5k4ca_sensor_init,
	s5k4ca_sensor_exit	
};

#ifdef CONFIG_VIDEO_SAMSUNG_MODULE
static int s5k4ca_sensor_add(void)
{
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_init();
#endif
	return s3c_camif_add_sensor(&s5k4ca_input, &s5k4ca_input_handler);
}

static void s5k4ca_sensor_remove(void)
{
	if (s5k4ca_data.sensor != NULL)
		i2c_del_driver(&s5k4ca_driver);
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_exit();
#endif
	s3c_camif_remove_sensor(&s5k4ca_input, &s5k4ca_input_handler);
}

module_init(s5k4ca_sensor_add)
module_exit(s5k4ca_sensor_remove)

MODULE_AUTHOR("Jinsung, Yang <jsgood.yang@samsung.com>");
MODULE_DESCRIPTION("I2C Client Driver For FIMC V4L2 Driver");
MODULE_LICENSE("GPL");
#else
int s5k4ca_sensor_add(void)
{
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_init();
#endif	
#ifdef LOCAL_CONFIG_S5K4CA_I2C_TEST
	return s5k4ca_sensor_init();
#else
	return s3c_camif_add_sensor(&s5k4ca_input, &s5k4ca_input_handler);
#endif
}

void s5k4ca_sensor_remove(void)
{
	if (s5k4ca_data.sensor != NULL)
		i2c_del_driver(&s5k4ca_driver);
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_exit();
#endif
	s3c_camif_remove_sensor(&s5k4ca_input, &s5k4ca_input_handler);
}
#endif
