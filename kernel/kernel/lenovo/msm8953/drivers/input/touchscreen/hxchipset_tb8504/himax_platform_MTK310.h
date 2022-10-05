/* Himax Android Driver Sample Code for Himax chipset
*
* Copyright (C) 2015 Himax Corporation.
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

#ifndef HIMAX_PLATFORM_H
#define HIMAX_PLATFORM_H

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>

#include <mach/board.h>
#include <mach/eint.h>

#include <cust_eint.h>
#include "himax_common.h"
#include "cust_gpio_usage.h"
#include "tpd.h"

#undef CONFIG_OF

#define MTK
#define MTK_KERNEL_310

#define TPD_I2C_NUMBER				2
#define HIMAX_I2C_RETRY_TIMES 		10
#define TPD_DEVICE            		"mtk-tpd"

#if defined(CONFIG_TOUCHSCREEN_HIMAX_TB8504_DEBUG)
#define D(x...) printk("[HXTP] " x)
#define I(x...) printk("[HXTP] " x)
#define W(x...) printk("[HXTP][WARNING] " x)
#define E(x...) printk("[HXTP][ERROR] " x)
#define DIF(x...) \
	if (debug_flag) \
	printk("[HXTP][DEBUG] " x) \
} while(0)
#else
#define D(x...)
#define I(x...)
#define W(x...)
#define E(x...)
#define DIF(x...)
#endif

#define HIMAX_common_NAME 				"himax_tp"
#define HIMAX_I2C_ADDR					0x48
#define INPUT_DEV_NAME					"himax-touchscreen"

extern struct tpd_device *tpd;
static struct i2c_client *i2c_client_point = NULL;
static int tpd_flag = 0;
static unsigned short force[] = {0, 0x90, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };

static struct task_struct *touch_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);


struct himax_i2c_platform_data {	
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	int screenWidth;
	int screenHeight;
	uint8_t fw_version;
	uint8_t tw_id;
	uint8_t powerOff3V3;
	uint8_t cable_config[2];
	uint8_t protocol_type;
	int gpio_irq;
	int gpio_reset;
	int gpio_3v3_en;
	int (*power)(int on);
	void (*reset)(void);
	struct himax_virtual_key *virtual_key;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;

	struct himax_config *hx_config;
	int hx_config_size;
};

extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
#ifdef CONFIG_OF_TOUCH
irqreturn_t tpd_eint_interrupt_handler(int irq, void *desc);
#else
void tpd_eint_interrupt_handler(void);
#endif
extern int irq_enable_count;
extern int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry);
extern int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry);
extern int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry);
extern int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry);
extern int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry);
extern void himax_int_enable(int irqnum, int enable);
extern int himax_ts_register_interrupt(struct i2c_client *client);
extern void himax_rst_gpio_set(int pinnum, uint8_t value);
extern uint8_t himax_int_gpio_read(int pinnum);

extern int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata);

#if defined(CONFIG_FB)
extern int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

#endif
