/* Himax Android Driver Sample Code for HIMAX chipset
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

#include "himax_platform.h"

#if defined(CONFIG_TOUCHSCREEN_HIMAX_TB8504_DEBUG)
#define D(x...) printk("[HXTP] " x)
#define I(x...) printk("[HXTP] " x)
#define W(x...) printk("[HXTP][WARNING] " x)
#define E(x...) printk("[HXTP][ERROR] " x)
#endif

DEFINE_MUTEX(tp_wr_access);

//Custom set some config
static int hx_panel_coords[4] = {0,720,0,1280};//[1]=X resolution, [3]=Y resolution
static int hx_display_coords[4] = {0,720,0,1280};
static int report_type = PROTOCOL_TYPE_B;

int irq_enable_count = 0;
u8 *gpDMABuf_va = NULL;
u8 *gpDMABuf_pa = NULL;

extern struct himax_ic_data* ic_data;
extern struct himax_ts_data *private_ts;
extern void himax_ts_work(struct himax_ts_data *ts);
extern enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer);

#ifdef HX_TP_PROC_DIAG
extern uint8_t getDiagCommand(void);
#endif

int himax_parse_dt(struct himax_ts_data *ts,struct himax_i2c_platform_data *pdata)
{
	//Set device tree data
	//Set panel coordinates
	pdata->abs_x_min = hx_panel_coords[0], pdata->abs_x_max = hx_panel_coords[1];
	pdata->abs_y_min = hx_panel_coords[2], pdata->abs_y_max = hx_panel_coords[3];
	I(" %s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
	pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);

	//Set display coordinates
	pdata->screenWidth  = hx_display_coords[1];
	pdata->screenHeight = hx_display_coords[3];
	I(" %s:display-coords = (%d, %d)", __func__, pdata->screenWidth,
	pdata->screenHeight);
	//report type
	pdata->protocol_type = report_type;
	pdata->gpio_irq = GPIO_CTP_EINT_PIN;
	pdata->gpio_reset = GPIO_CTP_RST_PIN;
	return 0;
}

int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int ret=0;
	s32 retry = 0;
	u8 buffer[1];

	struct i2c_msg msg[] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buffer,
			.len = 1,
			.timing = 400
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = gpDMABuf_pa,
			.len = length,
			.timing = 400
		},
	};
	mutex_lock(&tp_wr_access);
	buffer[0] = command;

	if (data == NULL){
		mutex_unlock(&tp_wr_access);
		return -1;
	}
	for (retry = 0; retry < toRetry; ++retry)
	{
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
		{
			continue;
		}
		memcpy(data, gpDMABuf_va, length);
		mutex_unlock(&tp_wr_access);
		return 0;
	}
	E("Dma I2C Read Error: %d byte(s), err-code: %d", length, ret);
	mutex_unlock(&tp_wr_access);
	return ret;
}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
	int rc=0,retry=0;
	u8 *pWriteData = gpDMABuf_va;

	struct i2c_msg msg[]={
		{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = gpDMABuf_pa,
		.len = len+1,
		.timing = 400
		},
	};
	
	mutex_lock(&tp_wr_access);
	if(!pWriteData)
	{
		E("dma_alloc_coherent failed!\n");
		mutex_unlock(&tp_wr_access);
		return -1;
	}

	gpDMABuf_va[0] = command;

	memcpy(gpDMABuf_va+1, buf, len);

	for (retry = 0; retry < toRetry; ++retry)
	{
		rc = i2c_transfer(client->adapter, &msg[0], 1);
		if (rc < 0)
		{
			continue;
		}
		mutex_unlock(&tp_wr_access);
		return 0;
	}

	E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
	mutex_unlock(&tp_wr_access);
	return rc;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t len, uint8_t *buf, uint8_t *readlength, uint8_t toRetry)
{
	return 0;
}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
	int rc=0, retry=0;
	u8 *pWriteData = gpDMABuf_va;

	struct i2c_msg msg[] ={
		{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = gpDMABuf_pa,
		.len = len,
		.timing = 400
		},
	};

	mutex_lock(&tp_wr_access);
	if(!pWriteData)
	{
		E("dma_alloc_coherent failed!\n");
		mutex_unlock(&tp_wr_access);
		return -1;
	}

	memcpy(gpDMABuf_va, buf, len);
	for (retry = 0; retry < toRetry; ++retry)
	{
		rc = i2c_transfer(client->adapter, &msg[0], 1);
		if (rc < 0)
		{
			continue;
		}
		mutex_unlock(&tp_wr_access);
		return 0;
	}
	E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
	mutex_unlock(&tp_wr_access);
	return rc;
}

void himax_int_enable(int irqnum, int enable)
{
	if (enable == 1 && irq_enable_count == 0) {
		mt_eint_unmask(irqnum);
		irq_enable_count++;
	} else if (enable == 0 && irq_enable_count == 1) {
		mt_eint_mask(irqnum);
		irq_enable_count--;
	}
	I("irq_enable_count = %d", irq_enable_count);
}

void himax_rst_gpio_set(int pinnum, uint8_t value)
{
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	if(value)
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	
}

uint8_t himax_int_gpio_read(int pinnum)
{
	return mt_get_gpio_in(GPIO_CTP_EINT_PIN);
}

int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata)
{
	int error=0;
	
	//mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	//mdelay(10);

	// TODO Interrupt / Reset Pin Setup
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);

	//Himax: SET Interrupt GPIO, no setting PULL LOW or PULL HIGH  
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
	//mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	client->irq = CUST_EINT_TOUCH_PANEL_NUM;
	// TODO Power Pin Setup
	//hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
	//hwPowerOn(MT65XX_POWER_LDO_VGP5, VOL_1800, "TP_EINT");
	msleep(10);

	// HW Reset
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);	
	msleep(20);

return error;
}

static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);

	himax_ts_work(ts);
}

#ifdef CONFIG_OF_TOUCH
irqreturn_t tpd_eint_interrupt_handler(int irq, void *desc)
{
	tpd_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
void tpd_eint_interrupt_handler(void)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}
#endif

static void himax_ts_isr_func(void)
{
	struct himax_ts_data *ts = private_ts;

	himax_ts_work(ts);
}

static int touch_event_handler(void *ptr)
{
	struct timespec timeStart, timeEnd, timeDelta;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter,tpd_flag!=0);

		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

		if (private_ts->debug_log_level & BIT(2)) {
				getnstimeofday(&timeStart);
				/*I(" Irq start time = %ld.%06ld s\n",
					timeStart.tv_sec, timeStart.tv_nsec/1000);*/
		}
#ifdef HX_SMART_WAKEUP
		if (atomic_read(&private_ts->suspend_mode)&&(!FAKE_POWER_KEY_SEND)&&(private_ts->SMWP_enable)) {
			I("Start to parse wake event\n");
			wake_lock_timeout(&private_ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
			msleep(200);
			himax_wake_check_func();
			continue;
		}
#endif
		himax_ts_isr_func();

		if(private_ts->debug_log_level & BIT(2)) {
				getnstimeofday(&timeEnd);
					timeDelta.tv_nsec = (timeEnd.tv_sec*1000000000+timeEnd.tv_nsec)
					-(timeStart.tv_sec*1000000000+timeStart.tv_nsec);
				/*I("Irq finish time = %ld.%06ld s\n",
					timeEnd.tv_sec, timeEnd.tv_nsec/1000);*/
				I("Touch latency = %ld us\n", timeDelta.tv_nsec/1000);
		}
	}
	while(!kthread_should_stop());

	return 0;
}


int himax_ts_register_interrupt(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;

	ts->irq_enabled = 0;
	ts->use_irq = 0;
	//Work functon
	if (client->irq) {/*INT mode*/
	ts->use_irq = 1;
	if(ic_data->HX_INT_IS_EDGE)
		{
			I("%s edge triiger falling\n ",__func__);
#ifdef CONFIG_OF_TOUCH
			ret = request_irq(client->irq, tpd_eint_interrupt_handler, EINTF_TRIGGER_FALLING,
			"TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
			}
#else
//			mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
//			mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
			mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
#endif
		}
	else
		{
			I("%s level trigger low\n ",__func__);
#ifdef CONFIG_OF_TOUCH
			ret = request_irq(client->irq, tpd_eint_interrupt_handler, EINTF_TRIGGER_LOW, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
			}
#else
//			mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE);
//			mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
//			mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 1);
			mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
#endif
		}

		ts->irq_enabled = 1;
		irq_enable_count = 1;
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		I("%s: irq register at qpio: %02X\n", __func__, CUST_EINT_TOUCH_PANEL_NUM);
#ifdef HX_SMART_WAKEUP
		irq_set_irq_wake(client->irq, 1);
#endif
		touch_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
		if (IS_ERR(touch_thread))
		{
			ret = PTR_ERR(touch_thread);
			E(" Failed to create kernel thread: %d\n", ret);
			return ret;
		}
	}
	else {
		I("%s: client->irq is empty, use polling mode.\n", __func__);
	}

	if (!ts->use_irq) {/*if use polling mode need to disable HX_ESD_WORKAROUND function*/
		ts->himax_wq = create_singlethread_workqueue("himax_touch");

		INIT_WORK(&ts->work, himax_ts_work_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		I("%s: polling mode enabled\n", __func__);
	}
	tpd_load_status = 1;
	return 0;
}

int himax_common_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, 4096, (dma_addr_t *)&gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va)
	{
		E("Allocate DMA I2C Buffer failed\n");
		ret = -ENODEV;
		goto err_alloc_MTK_DMA_failed;
	}
	memset(gpDMABuf_va, 0, 4096);
	i2c_client_point = client;

	ret = himax_chip_common_probe(client,id);
	if(ret)
		{
			if(gpDMABuf_va)
			{
				dma_free_coherent(&client->dev, 4096, gpDMABuf_va, (dma_addr_t)gpDMABuf_pa);
				gpDMABuf_va = NULL;
				gpDMABuf_pa = NULL;
			}
		}
err_alloc_MTK_DMA_failed:
	return ret;
}

int himax_common_remove(struct i2c_client *client)
{
	int ret = 0;

	himax_chip_common_remove(client);

	if(gpDMABuf_va)
	{
		dma_free_coherent(&client->dev, 4096, gpDMABuf_va, (dma_addr_t)gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = NULL;
	}
	return ret;
}

static void himax_common_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts = dev_get_drvdata(&i2c_client_point->dev);

	I("%s: enter \n", __func__);

	himax_chip_common_suspend(ts);

	return;
}

static void himax_common_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts = dev_get_drvdata(&i2c_client_point->dev);

	I("%s: enter \n", __func__);

	himax_chip_common_resume(ts);

	return;
}

#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *ts=
		container_of(self, struct himax_ts_data, fb_notif);

	I(" %s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
			ts->client) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			himax_common_resume(&ts->client->dev);
		break;

		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			himax_common_suspend(&ts->client->dev);
		break;
		}
	}

	return 0;
}
#endif
static int himax_common_detect (struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);
	 return 0;
}

static const struct i2c_device_id himax_common_ts_id[] = {
	{HIMAX_common_NAME, 0 },
	{}
};

static struct i2c_board_info __initdata himax_common_i2c = { I2C_BOARD_INFO(HIMAX_common_NAME, (0x90 >> 1))};

static struct i2c_driver tpd_i2c_driver =
{
    .probe = himax_common_probe,
    .remove = himax_common_remove,
    .detect = himax_common_detect,
    .driver.name = HIMAX_common_NAME,
    .id_table = himax_common_ts_id,
    .address_list = (const unsigned short *) forces,
};

static int himax_common_local_init(void)
{
	I("[Himax] Himax_ts I2C Touchscreen Driver local init\n");

    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        I("unable to add i2c driver.\n");
        return -1;
    }

    //input_set_abs_params(tpd->input_dev, ABS_MT_TRACKING_ID, 0, (HIMAX_MAX_TOUCH-1), 0, 0);

    // set vendor string
    //client->input_devid.vendor = 0x00;
    //client->input_dev->id.product = tpd_info.pid;
    //client-->input_dev->id.version = tpd_info.vid;

    I("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;

    return 0;
}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = HIMAX_common_NAME,
    .tpd_local_init = himax_common_local_init,
    .suspend = himax_common_suspend,
    .resume = himax_common_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

static void __init himax_common_init_async(void *unused, async_cookie_t cookie)
{
	I("%s:Enter \n", __func__);
	i2c_register_board_info(TPD_I2C_NUMBER, &himax_common_i2c, 1);
	tpd_driver_add(&tpd_device_driver);
}

static int __init himax_common_init(void)
{
	I("Himax common touch panel driver init\n");
	async_schedule(himax_common_init_async, NULL);
	return 0;
}

static void __exit himax_common_exit(void)
{
	tpd_driver_remove(&tpd_device_driver);
}

module_init(himax_common_init);
module_exit(himax_common_exit);

MODULE_DESCRIPTION("Himax_common driver");
MODULE_LICENSE("GPL");

