/*
 * drivers/input/touchscreen/LU201x_ts.c
 *
 * LeadingUi LU201x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/syscalls.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>
#include <linux/sysdev.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/file.h>
#include <linux/uaccess.h>

#if defined (CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "lu201x_driver.h"

#define MAX_POINT_SIZE_FOR_LPWG		12

static char knock_on_type = 1;
static u8 multi_tap_count = 2;
static u8 KnockCode_over_check = 0;
static u8 gesture_property[MAX_POINT_SIZE_FOR_LPWG * 4] = {0};
static struct point lpwg_data[MAX_POINT_SIZE_FOR_LPWG + 1];
static char *lpwg_uevent[2][2] = {{ "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL },	{ "TOUCH_GESTURE_WAKEUP=PASSWORD", NULL }};
static struct wake_lock knock_code_lock;
static struct wake_lock touch_wake_lock;
static struct mutex irq_lock;
static struct mutex i2c_suspend_lock;
static struct mutex sys_lock;

static bool touch_irq_mask = 1;
static bool touch_irq_wake_mask = 0;

//static int debug = 1;
//module_param(debug, int, S_IRUGO|S_IWUSR);

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define LU201x_NAME	"touch_dev"

//SYSFS
static struct sys_device lge_touch_sys_device;
struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct lu201x_data *data, char *buf);
	ssize_t (*store)(struct lu201x_data *data, const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name = __ATTR(_name, _mode, _show, _store)

static void lu201x_irq_mask(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if(desc->irq_data.chip->irq_mask) {
		desc->irq_data.chip->irq_mask(&desc->irq_data);
	}
}

static void lu201x_irq_unmask(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if(desc->irq_data.chip->irq_unmask) {
		desc->irq_data.chip->irq_unmask(&desc->irq_data);
	}
}

static void touch_enable_irq(unsigned int irq)
{
	mutex_lock(&irq_lock);

	if (!touch_irq_mask) {
		touch_irq_mask = 1;
		lu201x_irq_unmask(irq);
		enable_irq(irq);
		TOUCH_INFO_MSG("%s\n",__FUNCTION__);
	}

	mutex_unlock(&irq_lock);
}

static void touch_disable_irq(unsigned int irq)
{
	mutex_lock(&irq_lock);

	if (touch_irq_mask) {
		touch_irq_mask = 0;
		disable_irq_nosync(irq);
		lu201x_irq_mask(irq);
		TOUCH_INFO_MSG("%s\n",__FUNCTION__);
	}

	mutex_unlock(&irq_lock);
}

static int touch_enable_irq_wake(unsigned int irq)
{
	int ret = 0;

	mutex_lock(&irq_lock);

	if (!touch_irq_wake_mask) {
		touch_irq_wake_mask = 1;
		ret = enable_irq_wake(irq);
		if (ret != 0)
			TOUCH_INFO_MSG("%s : %d \n", __func__, ret);
	}

	mutex_unlock(&irq_lock);
	return ret;
}

static int touch_disable_irq_wake(unsigned int irq)
{
	int ret = 0;

	mutex_lock(&irq_lock);

	if (touch_irq_wake_mask) {
		touch_irq_wake_mask = 0;
		ret = disable_irq_wake(irq);
		if (ret != 0)
			TOUCH_INFO_MSG("%s : %d \n", __func__, ret);
	}

	mutex_unlock(&irq_lock);

	return ret;
}


/* Resume code */
static void lu201x_resume_work_func(struct work_struct *resume_work)
{
	struct lu201x_data *data = container_of(to_delayed_work(resume_work), struct lu201x_data, resume_work);
	struct lu201x_platform_data *pdata = data->pdata;

	gpio_direction_output(pdata->gpio_reset, 1);
	gpio_direction_input(pdata->gpio_int);
	LU201x_reset(data, 0);
	LU201x_reset(data, 1);

	touch_enable_irq(data->irq);
	touch_disable_irq_wake(data->irq);
	LU201x_release_all_finger(data);
}

static int lu201x_update_file_name(struct device *dev, char **file_name, const char *buf, size_t count)
{
	char *file_name_tmp = NULL;

	/* Simple sanity check */
	if (count > 128) {
		TOUCH_INFO_MSG("File name too long %d\n", count);
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		TOUCH_INFO_MSG("no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

/****************************************************************************
* LU201x  I2C  Read / Write Funtions
****************************************************************************/
#define MAX_I2C_TRANSFER_SIZE (MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)
 static int i2c_read_bytes_LU201x( struct i2c_client *client, u16 addr, u8 *rxbuf, int len )
 {
	u8 buffer[I2C_DEVICE_ADDRESS_LEN];
	u8 retry;
	u16 left = len;
	u16 offset = 0;
	struct i2c_msg msg[2] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.buf = buffer,
			.len = I2C_DEVICE_ADDRESS_LEN,
			// .scl_rate = 100*1000
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			// .scl_rate = 100*1000
		},
	};

	if(rxbuf == NULL)
		 return -1;
#if 0
	TOUCH_INFO_MSG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len );
#endif
	while(left > 0) {
		buffer[0] = ((addr+offset) >> 8) & 0xFF;
		buffer[1] = (addr+offset) & 0xFF;

		msg[1].buf = &rxbuf[offset];

		if (left > MAX_TRANSACTION_LENGTH) {
			msg[1].len = MAX_TRANSACTION_LENGTH;
			left -= MAX_TRANSACTION_LENGTH;
			offset += MAX_TRANSACTION_LENGTH;
		} else {
			msg[1].len = left;
			left = 0;
		}

		retry = 0;

		while(i2c_transfer(client->adapter, &msg[0], 2) != 2) {
			retry++;

			if ( retry == 3 ) {
				TOUCH_INFO_MSG("I2C read 0x%X length=%d failed\n", addr + offset, len);
				return -1;
			}
		}
	}
	return 0;
 }

 static int i2c_write_bytes_LU201x( struct i2c_client *client, u16 addr, u8 *txbuf, int len )
 {
	 u8 buffer[MAX_TRANSACTION_LENGTH];
	 u16 left = len;
	 u16 offset = 0;
	 u8 retry = 0;

	 struct i2c_msg msg =
	 {
		 .addr = client->addr,
		 .flags = 0,
		 .buf = buffer,
		// .scl_rate=100*1000
	 };

	 if ( txbuf == NULL )
		 return -1;
#if 0
	TOUCH_INFO_MSG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len );
#endif
	 while (left > 0) {
		 retry = 0;
		 buffer[0] = ((addr+offset) >> 8) & 0xFF;
		 buffer[1] = (addr+offset ) & 0xFF;

		if (left > MAX_I2C_TRANSFER_SIZE) {
			 memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE );
			 msg.len = MAX_TRANSACTION_LENGTH;
			 left -= MAX_I2C_TRANSFER_SIZE;
			 offset += MAX_I2C_TRANSFER_SIZE;
		} else {
			 memcpy(&buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left);
			 msg.len = left + I2C_DEVICE_ADDRESS_LEN;
			 left = 0;
		}

		 //TOUCH_INFO_MSG("byte left %d offset %d\n", left, offset );
		 while (i2c_transfer(client->adapter, &msg, 1) != 1)
		{
			retry++;
			if (retry == 3) {
				TOUCH_INFO_MSG("I2C write 0x%02X%02X length=%d failed\n", buffer[0], buffer[1], len);
				return -1;
			} else
				TOUCH_INFO_MSG("I2C write retry %d addr 0x%02X%02X\n", retry, buffer[0], buffer[1]);
		}
	}

	 return 0;
 }

static void LU201x_i2c_done(struct lu201x_data *data)
{
	u8 i2c_done = CMD_I2C_DONE;

	i2c_write_bytes_LU201x(data->client, LU201x_I2CDONE_ADDR, &i2c_done, 1);
}

static void LU201x_set_i2c_mode(struct lu201x_data *data)
{
	// Add interrupt pin change for output mode
	gpio_direction_output(data->pdata->gpio_int, 0);
	// Add here interrupt pin goes to low
	gpio_set_value(data->pdata->gpio_int, 0);
	udelay(100);

	// Add here interrupt pin goes to high
	gpio_set_value(data->pdata->gpio_int, 1);
	// Add here interrupt pin change for input mode
	gpio_direction_input(data->pdata->gpio_int);
	mdelay(1);
}

/* Add here interrupt check routine */
static int LU201x_intPort_check(struct lu201x_data *data)
{
	// return value is high when interrupt pin is high
	return gpio_get_value(data->pdata->gpio_int);
}

static int LU201x_wait_ready_i2c(struct lu201x_data *data, u8 mode)
{
	int ret = 0;
	int dlycnt = 200;

	if (mode == 1) {
		dlycnt = 10;
	}
	/* wait till interrup pin goes low */
	while (LU201x_intPort_check(data) && dlycnt) {
		msleep(1);
		dlycnt--;
	}

	if (dlycnt == 0)
		ret = -1;

	return ret;
}

static int LU201x_read_register(struct lu201x_data *data, u16 addr,u8 *buf,u8 size)
{
	int ret = 1;

	LU201x_set_i2c_mode(data);
	ret = LU201x_wait_ready_i2c(data, 0);
	if (ret == -1) {
		TOUCH_INFO_MSG("Register Read failure\n");
	} else {
		ret = i2c_read_bytes_LU201x(data->client, addr, buf, size);
		LU201x_i2c_done(data);
	}

	return ret;
}

static int LU201x_set_mode(struct lu201x_data *data, u8 mode, u8 sub_mode)
{
	struct lu201x_platform_data *pdata = data->pdata;
	u8 buf[5]={CMD_LU201x_CHANGEMODE, 0, 0, 0, 0};
	u8 temp[2] = {0, 0};
	u16 tab_dlytime = 0;
	u16 code_tap_dlytime = 0;
	int ret = -1;

	buf[1] = (u8)(mode & 0xff);
	buf[3] = (u8)(sub_mode& 0xff);

	TOUCH_INFO_MSG("******enter %s LU201x_set_mode(0x%02x) - sub_mode(0x%02x)\n", \
		(mode == CMD_LU201x_PDN) ? "Power Down" : "LPWG mode", mode, sub_mode);
	LU201x_set_i2c_mode(data);

	/* wait till interrupt pin goes low */
	ret = LU201x_wait_ready_i2c(data, 1);
	if (ret == -1) {
		TOUCH_INFO_MSG("Register Read failure\n");
		goto exit;
	}

	ret = i2c_write_bytes_LU201x(data->client, LU201x_MODE_ADDR, buf, sizeof(buf));
	if (ret == -1) {
		TOUCH_INFO_MSG("Mode change Fail\n");
		goto exit;
	}

	if (mode != CMD_LU201x_PDN) {
		if (pdata->lpwg_mode == LPWG_DOUBLE_TAP) {
			TOUCH_INFO_MSG("Knock On mode\n");
			temp[0] = 1;
			i2c_write_bytes_LU201x(data->client, KNOCK_STATUS, temp, 1);
			temp[0] = 150;
			i2c_write_bytes_LU201x(data->client, KNOCK_TAP_THON, temp, 1);
			temp[0] = 8;
			i2c_write_bytes_LU201x(data->client, KNOCK_EXCEPT_PALM_ONCH, temp, 1);
			temp[0] = 70;
			i2c_write_bytes_LU201x(data->client, KNOCK_WAKEUP_INTERVAL, temp, 1);
			tab_dlytime = 500;
			temp[0] = tab_dlytime & 0x00FF;
			temp[1] = (tab_dlytime & 0xFF00) >> 8;
			i2c_write_bytes_LU201x(data->client, KNOCK_TAPOFF_TIMEOUT, temp, 2);
			temp[0] = 50;
			i2c_write_bytes_LU201x(data->client, KNOCK_ON_REPORT_WAIT_TIME, temp, 1);
			multi_tap_count = 2;
			i2c_write_bytes_LU201x(data->client, KNOCK_ON_TAP_COUNT, &multi_tap_count, 1);
		} else if (pdata->lpwg_mode == LPWG_MULTI_TAP) {
			/* enable both a Knock on and code */
			TOUCH_INFO_MSG("Knock On & Knock Code mode\n");
			temp[0] = 3;
			i2c_write_bytes_LU201x(data->client, KNOCK_STATUS, temp, 1);
			temp[0] = 150;
			i2c_write_bytes_LU201x(data->client, KNOCK_TAP_THON, temp, 1);
			temp[0] = 8;
			i2c_write_bytes_LU201x(data->client, KNOCK_EXCEPT_PALM_ONCH, temp, 1);
			temp[0] = 70;
			i2c_write_bytes_LU201x(data->client, KNOCK_WAKEUP_INTERVAL, temp, 1);

			if (pdata->double_tap_check == 1) {
				code_tap_dlytime = 500;
			} else {
				code_tap_dlytime = 300;
			}
			temp[0] = code_tap_dlytime & 0x00FF;
			temp[1] = (code_tap_dlytime & 0xFF00) >> 8;
			i2c_write_bytes_LU201x(data->client, KNOCK_ON_REPORT_WAIT_TIME, temp, 2);

			tab_dlytime = 700;
			temp[0] = tab_dlytime & 0x00FF;
			temp[1] = (tab_dlytime & 0xFF00) >> 8;
			i2c_write_bytes_LU201x(data->client, KNOCK_CODE_EXCEPT_TAP_WAIT_TIME, temp, 2);
			temp[0] = multi_tap_count;
			i2c_write_bytes_LU201x(data->client, KNOCK_CODE_TAP_COUNT, temp, 1);
		}
	}
	LU201x_i2c_done(data);
	msleep(2);

	/* wait till interrupt pin goes low */
	ret = LU201x_wait_ready_i2c(data, 1);
	if(ret == -1) {
		TOUCH_INFO_MSG("Register Read failure\n");
		goto exit;
	} else {
		i2c_read_bytes_LU201x(data->client, LU201x_CMDACK_ADDR, buf, sizeof(buf));
		LU201x_i2c_done(data);
	}

	if (buf[2] != CMD_LU201x_CHANGEMODE) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		goto exit;
	}

	return buf[2];
exit:
	gpio_direction_input(data->pdata->gpio_int);
	return -1;
}

static char *get_touch_button_string(u16 key_code)
{
	static char str[16] = {0};

	switch(key_code) {
		case KEY_BACK : /*158 0x9E*/
			sprintf(str, "BACK");
			break;
		case KEY_HOMEPAGE : /*172 0xAC*/
			sprintf(str, "HOME");
			break;
		case KEY_MENU : /* 139 0x8B*/
			sprintf(str, "MENU");
			break;
		case KEY_SIMSWITCH : /*249 0xF9*/
			sprintf(str, "APP_SWITCH");
			break;
		default :
			sprintf(str, "Unknown");
			break;
	}
	return str;
}

static void LU201x_release_all_finger(struct lu201x_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	int id = 0;

	TOUCH_INFO_MSG("Release all touch event!\n");

	for(id  = 0; id < MAX_FINGER_NUM; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	input_sync(input_dev);
}

static int LU201x_knock_check(int reason)
{
	static char str[16] = {0};
	int ret = 0;

	switch (reason) {
		case 1 :
			sprintf(str, "KNOCK ON");
			ret = 1;
			break;
		case 2:
			sprintf(str, "KNOCK CODE");
			ret = 1;
			break;
		case 255:
			sprintf(str, "OVER TAP");
			ret = 1;
			break;
		default :
			sprintf(str, "ERROR");
			ret = 0;
	}
	TOUCH_INFO_MSG("LPWG Mode-%s %d\n", str,reason);

	return ret;
}

static int LU201x_read_data(struct lu201x_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	struct ts_event *event = &data->event;
	struct LU201xTPD_INFO *LU201x_tpd = &(data->LU201x_tpd);
	static u8 pressed_key = 0;
	u8 *key_maps = data->pdata->key_maps;
	u8 tap_count = 0;
	u8 i = 0;
	int ret = 0;

	/* Knock On or Knock Code Check */
	if (data->pdata->lpwg_panel_on == LCD_OFF && data->pdata->lpwg_prox == PROX_FAR) {
		wake_lock_timeout(&knock_code_lock, msecs_to_jiffies(2000));
		ret = i2c_read_bytes_LU201x(data->client, KNOCK_ON_STAUS, &tap_count, 1);

		if (LU201x_knock_check(tap_count)) {
			ret = touch_knock_check(data);
			if ( ret != 0 )
			{
				TOUCH_INFO_MSG("touch_knock_check fail\n");
			}
		}
		goto exit_work;
	} else if(data->pdata->lpwg_panel_on == LCD_ON) {
		// Read touch information
		if (i2c_read_bytes_LU201x(data->client, FW_STATUS_REG, (u8 *)LU201x_tpd, sizeof(*LU201x_tpd)) == 0)
		{
			LU201x_i2c_done(data);
		} else {
			TOUCH_INFO_MSG(" LU201x I2C Communication error \n");
			LU201x_i2c_done(data);

			LU201x_reset(data, 0);
			LU201x_reset(data, 1);
			LU201x_release_all_finger(data);
			return -1;
		}

		if(LU201x_tpd->EventType == EVENT_KEY) {	/* Button Input */
			if (LU201x_tpd->KeyData[0] == 0) {
				input_report_key(input_dev, key_maps[pressed_key-1], KEY_RELEASED);
				TOUCH_INFO_MSG( "Touch Key[%d:%s] is released\n", pressed_key, get_touch_button_string(key_maps[pressed_key-1]));
				pressed_key = 0;
			}
			else
			{
				pressed_key = LU201x_tpd->KeyData[0];
				input_report_key(input_dev, key_maps[pressed_key-1], KEY_PRESSED);
				TOUCH_INFO_MSG( "Touch Key[%d:%s] is pressed\n", pressed_key, get_touch_button_string(key_maps[pressed_key-1]));
			}
			input_sync(input_dev);
			return 1;
		} else if (LU201x_tpd->EventType == EVENT_ABS) { /* Touch Input*/
			/*get previous touch info */
			for(i = 0; i < LU201x_tpd->VPCount; i++)	{
				if (event->point[i].status == TTYPE_PRESS || event->point[i].status == TTYPE_MOVE) {
					memcpy(&(event->prev_point[i]), &(event->point[i]), sizeof(event->prev_point[i]));
				}
			}
			event->prev_touch_point = event->touch_point;

			/*get the number of the touch finger*/
			event->touch_point = LU201x_tpd->VPCount;
			for(i = 0; i < event->touch_point; i++)	{
				event->point[i].x = ((LU201x_tpd->Point[i*4+1] & 0x07)<<8) | LU201x_tpd->Point[i*4];
				event->point[i].y = ((LU201x_tpd->Point[i*4+2] & 0x38)<<5) | ((LU201x_tpd->Point[i*4+2] & 0x07)<<5) |((LU201x_tpd->Point[i*4+1]>>3)&0x1f);
				event->point[i].id = ((LU201x_tpd->Point[i*4+3] & 0x07)<<3) | ((LU201x_tpd->Point[i*4+2] >> 6) &0x03);
				event->point[i].status = (LU201x_tpd->Point[i*4+3] >> 3) & 0x03;
			}
		} else {
			TOUCH_INFO_MSG(" Unknown Event occured from LU201x %d\n", LU201x_tpd->EventType);
			return -1;
		}

		return 0;
	}

exit_work:
	return -1;
}

static void print_touch_log(struct ts_event *event, u8 finger) {
	static u32 touch_log_cnt[MAX_FINGER_NUM] = {0, };
	static u8 touch_status[MAX_FINGER_NUM] = {TTYPE_RELEASE, TTYPE_RELEASE};
	u8 rel_chk = 0;
	u8 status = event->point[finger].status;
	u8 id = event->point[finger].id;

	switch (status) {
		case TTYPE_PRESS :
		case TTYPE_MOVE :
			if (touch_log_cnt[id] > 0xFFFF0000) {
				touch_log_cnt[id] = 2;
			}
			touch_log_cnt[id]++;
			touch_status[id] = status;
			if (touch_log_cnt[id] == 1) {
				TOUCH_INFO_MSG("Touch[%d] pressed [%d,%d]\n",
					event->point[finger].id, event->point[finger].x, event->point[finger].y);
			}
			break;
		case TTYPE_RELEASE:
			rel_chk = touch_status[id] ^ TTYPE_RELEASE;
			touch_log_cnt[id] = 0;
			touch_status[id] = TTYPE_RELEASE;
			if (rel_chk) {
				TOUCH_INFO_MSG("Touch[%d] release [%d,%d]\n",
					event->prev_point[finger].id, event->prev_point[finger].x, event->prev_point[finger].y);
			}
		default :
			break;
	}

	/* Buffer clear */
	if (touch_status[0] == TTYPE_RELEASE && touch_status[1] == TTYPE_RELEASE) {
		memset(touch_log_cnt, 0, sizeof(touch_log_cnt));
		memset(touch_status, TTYPE_RELEASE, sizeof(touch_status));
	}
}

static void LU201x_report_value(struct lu201x_data *data)
{
	struct ts_event *event = &data->event;
	struct input_dev *input_dev = data->input_dev;
	static u8 touch_pressure = 0;
	u8 i = 0;

#if 0
	for (i=0; i<event->touch_point; i++) {
		TOUCH_INFO_MSG("[%d] point ID:%d is %d x0 is %d y0 is %d\n",
				event->point[i].status,
				event->point[i].id,
				event->touch_point,
				event->point[i].x, event->point[i].y);
	}
#endif
	touch_pressure ^= 1;

	for(i = 0; i < event->touch_point; i++) {
		print_touch_log(event, i);
		if (event->point[i].status == TTYPE_PRESS || event->point[i].status == TTYPE_MOVE) {
			input_mt_slot(input_dev, event->point[i].id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, event->point[i].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, event->point[i].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE, 100+touch_pressure);
			input_report_abs(input_dev,ABS_MT_WIDTH_MAJOR, 10);
			input_report_abs(input_dev,ABS_MT_WIDTH_MINOR, 4);
		} else if (event->point[i].status == TTYPE_RELEASE) {
			input_mt_slot(input_dev, event->prev_point[i].id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		}
	}
	input_sync(data->input_dev);
}

static void LU201x_ts_pen_irq_work(struct work_struct *work)
{
	struct lu201x_data *data = container_of(work, struct lu201x_data, pen_event_work);
	int ret = -1;

	mutex_lock(&i2c_suspend_lock);
	ret = LU201x_read_data(data);
	if (ret == 0) {
		LU201x_report_value(data);
	}
	mutex_unlock(&i2c_suspend_lock);
}

static irqreturn_t LU201x_ts_interrupt(int irq, void *dev_id)
{
	struct lu201x_data *data = dev_id;

	queue_work(data->ts_workqueue, &data->pen_event_work);

	return IRQ_HANDLED;
}

/****************************************************************************
* Touch KNOCK ON/CODE Function
****************************************************************************/
static void lpwg_data_read(void)
{
	int i = 0;

	if (KnockCode_over_check) {
		lpwg_data[0].x = 1;
		lpwg_data[0].y = 1;
		lpwg_data[1].x = -1;
		lpwg_data[1].y = -1;
		KnockCode_over_check = 0;
	} else {
		for (i = 0; i < MAX_POINT_SIZE_FOR_LPWG; i++) {
			lpwg_data[i].x = (gesture_property[4*i+1] << 8 | gesture_property[4*i]);
			lpwg_data[i].y = (gesture_property[4*i+3] << 8 | gesture_property[4*i+2]);
			//TOUCH_INFO_MSG("TAP Position x[%3d], y[%3d]\n", lpwg_data[i].x, lpwg_data[i].y);
			// '-1' should be assinged to the last data.
			// Each data should be converted to LCD-resolution.
		}
	}
}

static int touch_knock_check(struct lu201x_data *data)
{
	int ret = 0;
	u8 tap = 0;

	ret = i2c_read_bytes_LU201x(data->client, KNOCK_TAP_COUNT, &tap, 1);

	if (tap > MAX_POINT_SIZE_FOR_LPWG) {
		TOUCH_INFO_MSG("Tap count exceed 12 tap - %d\n", tap);
		return -1;
	}

	ret = i2c_read_bytes_LU201x(data->client, KNOCK_TAP_COUNT+1, gesture_property, tap * 4);
	LU201x_i2c_done(data);
	if (ret != 0)
	{
		TOUCH_INFO_MSG("KNOCK_TAP_COUNT read fail\n");
		return -1;
	}

	if (tap == 2) {
		TOUCH_INFO_MSG( "Knock On occured!!\n" );
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_DOUBLE_TAP-1] );
	} else if (multi_tap_count > 2 && 2 < tap && tap <= MAX_POINT_SIZE_FOR_LPWG) {
		if (multi_tap_count < tap) {
			KnockCode_over_check = 1;
		}
		TOUCH_INFO_MSG( "Knock Code occured!!\n" );
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_MULTI_TAP-1] );
	}

	return 0;
}

static void lpwg_mode_change(struct lu201x_data *data, int lpwg_mode) {
	struct lu201x_platform_data *pdata = data->pdata;
	int ret = 0;
	int retry = 0;

	switch (lpwg_mode) {
		case LPWG_DOUBLE_TAP :
			do {
				ret = LU201x_set_mode(data, CMD_LU201x_IDLE_DOUBLETAB, 0);
				if (ret == -1) {
					gpio_direction_output(pdata->gpio_reset, 1);
					gpio_direction_input(pdata->gpio_int);
					LU201x_reset(data, 0);
					LU201x_reset(data, 1);
					retry++;
					TOUCH_INFO_MSG ( "set_mode retry %d\n", retry );
				}
			} while (ret == -1 && retry < 2);
			break;

		case LPWG_MULTI_TAP :
			do {
				ret = LU201x_set_mode(data, CMD_LU201x_IDLE_MULTITAB, 0);
				if (ret == -1) {
					gpio_direction_output(pdata->gpio_reset, 1);
					gpio_direction_input(pdata->gpio_int);
					LU201x_reset(data, 0);
					LU201x_reset(data, 1);
					retry++;
					TOUCH_INFO_MSG ( "set_mode retry %d\n", retry );
				}
			} while (ret == -1 && retry < 2);
			break;

		case LPWG_DISABLE :
			ret = LU201x_set_mode(data, CMD_LU201x_PDN, 0);
			if(ret != CMD_LU201x_CHANGEMODE) {
				LU201x_reset(data, 0);
				LU201x_reset(data, 1);
				TOUCH_INFO_MSG("******enter ts_suspend mode failure\n");
			} else {
				TOUCH_INFO_MSG("******enter ts_suspend mode ok\n");
			}
			break;

		default :
			TOUCH_INFO_MSG("Unknown mode\n");
			break;
		}
}

static void lpwg_status(struct lu201x_data *data) {
	struct lu201x_platform_data *pdata = data->pdata;

	TOUCH_INFO_MSG("SUSPEND AND SET TAP count\n");

	if(pdata->lpwg_panel_on == LCD_ON) {
		cancel_delayed_work_sync(&data->resume_work);
		queue_delayed_work(data->ts_workqueue, &data->resume_work, msecs_to_jiffies(500));
		TOUCH_INFO_MSG("RESUME AND SET power on\n");
	} else {
		cancel_delayed_work_sync(&data->resume_work);
		touch_disable_irq(data->irq);
		if (pdata->lpwg_mode == LPWG_DISABLE) {
			touch_disable_irq_wake(data->irq);

			lpwg_mode_change(data, LPWG_DISABLE);
			TOUCH_INFO_MSG("SUSPEND AND SET power off\n");
		} else {
			if (pdata->lpwg_mode == LPWG_DOUBLE_TAP) {
				lpwg_mode_change(data, LPWG_DOUBLE_TAP);
			} else if (pdata->lpwg_mode == LPWG_MULTI_TAP) {
				lpwg_mode_change(data, LPWG_MULTI_TAP);
			}

			TOUCH_INFO_MSG("SUSPEND AND SET power on\n");
			touch_enable_irq(data->irq);
			touch_enable_irq_wake(data->irq);
		}
	}
}

/****************************************************************************
* Touch Firmware Update Function
****************************************************************************/
static int LU201x_readFW(struct lu201x_data *data, unsigned char *pBuf, int addr, int size)
{
	unsigned char temp[64] = {0,};
	int i;
	u8 Cmd[2] = {0, 0};

	/* Set Read Command */
	Cmd[0] = 0x84;
	if (i2c_write_bytes_LU201x(data->client, 0x1000, Cmd, 1) == -1)	{
		TOUCH_INFO_MSG("Read Cmd operation failed\n" );
		return -1;
	}

	for (i = 0; i < size; i += 64) {
		if (i2c_read_bytes_LU201x(data->client, addr+i, temp, sizeof(temp)) == -1) {
			TOUCH_INFO_MSG("bin write operation failed %d\n", i );
			return -1;
		}
#if 1
{
		int j = 0;
		if (i == (7 * 64)) {
			for (j = 0; j<64; j += 16) {
				TOUCH_INFO_MSG("%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x", \
					temp[j], temp[j+1], temp[j+2], temp[j+3], \
					temp[j+4], temp[j+5], temp[j+6], temp[j+7], \
					temp[j+8], temp[j+9], temp[j+10], temp[j+11], \
					temp[j+12], temp[j+13], temp[j+14], temp[j+15]);
			}
		}
}
#endif
		memcpy(&pBuf[i], temp, 64);
	}

	return 1;
}

static int LU201x_programFW(struct lu201x_data *data, u8 *pBuf, int addr, int size)
{
	u8 Cmd[2] = {0, 0};
	u8 Status = 0;
	int i;

	for (i = 0; i < size; i += 64) {
		Cmd[0] = 0x83;
		if (i2c_write_bytes_LU201x(data->client, 0x1000, Cmd, 1) == -1)	{
			TOUCH_INFO_MSG("Set load bit operation failed\n" );
			return -1;
		}

		if (i%(80*64) == 0) {
			TOUCH_INFO_MSG("%s Writing (%d/%d) bytes\n",
				(size == FAC_SIZE) ? "FACTORY" : "FIRMWARE", i, size);
		}

		if (i2c_write_bytes_LU201x(data->client, addr+i, (pBuf+i), 64) == -1 ) {
			TOUCH_INFO_MSG("bin write operation failed %d\n", i );
			return -1;
		}

		Cmd[0] = 0x82;
		if (i2c_write_bytes_LU201x(data->client, 0x1000, Cmd, 1) == -1)	{
			TOUCH_INFO_MSG("Reset load bit operation failed\n" );
			return -1;
		}
		while (1) {
			if (i2c_read_bytes_LU201x(data->client, 0x1000, &Status, 1) == -1) {
				TOUCH_INFO_MSG("Read Status operation failed\n" );
				return -1;
			}

			if ((Status & 0x40) == 0x40) {
				break;
			}
		}
	}

	if(i == size) {
		TOUCH_INFO_MSG("%s Writing (%d/%d) bytes download done.\n",
			(size == FAC_SIZE) ? "FACTORY" : "FIRMWARE", i, size);
	}

	return 0;
}

static int LU201x_erase(struct lu201x_data *data)
{
	struct lu201x_fw_info *fw_info = data->fw_info;
	u8 Cmd[2] = {0, 0};
	u8 status = 0;
	u8 *pBuf;

	pBuf = kmalloc(FAC_SIZE, GFP_KERNEL);
	fw_info->fac_raw_data = pBuf;

	/* Read Cal Data */
	if (LU201x_readFW(data, pBuf, FAC_POS, FAC_SIZE) == -1) {
		TOUCH_INFO_MSG("Read Cal Data failed\n" );
		goto ERASE_FAIL;
	}

	/* Erase */
	Cmd[0] = 0x80;
	if (i2c_write_bytes_LU201x(data->client, 0x1000, Cmd, 1) == -1)	{
		TOUCH_INFO_MSG("Set mode command operation failed\n" );
		goto ERASE_FAIL;
	}

	Cmd[0] = 0x04;
	if (i2c_write_bytes_LU201x(data->client, 0x1005, Cmd, 1) == -1) {
		TOUCH_INFO_MSG("Erase command operation failed\n" );
		goto ERASE_FAIL;
	}

	while (1) {
		if (i2c_read_bytes_LU201x(data->client, 0x1000, &status, 1) == -1) {
			TOUCH_INFO_MSG("Read status operation failed\n" );
			goto ERASE_FAIL;
		}

		if ((status & 0x40) == 0x40) {
			break;
		}
	}
	msleep(100);

	/* Write Cal Data */
	if (LU201x_programFW(data, pBuf, FAC_POS, FAC_SIZE) == -1) {
		TOUCH_INFO_MSG("Read Cal Data failed\n" );
		goto ERASE_FAIL;
	}
	kfree(pBuf);
	return 0;

ERASE_FAIL:
	kfree(pBuf);
	return -1;
}

static int LU201x_update_setmode(struct lu201x_data *data, int mode)
{
	struct lu201x_platform_data *pdata = data->pdata;
	u8 Cmd[2] = {0, 0};

	/* FW Upgrade mode */
	if (mode) {
		disable_irq(data->irq);

		Cmd[0] = 0x80;
		if (i2c_write_bytes_LU201x(data->client, 0x1000, Cmd, 1) == -1) {
			TOUCH_INFO_MSG("Set mode operation failed\n" );
			return -1;
		}

		Cmd[0] = 0x75;
		Cmd[1] = 0x6C;
		if (i2c_write_bytes_LU201x(data->client, 0x1003, Cmd, 2) == -1) {
			TOUCH_INFO_MSG("Set password operation failed\n" );
			return -1;
		}
	} else {
		gpio_direction_input(pdata->gpio_int);
		enable_irq(data->irq);
	}
	TOUCH_INFO_MSG("LU201x_update_setmode is success\n");

	return 0;
}

static int LU201x_do_update(struct lu201x_data *data)
{
	struct lu201x_fw_info *fw_info = data->fw_info;

	/* Set mode */
	if (LU201x_update_setmode(data, 1) == -1) {
		TOUCH_INFO_MSG("Set Mode failed\n");
		goto UPDATE_FAIL;
	}

	/* Erase and Write Factory region*/
	if (LU201x_erase(data) == -1) {
		TOUCH_INFO_MSG("Erase failed\n");
		goto UPDATE_FAIL;
	}

	/* Program */
	if (LU201x_programFW(data, fw_info->fw_raw_data, FW_POS, FW_SIZE + CFG_SIZE) == -1) {
		TOUCH_INFO_MSG("Bin program failed\n");
		goto UPDATE_FAIL;
	}

	LU201x_update_setmode(data, 0);
	LU201x_reset(data, 0);
	LU201x_reset(data, 1);

	return 0;

UPDATE_FAIL:
	LU201x_update_setmode(data, 0);
	return -1;
}

static int LU201x_parse_fw_header(struct lu201x_data *data, const struct firmware *fw)
{
	struct lu201x_fw_info *fw_info = data->fw_info;
	struct lu201x_fw_image *fw_image = data->fw_image;
	struct lu201x_self_diagnostic *sd = &(fw_image->sd);
	int sd_size = 0;
	int i = 0;

	fw_image = (struct lu201x_fw_image *)fw->data;

	fw_image->fw_header_size = sizeof(struct lu201x_fw_image) - sizeof(struct lu201x_self_diagnostic);
	sd_size = sizeof(struct lu201x_self_diagnostic);
	TOUCH_INFO_MSG("firmware size : %d header : 0x%02x\n", fw->size, fw_image->fw_header_size + sd_size);
	if (fw->size > FW_SIZE + CFG_SIZE + fw_image->fw_header_size + sd_size) {
		TOUCH_INFO_MSG("Firmware Size exceed 31K : %d\n", fw->size);
		return -1;
	}

	memcpy(fw_info, fw_image, fw_image->fw_header_size - 1);
	fw_info->model_name[sizeof(fw_image->model_name)-1] = '\0';
	fw_info->fw_bin_size = fw->size - fw_image->fw_header_size - sd_size;

	fw_info->fw_raw_data = kzalloc(fw_info->fw_bin_size, GFP_KERNEL);
	if (!fw_info->fw_raw_data) {
		TOUCH_INFO_MSG("%s Failed to allocate memory\n", __func__);
		return 0;
	}
	memcpy(fw_info->fw_raw_data, fw_image->fw_bin, fw_info->fw_bin_size);

	/* get self-diagnostic data */
	memcpy(sd, fw->data + fw_image->fw_header_size, sd_size);
	for (i = 0; i < MAX_CHANNEL; i++) {
		TOUCH_INFO_MSG(" ch[%d] max = %d, ch[%d] min = %d\n", \
			i, sd->ch_max[i], i, sd->ch_min[i]);
	}
	TOUCH_INFO_MSG("Jitter : %d", sd->ch_jitter);

	return 0;
}

static int LU201x_fw_checksum(struct lu201x_data *data)
{
	struct lu201x_fw_info *fw_info = data->fw_info ;
	u8 temp[3] = {CMD_LU201x_DEVINFO, 0, 0};
	u8 check_sum[6] = {0,};

	LU201x_set_i2c_mode(data);
	if (LU201x_wait_ready_i2c(data, 0) == -1) {
		TOUCH_INFO_MSG("Set mode Int ready error\n");
		goto exit;
	}

	/* Change Checksum Mode */
	i2c_write_bytes_LU201x(data->client, LU201x_MODE_ADDR, temp, 3);
	LU201x_i2c_done(data);

	msleep(2);
	if (LU201x_wait_ready_i2c(data, 0) == -1) {
		TOUCH_INFO_MSG ("Set mode read ready error\n");
		goto exit;
	}

	/* Checksum Mode check */
	i2c_read_bytes_LU201x(data->client, LU201x_CMDACK_ADDR, temp, 3);
	if (temp[2] != CMD_LU201x_DEVINFO) {
		TOUCH_INFO_MSG("Read reg failed! %#x\n", temp[0]);
		goto exit;
	}
	mdelay(2);

	i2c_read_bytes_LU201x(data->client, LU201x_CMD_FW_CHECKSUM_ADDR, check_sum, sizeof(check_sum));
	LU201x_i2c_done(data);

	TOUCH_INFO_MSG("TOUCH Binary Firm_checksum [%02x.%02x.%02x.%02x], Parm_checksum [%02x.%02x]\n", \
		fw_info->fw_crc[0], fw_info->fw_crc[1], fw_info->fw_crc[2], fw_info->fw_crc[3], \
		fw_info->parm_crc[0], fw_info->parm_crc[1]);
	TOUCH_INFO_MSG("TOUCH IC Firm_checksum [%02x.%02x.%02x.%02x], Parm_checksum [%02x.%02x]\n", \
		check_sum[0], check_sum[1], check_sum[2], check_sum[3], check_sum[4], check_sum[5]);

	if(memcmp(&fw_info->fw_crc, &check_sum[0], sizeof(fw_info->fw_crc)) || \
		memcmp(&fw_info->parm_crc, &check_sum[4], sizeof(fw_info->parm_crc))) {
		TOUCH_INFO_MSG("Did not verify Touch IC firmware");
		goto exit;
	}

	return 0;
exit:
	return -1;
}

static int LU201x_verify_firmware(struct lu201x_data *data, const char *fw_name)
{
	const struct firmware *fw = NULL;
	struct lu201x_fw_info *fw_info = data->fw_info;
	u8 fw_ver[4] = {0, };
	int err = 0;

	/* firmware region */
	if (fw_name != NULL) {
		err = request_firmware(&fw, fw_name, &data->client->dev);
		if (err) {
			TOUCH_INFO_MSG("%s error request_firmware \n", __func__);
			goto fw_fail;
		}
	} else {
		TOUCH_INFO_MSG("Firmware Name is NULL");
		goto fw_fail;
	}

	err = LU201x_parse_fw_header(data, fw);
	if(err < 0) {
		TOUCH_INFO_MSG("Firmware header parsing error");
		goto exit;
	}

	err = LU201x_read_register(data, FW_VERSION_REG-2, &fw_ver[0], 4);
	if (err < 0) {
		TOUCH_INFO_MSG("FW_VERSION_REG read fail\n");
		goto exit;
	} else {
		TOUCH_INFO_MSG("Touch IC: FW_Version [%d.%d], Release Version [%d.%d]\n", fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3]);
		TOUCH_INFO_MSG("Binary: FW_Version [%d.%d], Release Version [%d.%d]\n", \
			fw_info->fw_ver[0], fw_info->fw_ver[1], fw_info->rel_ver[0], fw_info->rel_ver[1]);

		if (memcmp(&fw_info->fw_ver, &fw_ver[0], sizeof(fw_info->fw_ver)) ||
			memcmp(&fw_info->rel_ver, &fw_ver[2], sizeof(fw_info->rel_ver))) {
			TOUCH_INFO_MSG("Touch FW Update!! [%d.%d ==> %d.%d]\n",
				fw_ver[2], fw_ver[3], fw_info->rel_ver[0], fw_info->rel_ver[1]);
			data->need_fw = FIRMWARE;
		} else {
			TOUCH_INFO_MSG("don't have to update FW\n");
			data->need_fw = NORMAL;
		}
	}

	release_firmware(fw);
	return 0;
exit:
	return -1;
fw_fail:
	return -2;
}

static int LU201x_update_firmware(struct lu201x_data *data, const char *fw_name)
{
	int err = 0;
	u8 info[8] = { 0, };
	u8 Cmd[2] = { 0, };

	TOUCH_INFO_MSG("Touch FW name : %s\n", fw_name);
	msleep(5);

	err = LU201x_verify_firmware(data, fw_name);
	if (err == -2) {
		LU201x_reset(data, 0);
		LU201x_reset(data, 1);
		TOUCH_INFO_MSG("Binary didn't have the Touch Firmware\n");
		return 0;
	} else if (err < 0) {
		TOUCH_INFO_MSG("LU201x_verify_firmware fail\n");
		data->need_fw = FIRMWARE;
	}

	if (data->need_fw == FIRMWARE) {
		/* read Product ID */
		//User mode enter
		Cmd[0] = 0x75;
		Cmd[1] = 0x6C;
		if (i2c_write_bytes_LU201x(data->client, 0x1003, Cmd, 2) == -1) {
			TOUCH_INFO_MSG("Set password operation failed\n" );
			return -1;
		}
		Cmd[0] = 0x88;
		if (i2c_write_bytes_LU201x(data->client, 0x1000, Cmd, 1) == -1) {
			TOUCH_INFO_MSG("User mode go operation failed\n" );
			return -1;
		}
		//Get information
		Cmd[0] = 0x8C;
		if (i2c_write_bytes_LU201x(data->client, 0x1000, Cmd, 1) == -1) {
			TOUCH_INFO_MSG("User Area-Read Cmd operation failed\n" );
			return -1;
		}
		if (i2c_read_bytes_LU201x(data->client, 0x0038, info, 8) == -1) {
			TOUCH_INFO_MSG("User Area-Read  failed\n" );
			return -1;
		}

		err = LU201x_do_update(data);
		if (err < 0) {
			TOUCH_INFO_MSG("update tsp fail\n" );
			return -1;
		} else {
			TOUCH_INFO_MSG("update tsp success\n");
		}
	} else {
		TOUCH_INFO_MSG("do not update FW\n");
	}

	err = LU201x_fw_checksum(data);
	if (err < 0) {
		TOUCH_INFO_MSG("LU201x_fw_checksum fail\n" );
		return -1;
	}

	kfree(data->fw_info->fw_raw_data);
	return 0;
}

/****************************************************************************
* Touch SYSFS
****************************************************************************/
static void lu201x_write_file(char *filename, char *data, int time, int size)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu \n", \
		my_date.tm_mon + 1,my_date.tm_mday, \
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec, \
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	if (!strcmp(filename, FW_FILE_PATH)) {
		fd = sys_open(filename, O_WRONLY|O_CREAT, 0666);
	} else if (!strcmp(filename, SELF_DIAGNOSTIC_FILE_PATH)) {
		fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	}
	TOUCH_INFO_MSG("write open %s, fd : %d\n", (fd >= 0)? "success":"fail",fd);

	if (fd >= 0) {
		if (time > 0) {
			sys_write(fd, time_string, strlen(time_string));
			TOUCH_INFO_MSG("Time write success.\n");
		}
		if (size > 0) {
			sys_write(fd, data, size);
		}
		sys_close(fd);
	}
	set_fs(old_fs);
}

static ssize_t lu201x_read_fw_store(struct lu201x_data *data, const char *buf, size_t count)
{
	int mode = 0;
	u8 *pBuf = NULL;
	u32 size = 0;
	u32 position = 0;

	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));
	touch_disable_irq(data->irq);

	sscanf(buf, "%d", &mode);
	TOUCH_INFO_MSG("%s  mode = %d \n", __func__, mode);

	/* Set mode */
	if (LU201x_update_setmode(data, 1) == -1) {
		TOUCH_INFO_MSG("Set Mode failed\n");
		goto exit;
	}

	switch (mode) {
		case FACTORY_REGION :
			size = FAC_SIZE;
			position = FAC_POS;
			break;

		case FIRMWARE_REGION :
			size = FW_SIZE + CFG_SIZE;
			position = FW_POS;
			break;

		default:
			goto exit;
			break;
	}

	pBuf = kmalloc(size, GFP_KERNEL);
	/* Read Cal Data */
	if (LU201x_readFW(data, pBuf, position, size) == -1) {
		TOUCH_INFO_MSG("Read Cal Data failed\n" );
		goto exit;
	}
	/* IC Firmware binary write to file */
	lu201x_write_file(FW_FILE_PATH, pBuf, 0, size);

exit :
	LU201x_update_setmode(data, 0);
	LU201x_reset(data, 0);
	LU201x_reset(data, 1);

	kfree(pBuf);
	touch_enable_irq(data->irq);

	return count;
}

static ssize_t lu201x_global_access_pixel_show(struct lu201x_data *data, char *buf)
{
	int len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", data->pdata->global_access_pixel);

	return len;
}

static ssize_t lu201x_global_access_pixel_store(struct lu201x_data *data, const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	TOUCH_INFO_MSG("%s = %d \n", __func__, value);

	data->pdata->global_access_pixel = value;

	return count;
}

static ssize_t lu201x_power_ctl_store(struct lu201x_data *data, const char *buf, size_t count)
{
	int mode;

	sscanf(buf, "%d", &mode);

	switch (mode) {
		case HW_RESET :
			gpio_direction_input(data->pdata->gpio_int);
			gpio_direction_output(data->pdata->gpio_reset, 1);
			LU201x_power(data, 0);
			LU201x_power(data, 1);
			break;

		case SW_RESET :
			gpio_direction_input(data->pdata->gpio_int);
			gpio_direction_output(data->pdata->gpio_reset, 1);
			LU201x_reset(data, 0);
			LU201x_reset(data, 1);
			break;

		default :
			TOUCH_INFO_MSG("error command");
			break;
	}

	return count;
}

static ssize_t show_knock_on_type(struct lu201x_data *data, char *buf)
{
	int ret = 0;

	TOUCH_INFO_MSG("choice the knock on/code device %d", knock_on_type);
	ret += sprintf(buf+ret, "%d", knock_on_type);

	return ret;
}

static ssize_t show_lpwg_data(struct lu201x_data *data, char *buf)
{
	int i = 0, ret = 0;

	memset(lpwg_data, 0, sizeof(struct point) * MAX_POINT_SIZE_FOR_LPWG);

	lpwg_data_read();
	for (i = 0; i < MAX_POINT_SIZE_FOR_LPWG; i++) {
		if (lpwg_data[i].x == 0 && lpwg_data[i].y == 0) {
			break;
		}
		ret += sprintf(buf + ret, "%d %d\n", lpwg_data[i].x, lpwg_data[i].y);
	}

	return ret;
}

static ssize_t store_lpwg_data(struct lu201x_data *data, const char *buf, size_t count)
{
	int reply = 0;

	sscanf(buf, "%d", &reply);
	TOUCH_INFO_MSG("LPWG RESULT = %d ", reply);

	if (reply == 1) {
		cancel_delayed_work_sync(&data->resume_work);
		queue_delayed_work(data->ts_workqueue, &data->resume_work, msecs_to_jiffies(0));
	}
	/* Initialize the lpwg_data */
	memset(gesture_property, 0, sizeof(u8) * MAX_POINT_SIZE_FOR_LPWG * 4);
	memset(lpwg_data, 0, sizeof(struct point) * MAX_POINT_SIZE_FOR_LPWG);


	return count;
}

static ssize_t store_lpwg_notify(struct lu201x_data *data, const char *buf, size_t count)
{
	static int pre_lpwg_prox = 0;
	static int pre_panel_status = 0;
	int type = 0;
	int value[4] = {0};



	sscanf(buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3]);
	TOUCH_INFO_MSG("touch notify type = %d , value[0] = %d, value[1] = %d, valeu[2] = %d, value[3] = %d ", \
		type, value[0], value[1], value[2], value[3]);

	mutex_lock(&sys_lock);
	if (mutex_is_locked(&i2c_suspend_lock)) {
		TOUCH_INFO_MSG("%s mutex_is_locked \n", __func__);
	}
	switch (type) {
		case CMD_LPWG_ENABLE :
			if (value[0] == LPWG_DISABLE) {
				data->pdata->lpwg_mode = LPWG_DISABLE;
			} else if (value[0] == LPWG_DOUBLE_TAP) {
				multi_tap_count = 2;
				data->pdata->lpwg_mode = LPWG_DOUBLE_TAP;
			} else if (value[0] == LPWG_MULTI_TAP) {
				data->pdata->lpwg_mode = LPWG_MULTI_TAP;
			}
			break;

		case CMD_LPWG_LCD :
			break;

		case CMD_LPWG_ACTIVE_AREA :
			break;

		case CMD_LPWG_TAP_COUNT :
			if (value[0]) {
				multi_tap_count = value[0];
				if (MAX_POINT_SIZE_FOR_LPWG < value[0]) {
					multi_tap_count = MAX_POINT_SIZE_FOR_LPWG;
				}
			}
			break;

		case CMD_LPWG_LCD_RESUME_SUSPEND :
			if (value[0] == 0) {
				data->pdata->lpwg_panel_on = LCD_OFF;
			} else {
				data->pdata->lpwg_panel_on = LCD_ON;
			}
			break;

		case CMD_LPWG_PROX :
			break;

		case CMD_LPWG_DOUBLE_TAP_CHECK :
			if (value[0] == 1) {
				data->pdata->double_tap_check = 1;
			} else {
				data->pdata->double_tap_check = 0;
			}
			break;

		case CMD_LPWG_TOTAL_STATUS :
			data->pdata->lpwg_panel_on = (int)value[1];
			data->pdata->lpwg_prox = (int)value[2];

			if (pre_lpwg_prox != data->pdata->lpwg_prox || pre_panel_status !=data->pdata->lpwg_panel_on) {
				lpwg_status(data);
				pre_lpwg_prox = data->pdata->lpwg_prox;
				pre_panel_status = data->pdata->lpwg_panel_on;
			}
			break;
	}

	mutex_unlock(&sys_lock);
	return count;
}

static ssize_t lu201x_release_finger_show(struct lu201x_data *data, char *buf)
{
	int ret = 0;

	LU201x_release_all_finger(data);

	return  ret;
}

static ssize_t lu201x_version_show(struct lu201x_data *data, char *buf)
{
	struct lu201x_platform_data *pdata = data->pdata;
	u8 fw_ver[4] = {0,};
	int ret = 0;
	int err = 0;

	touch_disable_irq(data->irq);
	if (pdata->lpwg_panel_on == LCD_OFF) {
		ret += snprintf(buf+ret, PAGE_SIZE, "========================================\n");
		ret += snprintf(buf+ret, PAGE_SIZE, "  LCD_OFF Please retry after LCD on\n");
		ret += snprintf(buf+ret, PAGE_SIZE, "========================================\n");
	} else {
		/* ready for i2c */
		LU201x_set_i2c_mode(data);
		if (LU201x_wait_ready_i2c(data, 0) == -1) {
			TOUCH_INFO_MSG("Set mode Int ready error\n");
		}

		err = LU201x_read_register(data, FW_VERSION_REG-2, &fw_ver[0], 4);
		if (err < 0) {
			ret += snprintf(buf+ret, PAGE_SIZE, "========================================\n");
			ret += snprintf(buf+ret, PAGE_SIZE, "       FW_VERSION_REG read fail\n");
			ret += snprintf(buf+ret, PAGE_SIZE, "========================================\n");
		} else {
			TOUCH_INFO_MSG("Touch IC: FW_Version [%d.%d], Release Version [%d.%d]\n",
				fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3]);

			ret += snprintf(buf+ret, PAGE_SIZE, "========================================\n");
			ret += snprintf(buf+ret, PAGE_SIZE, "   FW_Version      = [%02d.%02d]\n", fw_ver[0], fw_ver[1]);
			ret += snprintf(buf+ret, PAGE_SIZE, "   RELEASE_Version = [%02d.%02d]\n", fw_ver[2], fw_ver[3]);
			ret += snprintf(buf+ret, PAGE_SIZE, "   Num of channel  = %d\n", MAX_CHANNEL);
			ret += snprintf(buf+ret, PAGE_SIZE, "========================================\n");
		}
	}
	touch_enable_irq(data->irq);

	return ret;
}

static ssize_t lu201x_update_fw_store(struct lu201x_data *data, const char *buf, size_t count)
{
	char *package_name = NULL;
	int ret = 0;
	int err = 0;

	mutex_lock(&sys_lock);
	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(5000));
	TOUCH_INFO_MSG("%s\n", __func__);

	touch_disable_irq(data->irq);

	err = lu201x_update_file_name(&data->client->dev, &package_name, buf, count);
	if (err) {
		TOUCH_INFO_MSG("%s error package_name [%s] \n", __func__, package_name);
		goto exit;
	}

	LU201x_set_i2c_mode(data);
	ret = LU201x_wait_ready_i2c(data, 0);
	if ( ret == -1 ) {
		TOUCH_INFO_MSG("Register Read failure\n");
		goto exit;
	}

	/* Update the Firmware */
	LU201x_update_firmware(data, package_name);
exit:
	if (package_name) {
		kfree(package_name);
	}

	touch_enable_irq(data->irq);
	mutex_unlock(&sys_lock);

	return count;
}

static void lu201x_rawdata_read(struct lu201x_data *data, u16 *ch_cap_data, u8 mode)
{
	u8 buf[3] = {CMD_LU201x_CHCAPTEST, 0, 0};
	u8 reply[2] = {0,};
	u8 temp = 0, size = 0;
	u8 cpa_read_val[MAX_CHANNEL*2] = {0,};

	LU201x_set_i2c_mode(data);
	if (LU201x_wait_ready_i2c(data, 0) == -1) {
		TOUCH_INFO_MSG("Set mode Int ready error\n");
		goto exit;
	}

	/* Change Cap Test Mode */
	i2c_write_bytes_LU201x(data->client, LU201x_MODE_ADDR, buf, 3);
	LU201x_i2c_done(data);

	//msleep(2);
	msleep(50);
	if (LU201x_wait_ready_i2c(data, 0) == -1) {
		TOUCH_INFO_MSG ("Set mode read ready error\n");
		goto exit;
	}

	/* Cap Test Mode check */
	i2c_read_bytes_LU201x(data->client, LU201x_CMDACK_ADDR, buf, 3);
	if (buf[2] != CMD_LU201x_CHCAPTEST) {
		TOUCH_INFO_MSG("Read reg failed! %#x\n", buf[0]);
		goto exit;
	}

	/* Read channel of panel size */
	i2c_read_bytes_LU201x(data->client, LU201x_CMDReply_ADDR, reply, 2);
	if (reply[0] != LU201x_CHCAPTEST_Reply) {
		TOUCH_INFO_MSG("Captest reply error! %#x\n", reply[0]);
		goto exit;
	}
	size = reply[1] - 6;
	TOUCH_INFO_MSG("Channel Size : %d", size / 4);
	if(size > MAX_CHANNEL*4) {
		size = MAX_CHANNEL*4;
	}

	/* Read data of channel */
	if (mode == GET_REFERENCE) {
		i2c_read_bytes_LU201x(data->client, LU201x_CMDReply_ADDR + 2, cpa_read_val, size / 2);
	} else if (mode == GET_JITTER) {
		i2c_read_bytes_LU201x(data->client, LU201x_CMDReply_ADDR + 2 + (MAX_CHANNEL * 2), cpa_read_val, size / 2);
	}
	LU201x_i2c_done(data);

	for (temp = 0; temp < (size / 4); temp++) {
		ch_cap_data[temp] = cpa_read_val[temp * 2] | (cpa_read_val[temp * 2 + 1] << 8);
		TOUCH_INFO_MSG("Ch[%d] Cap data = %d\n", temp, ch_cap_data[temp]);
	}

exit :
	return;
}

int print_raw(char *buf, u16 *ch_cap_data, u8 mode, int offset)
{
	int i = 0;

	offset += snprintf(buf + offset, PAGE_SIZE, "==============================================\n");
	offset += snprintf(buf + offset, PAGE_SIZE, "         Touch Panel %s value\n", \
			(mode == GET_REFERENCE) ? "Reference" : (mode == GET_JITTER) ? "Jitter" : "Delta");
	offset += snprintf(buf+offset, PAGE_SIZE, "==============================================\n");

	for (i = 0; i < MAX_CHANNEL / 2; i++) {
		if (i != MAX_CHANNEL / 2 - 1) {
			offset += snprintf(buf + offset, PAGE_SIZE, "left [%02d]-[%05d] | [%02d]-[%05d] right\n",
					i, ch_cap_data[i], i+MAX_CHANNEL/2, ch_cap_data[i+MAX_CHANNEL/2]);
		} else {
			offset += snprintf(buf + offset, PAGE_SIZE, "==============================================\n");
			offset += snprintf(buf + offset, PAGE_SIZE, "        Touch Key Reference value\n");
			offset += snprintf(buf + offset, PAGE_SIZE, "==============================================\n");
			offset += snprintf(buf + offset, PAGE_SIZE, " left [%02d]-[%05d] | [%02d]-[%05d] right\n",
					i, ch_cap_data[i], i+MAX_CHANNEL/2, ch_cap_data[i+MAX_CHANNEL/2]);
		}
	}

	return offset;
}

static ssize_t lu201x_reference_show(struct lu201x_data *data, char *buf)
{
	struct lu201x_platform_data *pdata = data->pdata;
	u16 ch_cap_data[MAX_CHANNEL] = {0,};
	int ret = 0;

	touch_disable_irq(data->irq);
	if(pdata->lpwg_panel_on == LCD_OFF) {
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_DOUBLE_TAP-1]);
	}
	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));

	lu201x_rawdata_read(data, ch_cap_data, GET_REFERENCE);
	ret = print_raw(buf, ch_cap_data, GET_REFERENCE, ret);

	touch_enable_irq(data->irq);

	return ret;
}

static ssize_t lu201x_jitter_cap_show(struct lu201x_data *data, char *buf)
{
	struct lu201x_platform_data *pdata = data->pdata;
	u16 ch_cap_data[MAX_CHANNEL] = {0,};
	int ret = 0;

	touch_disable_irq(data->irq);
	if(pdata->lpwg_panel_on == LCD_OFF) {
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_DOUBLE_TAP-1]);
	}
	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));

	lu201x_rawdata_read(data, ch_cap_data, GET_JITTER);
	ret = print_raw(buf, ch_cap_data, GET_JITTER, ret);

	touch_enable_irq(data->irq);

	return ret;
}

static void lu201x_debug_rawdata_read(struct lu201x_data *data, u16 *ch_cap_data, u8 mode)
{
	u8 temp = 0,size = 0;
	u8 cpa_read_val[80] = {0,};
	int ret = -1;

	ret = LU201x_set_mode(data, CMD_LU201x_DEBUG, mode);
	if (ret == -1) {
		LU201x_reset(data, 0);
		LU201x_reset(data, 1);
		TOUCH_INFO_MSG ( "set_mode(%d) fail\n", mode);
		goto exit;
	}

	if (LU201x_wait_ready_i2c(data, 0) == -1) {
		TOUCH_INFO_MSG("read wait time over fail\n");
		goto exit;
	}

	i2c_read_bytes_LU201x(data->client, LU201x_CMDReply_ADDR, cpa_read_val, MAX_CHANNEL*2 + 4);
	LU201x_i2c_done(data);

	size = cpa_read_val[2];
	TOUCH_INFO_MSG("debug data size = %d\n", size);

	if(mode == GET_SUMHISTO || mode == GET_SUMDIVHISTO) {
		size /= 2;
	}
	for (temp = 0; temp < (size); temp++) {
		ch_cap_data[temp] = cpa_read_val[temp*2 + 4] | (cpa_read_val[temp*2 + 5] << 8);
		//TOUCH_INFO_MSG("Ch[%d] Cap data = %d\n", temp, ch_cap_data[temp]);
	}

	ret = LU201x_set_mode(data, CMD_LU201x_NORMODE, 0);
	if (ret == -1) {
		LU201x_reset(data, 0);
		LU201x_reset(data, 1);
		TOUCH_INFO_MSG ( "set_mode(%d) fail\n", CMD_LU201x_NORMODE);
	}

exit :
	return;
}

static u8 delta_mode = GET_HISTO;
static ssize_t lu201x_delta_cap_show(struct lu201x_data *data, char *buf)
{
	struct lu201x_platform_data *pdata = data->pdata;
	u16 ch_cap_data[MAX_CHANNEL] = {0,};
	int ret = 0;

	touch_disable_irq(data->irq);
	if(pdata->lpwg_panel_on == LCD_OFF) {
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_DOUBLE_TAP-1]);
	}
	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));

	lu201x_debug_rawdata_read(data, ch_cap_data, delta_mode);
	ret = print_raw(buf, ch_cap_data, GET_JITTER, ret);

	touch_enable_irq(data->irq);

	return ret;
}

static ssize_t lu201x_delta_cap_store(struct lu201x_data *data, const char *buf, size_t count)
{
	sscanf(buf , "%d", (int *)&delta_mode);
	TOUCH_INFO_MSG("delta_mode : %d\n", delta_mode);

	if(GET_NONE >= delta_mode || delta_mode > GET_SUMDIVHISTO) {
		delta_mode = GET_HISTO;
	}

	return count;
}

u8 compare_raw_data(struct lu201x_data *data, u16 *ch_cap_data, u8 mode)
{
	struct lu201x_self_diagnostic *sd = &(data->fw_image->sd);
	int i = 0;

	if (mode == GET_REFERENCE) {
		for(i = 0; i < MAX_CHANNEL;
		i++) {
			if (sd->ch_min[i] > ch_cap_data[i]) {
				TOUCH_INFO_MSG("Reference Min(%d) ch-%d (%d)fail", sd->ch_min[i], i, ch_cap_data[i]);
				return i;
			} else if (sd->ch_max[i] < ch_cap_data[i]) {
				TOUCH_INFO_MSG("Reference Max(%d) ch-%d (%d)fail", sd->ch_max[i], i, ch_cap_data[i]);
				return i;
			}
		}
	} else if (mode == GET_JITTER) {
		for(i = 0; i < MAX_CHANNEL; i++) {
			if (sd->ch_jitter < ch_cap_data[i]) {
				TOUCH_INFO_MSG("Jitter ch-%d (%d)fail", i, ch_cap_data[i]);
				return i;
			}
		}
	}

	return SD_PASS;
}

int self_diagnostic(struct lu201x_data *data, u8 *result, char *temp_buf)
{
	u16 ch_cap_data[MAX_CHANNEL] = {0,};
	int ret = 0;

	lu201x_rawdata_read(data, ch_cap_data, GET_REFERENCE);
	result[GET_REFERENCE] = compare_raw_data(data, ch_cap_data, GET_REFERENCE);
	ret += print_raw(temp_buf, ch_cap_data, GET_REFERENCE, ret);

	mdelay(300);

	lu201x_rawdata_read(data, ch_cap_data, GET_JITTER);
	//result[GET_JITTER] = compare_raw_data(data, ch_cap_data, GET_JITTER);
	compare_raw_data(data, ch_cap_data, GET_JITTER);
	ret += print_raw(temp_buf, ch_cap_data, GET_JITTER, ret);

	/* check reference and jitter fail */
	TOUCH_INFO_MSG("self_diagnositc test is %s - [%d]", \
		(result[GET_REFERENCE] == SD_PASS) ? "Pass" : "Fail", result[GET_REFERENCE]);
	if (result[GET_REFERENCE] != SD_PASS) {
		return SD_FAIL;
	}
	return SD_PASS;
}

static int lu201x_get_fw_status(struct lu201x_data *data)
{
	u8 temp[3] = {CMD_LU201x_DEVINFO, 0, 0};
	u8 fw_status = -1;

	LU201x_set_i2c_mode(data);
	if (LU201x_wait_ready_i2c(data, 0) == -1) {
		TOUCH_INFO_MSG("Set mode Int ready error\n");
		goto exit;
	}

	/* Change DevInfo Mode */
	i2c_write_bytes_LU201x(data->client, LU201x_MODE_ADDR, temp, 3);
	LU201x_i2c_done(data);

	msleep(2);
	if (LU201x_wait_ready_i2c(data, 0) == -1) {
		TOUCH_INFO_MSG ("Set mode read ready error\n");
		goto exit;
	}

	/* DevInfo Mode check */
	i2c_read_bytes_LU201x(data->client, LU201x_CMDACK_ADDR, temp, 3);
	if (temp[2] != CMD_LU201x_DEVINFO) {
		TOUCH_INFO_MSG("Read reg failed! %#x\n", temp[0]);
		LU201x_i2c_done(data);
		goto exit;
	}
	mdelay(2);

	i2c_read_bytes_LU201x(data->client, LU201x_CMD_FW_STATUS_ADDR, &fw_status, 1);
	LU201x_i2c_done(data);

	return fw_status;
exit:
	return 3;
}

static ssize_t lu201x_sd_show(struct lu201x_data *data, char *buf)
{
	u8 result[GET_RAW_MAX] = {0, };
	char *temp_buf = NULL;
	int fac_reg_chk = 0;
	int raw_data_chk = 0;
	int ret_cnt = 0;
	int ret = 0;

	if (data->pdata->lpwg_panel_on == LCD_OFF) {
		ret += snprintf(buf + ret, PAGE_SIZE, "************************\n");
		ret += snprintf(buf + ret, PAGE_SIZE, "*** LCD STATUS : OFF ***\n");
		ret += snprintf(buf + ret, PAGE_SIZE, "************************\n");
		return ret;
	}

	mutex_lock(&sys_lock);
	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));
	touch_disable_irq(data->irq);

	/* need to IC stabilization time */
	msleep(1000);

	/*  Check Factory region of firmware */
	for (ret_cnt = 0; ret_cnt < 3; ret_cnt++) {
		fac_reg_chk = lu201x_get_fw_status(data);
		TOUCH_INFO_MSG("Touch Firmware Region check - %d", fac_reg_chk);
		if (fac_reg_chk != 3) {
			break;
		}
	}

	if (fac_reg_chk == FW_STATUS_CH_FAIL) {
		TOUCH_INFO_MSG("Channel Fail ( Open or Short ) - %d", fac_reg_chk);
	} else if (fac_reg_chk == FW_STATUS_CAL_FAIL) {
		TOUCH_INFO_MSG("Calibration Fail ( Empty Mass product data )  - %d", fac_reg_chk);
	}

	/* Check Channel status and Raw data */
	for (ret_cnt = 0; ret_cnt < 3; ret_cnt++) {
		temp_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
		raw_data_chk = self_diagnostic(data, result, temp_buf);
		TOUCH_INFO_MSG("sizeof(result) %d ret_cnt %d", sizeof(result), ret_cnt);
		if (raw_data_chk != SD_PASS && ret_cnt != 2) {
			mdelay(300);
			kfree(temp_buf);
			result[GET_REFERENCE] = 0;
		} else {
			break;
		}
	}

#if 0
	ret += snprintf(buf + ret, PAGE_SIZE, "=========================\n");
	ret += snprintf(buf + ret, PAGE_SIZE, " FW_Version      = [%02d.%02d]\n", data->fw_info->fw_ver[0], data->fw_info->fw_ver[1]);
	ret += snprintf(buf + ret, PAGE_SIZE, " RELEASE_Version = [%02d.%02d]\n", data->fw_info->rel_ver[0], data->fw_info->rel_ver[1]);
	ret += snprintf(buf + ret, PAGE_SIZE, " Num of channel  = %d\n", MAX_CHANNEL);
	ret += snprintf(buf + ret, PAGE_SIZE, "=========================\n");
	ret += snprintf(buf + ret, PAGE_SIZE, "FW Region check = %s\n", (fac_reg_chk == FW_STATUS_NORMAL) ? "PASS" : "FAIL");
	if (fac_reg_chk != FW_STATUS_NORMAL) {
		result[GET_REFERENCE] = 88;
	}
	ret += snprintf(buf + ret, PAGE_SIZE, "Channel Status : %s\n", (result[GET_REFERENCE] == SD_PASS) ? "PASS" : "FAIL");
	if (result[GET_REFERENCE] != SD_PASS) {
		ret += snprintf(buf + ret, PAGE_SIZE, " Channel Status channel[%d] Fail\n", result[GET_REFERENCE]);
	}

	ret += snprintf(buf+ret, PAGE_SIZE, "Raw Data : %s\n", (result[GET_JITTER] == SD_PASS) ? "PASS" : "FAIL");
	if (result[GET_JITTER] != SD_PASS) {
		ret += snprintf(buf + ret, PAGE_SIZE, " Raw Status channel[%d] Fail\n", result[GET_JITTER]);
	}
#else
	ret += snprintf(buf + ret, PAGE_SIZE, "=========================\n");
	ret += snprintf(buf + ret, PAGE_SIZE, " FW_Version      = [%02d.%02d]\n", data->fw_info->fw_ver[0], data->fw_info->fw_ver[1]);
	ret += snprintf(buf + ret, PAGE_SIZE, " RELEASE_Version = [%02d.%02d]\n", data->fw_info->rel_ver[0], data->fw_info->rel_ver[1]);
	ret += snprintf(buf + ret, PAGE_SIZE, " Num of channel  = %d\n", MAX_CHANNEL);
	ret += snprintf(buf + ret, PAGE_SIZE, "=========================\n");
	ret += snprintf(buf + ret, PAGE_SIZE, "Channel Status : %s\n", (fac_reg_chk == FW_STATUS_NORMAL) ? "PASS" : "FAIL");
	ret += snprintf(buf + ret, PAGE_SIZE, "Raw Data : %s\n", (result[GET_REFERENCE] == SD_PASS) ? "PASS" : "FAIL");
	if (result[GET_REFERENCE] != SD_PASS) {
		ret += snprintf(buf + ret, PAGE_SIZE, " Raw Status channel[%d] Fail\n", result[GET_REFERENCE]);
	}
#endif
	ret += snprintf(buf+ret, PAGE_SIZE, "=========================\n");

	/* Result of self-diagnostic write to file */
	lu201x_write_file(SELF_DIAGNOSTIC_FILE_PATH, buf, 1, ret);
	lu201x_write_file(SELF_DIAGNOSTIC_FILE_PATH, temp_buf, 0, strlen(temp_buf));
	kfree(temp_buf);

	touch_enable_irq(data->irq);
	mutex_unlock(&sys_lock);

	return ret;
}

static ssize_t lu201x_testmode_ver_show(struct lu201x_data *data, char *buf)
{
	struct lu201x_fw_info *fw_info = data->fw_info;
	int ret = 0;

	ret += snprintf(buf+ret, PAGE_SIZE, "F:%02d%02d R:%02d%02d\n", \
		fw_info->fw_ver[0], fw_info->fw_ver[1], fw_info->rel_ver[0], fw_info->rel_ver[1]);

	return ret;
}

static LGE_TOUCH_ATTR(read_fw, S_IWUSR, NULL, lu201x_read_fw_store);
static LGE_TOUCH_ATTR(global_access_pixel, S_IRUGO | S_IWUSR, lu201x_global_access_pixel_show, lu201x_global_access_pixel_store);
static LGE_TOUCH_ATTR(power_ctl, S_IWUSR, NULL, lu201x_power_ctl_store);
static LGE_TOUCH_ATTR(knock_on_type, S_IRUGO, show_knock_on_type, NULL);
static LGE_TOUCH_ATTR(lpwg_data, S_IRUGO | S_IWUSR, show_lpwg_data, store_lpwg_data);
static LGE_TOUCH_ATTR(lpwg_notify, S_IWUSR, NULL, store_lpwg_notify);
static LGE_TOUCH_ATTR(release_finger, S_IRUGO, lu201x_release_finger_show, NULL);
static LGE_TOUCH_ATTR(version, S_IRUGO, lu201x_version_show, NULL);
static LGE_TOUCH_ATTR(update_fw, S_IWUSR, NULL, lu201x_update_fw_store);
static LGE_TOUCH_ATTR(reference, S_IRUGO, lu201x_reference_show, NULL);
static LGE_TOUCH_ATTR(jitter_cap, S_IRUGO, lu201x_jitter_cap_show, NULL);
static LGE_TOUCH_ATTR(sd, S_IRUGO, lu201x_sd_show, NULL);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO, lu201x_testmode_ver_show, NULL);
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUSR, lu201x_delta_cap_show, lu201x_delta_cap_store);

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_read_fw.attr,
	&lge_touch_attr_global_access_pixel.attr,
	&lge_touch_attr_power_ctl.attr,
	&lge_touch_attr_knock_on_type.attr,
	&lge_touch_attr_lpwg_data.attr,
	&lge_touch_attr_lpwg_notify.attr,
	&lge_touch_attr_release_finger.attr,
	&lge_touch_attr_version.attr,
	&lge_touch_attr_update_fw.attr,
	&lge_touch_attr_reference.attr,
	&lge_touch_attr_jitter_cap.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_testmode_ver.attr,
	NULL
};

static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj, struct attribute *attr, char *buf)
{
	struct lu201x_data *data = container_of(lge_touch_kobj, struct lu201x_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv = container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(data, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj, struct attribute *attr, const char *buf, size_t count)
{
	struct lu201x_data *data = container_of(lge_touch_kobj, struct lu201x_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv = container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(data, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops	= &lge_touch_sysfs_ops,
	.default_attrs 	= lge_touch_attribute_list,
};

static struct sysdev_class lge_touch_sys_class = {
	.name = LGE_TOUCH_NAME,
};

static struct sys_device lge_touch_sys_device = {
	.id	= 0,
	.cls	= &lge_touch_sys_class,
};

static int lu201x_parse_dt(struct device *dev, struct lu201x_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	struct property *prop = NULL;
	u32 temp_array[16] = {0};
	int rc = 0;
	int i = 0;
	u32 temp_val = 0;

	TOUCH_INFO_MSG("%s\n", __func__);

	/* reset, irq gpio info */
	if (node == NULL)
		return -ENODEV;

	pdata->gpio_reset = of_get_named_gpio_flags(node, "lu201x,reset-gpio", 0, NULL);
	if (!pdata->gpio_reset) {
		TOUCH_INFO_MSG("DT : gpio_reset = %u\n", pdata->gpio_reset);
	} else {
		TOUCH_INFO_MSG("DT get gpio_reset error \n");
	}

	pdata->gpio_int = of_get_named_gpio_flags(node, "lu201x,irq-gpio", 0, NULL);
	if (pdata->gpio_int) {
		TOUCH_INFO_MSG("DT : gpio_int = %u\n", pdata->gpio_int);
	} else {
		TOUCH_INFO_MSG("DT get gpio_int error \n");
	}

	pdata->ldo_en = of_get_named_gpio_flags(node, "lu201x,ldo_en", 0, NULL);
	if (pdata->ldo_en) {
		TOUCH_INFO_MSG("DT : ldo_en = %u\n", pdata->ldo_en);
	} else {
		TOUCH_INFO_MSG("DT get ldo_en error \n");
	}

	rc = of_property_read_u32(node, "lu201x,num_touch", &temp_val);
	if (rc) {
		TOUCH_INFO_MSG("DT : Unable to read num_touch\n");
	} else {
		pdata->num_touch = temp_val;
		TOUCH_INFO_MSG("DT : numtouch = %d\n", pdata->num_touch);
	}

	rc = of_property_read_string(node, "lu201x,fw_name", &(pdata->fw_name));
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : lu201x,fw_name error \n");
		pdata->fw_name = NULL;
	} else {
		TOUCH_INFO_MSG("DT : fw_name : %s \n", pdata->fw_name);
	}

	rc = of_property_read_u32(node, "lu201x,global_access_pixel", &temp_val);
	if (rc) {
		TOUCH_INFO_MSG("DT : Unable to read global_access_pixel - set as 0\n" );
		pdata->global_access_pixel = 0;
	} else {
		pdata->global_access_pixel = temp_val;
	}
	TOUCH_INFO_MSG("DT : global_access_pixel = %d \n", pdata->global_access_pixel);

	rc = of_property_read_u32(node, "lu201x,max_x", &temp_val);
	if (rc) {
		TOUCH_INFO_MSG("DT : Unable to read max_x\n");
	} else {
		pdata->lcd_max_x = temp_val;
		TOUCH_INFO_MSG("DT : max_x = %d\n", pdata->lcd_max_x);
	}

	rc = of_property_read_u32(node, "lu201x,max_y", &temp_val);
	if (rc) {
		TOUCH_INFO_MSG("DT : Unable to read max_y\n");
	} else {
		pdata->lcd_max_y = temp_val;
		TOUCH_INFO_MSG("DT : max_y = %d\n", pdata->lcd_max_y);
	}

	prop = of_find_property(node, "lu201x,key_map", NULL);
	if (prop) {
		pdata->num_keys = prop->length / sizeof(temp_val);
		temp_val = pdata->num_keys;

		if (temp_val <= LU201X_MAX_KEY) {
			rc = of_property_read_u32_array(node, "lu201x,key_map", temp_array, temp_val);
			if (rc) {
				TOUCH_INFO_MSG("DT : Unable to read key codes\n");
				return rc;
			}

			for(i=0; i<temp_val; i++) {
				pdata->key_maps[i] = temp_array[i];
				TOUCH_INFO_MSG("DT : button[%d] = [%s] \n", i, get_touch_button_string(pdata->key_maps[i]));
			}
		}
	}

	return 0;
}

static void lu201x_probe_regulators(struct lu201x_data *data)
{
	struct device *dev = &data->client->dev;
	int error = 0;

	if (data->pdata->gpio_reset) {
		TOUCH_INFO_MSG("Must have reset GPIO to use regulator support\n");
		goto fail;
	}

	data->vdd_io = regulator_get(dev, "vdd_io");
	if (IS_ERR(data->vdd_io)) {
		error = PTR_ERR(data->vdd_io);
		TOUCH_INFO_MSG("Error %d getting vdd_io regulator\n", error);
		goto fail;
	}
#ifdef CONFIG_MACH_MSM8X10_Y30
	error = regulator_set_voltage(data->vdd_io, 2950000, 2950000);
#else
	error = regulator_set_voltage(data->vdd_io, 1800000, 1800000);
#endif
	if (error < 0) {
		TOUCH_INFO_MSG("Error %d cannot control vdd_io regulator\n", error);
		goto fail;
	}

	error = regulator_enable(data->vdd_io);
	if (error < 0) {
			TOUCH_INFO_MSG("vdd_io regulator enable fail\n");
			goto fail;
	}
	mdelay(1);
	TOUCH_INFO_MSG("%s VDD_IO value : %d\n", __func__, regulator_get_voltage(data->vdd_io));

	data->use_regulator = true;
	return;

fail:
	TOUCH_INFO_MSG("%s fail\n", __func__);
	data->vdd_io = NULL;
	data->use_regulator = false;
}


void LU201x_reset(struct lu201x_data *data, unsigned int on)
{
	if (on) {
		msleep(10);
		TOUCH_INFO_MSG("Reset pin is set high\n");
		gpio_set_value(data->pdata->gpio_reset, 1);
		//msleep(100);
		msleep(200);
	} else {
		TOUCH_INFO_MSG("Reset pin is set low\n");
		gpio_set_value(data->pdata->gpio_reset, 0);
	}
}

void LU201x_power(struct lu201x_data *data, unsigned int on)
{
	if (on) {
		gpio_set_value(data->pdata->ldo_en, 1);
		msleep(15);
		TOUCH_INFO_MSG("turned on the power %d\n", gpio_get_value(data->pdata->ldo_en));
		LU201x_reset(data, 1);
		//msleep ( 300 );
	} else {
		LU201x_reset(data, 0);
		gpio_set_value(data->pdata->ldo_en, 0);
		TOUCH_INFO_MSG("turned off the power %d\n", gpio_get_value(data->pdata->ldo_en));
	}
}

int lu201x_initialize_input_device(struct lu201x_data *data) {
	struct input_dev *input_dev = NULL;
	int err = 0;
	int i = 0;

	if(data->input_dev) {
		TOUCH_INFO_MSG("ignore %s\n", __func__);
		return 0;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&data->client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	input_dev->name	= LU201x_NAME;
	input_dev->id.bustype 	= BUS_I2C;
	input_dev->id.vendor	= 0xDEAD;
	input_dev->id.product	= 0xBEEF;
	input_dev->id.version	= 10427;	//screen firmware version

	err = input_mt_init_slots(input_dev, data->pdata->num_touch);
	if (err) {
		TOUCH_INFO_MSG("Error %d initialising slots\n", err);
		goto exit_input_register_device_failed;
	}

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit); /* My device is Touch screen */
	set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, data->pdata->lcd_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, data->pdata->lcd_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 100, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 100, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 15, 0, 0);

	/* Touch Key Event */
	//input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

	for (i = 0; i < data->pdata->num_keys; i++) {
		input_set_capability(input_dev, EV_KEY, data->pdata->key_maps[i]);
		input_dev->keybit[BIT_WORD(data->pdata->key_maps[i])] |= BIT_MASK(data->pdata->key_maps[i]);
	}

	input_set_drvdata(input_dev, data);
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&data->client->dev,
		"LU201x_ts_probe: failed to register input device: %s\n",
		dev_name(&data->client->dev));
		goto exit_input_register_device_failed;
	}

	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
	return err;
}

#if defined (CONFIG_HAS_EARLYSUSPEND)
static void LU201x_ts_suspend(struct early_suspend *handler)
{
	TOUCH_INFO_MSG("%s\n", __func__);
}

static void LU201x_ts_resume(struct early_suspend *handler)
{
	TOUCH_INFO_MSG("%s\n", __func__);
}
#endif

#if defined (CONFIG_FB)
static void LU201x_suspend(struct lu201x_data *data)
{
	TOUCH_INFO_MSG("******enter ts_suspend\n");
}

static void LU201x_resume(struct lu201x_data *data)
{
	TOUCH_INFO_MSG("******enter ts_wakeup\n");
}

int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	struct lu201x_data *ts = container_of(self, struct lu201x_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			LU201x_resume(ts);
		else if (*blank == FB_BLANK_POWERDOWN)
			LU201x_suspend(ts);
	}

	return 0;
}
#endif

static int LU201x_pm_suspend(struct device *device)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	mutex_lock(&i2c_suspend_lock);
	return 0;
}


static int LU201x_pm_resume(struct device *device)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	mutex_unlock(&i2c_suspend_lock);
	return 0;
}

static int LU201x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lu201x_data *data;
	int err = 0;
	int ret = 0, i = 0;
#ifndef CONFIG_MACH_MSM8X10_Y30
	u8 DeviceID[2] = {0,};

#endif 
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	TOUCH_INFO_MSG("%s\n", __func__);
	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch_wakelock");
	wake_lock_init(&knock_code_lock, WAKE_LOCK_SUSPEND, "knock_code");

	mutex_init(&irq_lock);
	mutex_init(&i2c_suspend_lock);
	mutex_init(&sys_lock);

	data = kzalloc(sizeof(struct lu201x_data), GFP_KERNEL);
	if (data == NULL) {
		TOUCH_INFO_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Read dtsi data */
	if (client->dev.of_node) {
		data->pdata = devm_kzalloc(&client->dev, sizeof(struct lu201x_platform_data), GFP_KERNEL);
		if (!data->pdata) {
			dev_err(&client->dev, "%s: platform data is null\n", __func__);
			err = -ENOMEM;
			goto exit_platform_data_null;
		}
		err = lu201x_parse_dt(&client->dev, data->pdata);
		if (err)
			goto exit_dts_error;
	}

	data->client = client;
	data->irq = client->irq;
	i2c_set_clientdata(client, data);

	/* GPIO Configuration */
	ret = gpio_request(data->pdata->gpio_int, "lu201x_irq");
        if (ret) {
                TOUCH_INFO_MSG("unable to request interrupt gpio-%d\n",data->pdata->gpio_int);
                return -1;
        }
	gpio_direction_input(data->pdata->gpio_int);

	ret = gpio_request(data->pdata->gpio_reset, "lu201x_reset");
        if (ret) {
                TOUCH_INFO_MSG("unable to request reset gpio-%d\n",data->pdata->gpio_reset);
                return -1;
        }
	gpio_direction_output(data->pdata->gpio_reset, 1);

	/* Turn On Touch IC */
	LU201x_power(data, 0);
	LU201x_power(data, 1);
	lu201x_probe_regulators(data);

	/* Check Touch Deivce ID */
#ifndef CONFIG_MACH_MSM8X10_Y30
	err = i2c_read_bytes_LU201x(client, LU201x_DEVICEID_ADDR, DeviceID, sizeof(DeviceID));
	if (err < 0) {
		TOUCH_INFO_MSG("Touch IC is not LeadingUI\n");
		return -1;
	} else if((DeviceID[1] != 0x20 ) || (DeviceID[0] != 0x10)) {
		TOUCH_INFO_MSG("Device ID is fail (%x%x)\n", DeviceID[1], DeviceID[0]);
		return -1;
	}
#endif
	/* Update the Firmware */
	if (data->pdata->fw_name != NULL) {
		data->fw_info = (struct lu201x_fw_info *) kmalloc(sizeof(struct lu201x_fw_info), GFP_KERNEL);
		data->fw_image = (struct lu201x_fw_image *) kmalloc(sizeof(struct lu201x_fw_image), GFP_KERNEL);

		for (i = 0; i < 3; i++) {
#ifndef CONFIG_MACH_MSM8X10_Y30
			err = LU201x_update_firmware(data, data->pdata->fw_name);
#endif
			if (err == 0 || i == 2) {
				break;
			} else {
				LU201x_reset(data, 0);
				LU201x_reset(data, 1);
				TOUCH_INFO_MSG("Firmware check Retry %d\n", i+1);
			}
		}
	}
#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = LU201x_ts_suspend;
	data->early_suspend.resume = LU201x_ts_resume;
	register_early_suspend(&data->early_suspend);
#endif

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);
	if (err)
		TOUCH_INFO_MSG("Unable to register fb_notifier: %d\n", err);
#endif

	err = lu201x_initialize_input_device(data);
	if (err) {
		TOUCH_INFO_MSG("Failed to lu201x_initialize_input_device\n");
		goto exit_input_register_fail;
	}

	/* Register sysfs for making fixed communication path to framework layer */
	err = sysdev_class_register(&lge_touch_sys_class);
	if (err < 0) {
		TOUCH_INFO_MSG("sysdev_class_register is failed\n");
		goto exit_lge_touch_sys_class_register;
	}

	err = sysdev_register(&lge_touch_sys_device);
	if (err < 0) {
		TOUCH_INFO_MSG("sysdev_register is failed\n");
		goto exit_lge_touch_sys_dev_register;
	}

	err = kobject_init_and_add(&data->lge_touch_kobj, &lge_touch_kobj_type,
			data->input_dev->dev.kobj.parent, "%s", LGE_TOUCH_NAME);
	if (err < 0) {
		TOUCH_INFO_MSG("kobject_init_and_add is failed\n");
		goto exit_lge_touch_sysfs_init_and_add;
	}

	INIT_WORK(&data->pen_event_work, LU201x_ts_pen_irq_work);
	INIT_DELAYED_WORK(&data->resume_work, lu201x_resume_work_func);

	data->ts_workqueue = create_singlethread_workqueue(LU201x_NAME);
	if (!data->ts_workqueue) {
		err = -ESRCH;
		goto exit_singlethread_fail;
	}

	err = request_irq(data->irq, LU201x_ts_interrupt, \
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, "LU201x_ts", data);
	if (err < 0) {
		dev_err(&client->dev, "LU201x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	touch_disable_irq(data->irq);
	data->pdata->lpwg_panel_on = 1;
	queue_delayed_work(data->ts_workqueue, &data->resume_work, msecs_to_jiffies(2000));

	TOUCH_INFO_MSG("*********probe end\n");
	return 0;

exit_irq_request_failed:
exit_singlethread_fail:
exit_lge_touch_sysfs_init_and_add:
	kobject_del(&data->lge_touch_kobj);

exit_lge_touch_sys_dev_register:
	sysdev_unregister(&lge_touch_sys_device);

exit_lge_touch_sys_class_register:
	sysdev_class_unregister(&lge_touch_sys_class);
	i2c_set_clientdata(client, NULL);
	TOUCH_INFO_MSG("lu201x_free_input_device\n");
	if (fb_unregister_client(&data->fb_notif))
		TOUCH_INFO_MSG("Error occurred while unregistering fb_notifier.\n");
	input_unregister_device(data->input_dev);
	data->input_dev = NULL;

exit_input_register_fail:
	kfree(data->fw_info);
	kfree(data->fw_image);
	gpio_free(data->pdata->gpio_reset);
	gpio_free(data->pdata->gpio_int);

exit_dts_error:
	kfree(data->pdata);

exit_platform_data_null:
	kfree(data);

exit_check_functionality_failed:
	return err;
}

static int __devexit LU201x_ts_remove(struct i2c_client *client)
{
	struct lu201x_data *data = i2c_get_clientdata(client);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);
	input_unregister_device(data->input_dev);
	kobject_del(&data->lge_touch_kobj);
	sysdev_unregister(&lge_touch_sys_device);
	sysdev_class_unregister(&lge_touch_sys_class);

	mutex_destroy(&irq_lock);
	mutex_destroy(&i2c_suspend_lock);
	mutex_destroy(&sys_lock);

	cancel_work_sync(&data->pen_event_work);
	destroy_workqueue(data->ts_workqueue);

	kfree(data->fw_info);
	kfree(data->fw_image);
	kfree(data);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static struct of_device_id lu201x_match_table[] = {
	{ .compatible = "leadingUI,lu201x",},
	{ },
};

static const struct i2c_device_id LU201x_ts_id[] = {
	{ LU201x_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, LU201x_ts_id);

static struct dev_pm_ops touch_pm_ops = {
	.suspend 	= LU201x_pm_suspend,
	.resume 	= LU201x_pm_resume,
};

static struct i2c_driver LU201x_ts_driver = {
	.probe		= LU201x_ts_probe,
	.remove		= __devexit_p(LU201x_ts_remove),
	.id_table	= LU201x_ts_id,
	.driver	= {
		.name	= LU201x_NAME,
		.owner	= THIS_MODULE,
		.pm = &touch_pm_ops,
		.of_match_table = lu201x_match_table,
	},
};

static int __init LU201x_ts_init(void)
{
	return i2c_add_driver(&LU201x_ts_driver);
}

static void __exit LU201x_ts_exit(void)
{
	i2c_del_driver(&LU201x_ts_driver);
}

module_init(LU201x_ts_init);
module_exit(LU201x_ts_exit);

MODULE_AUTHOR("<taegyun.an@lge.com>");
MODULE_DESCRIPTION("LeadingUI LU201x TouchScreen driver");
MODULE_LICENSE("GPL");
