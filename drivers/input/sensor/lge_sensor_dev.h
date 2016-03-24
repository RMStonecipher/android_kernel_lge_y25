/* lge_sensor_dev.h
 *
 * Copyright (C) 2014 LGE.
 *
 * Author: jiwon.seo@lge.com
 * Modified : jude.lee@lge.com
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

#ifndef __LGE_SENSOR_DEV_H__
#define __LGE_SENSOR_DEV_H__

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/string.h>

#define	LGE_SENSOR_DEV_NAME		"lge_sensor"
/* set i2c bus & number of sensors which to select : start */
#define ACCEL_I2C_BUS_NUM 3
#define MAX_CHOOSE_ACC_NUM	2
/* set i2c bus & number of sensors which to select : end */
/*----------------------------------------------------------------------------*/
#define SEN_TAG                  "[DualSenor] "
#define SEN_FUN(f)               printk(KERN_INFO SEN_TAG"%s\n", __FUNCTION__)
#define SEN_ERR(fmt, args...)    printk(KERN_ERR SEN_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SEN_LOG(fmt, args...)    printk(KERN_INFO SEN_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/

/* sensor basic information */
struct accel_sensor_info{
	char vendor[20];  /* sensor company */
    char name[20];    /* sensor name */
    int slave_address;  /* slave_address */
    int who_am_I_reg;   /* who_am_I_reg */
    int who_am_I_value;  /* who_am_I_value */
};

struct sensor_init_info {
    char *name;
    int (*init)(void);
	int (*uninit)(void);
};

/*----------------------------------------------------------------------------*/
extern int lge_sensor_acc_add(struct sensor_init_info* obj);
/*----------------------------------------------------------------------------*/

#endif