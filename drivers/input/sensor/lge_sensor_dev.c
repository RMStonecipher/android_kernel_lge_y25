/* lge_sensor_dev.c
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

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "lge_sensor_dev.h"

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES			5

/* sensor name used only in algorithm */
char SENSOR_NAME[20] = "none";

/* detail information of sensors : start */
struct accel_sensor_info accel_sensor_list[MAX_CHOOSE_ACC_NUM]={
	/* STM, K3DH K2DH */
	{
		"STMicron",		/* sensor company */
		"K3DH",			/* sensor name */
		0x18,			/* slave_address */
		0x0f,			/* who_am_I_reg */
		0x33			/* who_am_I_value */
	},
	/* STM, K2HH */
	{
		"STMicron",		/* sensor company */
		"K2HH",			/* sensor name */
		0x1e,			/* slave_address */
		0x0f,			/* who_am_I_reg */
		0x41,			/* who_am_I_value */
	}
};

enum sensor_list{
      STMicron,
      BOSCH
};
/* detail information of sensors : end */

struct sensor_init_info* acc_sensor_init_list[MAX_CHOOSE_ACC_NUM] = {0};
static int Is_Sensor_Checked = 0;

static int lge_sensor_i2c_read(struct accel_sensor_info *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_adapter *adap;

	struct i2c_msg msgs[] = {
		{
			.addr = acc->slave_address,
			.flags = 0,
			.len = 1,
			.buf = buf, },
		{
			.addr = acc->slave_address,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf, },
	};

		adap = i2c_get_adapter(ACCEL_I2C_BUS_NUM);
		if (!adap) {
			SEN_ERR("get i2c adapter faild\n");
			return -ENODEV;
		}

		do {
			err = i2c_transfer(adap, msgs, 2);
			if (err != 2)
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((err != 2) && (++tries < I2C_RETRIES));

		if (err != 2) {
			SEN_ERR("read transfer error\n");
			err = -EIO;
		} else {
			err = 0;
		}

	return err;

}

int Accel_Dev_Check(void)
{

	int err = -1;
	int i = 0;
	u8 acc_data[2] = {0, };

	if(Is_Sensor_Checked == 0) {
		for(i = 0; i < MAX_CHOOSE_ACC_NUM; i++) {
			acc_data[0] = accel_sensor_list[i].who_am_I_reg;
			err = lge_sensor_i2c_read(&accel_sensor_list[i], acc_data, 1);

			if( err < 0) {
				SEN_ERR("%d I2C fail lge_sensor_i2c_read\n",err);
				SEN_ERR("Accelerometer %s does not exist",accel_sensor_list[i].name);
			}

			if(acc_data[0] == accel_sensor_list[i].who_am_I_value) {
				SEN_LOG("Accel_Dev_Check: Read Value = 0x%x\n",acc_data[0]);
				SEN_LOG("Accel_Dev_Check: %s Sensor\n",accel_sensor_list[i].name);
				strcpy(SENSOR_NAME,accel_sensor_list[i].name);
				break;
			}
		}

		Is_Sensor_Checked = 1;

	} else {
		SEN_LOG("Accel_Dev_Check : %s Sensor(Checked)\n",SENSOR_NAME);
		err = 1;
		return err;
	}

	return err;
}

int Accel_Dev_Register(void)
{
	int err = -1;
	int i=0;

	for(i = 0; i < MAX_CHOOSE_ACC_NUM; i++)	{
		SEN_LOG("Accel_Dev_Register : i=%d\n",i);
		if (acc_sensor_init_list[i] == NULL) {
			return err;
		}

		if(!strcmp(SENSOR_NAME,acc_sensor_init_list[i]->name))
			err = acc_sensor_init_list[i]->init();

		if(err == 0) {
			SEN_LOG("LGE Accelerometer %s probe ok\n", SENSOR_NAME);
			return err;
		}
	}

	if(err != 0) {
		SEN_LOG("Accel_Dev_Register : re-check all drivers\n");
		for(i = 0; i < MAX_CHOOSE_ACC_NUM; i++) {
			if(acc_sensor_init_list[i] != 0) {
				err = acc_sensor_init_list[i]->init();
				if(err == 0) {
					SEN_LOG("LGE Accelerometer %s probe ok\n", SENSOR_NAME);
					break;
				}
			}
		}
	}

	if(i == MAX_CHOOSE_ACC_NUM)
			SEN_LOG("LGE Accelerometer probe fail\n");

	return err;
}

int Accel_Probe_Available(struct i2c_client *client)
{
	/* result 1 : Doing probe */
	int result = 0;
	int i=0;

	if(Is_Sensor_Checked==0) {
		SEN_LOG("Accel_Probe_Available : Is_Sensor_Checked==0\n");
		result = 1;
   }
	else {
	SEN_LOG("Accel_Probe_Available : Is_Sensor_Checked==1\n");
	for(i = 0; i < MAX_CHOOSE_ACC_NUM; i++) {
		if(accel_sensor_list[i].slave_address==client->addr) {
			if(!strcmp(SENSOR_NAME,accel_sensor_list[i].name)) {
				SEN_LOG("TEST8 : strcmp is success\n");
				SEN_LOG("i2c = 0x%x, name = %s\n",client->addr,SENSOR_NAME);
				result = 1;
				}
			}
		}
	}

	return result;
}

static int lge_sensor_acc_probe(struct platform_device *pdev)
{
	SEN_LOG("%s LGE lge_sensor_acc_probe init\n", LGE_SENSOR_DEV_NAME);

	if (Accel_Dev_Check() < 0)
		SEN_ERR("Accelerometer check failed\n");

	if (Accel_Dev_Register() < 0)
		SEN_ERR("Accelerometer register failed\n");

	return 0;
}

static int lge_sensor_acc_remove(struct platform_device *pdev)
{
	int i = 0;

	for(i = 0 ; i < MAX_CHOOSE_ACC_NUM ; i++) {
		if(strcmp(SENSOR_NAME, acc_sensor_init_list[i]->name) == 0) {
			if(acc_sensor_init_list[i]->uninit == NULL) {
				SEN_ERR("lge_sensor_acc_remove null pointer\n");
				return -1;
			}
			acc_sensor_init_list[i]->uninit();
		}
	}
	return 0;
}

static struct miscdevice lge_acc_misc_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = LGE_SENSOR_DEV_NAME,
};

static const struct of_device_id lge_sensor_dt_match[] = {
	{.compatible = "lge,sensor"},
	{},
};
MODULE_DEVICE_TABLE(of, lge_sensor_dt_match);

static struct platform_driver lge_sensor_acc_driver = {
	.driver	= {
			.name  = LGE_SENSOR_DEV_NAME,
			.owner = THIS_MODULE,
			.of_match_table = lge_sensor_dt_match,
	},
	.probe	= lge_sensor_acc_probe,
	.remove	= lge_sensor_acc_remove,
};

int lge_sensor_acc_add(struct sensor_init_info* obj)
{
    int err = 0;
	int i = 0;
	SEN_FUN(f);

	for(i = 0; i < MAX_CHOOSE_ACC_NUM; i++ ) {
		if(acc_sensor_init_list[i] == NULL) {
			acc_sensor_init_list[i] = kzalloc(sizeof(struct sensor_init_info), GFP_KERNEL);
			if(acc_sensor_init_list[i] == NULL) {
				SEN_ERR("kzalloc error");
				return -1;
		  }
		  acc_sensor_init_list[i] = obj;
		  break;
	    }
	}

	return err;
}
EXPORT_SYMBOL_GPL(lge_sensor_acc_add);

static int __init lge_sensor_init(void)
{
	int err = 0;
	SEN_LOG("LGE Sensor driver: init start\n");
	err = misc_register(&lge_acc_misc_device);
	if (err < 0) {
		SEN_LOG("misc LGE Sensor regitser failed\n");
	}

	SEN_LOG("LGE Sensor driver: init end\n");
	platform_driver_register(&lge_sensor_acc_driver);

	return 0;
}

static void __exit lge_sensor_exit(void)
{
	SEN_LOG("LGE sensor driver exit\n");
	misc_deregister(&lge_acc_misc_device);
}

module_init(lge_sensor_init);
module_exit(lge_sensor_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LGE sensor device driver");
MODULE_AUTHOR("Kangyoung Lee<jude.lee@lge.com>");
