/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#define BF3905_SENSOR_NAME "bf3905"
#define PLATFORM_DRIVER_NAME "msm_camera_bf3905"
#include "../../../../../base/base.h" /*                                                                            */

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


DEFINE_MSM_MUTEX(bf3905_mut);

typedef enum {
  BF3905_60HZ,
  BF3905_50HZ,
  BF3905_HZ_MAX_NUM,
}BF3905AntibandingType;

static int bf3905_antibanding = BF3905_60HZ;  //                                


/*                                                                                              */
static ssize_t bf3905_antibanding_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
       int val =0;

       sscanf(buf,"%d",&val);
       printk("bf3905: antibanding type [0x%x] \n",val);

       /* 1 : Antibanding 60Hz        * 2 : Antibanding 50Hz */
       switch(val)
       {
			case 1:
				bf3905_antibanding = BF3905_60HZ;
				break;
			case 2:
				bf3905_antibanding = BF3905_50HZ;
				break;
			default:
			pr_err("bf3905: invalid antibanding type[%d] \n",val);
			bf3905_antibanding = BF3905_50HZ;
			break;
		}

	return n;
}

static DEVICE_ATTR(antibanding, /*S_IRUGO|S_IWUGO*/ 0664, NULL, bf3905_antibanding_store);

static struct attribute* bf3905_sysfs_attrs[] = {
       &dev_attr_antibanding.attr,
};

static struct device_attribute* bf3905_sysfs_symlink[] = {
       &dev_attr_antibanding,
	   	NULL,
};

static int bf3905_sysfs_add(struct kobject* kobj)
{
	int i, n, ret;

	n = ARRAY_SIZE(bf3905_sysfs_attrs);
	for(i = 0; i < n; i++){
		if(bf3905_sysfs_attrs[i]){
			ret = sysfs_create_file(kobj, bf3905_sysfs_attrs[i]);
				if(ret < 0){
					pr_err("bf3905 sysfs is not created\n");
					}
			}
		}
	return 0;
};
static int bf3905_sysfs_add_symlink(struct device *dev)
{
	 int i = 0;
	 int rc = 0;
	 int n =0;
	 struct bus_type *bus = dev->bus;

	n = ARRAY_SIZE(bf3905_sysfs_symlink);
	for(i = 0; i < n; i++){
		if(bf3905_sysfs_symlink[i]){
			rc = device_create_file(dev, bf3905_sysfs_symlink[i]);
				if(rc < 0){
					pr_err("bf3905_sysfs_symlink is not created\n");
					goto out_unreg;
				}
			}
		}

	if(bus){
		//  PATH of bus->p->devices_kset = /sys/bus/platform/devices/
		rc = sysfs_create_link(&bus->p->devices_kset->kobj, &dev->kobj, "cam_sensor_vt");

		if(rc)
			goto out_unlink;
	}

	pr_err("bf3905_sysfs_add_symlink is created\n");
	return 0;

out_unreg:
	pr_err("fail to creat device file for antibanding");
	for (; i >= 0; i--)
		device_remove_file(dev, bf3905_sysfs_symlink[i]);

	return rc;

out_unlink:
	pr_err("fail to creat sys link for antibanding");
	sysfs_remove_link(&bus->p->devices_kset->kobj, "cam_sensor_vt");
	return rc;

};
/*                                                                                             */

static struct msm_sensor_ctrl_t bf3905_s_ctrl;

static struct msm_sensor_power_setting bf3905_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
		{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},

	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_camera_i2c_reg_conf bf3905_recommend_settings1[] = {
	{0x15, 0x12},
	{0x09, 0x01},
	{0x12, 0x00},
	{0x3a, 0x00},
	{0x1e, 0x70}, //0x40}, mirror and vetically flip
	{0x1b, 0x0e},
	{0x2a, 0x00},
	{0x2b, 0x10},
	{0x92, 0x09},
	{0x93, 0x00},
	{0x8a, 0x96},
	{0x8b, 0x7d},
	{0x13, 0x00},
	{0x01, 0x15},
	{0x02, 0x23},
	{0x9d, 0x20},
	{0x8c, 0x02},
	{0x8d, 0xee},
	{0x13, 0x07},
	{0x5d, 0xb3},
	{0xbf, 0x08},
	{0xc3, 0x08},
	{0xca, 0x10},
	{0x62, 0x00},
	{0x63, 0x00},
	{0xb9, 0x00},
	{0x64, 0x00},
	{0xbb, 0x10},
	{0x08, 0x02},
	{0x20, 0x09},
	{0x21, 0x4f},
	{0x3e, 0x83},
	{0x2f, 0x04},
	{0x16, 0xa3},//0xaf
	{0x6c, 0xc2},
	{0x27, 0x98},
	{0x71, 0x0f},
	{0x7e, 0x84},
	{0x7f, 0x3c},
	{0x60, 0xe5},
	{0x61, 0xf2},
	{0x6d, 0xc0},
	{0xd9, 0x25},
	{0xdf, 0x26},
	{0x17, 0x00},
	{0x18, 0xa0},
	{0x19, 0x00},
	{0x1a, 0x78},
	{0x03, 0xa0},
	{0x4a, 0x0c},
	{0xda, 0x00},
	{0xdb, 0xa2},
	{0xdc, 0x00},
	{0xdd, 0x7a},
	{0xde, 0x00},
	{0x34, 0x1d},
	{0x36, 0x45},
	{0x6e, 0x20},
	{0xbc, 0x0d},
	{0x35, 0x50},
	{0x65, 0x42},
	{0x66, 0x42},
	{0xbd, 0xf4},
	{0xbe, 0x44},
	{0x9b, 0xf4},
	{0x9c, 0x44},
	{0x37, 0xf4},
	{0x38, 0x44},
	{0x70, 0x0b},
	{0x73, 0x27},
	{0x79, 0x24},
	{0x7a, 0x12},
	{0x75, 0x8a},
	{0x76, 0x98},
	{0x77, 0x2a},
	{0x7b, 0x58},
	{0x7d, 0x00},
	{0x13, 0x07},
	{0x24, 0x4a},
	{0x25, 0x88},
	{0x97, 0x3c},
	{0x98, 0x8a},
};
static struct msm_camera_i2c_reg_conf bf3905_recommend_settings2[] = {
	{0x81, 0x00},
	{0x82, 0x2a},
	{0x83, 0x54},
	{0x84, 0x39},
	{0x85, 0x5d},
	{0x86, 0x88},
	{0x89, 0x63},
	{0x94, 0x82},
	{0x96, 0xb3},
	{0x9a, 0x50},
	{0x99, 0x10},
	{0x9f, 0x64},
	{0x39, 0x9c},
	{0x3f, 0x9c},
	{0x90, 0x20},
	{0x91, 0xd0},
	{0x40, 0x3b},
	{0x41, 0x36},
	{0x42, 0x2b},
	{0x43, 0x1d},
	{0x44, 0x1a},
	{0x45, 0x14},
	{0x46, 0x11},
	{0x47, 0x0f},
	{0x48, 0x0e},
	{0x49, 0x0d},
	{0x4b, 0x0c},
	{0x4c, 0x0b},
	{0x4e, 0x0a},
	{0x4f, 0x09},
	{0x50, 0x09},
	{0x5a, 0x56},
	{0x51, 0x13},
	{0x52, 0x05},
	{0x53, 0x91},
	{0x54, 0x72},
	{0x57, 0x96},
	{0x58, 0x35},
	{0x5a, 0xd6},
	{0x51, 0x28},
	{0x52, 0x35},
	{0x53, 0x9e},
	{0x54, 0x7d},
	{0x57, 0x50},
	{0x58, 0x15},
	{0x5c, 0x26},
	{0x6a, 0x81},
	{0x23, 0x55},
	{0xa1, 0x31},
	{0xa2, 0x0d},
	{0xa3, 0x27},
	{0xa4, 0x0a},
	{0xa5, 0x2c},
	{0xa6, 0x04},
	{0xa7, 0x1a},
	{0xa8, 0x18},
	{0xa9, 0x13},
	{0xaa, 0x18},
	{0xab, 0x1c},
	{0xac, 0x3c},
	{0xad, 0xf0},
	{0xae, 0x57},
	{0xd0, 0x92},
	{0xd1, 0x00},
	{0xd2, 0x58},
	{0xc5, 0xaa},
	{0xc6, 0x88},
	{0xc7, 0x10},
	{0xc8, 0x0d},
	{0xc9, 0x10},
	{0xd3, 0x09},
	{0xd4, 0x24},
	{0xee, 0x30},
	{0xb0, 0xe0},
	{0xb3, 0x48},
	{0xb4, 0xe3},
	{0xb1, 0xf0},
	{0xb2, 0xa0},
	{0xb4, 0x63},
	{0xb1, 0xb0},
	{0xb2, 0xa0},
	{0x55, 0x00},
	{0x56, 0x40},
};
static struct msm_camera_i2c_reg_conf bf3905_flicker_settings[] = {
	{0x80, 0x90},
	{0x80, 0x92},
};

static struct v4l2_subdev_info bf3905_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,/* For YUV type sensor (YUV422) */
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id bf3905_i2c_id[] = {
	{BF3905_SENSOR_NAME, (kernel_ulong_t)&bf3905_s_ctrl},
	{ }
};

static int32_t msm_bf3905_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("%s:%d\n", __func__, __LINE__);
	rc = msm_sensor_i2c_probe(client, id, &bf3905_s_ctrl);
	if(rc == 0){
		if(bf3905_sysfs_add(&client->dev.kobj) < 0)
			pr_err("bf3905: failed bf3905_sysfs_add\n");
	}
	return 0;
}

static struct i2c_driver bf3905_i2c_driver = {
	.id_table = bf3905_i2c_id,
	.probe  = msm_bf3905_i2c_probe,
	.driver = {
		.name = BF3905_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client bf3905_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id bf3905_dt_match[] = {
	{.compatible = "qcom,bf3905", .data = &bf3905_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, bf3905_dt_match);

static struct platform_driver bf3905_platform_driver = {
	.driver = {
		.name = "qcom,bf3905",
		.owner = THIS_MODULE,
		.of_match_table = bf3905_dt_match,
	},
};

static void bf3905_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}

}

static int32_t bf3905_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(bf3905_dt_match, &pdev->dev);
/*                                                              */
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	}
/*                                                              */
	rc = msm_sensor_platform_probe(pdev, match->data);
	if(rc == 0){
		rc = bf3905_sysfs_add_symlink(&pdev->dev);
		pr_info("%s bf3905_sysfs_add_symlink done\n",__func__);
	}
	return rc;
}

static int __init bf3905_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&bf3905_platform_driver,
		bf3905_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&bf3905_i2c_driver);
}

static void __exit bf3905_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (bf3905_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&bf3905_s_ctrl);
		platform_driver_unregister(&bf3905_platform_driver);
	} else
		i2c_del_driver(&bf3905_i2c_driver);
	return;
}

int32_t bf3905_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t rc = 0;
	int32_t i = 0;

	mutex_lock(s_ctrl->msm_sensor_mutex);

	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);

	switch (cdata->cfgtype) {

	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];		

		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		bf3905_i2c_write_table(s_ctrl,
				&bf3905_recommend_settings1[0],
				ARRAY_SIZE(bf3905_recommend_settings1));

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client, bf3905_flicker_settings[bf3905_antibanding].reg_addr,
			bf3905_flicker_settings[bf3905_antibanding].reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d CFG_SET_INIT_SETTING - %d\n", __func__, __LINE__,bf3905_antibanding);	

		bf3905_i2c_write_table(s_ctrl,
				&bf3905_recommend_settings2[0],
				ARRAY_SIZE(bf3905_recommend_settings2));
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id 0x%x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg 0x%x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id 0x%x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);

		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
		}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
		}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
		}
/*                                                              */
	case CFG_PAGE_MODE_READ_I2C_ARRAY:{
		int16_t size=0;
		uint16_t read_data_size = 0;
		uint16_t *read_data;
		uint16_t *read_data_head;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		size = conf_array.size;		//size for write(page_mode) and read
		read_data_size = size - 1;	//size for read

		pr_err("[WX] %s: size : %d rsize : %d\n", __func__, size, read_data_size);

		if (!size || !read_data_size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(size *(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			size * sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		read_data = kzalloc(read_data_size * (sizeof(uint16_t)), GFP_KERNEL);
		if (!read_data) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}

		//check if this code is needed;;;
		if (copy_from_user(read_data, (void *)conf_array.value,
			read_data_size * sizeof(uint16_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		read_data_head = read_data;

		for(i = 0; i < size; i++){
			if(i == 0){
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, conf_array.reg_setting->reg_addr, conf_array.reg_setting->reg_data, conf_array.data_type);
			}
			else{
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, conf_array.reg_setting->reg_addr, read_data, conf_array.data_type);
				pr_err("[WX] %s read_data : %d\n", __func__, *read_data);
				read_data++;
			}
			conf_array.reg_setting++;
		}

		read_data = read_data_head;

		if (copy_to_user((void *)conf_array.value, read_data, read_data_size * sizeof(uint16_t))) {
			pr_err("%s:%d copy failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		kfree(reg_setting);
		kfree(read_data);

		reg_setting = NULL;
		read_data = NULL;
		read_data_head = NULL;

		pr_err("[WX] %s done\n", __func__);

		break;
		}
/*                                                              */
/*                                                                                          */
	case CFG_POWER_UP:

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__, __LINE__,
					s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (s_ctrl->func_tbl->sensor_power_up) {
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);

			if (rc < 0) {
				pr_err("%s POWER_UP failed\n", __func__);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
					s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	
	case CFG_POWER_DOWN:

		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (s_ctrl->func_tbl->sensor_power_down){
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);

			if (rc < 0) {
				pr_err("%s POWER_DOWN failed\n", __func__);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);

		}else{
			rc = -EFAULT;
		}
		break;
	
/*                                                                                          */
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
		}
	default:
		rc =0;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t bf3905_sensor_func_tbl = {
	.sensor_config = bf3905_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t bf3905_s_ctrl = {
	.sensor_i2c_client = &bf3905_sensor_i2c_client,
	.power_setting_array.power_setting = bf3905_power_setting,
	.power_setting_array.size = ARRAY_SIZE(bf3905_power_setting),
	.msm_sensor_mutex = &bf3905_mut,
	.sensor_v4l2_subdev_info = bf3905_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(bf3905_subdev_info),
	.func_tbl = &bf3905_sensor_func_tbl,
};

module_init(bf3905_init_module);
module_exit(bf3905_exit_module);
MODULE_DESCRIPTION("BYD VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");

