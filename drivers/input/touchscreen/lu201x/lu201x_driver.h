/*
 * include/linux/lu2010.h
 *
 * Copyright (C) 2011 lu2010, Inc.
 *
 */
#ifndef 	__LU2010_H
#define	__LU2010_H

#define LGE_TOUCH_NAME	"lge_touch"

#define LU201X_MAX_KEY 4
#define MAX_FINGER_NUM 2
#define MAX_CHANNEL 34

/* LeadingUI Firmware */
#define FW_SIZE 		30*1024
#define CFG_SIZE 		1*1024
#define FAC_SIZE		1*1024

#define FAC_POS			0xFC00
#define FW_POS			0x8000

/* Firmware File Path */
#define FW_FILE_PATH "/mnt/sdcard/lu201x_fw.bin"
#define SELF_DIAGNOSTIC_FILE_PATH "/mnt/sdcard/touch_self_test.txt"

/* Knock On/Code */
#define KNOCK_ON_STAUS		0x0082
#define KNOCK_TAP_COUNT		0x0083
#define KNOCK_STATUS		0x00C0
#define KNOCK_TAP_THON		0x00C1
#define KNOCK_EXCEPT_PALM_ONCH	0x00C5
#define KNOCK_WAKEUP_INTERVAL	0x00C9
#define KNOCK_TAPOFF_TIMEOUT	0x00D2
#define KNOCK_ON_TAP_COUNT	0x00D4
#define KNOCK_ON_REPORT_WAIT_TIME	0x00D5

#define KNOCK_CODE_EXCEPT_TAP_WAIT_TIME	0x00DB
#define KNOCK_CODE_TAP_COUNT		0x00DD
#define KNOCK_CODE_REPORT_WAIT_TIME	0x00DE

/* Touch Event Type */
#define TTYPE_PRESS		0x01
#define TTYPE_MOVE		0x02
#define TTYPE_RELEASE		0x03

/* Key Event Type */
#define KEY_PRESSED		1
#define KEY_RELEASED		0
#define CANCEL_KEY		0xFF

#define SCREEN_MAX_X    	1280
#define SCREEN_MAX_Y    	800
#define PRESS_MAX       	255

#define EVENT_NONE		0x00
#define EVENT_ABS		0x01
#define EVENT_KEY		0x02
#define EVENT_GEST		0x04
#define EVENT_MOUSE		0x08

#define FWSTATUS_NORMAL		0x00
#define FWSTATUS_INITREQ	0xFF
#define FWSTATUS_CHFAIL		0xfe
#define FWSTATUS_CALFAIL	0xfd

#define I2C_DEVICE_ADDRESS_LEN	2
#define MAX_TRANSACTION_LENGTH	8

#define FW_STATUS_REG		0x0000
#define FW_VERSION_REG		0x0080

#define LU201x_MODE_ADDR	0x00E0
#define LU201x_CMDACK_ADDR	0x00ED

#define LU201x_DEVICEID_ADDR	0x10FD
#define LU201x_I2CDONE_ADDR	0x10FF
#define LU201x_CMDReply_ADDR	0x0100

#define CMD_I2C_DONE		0x01
#define CMD_LU201x_CHANGEMODE	0xA3
#define CMD_LU201x_DEVINFO 	0xAA
#define CMD_LU201x_CHCAPTEST	0xB6
#define LU201x_CHCAPTEST_Reply	0xC1

/* Command */
#define LU201x_CMD_FW_CHECKSUM_ADDR	0x0158
#define LU201x_CMD_FW_STATUS_ADDR	0x0102

/* Major Mode */
#define CMD_LU201x_NORMODE		0x00
#define CMD_LU201x_PDN			0x01
#define CMD_LU201x_DEBUG		0x02
#define CMD_LU201x_IDLE_DOUBLETAB	0x11
#define CMD_LU201x_IDLE_MULTITAB	0x11

/* Minor Mode */
#define CMD_LU201x_NONE		0x0000
#define CMD_LU201x_REFCAP	0x0000

enum {
	CMD_LPWG_ENABLE = 1,
	CMD_LPWG_LCD = 2,
	CMD_LPWG_ACTIVE_AREA = 3,
	CMD_LPWG_TAP_COUNT = 4,
	CMD_LPWG_LCD_RESUME_SUSPEND = 6,
	CMD_LPWG_PROX = 7,
	CMD_LPWG_DOUBLE_TAP_CHECK = 8,
	CMD_LPWG_TOTAL_STATUS = 9,
};

enum {
	LPWG_DISABLE = 0,
	LPWG_DOUBLE_TAP,
	LPWG_MULTI_TAP,
};

enum fw_region {
	FACTORY_REGION = 0,
	FIRMWARE_REGION,
};

enum flash_update_mode {
	NORMAL = 0,
	FIRMWARE
};

enum {
	LCD_OFF = 0,
	LCD_ON,
};

enum {
	TOUCH_OFF = 0,
	TOUCH_ON,
};

enum {
	PROX_NEAR = 0,
	PROX_FAR = 1,
};

enum power_mode {
	HW_RESET = 0,
	SW_RESET,
};

enum {
	SD_PASS = 100,
	SD_FAIL = 101,
};

enum {
	GET_REFERENCE = 0,
	GET_JITTER,
	GET_RAW_MAX,
};

enum {
	FW_STATUS_NORMAL = 0,
	FW_STATUS_CH_FAIL,
	FW_STATUS_CAL_FAIL,
};

enum {
	GET_NONE = 0,
	GET_HISTO,		//max channel
	GET_SUMHISTO,		//half channel
	GET_CURCAP,		//max channel
	GET_REFCAP,		//max channel
	GET_SUMDIVHISTO		//half channel
};

struct point {
    int x;
    int y;
};

struct lu201x_self_diagnostic {
	u16 ch_min[MAX_CHANNEL];
	u16 ch_max[MAX_CHANNEL];
	u8 ch_jitter;
	u8 reserve[7];
};

struct lu201x_fw_image {
	u8 model_name[5];
	u8 fw_ver[2];
	u8 rel_ver[2];
	u8 fw_crc[4];
	u8 parm_crc[2];
	u8 fw_header_size;
	struct lu201x_self_diagnostic	sd;
	u8 fw_bin[0];
};

struct lu201x_fw_info {
	u8 model_name[5];
	u8 fw_ver[2];
	u8 rel_ver[2];
	u8 fw_crc[4];
	u8 parm_crc[2];
	u32 fw_bin_size;

	u8 *fw_raw_data;	/* start address of firmware data */
	u8 *fac_raw_data;	/* start address of factory data */
	u8 *cfg_raw_data;	/* start address of configuration data */
};

struct point_data {
	u8 status;
	u8 id;
	u16 x;
	u16 y;
};

struct ts_event {
	u16 touch_point;
	u16 prev_touch_point;
	struct point_data point[MAX_FINGER_NUM];
	struct point_data prev_point[MAX_FINGER_NUM];
};

struct LU201xTPD_INFO {
	u8 FWStatus;		// 0x0000
	u8 EventType;		// 0x0001
	u8 VPCount;		// 0x0002
	u8 KeyData[2];		// 0x0003
	u8 Point[8];		// 0x0005 X0 position
};

struct lu201x_platform_data {
	u8 num_touch;
	int model;

	u8 gpio_reset;
	u8 gpio_int;
	u8 ldo_en;

	unsigned int lcd_max_x;
	unsigned int lcd_max_y;

	const char *fw_name;
	u8 num_keys;
	u8 key_maps[LU201X_MAX_KEY];
	u32 global_access_pixel;

	/* LPWG */
	int lpwg_panel_on;
	int lpwg_prox;
	int report_enable;
	u8 lpwg_mode;
	u16 double_tap_check;
};

struct lu201x_data {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct lu201x_platform_data 	*pdata;
	struct ts_event			event;
	struct work_struct 		pen_event_work;
	struct delayed_work		resume_work;
	struct workqueue_struct 	*ts_workqueue;
#if defined (CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_suspend;
#endif
#if defined (CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	struct kobject			lge_touch_kobj;
	int				irq;

	struct regulator 		*vdd_io;
	bool 				use_regulator;

	struct LU201xTPD_INFO 		LU201x_tpd;
	struct lu201x_fw_info		*fw_info;
	struct lu201x_fw_image		*fw_image;
	u8 				reportd_keycode;

	u8				need_fw;
};

#define TOUCH_INFO_MSG(fmt, args...) 	printk(KERN_ERR "[Touch] " fmt, ##args)
#define TOUCH_ERR_MSG(fmt, args...) printk(KERN_ERR "[Touch E] [%s %d] " fmt, __FUNCTION__, __LINE__, ##args)

/****************************************************************************
* Touch FUNCTION Declaration
****************************************************************************/
void LU201x_reset(struct lu201x_data *data, unsigned int on);
void LU201x_power(struct lu201x_data *data, unsigned int on);
static int touch_knock_check(struct lu201x_data *data);
static void LU201x_release_all_finger(struct lu201x_data *data);

#endif /* _LU2010_TOUCH_H */