/*
 * Core MDSS framebuffer driver.
 *
 * Copyright (C) 2007 Google Incorporated
 * Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/bootmem.h>
#include <linux/console.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/memory.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/msm_mdp.h>
#include <linux/proc_fs.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/sync.h>
#include <linux/sw_sync.h>
#include <linux/file.h>
#include <linux/memory_alloc.h>
#include <linux/kthread.h>

#include <mach/board.h>
#include <mach/memory.h>
#include <mach/iommu.h>
#include <mach/iommu_domains.h>
#include <mach/msm_memtypes.h>

#include "mdss_fb.h"
#include "mdss_dsi.h"

#if defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CN) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGNM_KDDI_JP) || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_AKA_KR) || defined(CONFIG_MACH_MSM8926_E2JPS_JP)
#include "mdss_debug.h"
#endif

#ifdef CONFIG_MACH_LGE
#include "mdss_mdp.h"
#include <mach/board_lge.h>
#if defined(CONFIG_LGE_MIPI_TOVIS_VIDEO_540P_PANEL) || defined(CONFIG_FB_MSM_MIPI_TIANMA_VIDEO_QHD_PT_PANEL)
extern int is_dsv_cont_splash_screening_f;
#endif
#if defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_WVGA_PT_INCELL_PANEL)
extern int is_dsv_cont_splash_screening_f;
#endif
static int force_set_bl_f;
unsigned long msm_fb_phys_addr_backup;
#endif

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MDSS_FB_NUM 3
#else
#define MDSS_FB_NUM 2
#endif

#if defined(CONFIG_MACH_MSM8926_X3_TRF_US) || defined(CONFIG_MACH_MSM8926_X3N_KR) || defined(CONFIG_MACH_MSM8926_F70N_KR) || defined(CONFIG_MACH_MSM8926_X3N_OPEN_EU) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_AME) || defined(CONFIG_MACH_MSM8926_F70_GLOBAL_COM) || \
	defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CN) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGNM_KDDI_JP) || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_E2JPS_JP)
#define UI_BL_OFF		0
#define UI_0_BL			10
#define UI_20_BL		60
#define UI_40_BL		110
#define UI_60_BL		157
#define UI_80_BL		207
#define UI_MAX_BL		255

#define LGE_BL_OFF		0
#define LGE_0_BL		100
#define LGE_20_BL		230
#define LGE_40_BL		630
#define LGE_60_BL		1280
#define LGE_80_BL		2480
#define LGE_MAX_BL		4095
#elif defined(CONFIG_MACH_MSM8926_E2_SPR_US) || defined(CONFIG_MACH_MSM8926_E2_MPCS_US) || defined(CONFIG_MACH_MSM8926_E2_VZW) || defined(CONFIG_MACH_MSM8926_E2_BELL_CA) || defined(CONFIG_MACH_MSM8926_E2_RGS_CA) || defined(CONFIG_MACH_MSM8926_E2_VTR_CA)
#define UI_BL_OFF		0
#define UI_0_BL			10
#define UI_20_BL		60
#define UI_40_BL		110
#define UI_60_BL		157
#define UI_80_BL		207
#define UI_MAX_BL		255

#define LGE_BL_OFF		0
#define LGE_0_BL		100
#define LGE_20_BL		230
#define LGE_40_BL		630
#define LGE_60_BL		1280
#define LGE_80_BL		2480
#define LGE_MAX_BL		4095
#elif defined(CONFIG_MACH_MSM8926_AKA_KR)
#define UI_BL_OFF		0
#define UI_0_BL			10
#define UI_20_BL		61
#define UI_40_BL		109
#define UI_60_BL		159
#define UI_80_BL		208
#define UI_MAX_BL		255

#define LGE_BL_OFF		0
#define LGE_0_BL		180
#define LGE_20_BL		280
#define LGE_40_BL		690
#define LGE_60_BL		1420
#define LGE_80_BL		2550
#define LGE_MAX_BL		4095
#endif

#define MAX_FBI_LIST 32
static struct fb_info *fbi_list[MAX_FBI_LIST];
static int fbi_list_index;

static u32 mdss_fb_pseudo_palette[16] = {
	0x00000000, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

#if defined(CONFIG_LGE_BROADCAST_TDMB)
extern struct mdp_csc_cfg dmb_csc_convert;
extern int pp_set_dmb_status(int flag);
#endif /*               */

#ifdef CONFIG_LGE_LCD_OFF_DIMMING
static bool fb_blank_called;
#endif

static struct msm_mdp_interface *mdp_instance;

static int mdss_fb_register(struct msm_fb_data_type *mfd);
static int mdss_fb_open(struct fb_info *info, int user);
static int mdss_fb_release(struct fb_info *info, int user);
static int mdss_fb_release_all(struct fb_info *info, bool release_all);
static int mdss_fb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info);
static int mdss_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info);
static int mdss_fb_set_par(struct fb_info *info);
static int mdss_fb_blank_sub(int blank_mode, struct fb_info *info,
			     int op_enable);
static int mdss_fb_suspend_sub(struct msm_fb_data_type *mfd);
static int mdss_fb_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg);
static int mdss_fb_mmap(struct fb_info *info, struct vm_area_struct *vma);
static void mdss_fb_release_fences(struct msm_fb_data_type *mfd);
static int __mdss_fb_sync_buf_done_callback(struct notifier_block *p,
		unsigned long val, void *data);

static int __mdss_fb_display_thread(void *data);
static int mdss_fb_pan_idle(struct msm_fb_data_type *mfd);
static int mdss_fb_send_panel_event(struct msm_fb_data_type *mfd,
					int event, void *arg);
void mdss_fb_no_update_notify_timer_cb(unsigned long data)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)data;
	if (!mfd) {
		pr_err("%s mfd NULL\n", __func__);
		return;
	}
	mfd->no_update.value = NOTIFY_TYPE_NO_UPDATE;
	complete(&mfd->no_update.comp);
}

static int mdss_fb_notify_update(struct msm_fb_data_type *mfd,
							unsigned long *argp)
{
	int ret;
	unsigned long notify = 0x0, to_user = 0x0;

	ret = copy_from_user(&notify, argp, sizeof(unsigned long));
	if (ret) {
		pr_err("%s:ioctl failed\n", __func__);
		return ret;
	}

	if (notify > NOTIFY_UPDATE_POWER_OFF)
		return -EINVAL;

	if (notify == NOTIFY_UPDATE_START) {
		INIT_COMPLETION(mfd->update.comp);
		ret = wait_for_completion_timeout(
						&mfd->update.comp, 4 * HZ);
		to_user = (unsigned int)mfd->update.value;
		if (mfd->update.type == NOTIFY_TYPE_SUSPEND) {
			to_user = (unsigned int)mfd->update.type;
			ret = 1;
		}
	} else if (notify == NOTIFY_UPDATE_STOP) {
		INIT_COMPLETION(mfd->no_update.comp);
		ret = wait_for_completion_timeout(
						&mfd->no_update.comp, 4 * HZ);
		to_user = (unsigned int)mfd->no_update.value;
	} else {
		if (mfd->panel_power_on) {
			INIT_COMPLETION(mfd->power_off_comp);
			ret = wait_for_completion_timeout(
						&mfd->power_off_comp, 1 * HZ);
		}
	}

	if (ret == 0)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = copy_to_user(argp, &to_user, sizeof(unsigned long));
	return ret;
}

#if defined (CONFIG_LGE_MIPI_LGD_LG4591_VIDEO_HD_PANEL)
static int mdss_fb_splash_thread(void *data)
{
	struct msm_fb_data_type *mfd = data;
	int ret = -EINVAL;
	struct fb_info *fbi = NULL;
	int ov_index[2];

	if (!mfd || !mfd->fbi || !mfd->mdp.splash_fnc) {
		pr_err("Invalid input parameter\n");
		goto end;
	}

	fbi = mfd->fbi;

	ret = mdss_fb_open(fbi, current->tgid);
	if (ret) {
		pr_err("fb_open failed\n");
		goto end;
	}

	mfd->bl_updated = true;
	mdss_fb_set_backlight(mfd, mfd->panel_info->bl_max >> 1);

	ret = mfd->mdp.splash_fnc(mfd, ov_index, MDP_CREATE_SPLASH_OV);
	if (ret) {
		pr_err("Splash image failed\n");
		goto splash_err;
	}

	do {
		schedule_timeout_interruptible(SPLASH_THREAD_WAIT_TIMEOUT * HZ);
	} while (!kthread_should_stop());

	mfd->mdp.splash_fnc(mfd, ov_index, MDP_REMOVE_SPLASH_OV);

splash_err:
	mdss_fb_release(fbi, current->tgid);
end:
	return ret;
}
#endif

static int lcd_backlight_registered;

#if defined(CONFIG_FB_MSM_MIPI_TIANMA_VIDEO_QHD_PT_PANEL) || defined(CONFIG_MACH_MSM8926_X5_VZW)
static int cal_value;
static const char mapped_value[256] = {
	0,  4,  4,  4,  4,  4,  4,  4,  4,  4, //9
	4,  4,  4,  4,  4,  4,  4,  4,  4,  4, //19
	4,  4,  4,  4,  4,  4,  4,  5,  5,  5, //29
	5,  5,  5,  5,  5,  5,  5,  6,  6,  6, //39
	6,  7,  7,  7,  7,  7,  8,  8,  8,  8, //49
	8,  9,  9,  9,  9,  9,  10, 10, 10, 11, //59
	11, 12, 12, 12, 12, 12, 13, 13, 14, 14, //69
	14, 15, 15, 16, 16, 16, 17, 17, 18, 18, //79
	18, 19, 19, 20, 21, 21, 22, 22, 23, 23, //89
	24, 25, 25, 26, 26, 27, 27, 28, 29, 29, //99
	30, 30, 30, 31, 31, 32, 32, 32, 34, 34, //109
	36, 36, 38, 38, 38, 41, 41, 42, 43, 44, //119
	45, 45, 46, 47, 48, 49, 49, 50, 51, 52, //129
	53, 53, 55, 55, 55, 56, 57, 59, 59, 60, //139
	61, 62, 64, 64, 65, 66, 67, 69, 69, 69, //149
	71, 71, 71, 71, 75, 75, 75, 78, 78, 82, //159
	83, 84, 85, 86, 88, 89, 90, 91, 92, 94, //169
	95, 96, 97, 98,100,101,102,104,105,107, //179
	107,107,107,110,110,113,113,116,116,116, //189
	120,120,123,123,127,127,127,130,130,130, //199
	134,134,138,138,142,142,146,146,146,150, //209
	150,150,154,154,154,158,158,162,162,162, //219
	166,166,171,171,175,175,175,179,179,184, //229
	184,188,188,193,193,193,198,198,202,202, //239
	202,207,207,207,214,214,217,217,222,222, //249
	222,228,228,233,233,239 //255
};
#elif defined(CONFIG_LGE_MIPI_TOVIS_VIDEO_540P_PANEL)
static int cal_value;
static const char mapped_value[256] = {
	0,  3,  3,  3,  3,  4,  4,  4,  4,  5,   //9
	5,  5,  5,  5,  5,  5,  5,  5,  5,  5,   //19
	5,  5,  5,  5,  5,  5,  5,  6,  6,  6,   //29
	6,  6,  6,  6,  6,  6,  6,  6,  6,  7,   //39
	7,  7,  7,  7,  7,  7,  7,  8,  8,  8,   //49
	9,  9,  9,  9,  9,  10, 10, 10, 10, 10,  //59
	11, 11, 11, 11, 12, 12, 12, 12, 13, 13,  //69
	13, 14, 14, 14, 15, 15, 15, 16, 16, 16,  //79
	17, 17, 17, 18, 18, 19, 20, 20, 21, 21,  //89
	21, 22, 22, 23, 23, 23, 24, 24, 24, 25,  //99
	25, 25, 27, 27, 28, 28, 29, 29, 31, 31,  //109
	31, 32, 32, 32, 33, 33, 35, 35, 37, 37,  //119
	37, 38, 38, 40, 40, 42, 42, 42, 42, 43,  //129
	43, 45, 47, 47, 49, 49, 51, 51, 51, 53,  //139
	53, 55, 55, 55, 57, 57, 57, 59, 59, 59,  //149
	61, 61, 64, 64, 66, 66, 66, 68, 68, 71,  //159
	71, 71, 73, 73, 73, 76, 76, 76, 78, 78,  //169
	81, 81, 84, 84, 86, 86, 86, 86, 89, 89,  //179
	92, 92, 95, 95, 95, 98, 98, 98,101,104,  //189
	104,104,107,107,110,110,112,114,114,114, //199
	117,117,120,120,124,124,124,128,128,132, //209
	132,132,135,135,135,139,143,143,145,147, //219
	147,147,151,151,159,159,159,159,161,163, //229
	163,163,167,172,172,174,176,176,180,180, //239
	185,185,185,187,189,189,189,194,198,200, //249
	201,203,205,208,210,213                  //255
};
#endif

static void mdss_fb_set_bl_brightness(struct led_classdev *led_cdev,
				      enum led_brightness value)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(led_cdev->dev->parent);
	int bl_lvl;

#ifdef CONFIG_LGE_LCD_OFF_DIMMING
	if(lge_get_bootreason() == 0x77665560 && !fb_blank_called){
		pr_info("%s : lcd off mode! Will not turn on backlight.\n",__func__);
		return;
	}
#endif

	if (value > mfd->panel_info->brightness_max)
		value = mfd->panel_info->brightness_max;

	/* This maps android backlight level 0 to 255 into
	   driver backlight level 0 to bl_max with rounding */
#if defined(CONFIG_LGE_MIPI_TOVIS_VIDEO_540P_PANEL) || defined(CONFIG_FB_MSM_MIPI_TIANMA_VIDEO_QHD_PT_PANEL)
	cal_value = mapped_value[value];
	MDSS_BRIGHT_TO_BL(bl_lvl, cal_value, mfd->panel_info->bl_max,
			MDSS_MAX_BL_BRIGHTNESS);
	pr_info("value=%d, cal_value=%d\n", value, cal_value);

#elif defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_WVGA_PT_INCELL_PANEL)
#if defined(CONFIG_MACH_MSM8X10_L70P)
	MDSS_BRIGHT_TO_BL(bl_lvl, value, mfd->panel_info->bl_max,
                                mfd->panel_info->brightness_max);
	pr_info("value=%d, bl_lvl=%d\n", value, bl_lvl);
#elif defined(CONFIG_MACH_MSM8926_E2_SPR_US) || defined(CONFIG_MACH_MSM8926_E2_MPCS_US) || defined(CONFIG_MACH_MSM8926_E2_VZW) || defined(CONFIG_MACH_MSM8926_E2_BELL_CA) || defined(CONFIG_MACH_MSM8926_E2_RGS_CA) || defined(CONFIG_MACH_MSM8926_E2_VTR_CA)
	if(value >= UI_BL_OFF && value <= UI_0_BL)
		bl_lvl = (value - UI_BL_OFF) * (LGE_0_BL - LGE_BL_OFF) / (UI_0_BL - UI_BL_OFF) + LGE_BL_OFF;
	else if(value >= UI_0_BL && value <= UI_20_BL)
		bl_lvl = (value - UI_0_BL) * (LGE_20_BL - LGE_0_BL) / (UI_20_BL - UI_0_BL) + LGE_0_BL;
	else if(value >UI_20_BL && value <= UI_40_BL)
		bl_lvl = (value - UI_20_BL) * (LGE_40_BL - LGE_20_BL) / (UI_40_BL - UI_20_BL) + LGE_20_BL;
	else if(value >UI_40_BL && value <= UI_60_BL)
		bl_lvl = (value - UI_40_BL) * (LGE_60_BL - LGE_40_BL) / (UI_60_BL - UI_40_BL) + LGE_40_BL;
	else if(value >UI_60_BL && value <= UI_80_BL)
		bl_lvl = (value - UI_60_BL) * (LGE_80_BL - LGE_60_BL) / (UI_80_BL - UI_60_BL) + LGE_60_BL;
	else if(value >UI_80_BL && value <= UI_MAX_BL)
		bl_lvl = (value - UI_80_BL) * (LGE_MAX_BL - LGE_80_BL) / (UI_MAX_BL - UI_80_BL) + LGE_80_BL;

	pr_info("value=%d, bl_lvl=%d\n", value, bl_lvl);
#endif

#else

#if defined(CONFIG_MACH_MSM8926_X3_TRF_US) || defined(CONFIG_MACH_MSM8926_X3N_KR) || defined(CONFIG_MACH_MSM8926_F70N_KR) || defined(CONFIG_MACH_MSM8926_X3N_OPEN_EU) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_AME) || defined(CONFIG_MACH_MSM8926_F70_GLOBAL_COM) || \
	defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CN) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGNM_KDDI_JP) || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM) ||  defined(CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_AKA_KR) || defined(CONFIG_MACH_MSM8926_E2JPS_JP)
	if(value >= UI_BL_OFF && value <= UI_0_BL)
		bl_lvl = (value - UI_BL_OFF) * (LGE_0_BL - LGE_BL_OFF) / (UI_0_BL - UI_BL_OFF) + LGE_BL_OFF;
	else if(value >= UI_0_BL && value <= UI_20_BL)
		bl_lvl = (value - UI_0_BL) * (LGE_20_BL - LGE_0_BL) / (UI_20_BL - UI_0_BL) + LGE_0_BL;
	else if(value >UI_20_BL && value <= UI_40_BL)
		bl_lvl = (value - UI_20_BL) * (LGE_40_BL - LGE_20_BL) / (UI_40_BL - UI_20_BL) + LGE_20_BL;
	else if(value >UI_40_BL && value <= UI_60_BL)
		bl_lvl = (value - UI_40_BL) * (LGE_60_BL - LGE_40_BL) / (UI_60_BL - UI_40_BL) + LGE_40_BL;
	else if(value >UI_60_BL && value <= UI_80_BL)
		bl_lvl = (value - UI_60_BL) * (LGE_80_BL - LGE_60_BL) / (UI_80_BL - UI_60_BL) + LGE_60_BL;
	else if(value >UI_80_BL && value <= UI_MAX_BL)
		bl_lvl = (value - UI_80_BL) * (LGE_MAX_BL - LGE_80_BL) / (UI_MAX_BL - UI_80_BL) + LGE_80_BL;

	pr_info("value=%d, bl_lvl=%d\n", value, bl_lvl);
#else
	MDSS_BRIGHT_TO_BL(bl_lvl, value, mfd->panel_info->bl_max,
                                mfd->panel_info->brightness_max);
#endif

#endif

	if (!bl_lvl && value)
		bl_lvl = 1;

	if (!IS_CALIB_MODE_BL(mfd) && (!mfd->ext_bl_ctrl || !value ||
							!mfd->bl_level)) {
		mutex_lock(&mfd->bl_lock);
		mdss_fb_set_backlight(mfd, bl_lvl);
		mutex_unlock(&mfd->bl_lock);
	}
}

static struct led_classdev backlight_led = {
	.name           = "lcd-backlight",
	.brightness     = MDSS_MAX_BL_BRIGHTNESS,
	.brightness_set = mdss_fb_set_bl_brightness,
};

static ssize_t mdss_fb_get_type(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;

	switch (mfd->panel.type) {
	case NO_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "no panel\n");
		break;
	case HDMI_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "hdmi panel\n");
		break;
	case LVDS_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "lvds panel\n");
		break;
	case DTV_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "dtv panel\n");
		break;
	case MIPI_VIDEO_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "mipi dsi video panel\n");
		break;
	case MIPI_CMD_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "mipi dsi cmd panel\n");
		break;
	case WRITEBACK_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "writeback panel\n");
		break;
	case EDP_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "edp panel\n");
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "unknown panel\n");
		break;
	}
	/* Notify listeners */
	sysfs_notify(&mfd->fbi->dev->kobj, NULL, "show_blank_event");

	return ret;
}

static void mdss_fb_parse_dt_split(struct msm_fb_data_type *mfd)
{
	u32 data[2];
	struct platform_device *pdev = mfd->pdev;

#if defined (CONFIG_LGE_MIPI_LGD_LG4591_VIDEO_HD_PANEL)
	pr_debug("%s: boot_mode : %d, laf_mode : %d\n", __func__, lge_get_boot_mode(), lge_get_laf_mode());
	if (!lge_get_laf_mode()
		&& !lge_get_fota_mode()
	#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
		&& lge_get_boot_mode() != LGE_BOOT_MODE_CHARGERLOGO)
	#endif
	{
		mfd->splash_logo_enabled = of_property_read_bool(pdev->dev.of_node,
					"qcom,mdss-fb-splash-logo-enabled");
	}
#endif

	if (of_property_read_u32_array(pdev->dev.of_node, "qcom,mdss-fb-split",
				       data, 2))
		return;
	if (data[0] && data[1] &&
	    (mfd->panel_info->xres == (data[0] + data[1]))) {
		mfd->split_fb_left = data[0];
		mfd->split_fb_right = data[1];
		pr_info("split framebuffer left=%d right=%d\n",
			mfd->split_fb_left, mfd->split_fb_right);
	} else {
		mfd->split_fb_left = 0;
		mfd->split_fb_right = 0;
	}
}

static ssize_t mdss_fb_get_split(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	ret = snprintf(buf, PAGE_SIZE, "%d %d\n",
		       mfd->split_fb_left, mfd->split_fb_right);
	return ret;
}

static ssize_t mdss_mdp_show_blank_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	int ret;

	pr_debug("fb%d panel_power_on = %d\n", mfd->index, mfd->panel_power_on);
	ret = scnprintf(buf, PAGE_SIZE, "panel_power_on = %d\n",
						mfd->panel_power_on);

	return ret;
}

#ifdef CONFIG_LGE_LCD_TUNING
//porch
static ssize_t mdss_get_porch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	ret = sprintf(buf, "vfp:%d vbp:%d vpw:%d hfp:%d hbp:%d hpw:%d\n",
						panel_info->lcdc.v_front_porch,
						panel_info->lcdc.v_back_porch,
						panel_info->lcdc.v_pulse_width,
						panel_info->lcdc.h_front_porch,
						panel_info->lcdc.h_back_porch,
						panel_info->lcdc.h_pulse_width);

	return ret;
}

static ssize_t mdss_set_porch_store(struct device *dev, struct device_attribute *addr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	int c_v_front_porch;
	int c_v_back_porch;
	int c_v_pulse_width;
	int c_h_front_porch;
	int c_h_back_porch;
	int c_h_pulse_width;

	sscanf(buf, "%d %d %d %d %d %d",
				&c_v_front_porch, &c_v_back_porch,&c_v_pulse_width,
				&c_h_front_porch, &c_h_back_porch, &c_h_pulse_width);
				panel_info->lcdc.v_front_porch = c_v_front_porch;
				panel_info->lcdc.v_back_porch = c_v_back_porch;
				panel_info->lcdc.v_pulse_width = c_v_pulse_width;
				panel_info->lcdc.h_front_porch = c_h_front_porch;
				panel_info->lcdc.h_back_porch = c_h_back_porch;
				panel_info->lcdc.h_pulse_width = c_h_pulse_width;

	return count;
}

//timing
static ssize_t mdss_get_timing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	ret = sprintf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x\n",
						panel_info->mipi.dsi_phy_db.timing[0],
						panel_info->mipi.dsi_phy_db.timing[1],
						panel_info->mipi.dsi_phy_db.timing[2],
						panel_info->mipi.dsi_phy_db.timing[3],
						panel_info->mipi.dsi_phy_db.timing[4],
						panel_info->mipi.dsi_phy_db.timing[5],
						panel_info->mipi.dsi_phy_db.timing[6],
						panel_info->mipi.dsi_phy_db.timing[7],
						panel_info->mipi.dsi_phy_db.timing[8],
						panel_info->mipi.dsi_phy_db.timing[9],
						panel_info->mipi.dsi_phy_db.timing[10],
						panel_info->mipi.dsi_phy_db.timing[11] );

	return ret;
}

static ssize_t mdss_set_timing_store(struct device *dev, struct device_attribute *addr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	int timing_number[12];
	int i;

	sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x",
				&timing_number[0], &timing_number[1],
				&timing_number[2], &timing_number[3],
				&timing_number[4], &timing_number[5],
				&timing_number[6], &timing_number[7],
				&timing_number[8], &timing_number[9],
				&timing_number[10], &timing_number[11]);

	for(i=0; i<12; i++)
	{
		panel_info->mipi.dsi_phy_db.timing[i] = timing_number[i];
	}

	return count;
}

//tclk 	507
static ssize_t mdss_get_tclk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	ret = sprintf(buf, "post:%x pre:%x\n",
						panel_info->mipi.t_clk_post,
						panel_info->mipi.t_clk_pre
);

	return ret;
}

static ssize_t mdss_set_tclk_store(struct device *dev, struct device_attribute *addr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	int c_t_clk_post;
	int c_t_clk_pre;
	sscanf(buf, "%x %x", &c_t_clk_post, &c_t_clk_pre);
	panel_info->mipi.t_clk_post = c_t_clk_post;
	panel_info->mipi.t_clk_pre = c_t_clk_pre;

	return count;
}

static DEVICE_ATTR(porch, S_IRUSR | S_IWUSR, mdss_get_porch_show, mdss_set_porch_store);
static DEVICE_ATTR(timing_value, S_IRUSR | S_IWUSR, mdss_get_timing_show, mdss_set_timing_store);
static DEVICE_ATTR(tclk, S_IRUSR | S_IWUSR, mdss_get_tclk_show, mdss_set_tclk_store);
#endif

static DEVICE_ATTR(msm_fb_type, S_IRUGO, mdss_fb_get_type, NULL);
static DEVICE_ATTR(msm_fb_split, S_IRUGO, mdss_fb_get_split, NULL);
static DEVICE_ATTR(show_blank_event, S_IRUGO, mdss_mdp_show_blank_event, NULL);

static struct attribute *mdss_fb_attrs[] = {
	&dev_attr_msm_fb_type.attr,
	&dev_attr_msm_fb_split.attr,
	&dev_attr_show_blank_event.attr,
#ifdef CONFIG_LGE_LCD_TUNING
	&dev_attr_porch.attr,
	&dev_attr_timing_value.attr,
	&dev_attr_tclk.attr,
#endif
	NULL,
};

static struct attribute_group mdss_fb_attr_group = {
	.attrs = mdss_fb_attrs,
};

static int mdss_fb_create_sysfs(struct msm_fb_data_type *mfd)
{
	int rc;

	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &mdss_fb_attr_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);
	return rc;
}

static void mdss_fb_remove_sysfs(struct msm_fb_data_type *mfd)
{
	sysfs_remove_group(&mfd->fbi->dev->kobj, &mdss_fb_attr_group);
}
#if defined(CONFIG_MACH_MSM8926_AKA_KR)
int is_shutdown;
#endif
static void mdss_fb_shutdown(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);

	mfd->shutdown_pending = true;
#if defined(CONFIG_MACH_MSM8926_AKA_KR)
	is_shutdown = 1;
#endif
	lock_fb_info(mfd->fbi);
	mdss_fb_release_all(mfd->fbi, true);
	unlock_fb_info(mfd->fbi);
}

#if defined (CONFIG_MACH_MSM8926_VFP_KR)
int is_fboot;
#endif


static int mdss_fb_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = NULL;
	struct mdss_panel_data *pdata;
	struct fb_info *fbi;
	int rc;

	if (fbi_list_index >= MAX_FBI_LIST)
		return -ENOMEM;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata)
		return -EPROBE_DEFER;

	/*
	 * alloc framebuffer info + par data
	 */
	fbi = framebuffer_alloc(sizeof(struct msm_fb_data_type), NULL);
	if (fbi == NULL) {
		pr_err("can't allocate framebuffer info data!\n");
		return -ENOMEM;
	}

	mfd = (struct msm_fb_data_type *)fbi->par;
	mfd->key = MFD_KEY;
	mfd->fbi = fbi;
	mfd->panel_info = &pdata->panel_info;
	mfd->panel.type = pdata->panel_info.type;
	mfd->panel.id = mfd->index;
	mfd->fb_page = MDSS_FB_NUM;
	mfd->index = fbi_list_index;
	mfd->mdp_fb_page_protection = MDP_FB_PAGE_PROTECTION_WRITECOMBINE;

	mfd->ext_ad_ctrl = -1;
	mfd->bl_level = 0;
	mfd->bl_scale = 1024;
	mfd->bl_min_lvl = 30;
#if defined(CONFIG_FB_MSM_MIPI_TIANMA_CMD_HVGA_PT) //&& !defined (CONFIG_MACH_MSM8926_VFP_KR)
	mfd->fb_imgType = MDP_RGB_565;
#else
	mfd->fb_imgType = MDP_RGBA_8888;
#endif

	mfd->pdev = pdev;
	if (pdata->next)
		mfd->split_display = true;
	mfd->mdp = *mdp_instance;
	INIT_LIST_HEAD(&mfd->proc_list);

	mutex_init(&mfd->lock);
	mutex_init(&mfd->bl_lock);

	fbi_list[fbi_list_index++] = fbi;

	platform_set_drvdata(pdev, mfd);

	rc = mdss_fb_register(mfd);
	if (rc)
		return rc;

	if (mfd->mdp.init_fnc) {
		rc = mfd->mdp.init_fnc(mfd);
		if (rc) {
			pr_err("init_fnc failed\n");
			return rc;
		}
	}

	rc = pm_runtime_set_active(mfd->fbi->dev);
	if (rc < 0)
		pr_err("pm_runtime: fail to set active.\n");
	pm_runtime_enable(mfd->fbi->dev);

	/* android supports only one lcd-backlight/lcd for now */
	if (!lcd_backlight_registered) {

		backlight_led.brightness = mfd->panel_info->brightness_max;
		backlight_led.max_brightness = mfd->panel_info->brightness_max;
		if (led_classdev_register(&pdev->dev, &backlight_led))
			pr_err("led_classdev_register failed\n");
		else
			lcd_backlight_registered = 1;
	}

	mdss_fb_create_sysfs(mfd);
	mdss_fb_send_panel_event(mfd, MDSS_EVENT_FB_REGISTERED, fbi);

	mfd->mdp_sync_pt_data.fence_name = "mdp-fence";
	if (mfd->mdp_sync_pt_data.timeline == NULL) {
		char timeline_name[16];
		snprintf(timeline_name, sizeof(timeline_name),
			"mdss_fb_%d", mfd->index);
		 mfd->mdp_sync_pt_data.timeline =
				sw_sync_timeline_create(timeline_name);
		if (mfd->mdp_sync_pt_data.timeline == NULL) {
			pr_err("%s: cannot create time line", __func__);
			return -ENOMEM;
		}
		mfd->mdp_sync_pt_data.notifier.notifier_call =
			__mdss_fb_sync_buf_done_callback;
	}
#ifdef CONFIG_ARCH_MSM8610
	if (mfd->panel.type == WRITEBACK_PANEL)
#else
	if ((mfd->panel.type == WRITEBACK_PANEL) ||
			(mfd->panel.type == MIPI_CMD_PANEL))
#endif
		mfd->mdp_sync_pt_data.threshold = 1;
	else
		mfd->mdp_sync_pt_data.threshold = 2;

#if defined (CONFIG_LGE_MIPI_LGD_LG4591_VIDEO_HD_PANEL)
	if (mfd->splash_logo_enabled) {
		mfd->splash_thread = kthread_run(mdss_fb_splash_thread, mfd,
						"mdss_fb_splash");
	if (IS_ERR(mfd->splash_thread)) {
		pr_err("unable to start splash thread %d\n",
				mfd->index);
		mfd->splash_thread = NULL;
	}
}
#endif
#if defined (CONFIG_MACH_MSM8926_VFP_KR)
	pr_info("%s: boot_mode : %d, laf_mode : %d\n", __func__, lge_get_boot_mode(), lge_get_laf_mode());
	if ((lge_get_boot_mode() == LGE_BOOT_MODE_QEM_130K))
			is_fboot = 1;
#endif

	return rc;
}

static int mdss_fb_remove(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	mdss_fb_remove_sysfs(mfd);

	pm_runtime_disable(mfd->fbi->dev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (mdss_fb_suspend_sub(mfd))
		pr_err("msm_fb_remove: can't stop the device %d\n",
			    mfd->index);

	/* remove /dev/fb* */
	unregister_framebuffer(mfd->fbi);

	if (lcd_backlight_registered) {
		lcd_backlight_registered = 0;
		led_classdev_unregister(&backlight_led);
	}

	return 0;
}

static int mdss_fb_send_panel_event(struct msm_fb_data_type *mfd,
					int event, void *arg)
{
	struct mdss_panel_data *pdata;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("no panel connected\n");
		return -ENODEV;
	}

	pr_debug("sending event=%d for fb%d\n", event, mfd->index);

	if (pdata->event_handler)
		return pdata->event_handler(pdata, event, arg);

	return 0;
}

static int mdss_fb_suspend_sub(struct msm_fb_data_type *mfd)
{
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	pr_debug("mdss_fb suspend index=%d\n", mfd->index);

	mdss_fb_pan_idle(mfd);
	ret = mdss_fb_send_panel_event(mfd, MDSS_EVENT_SUSPEND, NULL);
	if (ret) {
		pr_warn("unable to suspend fb%d (%d)\n", mfd->index, ret);
		return ret;
	}

	mfd->suspend.op_enable = mfd->op_enable;
	mfd->suspend.panel_power_on = mfd->panel_power_on;

	if (mfd->op_enable) {
		ret = mdss_fb_blank_sub(FB_BLANK_POWERDOWN, mfd->fbi,
				mfd->suspend.op_enable);
		if (ret) {
			pr_warn("can't turn off display!\n");
			return ret;
		}
		mfd->op_enable = false;
		fb_set_suspend(mfd->fbi, FBINFO_STATE_SUSPENDED);
	}

	return 0;
}

static int mdss_fb_resume_sub(struct msm_fb_data_type *mfd)
{
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	INIT_COMPLETION(mfd->power_set_comp);
	mfd->is_power_setting = true;
	pr_debug("mdss_fb resume index=%d\n", mfd->index);

	mdss_fb_pan_idle(mfd);
	ret = mdss_fb_send_panel_event(mfd, MDSS_EVENT_RESUME, NULL);
	if (ret) {
		pr_warn("unable to resume fb%d (%d)\n", mfd->index, ret);
		return ret;
	}

	/* resume state var recover */
	mfd->op_enable = mfd->suspend.op_enable;

	if (mfd->suspend.panel_power_on) {
		ret = mdss_fb_blank_sub(FB_BLANK_UNBLANK, mfd->fbi,
					mfd->op_enable);
		if (ret)
			pr_warn("can't turn on display!\n");
		else
			fb_set_suspend(mfd->fbi, FBINFO_STATE_RUNNING);
	}
	mfd->is_power_setting = false;
	complete_all(&mfd->power_set_comp);

	return ret;
}

#if defined(CONFIG_PM) && !defined(CONFIG_PM_SLEEP)
static int mdss_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;

	dev_dbg(&pdev->dev, "display suspend\n");

	return mdss_fb_suspend_sub(mfd);
}

static int mdss_fb_resume(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;

	dev_dbg(&pdev->dev, "display resume\n");

	return mdss_fb_resume_sub(mfd);
}
#else
#define mdss_fb_suspend NULL
#define mdss_fb_resume NULL
#endif

#ifdef CONFIG_PM_SLEEP
static int mdss_fb_pm_suspend(struct device *dev)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev);

	if (!mfd)
		return -ENODEV;

	dev_dbg(dev, "display pm suspend\n");

	return mdss_fb_suspend_sub(mfd);
}

static int mdss_fb_pm_resume(struct device *dev)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev);
	if (!mfd)
		return -ENODEV;

	dev_dbg(dev, "display pm resume\n");

	return mdss_fb_resume_sub(mfd);
}
#endif

static const struct dev_pm_ops mdss_fb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mdss_fb_pm_suspend, mdss_fb_pm_resume)
};

static const struct of_device_id mdss_fb_dt_match[] = {
	{ .compatible = "qcom,mdss-fb",},
	{}
};
EXPORT_COMPAT("qcom,mdss-fb");

static struct platform_driver mdss_fb_driver = {
	.probe = mdss_fb_probe,
	.remove = mdss_fb_remove,
	.suspend = mdss_fb_suspend,
	.resume = mdss_fb_resume,
	.shutdown = mdss_fb_shutdown,
	.driver = {
		.name = "mdss_fb",
		.of_match_table = mdss_fb_dt_match,
		.pm = &mdss_fb_pm_ops,
	},
};

static void mdss_fb_scale_bl(struct msm_fb_data_type *mfd, u32 *bl_lvl)
{
	u32 temp = *bl_lvl;

	pr_debug("input = %d, scale = %d", temp, mfd->bl_scale);
	if (temp >= mfd->bl_min_lvl) {
		if (temp > mfd->panel_info->bl_max) {
			pr_warn("%s: invalid bl level\n",
				__func__);
			temp = mfd->panel_info->bl_max;
		}
		if (mfd->bl_scale > 1024) {
			pr_warn("%s: invalid bl scale\n",
				__func__);
			mfd->bl_scale = 1024;
		}
		/*
		 * bl_scale is the numerator of
		 * scaling fraction (x/1024)
		 */
		temp = (temp * mfd->bl_scale) / 1024;

		/*if less than minimum level, use min level*/
		if (temp < mfd->bl_min_lvl)
			temp = mfd->bl_min_lvl;
	}
	pr_debug("output = %d", temp);

	(*bl_lvl) = temp;
}

/* must call this function from within mfd->bl_lock */
void mdss_fb_set_backlight(struct msm_fb_data_type *mfd, u32 bkl_lvl)
{
	struct mdss_panel_data *pdata;
	int (*update_ad_input)(struct msm_fb_data_type *mfd);
	u32 temp = bkl_lvl;

  #ifdef CONFIG_MACH_LGE
  if( force_set_bl_f || lge_get_boot_mode()== LGE_BOOT_MODE_QEM_130K
  	/*                                                     
                                                      */ ) {
    pdata = dev_get_platdata(&mfd->pdev->dev);
    if ((pdata) && (pdata->set_backlight)) {
      mdss_fb_scale_bl(mfd, &temp);
      pdata->set_backlight(pdata, temp);
      pr_info("regardless bl_updated, force to set bl. level=%d, laf_mode=%d\n",/*cont_sp=%d",*/
                temp, lge_get_laf_mode()/*, is_dsv_cont_splash_screening_f */);
    }
    return;
  }
  #endif

	if (((!mfd->panel_power_on && mfd->dcm_state != DCM_ENTER)
		|| !mfd->bl_updated) && !IS_CALIB_MODE_BL(mfd)) {
		mfd->unset_bl_level = bkl_lvl;
		return;
	} else {
		mfd->unset_bl_level = 0;
	}

	pdata = dev_get_platdata(&mfd->pdev->dev);

	if ((pdata) && (pdata->set_backlight)) {
#if !defined(CONFIG_MACH_MSM8X10_W5) && !defined(CONFIG_MACH_MSM8X10_L70P) && !defined(CONFIG_MACH_MSM8X10_Y30)
		if (!IS_CALIB_MODE_BL(mfd))
			mdss_fb_scale_bl(mfd, &temp);
		/*
		 * Even though backlight has been scaled, want to show that
		 * backlight has been set to bkl_lvl to those that read from
		 * sysfs node. Thus, need to set bl_level even if it appears
		 * the backlight has already been set to the level it is at,
		 * as well as setting bl_level to bkl_lvl even though the
		 * backlight has been set to the scaled value.
		 */
		if (mfd->bl_level_old == temp) {
			mfd->bl_level = bkl_lvl;
			return;
		}
#endif
		pdata->set_backlight(pdata, temp);
		mfd->bl_level = bkl_lvl;
		mfd->bl_level_old = temp;

		if (mfd->mdp.update_ad_input) {
			update_ad_input = mfd->mdp.update_ad_input;
			mutex_unlock(&mfd->bl_lock);
			/* Will trigger ad_setup which will grab bl_lock */
			update_ad_input(mfd);
			mutex_lock(&mfd->bl_lock);
		}
	}
}

void mdss_fb_update_backlight(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;

	if (mfd->unset_bl_level && !mfd->bl_updated) {
		pdata = dev_get_platdata(&mfd->pdev->dev);
		if ((pdata) && (pdata->set_backlight)) {
			mutex_lock(&mfd->bl_lock);
                        mfd->bl_level = mfd->unset_bl_level;
			pr_info("backlight on.bl_level=%d \n",mfd->bl_level); /*lge_changed*/
#if defined(CONFIG_MACH_MSM8X10_Y30)
			mdelay(50); /* added delay to fix whitescreen issue during device resume */
#endif
			pdata->set_backlight(pdata, mfd->bl_level);
			mfd->bl_level_old = mfd->unset_bl_level;
			mutex_unlock(&mfd->bl_lock);
			mfd->bl_updated = 1;
		}
	}
}

static int mdss_fb_blank_sub(int blank_mode, struct fb_info *info,
			     int op_enable)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
#if defined(CONFIG_FB_MSM_MIPI_TIANMA_CMD_HVGA_PT) || defined(CONFIG_LGE_MIPI_DSI_LGD_LVDS_WXGA) || defined(CONFIG_FB_MSM_MIPI_LGD_LH500WX9_VIDEO_HD_PT_PANEL) || defined(CONFIG_MACH_MSM8X10_L70P) || defined(CONFIG_MACH_MSM8X10_Y30)
	struct mdss_panel_data *pdata = dev_get_platdata(&mfd->pdev->dev);
#endif
	int ret = 0;

	if (!op_enable)
		return -EPERM;

  pr_info("%s: blank_mode=%d\n", __func__, blank_mode);
	if (mfd->dcm_state == DCM_ENTER)
		return -EPERM;

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		if (!mfd->panel_power_on && mfd->mdp.on_fnc) {
			ret = mfd->mdp.on_fnc(mfd);
			if (ret == 0) {
				mfd->panel_power_on = true;
				mfd->panel_info->panel_dead = false;
			}
			mutex_lock(&mfd->update.lock);
			mfd->update.type = NOTIFY_TYPE_UPDATE;
			mutex_unlock(&mfd->update.lock);
#if defined(CONFIG_FB_MSM_MIPI_LGD_LH500WX9_VIDEO_HD_PT_PANEL)
			mdelay(50);
			mdelay(70);
#endif
		}
		break;

	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
	case FB_BLANK_POWERDOWN:
	default:
		if (mfd->panel_power_on && mfd->mdp.off_fnc) {
			int curr_pwr_state;

#if defined(CONFIG_FB_MSM_MIPI_TIANMA_CMD_HVGA_PT) || defined(CONFIG_LGE_MIPI_DSI_LGD_LVDS_WXGA) || defined(CONFIG_FB_MSM_MIPI_LGD_LH500WX9_VIDEO_HD_PT_PANEL) || defined(CONFIG_MACH_MSM8X10_L70P) || defined(CONFIG_MACH_MSM8X10_Y30)
			if (mfd->index==0) {
					mfd->bl_updated = 1;
					pdata->set_backlight(pdata, 0);
					mfd->unset_bl_level = mfd->bl_level_old;
			}
#endif

#ifdef CONFIG_LGE_LCD_OFF_DIMMING
			if (mfd->index==0) {
				pr_info("fb blank is called: lcd dimming mode ends\n");
				fb_blank_called = true;
			}
#endif

			mutex_lock(&mfd->update.lock);
			mfd->update.type = NOTIFY_TYPE_SUSPEND;
			mutex_unlock(&mfd->update.lock);
			del_timer(&mfd->no_update.timer);
			mfd->no_update.value = NOTIFY_TYPE_SUSPEND;
			complete(&mfd->no_update.comp);

			mfd->op_enable = false;
			curr_pwr_state = mfd->panel_power_on;
			mfd->panel_power_on = false;
			mfd->bl_updated = 0;

			ret = mfd->mdp.off_fnc(mfd);
			if (ret)
				mfd->panel_power_on = curr_pwr_state;
			else
				mdss_fb_release_fences(mfd);
			mfd->op_enable = true;
			complete(&mfd->power_off_comp);
		}
		break;
	}
	/* Notify listeners */
	sysfs_notify(&mfd->fbi->dev->kobj, NULL, "show_blank_event");

	return ret;
}

static int mdss_fb_blank(int blank_mode, struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	mdss_fb_pan_idle(mfd);
	if (mfd->op_enable == 0) {
		if (blank_mode == FB_BLANK_UNBLANK)
			mfd->suspend.panel_power_on = true;
		else
			mfd->suspend.panel_power_on = false;
		return 0;
	}
	return mdss_fb_blank_sub(blank_mode, info, mfd->op_enable);
}

/*
 * Custom Framebuffer mmap() function for MSM driver.
 * Differs from standard mmap() function by allowing for customized
 * page-protection.
 */
static int mdss_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	/* Get frame buffer memory range. */
	unsigned long start = info->fix.smem_start;
	u32 len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	int ret = 0;

	if (!start) {
		pr_warn("No framebuffer memory is allocated.\n");
		return -ENOMEM;
	}

	ret = mdss_fb_pan_idle(mfd);
	if (ret) {
		pr_err("Shutdown pending. Aborting operation\n");
		return ret;
	}

	/* Set VM flags. */
	start &= PAGE_MASK;
	if ((vma->vm_end <= vma->vm_start) ||
	    (off >= len) ||
	    ((vma->vm_end - vma->vm_start) > (len - off)))
		return -EINVAL;
	off += start;
	if (off < start)
		return -EINVAL;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO | VM_RESERVED;

	/* Set VM page protection */
	if (mfd->mdp_fb_page_protection == MDP_FB_PAGE_PROTECTION_WRITECOMBINE)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
		 MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE)
		vma->vm_page_prot = pgprot_writethroughcache(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
		 MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE)
		vma->vm_page_prot = pgprot_writebackcache(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
		 MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE)
		vma->vm_page_prot = pgprot_writebackwacache(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap the frame buffer I/O range */
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static struct fb_ops mdss_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = mdss_fb_open,
	.fb_release = mdss_fb_release,
	.fb_check_var = mdss_fb_check_var,	/* vinfo check */
	.fb_set_par = mdss_fb_set_par,	/* set the video mode */
	.fb_blank = mdss_fb_blank,	/* blank display */
	.fb_pan_display = mdss_fb_pan_display,	/* pan display */
	.fb_ioctl = mdss_fb_ioctl,	/* perform fb specific ioctl */
	.fb_mmap = mdss_fb_mmap,
};

static int mdss_fb_alloc_fbmem_iommu(struct msm_fb_data_type *mfd, int dom)
{
	void *virt = NULL;
	unsigned long phys = 0;
	size_t size = 0;
	struct platform_device *pdev = mfd->pdev;

	if (!pdev || !pdev->dev.of_node) {
		pr_err("Invalid device node\n");
		return -ENODEV;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				 "qcom,memory-reservation-size",
				 &size) || !size) {
		mfd->fbi->screen_base = NULL;
		mfd->fbi->fix.smem_start = 0;
		mfd->fbi->fix.smem_len = 0;
		return 0;
	}

	pr_info("%s frame buffer reserve_size=0x%x\n", __func__, size);

	if (size < PAGE_ALIGN(mfd->fbi->fix.line_length *
			      mfd->fbi->var.yres_virtual))
		pr_warn("reserve size is smaller than framebuffer size\n");

	virt = allocate_contiguous_memory(size, MEMTYPE_EBI1, SZ_1M, 0);
	if (!virt) {
		pr_err("unable to alloc fbmem size=%u\n", size);
		return -ENOMEM;
	}

	phys = memory_pool_node_paddr(virt);

	msm_iommu_map_contig_buffer(phys, dom, 0, size, SZ_4K, 0,
					    &mfd->iova);
	pr_info("allocating %u bytes at %p (%lx phys) for fb %d\n",
		 size, virt, phys, mfd->index);

	mfd->fbi->screen_base = virt;
	mfd->fbi->fix.smem_start = phys;
	mfd->fbi->fix.smem_len = size;
#ifdef CONFIG_MACH_LGE
	msm_fb_phys_addr_backup = phys;
	memset(virt,0,size);
#endif

	return 0;
}

static int mdss_fb_alloc_fbmem(struct msm_fb_data_type *mfd)
{

	if (mfd->mdp.fb_mem_alloc_fnc)
		return mfd->mdp.fb_mem_alloc_fnc(mfd);
	else if (mfd->mdp.fb_mem_get_iommu_domain) {
		int dom = mfd->mdp.fb_mem_get_iommu_domain();
		if (dom >= 0)
			return mdss_fb_alloc_fbmem_iommu(mfd, dom);
		else
			return -ENOMEM;
	} else {
		pr_err("no fb memory allocator function defined\n");
		return -ENOMEM;
	}
}

static int mdss_fb_register(struct msm_fb_data_type *mfd)
{
	int ret = -ENODEV;
	int bpp;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	struct fb_info *fbi = mfd->fbi;
	struct fb_fix_screeninfo *fix;
	struct fb_var_screeninfo *var;
	int *id;

	/*
	 * fb info initialization
	 */
	fix = &fbi->fix;
	var = &fbi->var;

	fix->type_aux = 0;	/* if type == FB_TYPE_INTERLEAVED_PLANES */
	fix->visual = FB_VISUAL_TRUECOLOR;	/* True Color */
	fix->ywrapstep = 0;	/* No support */
	fix->mmio_start = 0;	/* No MMIO Address */
	fix->mmio_len = 0;	/* No MMIO Address */
	fix->accel = FB_ACCEL_NONE;/* FB_ACCEL_MSM needes to be added in fb.h */

	var->xoffset = 0;	/* Offset from virtual to visible */
	var->yoffset = 0;	/* resolution */
	var->grayscale = 0;	/* No graylevels */
	var->nonstd = 0;	/* standard pixel format */
	var->activate = FB_ACTIVATE_VBL;	/* activate it at vsync */
	var->height = -1;	/* height of picture in mm */
	var->width = -1;	/* width of picture in mm */
	var->accel_flags = 0;	/* acceleration flags */
	var->sync = 0;	/* see FB_SYNC_* */
	var->rotate = 0;	/* angle we rotate counter clockwise */
	mfd->op_enable = false;

	switch (mfd->fb_imgType) {
	case MDP_RGB_565:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 5;
		var->red.offset = 11;
		var->blue.length = 5;
		var->green.length = 6;
		var->red.length = 5;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 2;
		break;

	case MDP_RGB_888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 8;
		var->red.offset = 16;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 3;
		break;

	case MDP_ARGB_8888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 8;
		var->red.offset = 16;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 24;
		var->transp.length = 8;
		bpp = 4;
		break;

	case MDP_RGBA_8888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 8;
		var->green.offset = 16;
		var->red.offset = 24;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 8;
		bpp = 4;
		break;

	case MDP_YCRYCB_H2V1:
		fix->type = FB_TYPE_INTERLEAVED_PLANES;
		fix->xpanstep = 2;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;

		/* how about R/G/B offset? */
		var->blue.offset = 0;
		var->green.offset = 5;
		var->red.offset = 11;
		var->blue.length = 5;
		var->green.length = 6;
		var->red.length = 5;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 2;
		break;

	default:
		pr_err("msm_fb_init: fb %d unkown image type!\n",
			    mfd->index);
		return ret;
	}

	var->xres = panel_info->xres;
	if (mfd->split_display)
		var->xres *= 2;

	fix->type = panel_info->is_3d_panel;
	if (mfd->mdp.fb_stride)
		fix->line_length = mfd->mdp.fb_stride(mfd->index, var->xres,
							bpp);
	else
		fix->line_length = var->xres * bpp;

	var->yres = panel_info->yres;
	if (panel_info->physical_width)
		var->width = panel_info->physical_width;
	if (panel_info->physical_height)
		var->height = panel_info->physical_height;
	var->xres_virtual = var->xres;
	var->yres_virtual = panel_info->yres * mfd->fb_page;
	var->bits_per_pixel = bpp * 8;	/* FrameBuffer color depth */
	var->upper_margin = panel_info->lcdc.v_back_porch;
	var->lower_margin = panel_info->lcdc.v_front_porch;
	var->vsync_len = panel_info->lcdc.v_pulse_width;
	var->left_margin = panel_info->lcdc.h_back_porch;
	var->right_margin = panel_info->lcdc.h_front_porch;
	var->hsync_len = panel_info->lcdc.h_pulse_width;
	var->pixclock = panel_info->clk_rate / 1000;

	/* id field for fb app  */

	id = (int *)&mfd->panel;

	snprintf(fix->id, sizeof(fix->id), "mdssfb_%x", (u32) *id);

	fbi->fbops = &mdss_fb_ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = mdss_fb_pseudo_palette;

	mfd->ref_cnt = 0;
	mfd->panel_power_on = false;
	mfd->dcm_state = DCM_UNINIT;

	mdss_fb_parse_dt_split(mfd);

	if (mdss_fb_alloc_fbmem(mfd)) {
		pr_err("unable to allocate framebuffer memory\n");
		return -ENOMEM;
	}

	mfd->op_enable = true;

	mutex_init(&mfd->update.lock);
	mutex_init(&mfd->no_update.lock);
	mutex_init(&mfd->mdp_sync_pt_data.sync_mutex);
	atomic_set(&mfd->mdp_sync_pt_data.commit_cnt, 0);
	atomic_set(&mfd->commits_pending, 0);

	init_timer(&mfd->no_update.timer);
	mfd->no_update.timer.function = mdss_fb_no_update_notify_timer_cb;
	mfd->no_update.timer.data = (unsigned long)mfd;
	init_completion(&mfd->update.comp);
	init_completion(&mfd->no_update.comp);
	init_completion(&mfd->power_off_comp);
	init_completion(&mfd->power_set_comp);
	init_waitqueue_head(&mfd->commit_wait_q);
	init_waitqueue_head(&mfd->idle_wait_q);

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret)
		pr_err("fb_alloc_cmap() failed!\n");

	if (register_framebuffer(fbi) < 0) {
		fb_dealloc_cmap(&fbi->cmap);

		mfd->op_enable = false;
		return -EPERM;
	}

	pr_info("FrameBuffer[%d] %dx%d size=%d registered successfully!\n",
		     mfd->index, fbi->var.xres, fbi->var.yres,
		     fbi->fix.smem_len);

	return 0;
}

static int mdss_fb_open(struct fb_info *info, int user)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct mdss_fb_proc_info *pinfo = NULL;
	int result;
	int pid = current->tgid;

	if (mfd->shutdown_pending) {
/* Temporaily fix , It will be activated after QMC case complete */
//		pr_err("Shutdown pending. Aborting operation\n");
		return -EPERM;
	}

	list_for_each_entry(pinfo, &mfd->proc_list, list) {
		if (pinfo->pid == pid)
			break;
	}

	if ((pinfo == NULL) || (pinfo->pid != pid)) {
		pinfo = kmalloc(sizeof(*pinfo), GFP_KERNEL);
		if (!pinfo) {
			pr_err("unable to alloc process info\n");
			return -ENOMEM;
		}
		pinfo->pid = pid;
		pinfo->ref_cnt = 0;
		list_add(&pinfo->list, &mfd->proc_list);
		pr_debug("new process entry pid=%d\n", pinfo->pid);
	}

	result = pm_runtime_get_sync(info->dev);

	if (result < 0) {
		pr_err("pm_runtime: fail to wake up\n");
		goto pm_error;
	}

	if (!mfd->ref_cnt) {
		mfd->disp_thread = kthread_run(__mdss_fb_display_thread, mfd,
				"mdss_fb%d", mfd->index);
		if (IS_ERR(mfd->disp_thread)) {
			pr_err("unable to start display thread %d\n",
				mfd->index);
			result = PTR_ERR(mfd->disp_thread);
			mfd->disp_thread = NULL;
			goto thread_error;
		}

		result = mdss_fb_blank_sub(FB_BLANK_UNBLANK, info,
					   mfd->op_enable);
		if (result) {
			pr_err("can't turn on fb%d! rc=%d\n", mfd->index,
				result);
			goto blank_error;
		}
	}

	pinfo->ref_cnt++;
	mfd->ref_cnt++;

#if defined (CONFIG_LGE_MIPI_LGD_LG4591_VIDEO_HD_PANEL)
/* Stop the splash thread once boot animation proc opened fb node */
if (mfd->splash_thread && mfd->ref_cnt > 2) {
	pr_info("%s : splash done!! ref_cnt = %d \n", __func__, mfd->ref_cnt);
	kthread_stop(mfd->splash_thread);
	mfd->splash_thread = NULL;
}
#endif

	return 0;

blank_error:
	kthread_stop(mfd->disp_thread);
	mfd->disp_thread = NULL;

thread_error:
	if (pinfo && !pinfo->ref_cnt) {
		list_del(&pinfo->list);
		kfree(pinfo);
	}
	pm_runtime_put(info->dev);

pm_error:
	return result;
}

static int mdss_fb_release_all(struct fb_info *info, bool release_all)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct mdss_fb_proc_info *pinfo = NULL, *temp_pinfo = NULL;
	int ret = 0;
	int pid = current->tgid;
	bool unknown_pid = true, release_needed = false;
	struct task_struct *task = current->group_leader;

	if (!mfd->ref_cnt) {
		pr_info("try to close unopened fb %d! from %s\n", mfd->index,
			task->comm);
		return -EINVAL;
	}

	mdss_fb_pan_idle(mfd);

	pr_debug("release_all = %s\n", release_all ? "true" : "false");

	list_for_each_entry_safe(pinfo, temp_pinfo, &mfd->proc_list, list) {
		if (!release_all && (pinfo->pid != pid))
			continue;

		unknown_pid = false;

		pr_debug("found process %s pid=%d mfd->ref=%d pinfo->ref=%d\n",
			task->comm, mfd->ref_cnt, pinfo->pid, pinfo->ref_cnt);

		do {
			if (mfd->ref_cnt < pinfo->ref_cnt)
				pr_warn("WARN:mfd->ref=%d < pinfo->ref=%d\n",
					mfd->ref_cnt, pinfo->ref_cnt);
			else
				mfd->ref_cnt--;

			pinfo->ref_cnt--;
			pm_runtime_put(info->dev);
		} while (release_all && pinfo->ref_cnt);

		if (release_all && mfd->disp_thread) {
			kthread_stop(mfd->disp_thread);
			mfd->disp_thread = NULL;
		}

		if (pinfo->ref_cnt == 0) {
			list_del(&pinfo->list);
			kfree(pinfo);
			release_needed = !release_all;
		}

		if (!release_all)
			break;
	}

	if (release_needed) {
		pr_debug("known process %s pid=%d mfd->ref=%d\n",
			task->comm, pid, mfd->ref_cnt);

		if (mfd->mdp.release_fnc) {
			ret = mfd->mdp.release_fnc(mfd, false);
			if (ret)
				pr_err("error releasing fb%d pid=%d\n",
					mfd->index, pid);
		}
	} else if (unknown_pid || release_all) {
		pr_warn("unknown process %s pid=%d mfd->ref=%d\n",
			task->comm, pid, mfd->ref_cnt);

		if (mfd->ref_cnt)
			mfd->ref_cnt--;

		if (mfd->mdp.release_fnc) {
			ret = mfd->mdp.release_fnc(mfd, true);
			if (ret)
				pr_err("error fb%d release process %s pid=%d\n",
					mfd->index, task->comm, pid);
		}
	}

	if (!mfd->ref_cnt) {
		if (mfd->disp_thread) {
			kthread_stop(mfd->disp_thread);
			mfd->disp_thread = NULL;
		}

		ret = mdss_fb_blank_sub(FB_BLANK_POWERDOWN, info,
			mfd->op_enable);
		if (ret) {
			pr_err("can't turn off fb%d! rc=%d process %s pid=%d\n",
				mfd->index, ret, task->comm, pid);
			return ret;
		}
	}

	return ret;
}

static int mdss_fb_release(struct fb_info *info, int user)
{
	return mdss_fb_release_all(info, false);
}

static void mdss_fb_power_setting_idle(struct msm_fb_data_type *mfd)
{
	int ret;

	if (mfd->is_power_setting) {
		ret = wait_for_completion_timeout(
				&mfd->power_set_comp,
			msecs_to_jiffies(WAIT_DISP_OP_TIMEOUT));
		if (ret < 0)
			ret = -ERESTARTSYS;
		else if (!ret)
			pr_err("%s wait for power_set_comp timeout %d %d",
				__func__, ret, mfd->is_power_setting);
		if (ret <= 0) {
			mfd->is_power_setting = false;
			complete_all(&mfd->power_set_comp);
		}
	}
}

void mdss_fb_wait_for_fence(struct msm_sync_pt_data *sync_pt_data)
{
	struct sync_fence *fences[MDP_MAX_FENCE_FD];
	int fence_cnt;
	int i, ret = 0;

	pr_debug("%s: wait for fences\n", sync_pt_data->fence_name);

	mutex_lock(&sync_pt_data->sync_mutex);
	/*
	 * Assuming that acq_fen_cnt is sanitized in bufsync ioctl
	 * to check for sync_pt_data->acq_fen_cnt) <= MDP_MAX_FENCE_FD
	 */
	fence_cnt = sync_pt_data->acq_fen_cnt;
	sync_pt_data->acq_fen_cnt = 0;
	if (fence_cnt)
		memcpy(fences, sync_pt_data->acq_fen,
				fence_cnt * sizeof(struct sync_fence *));
	mutex_unlock(&sync_pt_data->sync_mutex);

	/* buf sync */
#if defined (CONFIG_MACH_MSM8926_VFP_KR)
	if (is_fboot) {
		for (i = 0; i < fence_cnt && !ret; i++)
				sync_fence_put(fences[i]);
	} else {
		for (i = 0; i < fence_cnt && !ret; i++) {
			ret = sync_fence_wait(fences[i],
					WAIT_FENCE_FIRST_TIMEOUT);
			if (ret == -ETIME) {
				pr_warn("%s: sync_fence_wait timed out! ",
						sync_pt_data->fence_name);
				pr_cont("Waiting %ld more seconds\n",
						WAIT_FENCE_FINAL_TIMEOUT/MSEC_PER_SEC);
				ret = sync_fence_wait(fences[i],
						WAIT_FENCE_FINAL_TIMEOUT);
			}
			sync_fence_put(fences[i]);
		}
	}
#else
	for (i = 0; i < fence_cnt && !ret; i++) {
		ret = sync_fence_wait(fences[i],
				WAIT_FENCE_FIRST_TIMEOUT);
		if (ret == -ETIME) {
			pr_warn("%s: sync_fence_wait timed out! ",
					sync_pt_data->fence_name);
			pr_cont("Waiting %ld more seconds\n",
					WAIT_FENCE_FINAL_TIMEOUT/MSEC_PER_SEC);
			ret = sync_fence_wait(fences[i],
					WAIT_FENCE_FINAL_TIMEOUT);
		}
		sync_fence_put(fences[i]);
	}
#endif
	if (ret < 0) {
		pr_err("%s: sync_fence_wait failed! ret = %x\n",
				sync_pt_data->fence_name, ret);
		for (; i < fence_cnt; i++)
			sync_fence_put(fences[i]);
	}
}

/**
 * mdss_fb_signal_timeline() - signal a single release fence
 * @sync_pt_data:	Sync point data structure for the timeline which
 *			should be signaled.
 *
 * This is called after a frame has been pushed to display. This signals the
 * timeline to release the fences associated with this frame.
 */
void mdss_fb_signal_timeline(struct msm_sync_pt_data *sync_pt_data)
{
	mutex_lock(&sync_pt_data->sync_mutex);
	if (atomic_add_unless(&sync_pt_data->commit_cnt, -1, 0) &&
			sync_pt_data->timeline) {
		sw_sync_timeline_inc(sync_pt_data->timeline, 1);
		sync_pt_data->timeline_value++;

		pr_debug("%s: buffer signaled! timeline val=%d remaining=%d\n",
			sync_pt_data->fence_name, sync_pt_data->timeline_value,
			atomic_read(&sync_pt_data->commit_cnt));
	} else {
		pr_debug("%s timeline signaled without commits val=%d\n",
			sync_pt_data->fence_name, sync_pt_data->timeline_value);
	}
	mutex_unlock(&sync_pt_data->sync_mutex);
}

/**
 * mdss_fb_release_fences() - signal all pending release fences
 * @mfd:	Framebuffer data structure for display
 *
 * Release all currently pending release fences, including those that are in
 * the process to be commited.
 *
 * Note: this should only be called during close or suspend sequence.
 */
static void mdss_fb_release_fences(struct msm_fb_data_type *mfd)
{
	struct msm_sync_pt_data *sync_pt_data = &mfd->mdp_sync_pt_data;
	int val;

	mutex_lock(&sync_pt_data->sync_mutex);
	if (sync_pt_data->timeline) {
		val = sync_pt_data->threshold +
			atomic_read(&sync_pt_data->commit_cnt);
		sw_sync_timeline_inc(sync_pt_data->timeline, val);
		sync_pt_data->timeline_value += val;
		atomic_set(&sync_pt_data->commit_cnt, 0);
	}
	mutex_unlock(&sync_pt_data->sync_mutex);
}

/**
 * __mdss_fb_sync_buf_done_callback() - process async display events
 * @p:		Notifier block registered for async events.
 * @event:	Event enum to identify the event.
 * @data:	Optional argument provided with the event.
 *
 * See enum mdp_notify_event for events handled.
 */
static int __mdss_fb_sync_buf_done_callback(struct notifier_block *p,
		unsigned long event, void *data)
{
	struct msm_sync_pt_data *sync_pt_data;

	sync_pt_data = container_of(p, struct msm_sync_pt_data, notifier);

	switch (event) {
	case MDP_NOTIFY_FRAME_READY:
		if (sync_pt_data->async_wait_fences)
			mdss_fb_wait_for_fence(sync_pt_data);
		break;
	case MDP_NOTIFY_FRAME_FLUSHED:
		pr_debug("%s: frame flushed\n", sync_pt_data->fence_name);
		sync_pt_data->flushed = true;
		break;
	case MDP_NOTIFY_FRAME_TIMEOUT:
		pr_err("%s: frame timeout\n", sync_pt_data->fence_name);
		mdss_fb_signal_timeline(sync_pt_data);
		break;
	case MDP_NOTIFY_FRAME_DONE:
		pr_debug("%s: frame done\n", sync_pt_data->fence_name);
		mdss_fb_signal_timeline(sync_pt_data);
		break;
	}

	return NOTIFY_OK;
}

/**
 * mdss_fb_pan_idle() - wait for panel programming to be idle
 * @mfd:	Framebuffer data structure for display
 *
 * Wait for any pending programming to be done if in the process of programming
 * hardware configuration. After this function returns it is safe to perform
 * software updates for next frame.
 */
static int mdss_fb_pan_idle(struct msm_fb_data_type *mfd)
{
	int ret = 0;

	ret = wait_event_timeout(mfd->idle_wait_q,
			(!atomic_read(&mfd->commits_pending) ||
			 mfd->shutdown_pending),
			msecs_to_jiffies(WAIT_DISP_OP_TIMEOUT));
	if (!ret) {
		pr_err("wait for idle timeout %d pending=%d\n",
				ret, atomic_read(&mfd->commits_pending));

		mdss_fb_signal_timeline(&mfd->mdp_sync_pt_data);
	} else if (mfd->shutdown_pending) {
		pr_debug("Shutdown signalled\n");
		return -EPERM;
	}

	return 0;
}


static int mdss_fb_pan_display_ex(struct fb_info *info,
		struct mdp_display_commit *disp_commit)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_var_screeninfo *var = &disp_commit->var;
	u32 wait_for_finish = disp_commit->wait_for_finish;
	int ret = 0;

	if (!mfd || (!mfd->op_enable) || (!mfd->panel_power_on))
		return -EPERM;

	if (var->xoffset > (info->var.xres_virtual - info->var.xres))
		return -EINVAL;

	if (var->yoffset > (info->var.yres_virtual - info->var.yres))
		return -EINVAL;

	ret = mdss_fb_pan_idle(mfd);
	if (ret) {
		pr_err("Shutdown pending. Aborting operation\n");
		return ret;
	}

	mutex_lock(&mfd->mdp_sync_pt_data.sync_mutex);
	if (info->fix.xpanstep)
		info->var.xoffset =
		(var->xoffset / info->fix.xpanstep) * info->fix.xpanstep;

	if (info->fix.ypanstep)
		info->var.yoffset =
		(var->yoffset / info->fix.ypanstep) * info->fix.ypanstep;

	mfd->msm_fb_backup.info = *info;
	mfd->msm_fb_backup.disp_commit = *disp_commit;

	atomic_inc(&mfd->mdp_sync_pt_data.commit_cnt);
	atomic_inc(&mfd->commits_pending);
	wake_up_all(&mfd->commit_wait_q);
	mutex_unlock(&mfd->mdp_sync_pt_data.sync_mutex);
	if (wait_for_finish)
		mdss_fb_pan_idle(mfd);
	return ret;
}

static int mdss_fb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct mdp_display_commit disp_commit;
	memset(&disp_commit, 0, sizeof(disp_commit));
	disp_commit.wait_for_finish = true;
	memcpy(&disp_commit.var, var, sizeof(struct fb_var_screeninfo));
	return mdss_fb_pan_display_ex(info, &disp_commit);
}

static int mdss_fb_pan_display_sub(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if ((!mfd->op_enable) || (!mfd->panel_power_on))
		return -EPERM;

	if (var->xoffset > (info->var.xres_virtual - info->var.xres))
		return -EINVAL;

	if (var->yoffset > (info->var.yres_virtual - info->var.yres))
		return -EINVAL;

	if (info->fix.xpanstep)
		info->var.xoffset =
		(var->xoffset / info->fix.xpanstep) * info->fix.xpanstep;

	if (info->fix.ypanstep)
		info->var.yoffset =
		(var->yoffset / info->fix.ypanstep) * info->fix.ypanstep;

	if (mfd->mdp.dma_fnc)
#if defined (CONFIG_LGE_MIPI_LGD_LG4591_VIDEO_HD_PANEL)
		mfd->mdp.dma_fnc(mfd, NULL, 0, NULL);
#else
		mfd->mdp.dma_fnc(mfd);
#endif
	else
		pr_warn("dma function not set for panel type=%d\n",
				mfd->panel.type);

	return 0;
}

static void mdss_fb_var_to_panelinfo(struct fb_var_screeninfo *var,
	struct mdss_panel_info *pinfo)
{
	pinfo->xres = var->xres;
	pinfo->yres = var->yres;
	pinfo->lcdc.v_front_porch = var->lower_margin;
	pinfo->lcdc.v_back_porch = var->upper_margin;
	pinfo->lcdc.v_pulse_width = var->vsync_len;
	pinfo->lcdc.h_front_porch = var->right_margin;
	pinfo->lcdc.h_back_porch = var->left_margin;
	pinfo->lcdc.h_pulse_width = var->hsync_len;
	pinfo->clk_rate = var->pixclock;
}

/**
 * __mdss_fb_perform_commit() - process a frame to display
 * @mfd:	Framebuffer data structure for display
 *
 * Processes all layers and buffers programmed and ensures all pending release
 * fences are signaled once the buffer is transfered to display.
 */
static int __mdss_fb_perform_commit(struct msm_fb_data_type *mfd)
{
	struct msm_sync_pt_data *sync_pt_data = &mfd->mdp_sync_pt_data;
	struct msm_fb_backup_type *fb_backup = &mfd->msm_fb_backup;
	int ret = -ENOSYS;

	if (!sync_pt_data->async_wait_fences)
		mdss_fb_wait_for_fence(sync_pt_data);
	sync_pt_data->flushed = false;

	if (fb_backup->disp_commit.flags & MDP_DISPLAY_COMMIT_OVERLAY) {
		if (mfd->mdp.kickoff_fnc)
			ret = mfd->mdp.kickoff_fnc(mfd,
					&fb_backup->disp_commit);
		else
			pr_warn("no kickoff function setup for fb%d\n",
					mfd->index);
	} else {
		ret = mdss_fb_pan_display_sub(&fb_backup->disp_commit.var,
				&fb_backup->info);
		if (ret)
			pr_err("pan display failed %x on fb%d\n", ret,
					mfd->index);
	}
	if (!ret)
		mdss_fb_update_backlight(mfd);

	if (IS_ERR_VALUE(ret) || !sync_pt_data->flushed)
		mdss_fb_signal_timeline(sync_pt_data);

	return ret;
}

static int __mdss_fb_display_thread(void *data)
{
	struct msm_fb_data_type *mfd = data;
	int ret;
	struct sched_param param;

	param.sched_priority = 16;
	ret = sched_setscheduler(current, SCHED_FIFO, &param);
	if (ret)
		pr_warn("set priority failed for fb%d display thread\n",
				mfd->index);

	while (1) {
		wait_event(mfd->commit_wait_q,
				(atomic_read(&mfd->commits_pending) ||
				 kthread_should_stop()));

		if (kthread_should_stop())
			break;

		ret = __mdss_fb_perform_commit(mfd);
		atomic_dec(&mfd->commits_pending);
		wake_up_all(&mfd->idle_wait_q);
	}

	atomic_set(&mfd->commits_pending, 0);
	wake_up_all(&mfd->idle_wait_q);

	return ret;
}

static int mdss_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (var->rotate != FB_ROTATE_UR)
		return -EINVAL;
	if (var->grayscale != info->var.grayscale)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 16:
		if ((var->green.offset != 5) ||
		    !((var->blue.offset == 11)
		      || (var->blue.offset == 0)) ||
		    !((var->red.offset == 11)
		      || (var->red.offset == 0)) ||
		    (var->blue.length != 5) ||
		    (var->green.length != 6) ||
		    (var->red.length != 5) ||
		    (var->blue.msb_right != 0) ||
		    (var->green.msb_right != 0) ||
		    (var->red.msb_right != 0) ||
		    (var->transp.offset != 0) ||
		    (var->transp.length != 0))
			return -EINVAL;
		break;

	case 24:
		if ((var->blue.offset != 0) ||
		    (var->green.offset != 8) ||
		    (var->red.offset != 16) ||
		    (var->blue.length != 8) ||
		    (var->green.length != 8) ||
		    (var->red.length != 8) ||
		    (var->blue.msb_right != 0) ||
		    (var->green.msb_right != 0) ||
		    (var->red.msb_right != 0) ||
		    !(((var->transp.offset == 0) &&
		       (var->transp.length == 0)) ||
		      ((var->transp.offset == 24) &&
		       (var->transp.length == 8))))
			return -EINVAL;
		break;

	case 32:
		/* Figure out if the user meant RGBA or ARGB
		   and verify the position of the RGB components */

		if (var->transp.offset == 24) {
			if ((var->blue.offset != 0) ||
			    (var->green.offset != 8) ||
			    (var->red.offset != 16))
				return -EINVAL;
		} else if (var->transp.offset == 0) {
			if ((var->blue.offset != 8) ||
			    (var->green.offset != 16) ||
			    (var->red.offset != 24))
				return -EINVAL;
		} else
			return -EINVAL;

		/* Check the common values for both RGBA and ARGB */

		if ((var->blue.length != 8) ||
		    (var->green.length != 8) ||
		    (var->red.length != 8) ||
		    (var->transp.length != 8) ||
		    (var->blue.msb_right != 0) ||
		    (var->green.msb_right != 0) ||
		    (var->red.msb_right != 0))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	if ((var->xres_virtual <= 0) || (var->yres_virtual <= 0))
		return -EINVAL;

	if (info->fix.smem_start) {
		u32 len = var->xres_virtual * var->yres_virtual *
			(var->bits_per_pixel / 8);
		if (len > info->fix.smem_len)
			return -EINVAL;
	}

	if ((var->xres == 0) || (var->yres == 0))
		return -EINVAL;

	if (var->xoffset > (var->xres_virtual - var->xres))
		return -EINVAL;

	if (var->yoffset > (var->yres_virtual - var->yres))
		return -EINVAL;

	if (mfd->panel_info) {
		struct mdss_panel_info panel_info;
		int rc;

		memcpy(&panel_info, mfd->panel_info, sizeof(panel_info));
		mdss_fb_var_to_panelinfo(var, &panel_info);
		rc = mdss_fb_send_panel_event(mfd, MDSS_EVENT_CHECK_PARAMS,
			&panel_info);
		if (IS_ERR_VALUE(rc))
			return rc;
		mfd->panel_reconfig = rc;
	}

	return 0;
}

static int mdss_fb_set_par(struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_var_screeninfo *var = &info->var;
	int old_imgType;
	int ret = 0;

#if defined(CONFIG_FB_MSM_MIPI_TIANMA_CMD_HVGA_PT) || defined(CONFIG_MACH_MSM8X10_L70P) || defined(CONFIG_MACH_MSM8X10_Y30)

    struct mdss_panel_data *pdata;

    pdata = dev_get_platdata(&mfd->pdev->dev);
#endif

	ret = mdss_fb_pan_idle(mfd);
	if (ret) {
		pr_err("Shutdown pending. Aborting operation\n");
		return ret;
	}

	old_imgType = mfd->fb_imgType;
	switch (var->bits_per_pixel) {
	case 16:
		if (var->red.offset == 0)
			mfd->fb_imgType = MDP_BGR_565;
		else
			mfd->fb_imgType	= MDP_RGB_565;
		break;

	case 24:
		if ((var->transp.offset == 0) && (var->transp.length == 0))
			mfd->fb_imgType = MDP_RGB_888;
		else if ((var->transp.offset == 24) &&
			 (var->transp.length == 8)) {
			mfd->fb_imgType = MDP_ARGB_8888;
			info->var.bits_per_pixel = 32;
		}
		break;

	case 32:
		if (var->transp.offset == 24)
			mfd->fb_imgType = MDP_ARGB_8888;
		else
			mfd->fb_imgType	= MDP_RGBA_8888;
		break;

	default:
		return -EINVAL;
	}


	if (mfd->mdp.fb_stride)
		mfd->fbi->fix.line_length = mfd->mdp.fb_stride(mfd->index,
						var->xres,
						var->bits_per_pixel / 8);
	else
		mfd->fbi->fix.line_length = var->xres * var->bits_per_pixel / 8;


	if (mfd->panel_reconfig || (mfd->fb_imgType != old_imgType)) {
		force_set_bl_f = 1;
#if defined(CONFIG_FB_MSM_MIPI_TIANMA_CMD_HVGA_PT) || defined(CONFIG_MACH_MSM8X10_L70P) ||  defined(CONFIG_MACH_MSM8X10_Y30)

        if ((pdata) && (pdata->set_backlight)) {
            pdata->set_backlight(pdata,0);
        }
#endif
		mdss_fb_blank_sub(FB_BLANK_POWERDOWN, info, mfd->op_enable);
		mdss_fb_var_to_panelinfo(var, mfd->panel_info);
		mdss_fb_blank_sub(FB_BLANK_UNBLANK, info, mfd->op_enable);
		mfd->panel_reconfig = false;
		pr_info("blank/unblank reconfing =%d, mfd->fb_imgType =%d, old_imgType =%d",
								mfd->panel_reconfig,mfd->fb_imgType,old_imgType);
		force_set_bl_f = 0;
	}

	return ret;
}

int mdss_fb_dcm(struct msm_fb_data_type *mfd, int req_state)
{
	int ret = 0;

	if (req_state == mfd->dcm_state) {
		pr_warn("Already in correct DCM/DTM state");
		return ret;
	}

	switch (req_state) {
	case DCM_UNBLANK:
		if (mfd->dcm_state == DCM_UNINIT &&
			!mfd->panel_power_on && mfd->mdp.on_fnc) {
			ret = mfd->mdp.on_fnc(mfd);
			if (ret == 0) {
				mfd->panel_power_on = true;
				mfd->dcm_state = DCM_UNBLANK;
			}
		}
		break;
	case DCM_ENTER:
		if (mfd->dcm_state == DCM_UNBLANK) {
			/*
			 * Keep unblank path available for only
			 * DCM operation
			 */
			mfd->panel_power_on = false;
			mfd->dcm_state = DCM_ENTER;
		}
		break;
	case DCM_EXIT:
		if (mfd->dcm_state == DCM_ENTER) {
			/* Release the unblank path for exit */
			mfd->panel_power_on = true;
			mfd->dcm_state = DCM_EXIT;
		}
		break;
	case DCM_BLANK:
		if ((mfd->dcm_state == DCM_EXIT ||
			mfd->dcm_state == DCM_UNBLANK) &&
			mfd->panel_power_on && mfd->mdp.off_fnc) {
			ret = mfd->mdp.off_fnc(mfd);
			if (ret == 0) {
				mfd->panel_power_on = false;
				mfd->dcm_state = DCM_UNINIT;
			}
		}
		break;
	case DTM_ENTER:
		if (mfd->dcm_state == DCM_UNINIT)
			mfd->dcm_state = DTM_ENTER;
		break;
	case DTM_EXIT:
		if (mfd->dcm_state == DTM_ENTER)
			mfd->dcm_state = DCM_UNINIT;
		break;
	}

	return ret;
}

static int mdss_fb_cursor(struct fb_info *info, void __user *p)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_cursor cursor;
	int ret;

	if (!mfd->mdp.cursor_update)
		return -ENODEV;

	ret = copy_from_user(&cursor, p, sizeof(cursor));
	if (ret)
		return ret;

	return mfd->mdp.cursor_update(mfd, &cursor);
}

static int mdss_fb_set_lut(struct fb_info *info, void __user *p)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_cmap cmap;
	int ret;

	if (!mfd->mdp.lut_update)
		return -ENODEV;

	ret = copy_from_user(&cmap, p, sizeof(cmap));
	if (ret)
		return ret;

	mfd->mdp.lut_update(mfd, &cmap);
	return 0;
}

/**
 * mdss_fb_sync_get_rel_fence() - get release fence from sync pt timeline
 * @sync_pt_data:	Sync pt structure holding timeline and fence info.
 *
 * Function returns a release fence on the timeline associated with the
 * sync pt struct given and it's associated information. The release fence
 * created can be used to signal when buffers provided will be released.
 */
static struct sync_fence *__mdss_fb_sync_get_rel_fence(
		struct msm_sync_pt_data *sync_pt_data)
{
	struct sync_pt *rel_sync_pt;
	struct sync_fence *rel_fence;
	int val;

	val = sync_pt_data->timeline_value + sync_pt_data->threshold +
		atomic_read(&sync_pt_data->commit_cnt);

	pr_debug("%s: buf sync rel fence timeline=%d\n",
		sync_pt_data->fence_name, val);

	rel_sync_pt = sw_sync_pt_create(sync_pt_data->timeline, val);
	if (rel_sync_pt == NULL) {
		pr_err("%s: cannot create sync point\n",
				sync_pt_data->fence_name);
		return NULL;
	}

	/* create fence */
	rel_fence = sync_fence_create(sync_pt_data->fence_name, rel_sync_pt);
	if (rel_fence == NULL) {
		sync_pt_free(rel_sync_pt);
		pr_err("%s: cannot create fence\n", sync_pt_data->fence_name);
		return NULL;
	}

	return rel_fence;
}

static int mdss_fb_handle_buf_sync_ioctl(struct msm_sync_pt_data *sync_pt_data,
				 struct mdp_buf_sync *buf_sync)
{
	int i, ret = 0;
	int acq_fen_fd[MDP_MAX_FENCE_FD];
	struct sync_fence *fence, *rel_fence;
	int rel_fen_fd;

	if ((buf_sync->acq_fen_fd_cnt > MDP_MAX_FENCE_FD) ||
				(sync_pt_data->timeline == NULL))
		return -EINVAL;

	if (buf_sync->acq_fen_fd_cnt)
		ret = copy_from_user(acq_fen_fd, buf_sync->acq_fen_fd,
				buf_sync->acq_fen_fd_cnt * sizeof(int));
	if (ret) {
		pr_err("%s: copy_from_user failed", sync_pt_data->fence_name);
		return ret;
	}

	if (sync_pt_data->acq_fen_cnt) {
		pr_warn("%s: currently %d fences active. waiting...\n",
				sync_pt_data->fence_name,
				sync_pt_data->acq_fen_cnt);
		mdss_fb_wait_for_fence(sync_pt_data);
	}

	mutex_lock(&sync_pt_data->sync_mutex);
	for (i = 0; i < buf_sync->acq_fen_fd_cnt; i++) {
		fence = sync_fence_fdget(acq_fen_fd[i]);
		if (fence == NULL) {
			pr_err("%s: null fence! i=%d fd=%d\n",
					sync_pt_data->fence_name, i,
					acq_fen_fd[i]);
			ret = -EINVAL;
			break;
		}
		sync_pt_data->acq_fen[i] = fence;
	}
	sync_pt_data->acq_fen_cnt = i;
	if (ret)
		goto buf_sync_err_1;

	rel_fence = __mdss_fb_sync_get_rel_fence(sync_pt_data);
	if (IS_ERR_OR_NULL(rel_fence)) {
		pr_err("%s: unable to retrieve release fence\n",
				sync_pt_data->fence_name);
		ret = rel_fence ? PTR_ERR(rel_fence) : -ENOMEM;
		goto buf_sync_err_1;
	}

	/* create fd */
	rel_fen_fd = get_unused_fd_flags(0);
	if (rel_fen_fd < 0) {
		pr_err("%s: get_unused_fd_flags failed\n",
				sync_pt_data->fence_name);
		ret = -EIO;
		goto buf_sync_err_2;
	}

	sync_fence_install(rel_fence, rel_fen_fd);

	ret = copy_to_user(buf_sync->rel_fen_fd, &rel_fen_fd, sizeof(int));
	if (ret) {
		pr_err("%s: copy_to_user failed\n", sync_pt_data->fence_name);
		goto buf_sync_err_3;
	}
	mutex_unlock(&sync_pt_data->sync_mutex);

	if (buf_sync->flags & MDP_BUF_SYNC_FLAG_WAIT)
		mdss_fb_wait_for_fence(sync_pt_data);

	return ret;
buf_sync_err_3:
	put_unused_fd(rel_fen_fd);
buf_sync_err_2:
	sync_fence_put(rel_fence);
buf_sync_err_1:
	for (i = 0; i < sync_pt_data->acq_fen_cnt; i++)
		sync_fence_put(sync_pt_data->acq_fen[i]);
	sync_pt_data->acq_fen_cnt = 0;
	mutex_unlock(&sync_pt_data->sync_mutex);
	return ret;
}
static int mdss_fb_display_commit(struct fb_info *info,
						unsigned long *argp)
{
	int ret;
	struct mdp_display_commit disp_commit;
	ret = copy_from_user(&disp_commit, argp,
			sizeof(disp_commit));
	if (ret) {
		pr_err("%s:copy_from_user failed", __func__);
		return ret;
	}
	ret = mdss_fb_pan_display_ex(info, &disp_commit);
	return ret;
}


static int mdss_fb_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	struct msm_fb_data_type *mfd;
	void __user *argp = (void __user *)arg;
	struct mdp_page_protection fb_page_protection;
	int ret = -ENOSYS;
	struct mdp_buf_sync buf_sync;
	struct msm_sync_pt_data *sync_pt_data = NULL;

#ifdef CONFIG_MACH_LGE
	u32 dsi_panel_invert = 0;
#endif

#if defined(CONFIG_LGE_BROADCAST_TDMB)
	int dmb_flag = 0;
	struct mdp_csc_cfg dmb_csc_cfg;
#endif /*               */

	if (!info || !info->par)
		return -EINVAL;
	mfd = (struct msm_fb_data_type *)info->par;
	mdss_fb_power_setting_idle(mfd);
	if ((cmd != MSMFB_VSYNC_CTRL) && (cmd != MSMFB_OVERLAY_VSYNC_CTRL) &&
			(cmd != MSMFB_ASYNC_BLIT) && (cmd != MSMFB_BLIT) &&
			(cmd != MSMFB_NOTIFY_UPDATE)) {
		ret = mdss_fb_pan_idle(mfd);
		if (ret) {
			pr_debug("Shutdown pending. Aborting operation %x\n",
				cmd);
			return ret;
		}
	}

	switch (cmd) {
	case MSMFB_CURSOR:
		ret = mdss_fb_cursor(info, argp);
		break;

	case MSMFB_SET_LUT:
		ret = mdss_fb_set_lut(info, argp);
		break;

	case MSMFB_GET_PAGE_PROTECTION:
		fb_page_protection.page_protection =
			mfd->mdp_fb_page_protection;
		ret = copy_to_user(argp, &fb_page_protection,
				   sizeof(fb_page_protection));
		if (ret)
			return ret;
		break;

	case MSMFB_BUFFER_SYNC:
		ret = copy_from_user(&buf_sync, argp, sizeof(buf_sync));
		if (ret)
			return ret;
		if ((!mfd->op_enable) || (!mfd->panel_power_on))
			return -EPERM;
		if (mfd->mdp.get_sync_fnc)
			sync_pt_data = mfd->mdp.get_sync_fnc(mfd, &buf_sync);
		if (!sync_pt_data)
			sync_pt_data = &mfd->mdp_sync_pt_data;

		ret = mdss_fb_handle_buf_sync_ioctl(sync_pt_data, &buf_sync);

		if (!ret)
			ret = copy_to_user(argp, &buf_sync, sizeof(buf_sync));
		break;

	case MSMFB_NOTIFY_UPDATE:
		ret = mdss_fb_notify_update(mfd, argp);
		break;

	case MSMFB_DISPLAY_COMMIT:
		ret = mdss_fb_display_commit(info, argp);
		break;

#ifdef CONFIG_MACH_LGE
	case MSMFB_INVERT_PANEL:
		ret = copy_from_user(&dsi_panel_invert, argp, sizeof(int));
		if(ret)
			return ret;
		ret = mdss_dsi_panel_invert(dsi_panel_invert);
	break;
#endif

#if defined(CONFIG_LGE_BROADCAST_TDMB)
	case MSMFB_DMB_SET_FLAG:

		ret = copy_from_user(&dmb_flag, argp, sizeof(int));
		if (ret)
			return ret;
		ret = pp_set_dmb_status(dmb_flag);

		break;
	case MSMFB_DMB_SET_CSC_MATRIX:

		ret = copy_from_user(&dmb_csc_cfg, argp, sizeof(dmb_csc_cfg));
		if (ret)
			return ret;
		memcpy(dmb_csc_convert.csc_mv, dmb_csc_cfg.csc_mv, sizeof(dmb_csc_cfg.csc_mv));

		break;
#endif

	default:
		if (mfd->mdp.ioctl_handler)
			ret = mfd->mdp.ioctl_handler(mfd, cmd, argp);
		break;
	}

	if (ret == -ENOSYS)
		pr_err("unsupported ioctl (%x)\n", cmd);

	return ret;
}

struct fb_info *msm_fb_get_writeback_fb(void)
{
	int c = 0;
	for (c = 0; c < fbi_list_index; ++c) {
		struct msm_fb_data_type *mfd;
		mfd = (struct msm_fb_data_type *)fbi_list[c]->par;
		if (mfd->panel.type == WRITEBACK_PANEL)
			return fbi_list[c];
	}

	return NULL;
}
EXPORT_SYMBOL(msm_fb_get_writeback_fb);

static int mdss_fb_register_extra_panel(struct platform_device *pdev,
	struct mdss_panel_data *pdata)
{
	struct mdss_panel_data *fb_pdata;

	fb_pdata = dev_get_platdata(&pdev->dev);
	if (!fb_pdata) {
		pr_err("framebuffer device %s contains invalid panel data\n",
				dev_name(&pdev->dev));
		return -EINVAL;
	}

	if (fb_pdata->next) {
		pr_err("split panel already setup for framebuffer device %s\n",
				dev_name(&pdev->dev));
		return -EEXIST;
	}

	if ((fb_pdata->panel_info.type != MIPI_VIDEO_PANEL) ||
			(pdata->panel_info.type != MIPI_VIDEO_PANEL)) {
		pr_err("Split panel not supported for panel type %d\n",
				pdata->panel_info.type);
		return -EINVAL;
	}

	fb_pdata->next = pdata;

	return 0;
}

int mdss_register_panel(struct platform_device *pdev,
	struct mdss_panel_data *pdata)
{
	struct platform_device *fb_pdev, *mdss_pdev;
	struct device_node *node;
	int rc = 0;
	bool master_panel = true;

	if (!pdev || !pdev->dev.of_node) {
		pr_err("Invalid device node\n");
		return -ENODEV;
	}

	if (!mdp_instance) {
		pr_err("mdss mdp resource not initialized yet\n");
		return -EPROBE_DEFER;
	}

	node = of_parse_phandle(pdev->dev.of_node, "qcom,mdss-fb-map", 0);
	if (!node) {
		pr_err("Unable to find fb node for device: %s\n",
				pdev->name);
		return -ENODEV;
	}
	mdss_pdev = of_find_device_by_node(node->parent);
	if (!mdss_pdev) {
		pr_err("Unable to find mdss for node: %s\n", node->full_name);
		rc = -ENODEV;
		goto mdss_notfound;
	}

	fb_pdev = of_find_device_by_node(node);
	if (fb_pdev) {
		rc = mdss_fb_register_extra_panel(fb_pdev, pdata);
		if (rc == 0)
			master_panel = false;
	} else {
		pr_info("adding framebuffer device %s\n", dev_name(&pdev->dev));
		fb_pdev = of_platform_device_create(node, NULL,
				&mdss_pdev->dev);
		fb_pdev->dev.platform_data = pdata;
	}

	if (master_panel && mdp_instance->panel_register_done)
		mdp_instance->panel_register_done(pdata);

mdss_notfound:
	of_node_put(node);
	return rc;
}
EXPORT_SYMBOL(mdss_register_panel);

int mdss_fb_register_mdp_instance(struct msm_mdp_interface *mdp)
{
	if (mdp_instance) {
		pr_err("multiple MDP instance registration");
		return -EINVAL;
	}

	mdp_instance = mdp;
	return 0;
}
EXPORT_SYMBOL(mdss_fb_register_mdp_instance);

int mdss_fb_get_phys_info(unsigned long *start, unsigned long *len, int fb_num)
{
	struct fb_info *info;
	struct msm_fb_data_type *mfd;

	if (fb_num > MAX_FBI_LIST)
		return -EINVAL;

	info = fbi_list[fb_num];
	if (!info)
		return -ENOENT;

	mfd = (struct msm_fb_data_type *)info->par;
	if (!mfd)
		return -ENODEV;

	if (mfd->iova)
		*start = mfd->iova;
	else
		*start = info->fix.smem_start;
	*len = info->fix.smem_len;

	return 0;
}
EXPORT_SYMBOL(mdss_fb_get_phys_info);

int __init mdss_fb_init(void)
{
	int rc = -ENODEV;

	if (platform_driver_register(&mdss_fb_driver))
		return rc;

	return 0;
}

module_init(mdss_fb_init);