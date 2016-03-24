/*****************************************************************************
	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8180_drv_api.c

	Description : wrapper source of FC8180 tuner driver

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
******************************************************************************/

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8180_regs.h"
#include "fc8180_isr.h"
#include "fci_hal.h"
#include "fci_types.h"
#include "fc8180_drv_api.h"
#if defined(CONFIG_ARCH_MT6582)
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint_md1.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#endif

#define GPIO_ISDBT_IRQ 84
#define GPIO_ISDBT_PWR_EN 92
#define GPIO_ISDBT_RST 65
u32 bbm_xtal_freq;

/* user stop */
static int fc8180_user_stop_called;

int fc8180_if_test(void)
{
	int res = 0;
	int i;
	u16 wdata = 0;
	u32 ldata = 0;
	u8 data = 0;
	u8 temp = 0;

	print_log(0, "fc8180_if_test Start!!!\n");
	for (i = 0; i < 1000; i++) {
		bbm_com_byte_write(0, 0xa4, i & 0xff);
		bbm_com_byte_read(0, 0xa4, &data);
		if ((i & 0xff) != data) {
			print_log(0, "fc8180_if_btest!   i=0x%x, data=0x%x\n"
				, i & 0xff, data);
			res = 1;
		}
	}

	for (i = 0 ; i < 1000 ; i++) {
		bbm_com_word_write(0, 0xa4, i & 0xffff);
		bbm_com_word_read(0, 0xa4, &wdata);
		if ((i & 0xffff) != wdata) {
			print_log(0, "fc8180_if_wtest!   i=0x%x, data=0x%x\n"
				, i & 0xffff, wdata);
			res = 1;
		}
	}

	for (i = 0 ; i < 1000; i++) {
		bbm_com_long_write(0, 0xa4, i & 0xffffffff);
		bbm_com_long_read(0, 0xa4, &ldata);
		if ((i & 0xffffffff) != ldata) {
			print_log(0, "fc8180_if_ltest!   i=0x%x, data=0x%x\n"
				, i & 0xffffffff, ldata);
			res = 1;
		}
	}

	for (i = 0 ; i < 1000 ; i++) {
		temp = i & 0xff;
		bbm_com_tuner_write(NULL, 0x52, 0x01, &temp, 0x01);
		bbm_com_tuner_read(NULL, 0x52, 0x01, &data, 0x01);
		if ((i & 0xff) != data)
			print_log(0, "FC8180 tuner test (0x%x,0x%x)\n"
			, i & 0xff, data);
	}

	print_log(0, "fc8180_if_test End!!!\n");

	return res;
}

void tunerbb_drv_hw_setting(void)
{
#if defined(CONFIG_MACH_MSM8974_G2_KDDI)
	{
		use_pm8941_xo_a2_192000 = 0;
		bbm_xtal_freq = 26000;
		print_log(NULL, "[1seg] A1-KDDI : %d, xtal_freq : %d\n"
			, use_pm8941_xo_a2_192000, bbm_xtal_freq);
	}
#elif defined(CONFIG_MACH_MSM8974_G2_DCM)
	if (lge_get_board_revno() >= HW_REV_D) {
		use_pm8941_xo_a2_192000 = 1;
		bbm_xtal_freq = 19200;
		print_log(NULL, "[1seg] A1-DCM rev.D or later version: %d\
			, xtal_freq : %d\n"
			, use_pm8941_xo_a2_192000, bbm_xtal_freq);
	} else {
		use_pm8941_xo_a2_192000 = 0;
		bbm_xtal_freq = 26000;
		print_log(NULL, "[1seg] A1-DCM rev.C : %d, xtal_freq : %d\n"
			, use_pm8941_xo_a2_192000, bbm_xtal_freq);
	}
#else
	{
		bbm_xtal_freq = 26000;
	}
#endif
#if defined(CONFIG_ARCH_MT6582)
	print_log(0, "[1seg][MTK] GPIO_ISDBT_PWR_EN Port request!!!\n");
	mt_set_gpio_mode(GPIO_1SEG_EN, GPIO_1SEG_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_1SEG_EN, GPIO_PULL_DOWN);
	udelay(50);
	mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);

	print_log(0, "[1seg][MTK] ISDBT_IRQ_INT Port request!!!\n");
	mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_1SEG_INT, GPIO_PULL_DOWN);
	mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);

	print_log(0, "[1seg][MTK] GPIO_ISDBT_RST Port request set zero!!!\n");
	udelay(50);
	mt_set_gpio_mode(GPIO_1SEG_RESET_N,  GPIO_1SEG_RESET_N_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_1SEG_RESET_N, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_1SEG_RESET_N, GPIO_PULL_DOWN);
	mt_set_gpio_dir(GPIO_1SEG_RESET_N, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);

	mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO82|0x80000000, GPIO_PULL_DOWN);

	mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO83|0x80000000, GPIO_PULL_DOWN);

	mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO81|0x80000000, GPIO_PULL_DOWN);

	mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO80|0x80000000, GPIO_PULL_DOWN);
#else
	if (gpio_request(GPIO_ISDBT_PWR_EN, "ISDBT_EN"))
		print_log(0
		, "[1seg] GPIO_ISDBT_PWR_EN Port request error!!!\n");
	udelay(50);
	gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);

	if (gpio_request(GPIO_ISDBT_IRQ, "ISDBT_IRQ_INT"))
		print_log(0, "[1seg] ISDBT_IRQ_INT Port request error!!!\n");

	gpio_direction_input(GPIO_ISDBT_IRQ);
/*
	if (gpio_request(GPIO_ISDBT_RST, "ISDBT_RST"))
		print_log(0, "[1seg] GPIO_ISDBT_RST Port request error!!!\n");
	udelay(50);
	gpio_direction_output(GPIO_ISDBT_RST, 1);
*/
#endif
}

void tunerbb_drv_hw_init(void)
{

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
|| defined(CONFIG_MACH_MSM8974_G2_KDDI)
|| defined(CONFIG_ARCH_MT6582)
	int ret = 0;
#endif
#if defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_KDDI)
	if (use_pm8941_xo_a2_192000) {
		if (!IS_ERR_OR_NULL(clk)) {
			int ret = -1;
			ret = clk_prepare_enable(clk);
			if (ret) {
				print_log(0, "[1seg] LGE_BROADCAST_DMB_IOCTL_ON\
					enable clock error!!!\n");
				return BBM_NOK;
			}
		}
	}
#endif
#if defined(CONFIG_ARCH_MT6582)
	print_log(0, "[1seg][MTK] isdbt_hw_init\n");
	ret = hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "1V8_MTK_DTV");
	print_log(0, "[1seg][MTK] hwPowerOn MT6323_POWER_LDO_VGP3(%d)\n", ret);
	ms_wait(2);
	mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ONE);
	ms_wait(10);
	mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);
	ms_wait(25);
	mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);
	ms_wait(2);

	mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_DISABLE);

	mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_DISABLE);

	mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_DISABLE);

	mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_DISABLE);

	mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_DISABLE);
	mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_01);
	mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_IN);
#else
	print_log(0, "[1seg] isdbt_hw_init \n");
	//gpio_set_value(GPIO_ISDBT_RST, 1);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
	ms_wait(10);
	//gpio_set_value(GPIO_ISDBT_RST, 0);
	//ms_wait(25);
	//gpio_set_value(GPIO_ISDBT_RST, 1);
	//ms_wait(2);
#endif
       //driver_mode = ISDBT_POWERON;
	return;
}


void tunerbb_drv_hw_deinit(void)
{
       //driver_mode = ISDBT_POWEROFF;
#if defined(CONFIG_ARCH_MT6582)
	int ret = -1;

	mt_eint_mask(CUST_EINT_DTV_NUM);
	print_log(0, "[1seg][MTK] isdbt_hw_deinit\n");

	mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_1SEG_EN, GPIO_PULL_DOWN);
	mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
	ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);
	print_log(0, "[1seg][MTK] mt_set_gpio_out GPIO_1SEG_EN(%d)\n", ret);

	mt_set_gpio_pull_enable(GPIO_1SEG_RESET_N, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_1SEG_RESET_N, GPIO_PULL_DOWN);
	mt_set_gpio_dir(GPIO_1SEG_RESET_N, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);

	mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_1SEG_INT, GPIO_PULL_DOWN);

	mt_set_gpio_pull_enable(GPIO82|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO82|0x80000000, GPIO_PULL_DOWN);

	mt_set_gpio_pull_enable(GPIO83|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO83|0x80000000, GPIO_PULL_DOWN);

	mt_set_gpio_pull_enable(GPIO81|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO81|0x80000000, GPIO_PULL_DOWN);

	mt_set_gpio_mode(GPIO80|0x80000000, GPIO_MODE_00);
	mt_set_gpio_pull_enable(GPIO80|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO80|0x80000000, GPIO_PULL_DOWN);
	mt_set_gpio_dir(GPIO80|0x80000000, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO80|0x80000000, GPIO_OUT_ZERO);

	ret = hwPowerDown(MT6323_POWER_LDO_VGP3, "1V8_MTK_DTV");
	print_log(0, "[1seg][MTK] hwPowerDown MT6323_POWER_LDO_VGP3(%d)\n"
		, ret);
#else
	gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
#endif

#if defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_KDDI)
	if (use_pm8941_xo_a2_192000) {
		if (!IS_ERR_OR_NULL(clk))
			clk_disable_unprepare(clk);
	}
#endif
}



int tunerbb_drv_fc8180_init(int mode)
{
	int res;

	print_log(0, "[1seg] FC8180 tunerbb_drv_fc8180_init\n");
	res = bbm_com_hostif_select(NULL, BBM_SPI);
	res |= bbm_com_i2c_init(NULL, FCI_HPI_TYPE);

#ifdef BBM_SPI_IF
	res = bbm_com_byte_write(NULL, BBM_DM_DATA, 0x00);
        print_log(0, "[1seg] bbm_com_byte_write res : %d \n", res);
#endif

	res |= bbm_com_probe(NULL);
	print_log(0, "[1seg] FC8180 BBM_PROBE res : %d \n", res);
	/* fc8180_if_test(); */

	if (res)
		print_log(0, "[1seg] FC8180 Initialize Fail : %d \n", res);

	res |= bbm_com_init(NULL);
	res |= bbm_com_tuner_select(NULL, FC8180_TUNER, mode);

	if (res)
		print_log(0, "[1seg] IOCTL_ISDBT_POWER_ON FAIL \n");
	else
		print_log(0, "[1seg] IOCTL_ISDBT_POWER_OK \n");

	return res;
}

int tunerbb_drv_fc8180_stop(void)
{
	s32 res = BBM_OK;

	res = bbm_com_hostif_deselect(NULL);

	return res;
}

int tunerbb_drv_fc8180_set_channel(s32 f_rf, u16 mode)
{
	s32 res = BBM_OK;

	bbm_com_tuner_set_freq(NULL, f_rf);

	if (mode)
		res = bbm_com_scan_status(NULL);

	return res;
}

void tunerbb_drv_fc8180_set_user_stop(int ustop)
{
    fc8180_user_stop_called = ustop;
}

int get_fc8180_ustop_state(void)
{
    int ret = 0;
    ret = (fc8180_user_stop_called == 1) ? 1 : 0;

    return ret;
}



int tunerbb_drv_fc8180_Get_SyncStatus(void)
{
	u8 data;
	s32 sync = 0;


	bbm_com_read(NULL, 0x3025, &data);
	if (data & 0x08) {
		sync = 1;

		bbm_com_read(NULL, 0x3026, &data);
		if (data & 0x02) {
			sync = 2;

			bbm_com_read(NULL, 0x5053, &data);
			if (data)
				sync = 3;
		}
	} else
		sync = 0;

	return sync;
}

static int tmcc_inter_length(u8 inter_len, u8 mode)
{
	switch (inter_len) {
	case 0x0:
		return 0;
	case 0x04:
		switch (mode & 0x03) {
		case 3:
			return 1;
		case 2:
			return 2;
		case 1:
			return 4;
		}
	case 0x02:
		switch (mode & 0x03) {
		case 3:
			return 2;
		case 2:
			return 4;
		case 1:
			return 8;
		}
	case 0x06:
		switch (mode & 0x03) {
		case 3:
			return 4;
		case 2:
			return 8;
		case 1:
			return 16;
		}
	}

	return 63;
}

static int tmcc_mod_scheme(u8 mod_scheme)
{
	switch (mod_scheme) {
	case 0x0:
		return 0;	/* "DQPSK"; */
	case 0x04:
		return 1;	/* "QPSK"; */
	case 0x02:
		return 2;	/* "16QAM"; */
	case 0x06:
		return 3;	/* "64QAM"; */
	}

	return 7;
}

static int tmcc_code_rate(u8 code_rate)
{
	switch (code_rate) {
	case 0x0:
		return 0;	/* "1/2"; */
	case 0x04:
		return 1;	/* "2/3"; */
	case 0x02:
		return 2;	/* "3/4"; */
	case 0x06:
		return 3;	/* "5/6"; */
	case 0x01:
		return 4;	/* "7/8"; */
	}

	return 7;
}

static int tmcc_num_segment(u8 num_segment)
{
	switch (num_segment) {
	case 0x08:
		return 1;
	case 0x04:
		return 2;
	case 0x0c:
		return 3;
	case 0x02:
		return 4;
	case 0x0a:
		return 5;
	case 0x06:
		return 6;
	case 0x0e:
		return 7;
	case 0x01:
		return 8;
	case 0x09:
		return 9;
	case 0x05:
		return 10;
	case 0x0d:
		return 11;
	case 0x03:
		return 12;
	case 0x0b:
		return 13;
	}

	return 0x0f;
}

static int tmcc_system_identification(u8 id)
{
	switch (id) {
	case 0x00:
		return 0;
	case 0x02:
		return 1;
	case 0x01:
		return 2;
	}

	return id;
}

static int tmcc_GI_mode(u8 gi)
{
	switch (gi) {
	case 1:
		return 0;
	case 2:
		return 1;
	case 3:
		return 2;
	case 4:
		return 3;
	}

	return 0;
}

int tunerbb_drv_fc8180_get_oneseg_antenna(int ant
	, int ber, int CN)
{
	s32 antlvl;

	switch (ant) {
	case 0:
		if (ber < 650)
			antlvl = 1;
		else
			antlvl = ant;
	break;

	case 1:
		if ((ber > 700) || ((ber > 500) && (CN <= 300)))
			antlvl = 0;
		else if ((ber < 300) && (CN > 600))
			antlvl = 2;
		else
			antlvl = ant;
	break;

	case 2:
		if ((ber > 500) || ((ber > 300) && (CN <= 500)))
			antlvl = 1;
		else if ((ber < 100) && (CN >= 900))
			antlvl = 3;
		else
			antlvl = ant;
	break;

	case 3:
		if ((ber > 200) || ((ber > 100) && (CN <= 900)))
			antlvl = 2;
		//else if ((ber < 50) && (CN >= 12))
			//antlvl = 4;
		else
			antlvl = ant;
	break;

	//case 4:
		//if ((ber > 100) || (CN <= 14))
			//antlvl = 3;
		//else
			//antlvl = ant;
	//break;

	default:
		antlvl = 0;
	break;
	}

	return antlvl;
}


int tunerbb_drv_fc8180_get_fullseg_antenna(int ant
	, int ber, int cn)
{
	u32 antlvl;
	switch (ant) {
	case 0:
		if ((ber < 530) && (cn >= 1600))
			antlvl = ant = 1;
		else
			antlvl = ant;
	break;
	case 1:
		if ((ber > 550) && (cn < 1600))
			antlvl = ant = 0;
		if ((ber < 400) || (cn > 1720))
			antlvl = ant = 2;
		else
			antlvl = ant;
	break;
	case 2:
		if ((ber > 420) && (cn < 1650))
			antlvl = ant = 1;
		if ((ber < 250) || (cn > 2300))
			antlvl = ant = 3;
		else
			antlvl = ant;
	break;
	case 3:
		if ((ber > 300) && (cn < 1820))
			antlvl = ant = 2;
		else
			antlvl = ant;
	break;
	default:
		antlvl = ant = 0;
	break;
	}

	return antlvl;
}

#define DC_OFFSET 150
int tunerbb_drv_fc8180_Get_SignalInfo(struct fc8180Status_t *st
, s32 brd_type)
{
	u8 agc_reg;
	s32 res;
	u8 tmcc_data[26];
	u8 tmcc_done = 0;
	u8 mode;
	u8 sync, data;
	u8 transmission_parameter_switching;
	struct dm_st {
		u8  start;
		s8  rssi;
		u8  wscn;
		u8  reserved;
		u16 a_rxd_rsps;
		u16 a_err_rsps;
		u32 a_err_bits;
		u32 dmp_rxd_bits;
		u32 dmp_err_bits;
		u16 rxd_rsps;
		u16 err_rsps;
		u32 err_bits;
    } dm;

	res = bbm_com_bulk_read(NULL, BBM_DM_DATA, (u8 *) &dm, sizeof(dm));

	if (res)
		print_log(NULL, "mtv_signal_measure Error res : %d\n");

	dm.dmp_err_bits = dm.dmp_err_bits & 0x00ffffff;

       bbm_com_read(NULL, 0x106e, &agc_reg);

	if (dm.a_rxd_rsps)
		st->per_1seg = ((dm.a_err_rsps * 10000) / dm.a_rxd_rsps);
	else
		st->per_1seg = 10000;

	if (dm.dmp_rxd_bits)
		st->ber_1seg = ((dm.dmp_err_bits * 10000) / dm.dmp_rxd_bits);
	else
		st->ber_1seg = 10000;

	st->err_tsp_1seg = st->ErrTSP = dm.a_err_rsps;
	st->total_tsp_1seg = st->TotalTSP = dm.a_rxd_rsps;

	st->agc = agc_reg;
	bbm_com_tuner_get_rssi(NULL, &st->rssi);
	st->rssi = abs(st->rssi) * 100;
	st->cn = dm.wscn * 100;

	bbm_com_read(NULL, 0x410a, &tmcc_done);
	bbm_com_read(NULL, 0x302a, &mode);


	bbm_com_read(NULL, 0x3025, &data);
	st->lock = 0;

	if (data & 0x08) {
		sync = 2;

		bbm_com_read(NULL, 0x3026, &data);
		if (data & 0x02) {
			sync = 1;

			bbm_com_read(NULL, 0x5053, &data);
			if (data) {
				sync = 0;
				st->lock = 1;
			}
		}
	} else
		sync = 3;

	if (!(tmcc_done & 0x01)) {
		st->layerinfo_a = 0xffff;
		st->layerinfo_b = 0xffff;
		st->layerinfo_b = 0xffff;
	} else {
		bbm_com_bulk_read(NULL, 0x4110, &tmcc_data[0], 26);

		/* layerA */
		st->layerinfo_a = ((tmcc_mod_scheme((tmcc_data[3] & 0x70) >> 4))
		& 0x07) << 13;
		st->layerinfo_a |= ((tmcc_code_rate(((tmcc_data[4] & 0x03) << 1)
			+ ((tmcc_data[3] & 0x80) >> 7))) & 0x07) << 10;
		st->layerinfo_a |= ((tmcc_inter_length(
			(tmcc_data[4] & 0x1c) >> 2
			, mode)) & 0x3f) << 4 ;
		st->layerinfo_a |= (tmcc_num_segment(
			((tmcc_data[5] & 0x01) << 3)
			+ ((tmcc_data[4] & 0xe0) >> 5))) & 0x0f;


		/* layerB */
		st->layerinfo_b = ((tmcc_mod_scheme((tmcc_data[5] & 0x0e) >> 1))
		& 0x07) << 13;
		st->layerinfo_b |= ((tmcc_code_rate((tmcc_data[5] & 0x70) >> 4))
			& 0x07) << 10;
		st->layerinfo_b |= ((tmcc_inter_length(
			((tmcc_data[6] & 0x03) << 1)
			+ ((tmcc_data[5] & 0x80) >> 7), mode)) & 0x3f) << 4 ;
		st->layerinfo_b |= (tmcc_num_segment(
			(tmcc_data[6] & 0x3c) >> 2))
			& 0x0f;


		/* layerC */
		st->layerinfo_c = ((tmcc_mod_scheme(((tmcc_data[7] & 0x01) << 2)
		+ ((tmcc_data[6] & 0xc0) >> 6))) & 0x07) << 13;
		st->layerinfo_c |= ((tmcc_code_rate((tmcc_data[7] & 0x0e) >> 1))
			& 0x07) << 10;
		st->layerinfo_c |= ((tmcc_inter_length(
			(tmcc_data[7] & 0x70) >> 4
			, mode)) & 0x3f) << 4 ;
		st->layerinfo_c |= (tmcc_num_segment(
			((tmcc_data[8] & 0x07) << 1)
			+ ((tmcc_data[7] & 0x80) >> 7))) & 0x0f;
	}

	transmission_parameter_switching = ((tmcc_data[2] & 0x80) >> 5)
			+ ((tmcc_data[2] & 0x40) >> 3)
			+ ((tmcc_data[3] & 0x02) >> 1)
			+ ((tmcc_data[3] & 0x01) << 1);
	if (tmcc_done & 0x01) {
		st->tmccinfo = (tmcc_system_identification((tmcc_data[2]
			& 0x30) >> 4) << 6)
			| (((transmission_parameter_switching == 0x0f)
			? 0xf : transmission_parameter_switching + 1) << 2)
			| (((tmcc_data[3] & 0x04) >> 2) << 1)
			| ((tmcc_data[3] & 0x08) >> 3);
	} else
		st->tmccinfo = 0xff;

	st->receive_status = ((tmcc_data[3] & 0x04) >> 2) | sync;
	st->scan_status = 0;
	st->sysinfo = (((mode & 0x03) - 1) << 6)
		| (tmcc_GI_mode((mode & 0x70) >> 4) << 4);

	st->antenna_level_oneseg =
		tunerbb_drv_fc8180_get_oneseg_antenna(st->antenna_level_oneseg
		, st->ber_1seg, st->cn);
	print_log(NULL, "LOCK : %d, CN : %d, RSSI : %d, over : %d\n"
		, st->lock, st->cn, st->rssi, st->agc);
	return res;
}
