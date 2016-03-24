#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-20,0,25,40,60},
	.y		= {1845,1846,1844,1842,1832},
	.cols	= 5
};

static struct single_row_lut fcc_sf = {
	.x		= {0},
	.y		= {100},
	.cols	= 1
};

static struct sf_lut rbatt_sf = {
	.rows		= 30,
	.cols		= 5,
	.row_entries		= {-20,0,25,40,60},
	.percent	= {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,16,13,11,10,9,8,7,6,5,4,3,2,1},
	.sf		= {
				{1229,239,100,86,84},
				{1229,239,100,86,84},
				{1197,249,103,88,85},
				{1134,264,106,89,86},
				{1087,273,111,93,88},
				{1035,268,118,96,89},
				{1017,253,127,101,94},
				{1017,245,139,108,97},
				{1020,243,136,120,105},
				{1032,244,114,102,95},
				{1054,248,105,90,88},
				{1086,253,105,90,88},
				{1129,259,108,92,90},
				{1184,267,109,96,93},
				{1256,277,112,95,90},
				{1344,283,113,92,88},
				{1446,281,112,91,88},
				{1545,290,109,90,88},
				{1651,318,106,89,87},
				{1789,358,108,91,89},
				{1897,377,109,92,90},
				{2071,403,111,95,91},
				{2192,412,114,96,92},
				{2325,411,117,98,92},
				{2909,448,123,98,92},
				{3846,516,123,95,91},
				{5553,668,123,97,92},
				{8830,1008,133,102,95},
				{15842,1748,155,114,105},
				{31685,3492,238,161,127}
	}
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 31,
	.cols		= 5,
	.temp		= {-20,0,25,40,60},
	.percent	= {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,16,13,11,10,9,8,7,6,5,4,3,2,1,0},
	.ocv		= {
				{4332,4328,4324,4318,4310},
				{4244,4255,4254,4252,4246},
				{4176,4194,4195,4193,4189},
				{4108,4138,4139,4138,4134},
				{4049,4082,4086,4085,4080},
				{3986,4024,4036,4036,4032},
				{3938,3967,3992,3992,3988},
				{3899,3923,3952,3952,3948},
				{3864,3886,3908,3914,3911},
				{3832,3856,3863,3866,3865},
				{3806,3830,3831,3832,3830},
				{3786,3808,3809,3809,3807},
				{3768,3790,3792,3791,3790},
				{3752,3774,3778,3777,3775},
				{3736,3761,3768,3764,3757},
				{3722,3744,3757,3749,3736},
				{3705,3724,3737,3728,3714},
				{3688,3708,3710,3702,3691},
				{3670,3700,3688,3678,3666},
				{3654,3696,3682,3673,3662},
				{3644,3692,3680,3671,3660},
				{3632,3688,3678,3670,3658},
				{3618,3683,3676,3668,3656},
				{3602,3676,3672,3664,3652},
				{3580,3661,3666,3656,3642},
				{3550,3637,3646,3634,3616},
				{3515,3598,3600,3588,3569},
				{3468,3546,3534,3524,3506},
				{3401,3463,3444,3438,3422},
				{3282,3326,3320,3308,3292},
				{3000,3000,3000,3000,3000}
	}
};

struct bms_battery_data LGE_BL41ZH_BYD_Y30_1900mAh_BMS_data = {
	.fcc				= 1900,
	.fcc_temp_lut			= &fcc_temp,
	.fcc_sf_lut				= &fcc_sf,
	.pc_temp_ocv_lut		= &pc_temp_ocv,
	.rbatt_sf_lut			= &rbatt_sf,
	.default_rbatt_mohm	= 132
};
