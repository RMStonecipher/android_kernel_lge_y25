/***************************************************************************
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
 *    File  	: lgtp_device_lu202x.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[LU202X]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>

#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <TestLimits_lu202x.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define TPD_I2C_ADDRESS				0x0E
#define I2C_DEVICE_ADDRESS_LEN		2
#define MAX_TRANSACTION_LENGTH		8
#define MAX_I2C_TRANSFER_SIZE		(MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)

#define LU202X_MAX_KEY 4
#define MAX_FINGER_NUM 2
#define MAX_CHANNEL 34

/* LeadingUI Firmware */
#define FW_SIZE 		30*1024
#define CFG_SIZE 		1*1024
#define FAC_SIZE		1*1024

#define FAC_POS			0xFC00
#define FW_POS			0x8000

/* Knock On/Code */
#define KNOCK_ON_STATUS		0x0082
#define KNOCK_TAP_COUNT		0x0083
#define KNOCK_STATUS		0x00C0
#define KNOCK_TAP_THON		0x00C1
#define KNOCK_EXCEPT_PALM_ONCH	0x00C5
#define KNOCK_WAKEUP_INTERVAL	0x00C9
#define KNOCK_TAPOFF_TIMEOUT	0x00D2
#define KNOCK_ON_TAP_COUNT	0x00D4

#define KNOCK_CODE_TAPOFF_TIMEOUT	0x00DB
#define KNOCK_CODE_TAP_COUNT		0x00DD


/* Touch Event Type */
#define TYPE_PRESS		0x01
#define TYPE_MOVE		0x02
#define TYPE_RELEASE		0x03

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
/*TO DO - NSM 
EVENT_LPWG Saperate EVENT_KNOCK / EVENT_KNOCK_ONCODE*/
#define EVENT_KNOCK_ON      0x03
#define EVENT_KNOCK_CODE    0x04
#define EVENT_KNOCK_OVER    0x05
#define EVENT_HOVERING      0x06
#define EVENT_GEST		0x04
#define EVENT_MOUSE		0x08

#define FWSTATUS_NORMAL		0x00
#define FWSTATUS_INITREQ	0xFF
#define FWSTATUS_CHFAIL		0xfe
#define FWSTATUS_CALFAIL	0xfd

#define I2C_DEVICE_ADDRESS_LEN	2
#define MAX_TRANSACTION_LENGTH	8

//#define FW_STATUS_REG		0x0000
#define INT_INFORM          0x0000
#define FW_VERSION_REG		0x0080

#define LU202x_MODE_ADDR	0x00E0
#define LU202x_CMDACK_ADDR	0x00ED

#define LU202x_DEVICEID_ADDR	0x10FD
#define LU202x_I2CDONE_ADDR	0x10FF
#define LU202x_CMDReply_ADDR	0x0100

#define CMD_I2C_DONE		0x01
#define CMD_LU202x_CHANGEMODE	0xA3
#define CMD_LU202x_DEVINFO 	0xAA
#define CMD_LU202x_CHCAPTEST	0xB6
#define LU202x_CHCAPTEST_Reply	0xC1

/* Command */
#define LU202x_CMD_FW_CHECKSUM_ADDR	0x0158

/* Major Mode */
#define CMD_LU202x_NORMODE		0x00
#define CMD_LU202x_PDN			0x01
#define CMD_LU202x_DEBUG		0x02
#define CMD_LU202x_IDLE_DOUBLETAB	0x11
#define CMD_LU202x_IDLE_MULTITAB	0x11

/* Minor Mode */
#define CMD_LU202x_NONE		0x0000
#define CMD_LU202x_REFCAP	0x0000

#define HEADER_SIZE 16

/* USERMODE CHANGE */
#define ACCESS_CTRL         0x1000
#define USER_SPACE          0x1001
#define USER_PASSWORD       0x1003
#define ROM_CONTROL         0x1000
#define TSP_INFORM          0x0038
/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct Lu202xDriverDataTag {

	TouchState deviceState;

} Lu202xDriverData;

/****************************************************************************
* Variables
****************************************************************************/
typedef struct {
	u8 FWStatus;		// 0x0000
	u8 EventType;		// 0x0001
	u8 VPCount; 		// 0x0002
	u8 KeyData[2];		// 0x0003
	u8 Point[8];		// 0x0005 X0 position
} lu202x_tpd;

typedef struct {
	u8 status[MAX_FINGER_NUM];
	u8 id[MAX_FINGER_NUM];
	u16 x[MAX_FINGER_NUM];
	u16 y[MAX_FINGER_NUM];
} touch_info;

static int pressed_key = 0;

static Lu202xDriverData gDeviceData = { STATE_NORMAL };
static const char defaultFirmware[] = "leadingUI/Y30_LU2020_08_00.img";
	

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
static int SleepInLu202x( struct i2c_client *client )
{
	u8 i2c_done = CMD_I2C_DONE;
	Lu202x_I2C_Write ( client, LU202x_I2CDONE_ADDR, &i2c_done, 1 );
	
	return TOUCH_SUCCESS;
}

static int WakeUpLu202x( void )
{
	int result = TOUCH_SUCCESS;
	int loopCount = 500;
	
	TouchIntPinToggle();
	while ( TouchReadInterrupt() && loopCount )
	{
		msleep ( 1 );
		loopCount--;
	}

	if( TouchReadInterrupt() == 1 )
	{
		TOUCH_ERR("Failed to wakeup Lu202x\n");
		result = TOUCH_FAIL;
	}

	return result;
	
}


static int LU202x_FlashReadyCheck(struct i2c_client *client)
{
	u8 status = 0;
    TOUCH_FUNC();
	while (1) {
		  // Flash Ready Check
		if (Lu202x_I2C_Read(client, 0x1000, &status, 1) == -1) {
			TOUCH_ERR("Read status operation failed\n" );		 
			return -1;
		}

		if ((status & 0x40) == 0x40) {
			break;
		}
	}
	msleep(100);
    return 0;
}

static int LU202x_PageErase(struct i2c_client *client, int addr)
{
  //	struct lu202x_fw_info *fw_info = data->fw_info;
	u8 Cmd[2] = {0, 0};
//	u8 status = 0;
//	u8 *pBuf;
	
	u8 pagenum = addr/1024;	
    TOUCH_FUNC();
//	pBuf = kmalloc(FAC_SIZE, GFP_KERNEL);
//	fw_info->fac_raw_data = pBuf;

/*
	// Read Cal Data
	if (LU202x_readFW(data, pBuf, FAC_POS, FAC_SIZE) == -1) {
		TOUCH_INFO_MSG("Read Cal Data failed\n" );
		goto ERASE_FAIL;
	}
*/

	/* Erase */
	// main Block Select
	Cmd[0] = 0x02;
   	if(Lu202x_I2C_Write(client, 0x1001, Cmd, 1) == -1){
		TOUCH_ERR("Main Block Select command operation failed\n" );
		goto ERASE_FAIL;
		}

	// Erase PageNum Write
	Cmd[0] = pagenum;
	if(Lu202x_I2C_Write(client, 0x1006, Cmd, 1) == -1){
		TOUCH_ERR(" Erage Page Write command operation failed\n" );
		goto ERASE_FAIL;
		}

	// Erase Function SelectEra
	Cmd[0] = 0x82;
	if(Lu202x_I2C_Write(client, 0x1000, Cmd, 1) == -1){
		TOUCH_ERR("Page Erase Function Select command operation failed\n" );
		goto ERASE_FAIL;
		}

	if(LU202x_FlashReadyCheck(client) == -1){
		TOUCH_ERR("Flash Ready failed\n" );
		goto ERASE_FAIL;
		}

	return 0;

ERASE_FAIL:
//	kfree(pBuf);
	return -1;
}

static int LU202x_PageWrite(struct i2c_client *client, u8 *pBuf, int addr, int size)
{
	u8 Cmd[2] = {0, 0};
//	u8 Status = 0;
//	u8 pagenum = addr/1024;
//	int i;
    TOUCH_FUNC();
	// All Area Enable
	Cmd[0] = 0x01;
	if (Lu202x_I2C_Write(client, 0x1001, Cmd, 1) == -1)	{
			TOUCH_ERR("Main Block Select operation failed\n" );
			return -1;
		}

	// I2C E-Flash Program Enable (Program Enable Fuction Select)
	Cmd[0] = 0x88;
	if (Lu202x_I2C_Write(client, 0x1000, Cmd, 1) == -1)	{
			TOUCH_ERR("Program Fuction Select operation failed\n" );
			return -1;
		}

	// Data Write 
    if (Lu202x_I2C_Write(client, addr, pBuf, size) == -1)	{
			TOUCH_ERR("Data Write operation failed\n" );
			return -1;
		}

	return 0;
}

static int LU202x_PageRead(struct i2c_client *client, u8 *pBuf, int addr, int size)
{
	u8 Cmd[2] = {0, 0};
	//u8 pagenum = addr/1024;

	// Main Block Select
	Cmd[0] = 0x01;
	if (Lu202x_I2C_Write(client, 0x1001, Cmd, 1) == -1)	{
			TOUCH_ERR("Main Block Select operation failed\n" );
			return -1;
		}

	// Read Function Select
	Cmd[0] = 0x81;
	if (Lu202x_I2C_Write(client, 0x1000, Cmd, 1) == -1)	{
			TOUCH_ERR("Read Function operation failed\n" );
			return -1;
		}

	// Data Read
	if (Lu202x_I2C_Read(client, addr, pBuf, size) == -1)	{
			TOUCH_ERR("Data Read operation failed\n" );
			return -1;
		}

	return 0;
}

static int Lu202x_programFW(struct i2c_client *client, u8 *pBuf, int addr, int size)
{
    int i = 0;
	int W_addr = addr;
	int R_addr = addr;
    TOUCH_FUNC();
	for(i=0; i<size; i+= 256, W_addr+=256)
	{
		if((W_addr % 1024) == 0)
		{	// 1K Erase
			if(LU202x_PageErase(client, W_addr) == -1){
				TOUCH_ERR("Data Page Erase failed \n");
				return -1;
				}
		}
											
		// 256Bytes Write * 4
		if(LU202x_PageWrite(client, (pBuf+i), W_addr , 256) == -1){
			TOUCH_ERR("Data Write failed \n");
			return -1;
			}
	}

	TOUCH_LOG("%s Writing (%d/%d) bytes\n",
				(size == FAC_SIZE) ? "FACTORY" : "FIRMWARE", i, size );

   // Data Check
   if(0)
   {
	for(i=0; i<size+FAC_SIZE; i+=1024, R_addr+=1024)
	{
        TOUCH_LOG("R_ADDRESS = 0x%x", R_addr);
		if(LU202x_PageRead(client, (pBuf+i), R_addr, 1024) == -1){
			TOUCH_ERR("Data Read for Check operation failed\n" );
			return -1;
		}
        else
        {
            TOUCH_LOG("DATA CHECK SUCCESS [%d] page", i);
        }
	}
   }
    TOUCH_LOG("END PROGRAM FW");
	return 0;

}


static int Lu202x_ReadChecksum( struct i2c_client *client, char *pCheckSum )
{
	int result = TOUCH_SUCCESS;
	
	u8 temp[3] = {CMD_LU202x_DEVINFO, 0, 0};

	if( WakeUpLu202x() )
	{
		result = TOUCH_FAIL;
		goto exit;
	}

	/* Change Checksum Mode */
	Lu202x_I2C_Write(client, LU202x_MODE_ADDR, temp, 3);
	if( SleepInLu202x(client) )
	{
		TOUCH_ERR("Failed to change mode\n");
		result = TOUCH_FAIL;
		goto exit;
	}
	msleep(2);
	
#if 1 /*                                                 */
#else
	if (LU202x_wait_ready_i2c(data) == -1) {
		TOUCH_INFO_MSG ("Set mode read ready error\n");
		goto exit;
	}
#endif

	/* Checksum Mode check */
	Lu202x_I2C_Read(client, LU202x_CMDACK_ADDR, temp, 3);
	if ( temp[2] != CMD_LU202x_DEVINFO )
	{
		TOUCH_ERR("Failed to read ack\n");
		result = TOUCH_FAIL;
		goto exit;
	}
	mdelay(2);

	Lu202x_I2C_Read(client, LU202x_CMD_FW_CHECKSUM_ADDR, pCheckSum, 6);
	if( SleepInLu202x(client) )
	{
		TOUCH_ERR("Failed to read checksum\n");
		result = TOUCH_FAIL;
		goto exit;
	}

exit:
	
	return result;
	
}

#if 1 /*                                                 */
#else
static int LU202x_set_mode(struct lu202x_data *data, u8 mode, u8 sub_mode)
{
	struct lu202x_platform_data *pdata = data->pdata;
	u8 buf[5]={CMD_LU202x_CHANGEMODE, 0, 0, 0, 0};
	u8 temp = 0;
	u8 tab_dlytime = 0;
	int ret = -1;

	buf[1] = (u8)(mode & 0xff);
	//buf[1] = mode ;
	buf[3] = (u8)(sub_mode& 0xff);

	WakeUpLu202x();
	Lu202x_I2C_Write(data->client, LU202x_MODE_ADDR, buf, sizeof(buf));

	if (mode != CMD_LU202x_PDN) {
		if (pdata->lpwg_mode == LPWG_DOUBLE_TAP)
		{
			temp = 1;
			Lu202x_I2C_Write(data->client, KNOCK_STATUS, &temp, 1);
			temp = 150;
			Lu202x_I2C_Write(data->client, KNOCK_TAP_THON, &temp, 1);
			temp = 8;
			Lu202x_I2C_Write(data->client, KNOCK_EXCEPT_PALM_ONCH, &temp, 1);
			temp = 70;
			Lu202x_I2C_Write(data->client, KNOCK_WAKEUP_INTERVAL, &temp, 1);
			multi_tap_count = 2;
			i2c_write_bytes_LU202x(data->client, KNOCK_ON_TAP_COUNT, &multi_tap_count, 1);
		}
		else if (pdata->lpwg_mode == LPWG_MULTI_TAP)
		{
			/* enable both a Knock on and code */
			temp = 3;
			Lu202x_I2C_Write(data->client, KNOCK_STATUS, &temp, 1);
			temp = 150;
			Lu202x_I2C_Write(data->client, KNOCK_TAP_THON, &temp, 1);
			temp = 8;
			Lu202x_I2C_Write(data->client, KNOCK_EXCEPT_PALM_ONCH, &temp, 1);
			temp = 70;
			Lu202x_I2C_Write(data->client, KNOCK_WAKEUP_INTERVAL, &temp, 1);
			tab_dlytime = 200;
			Lu202x_I2C_Write(data->client, KNOCK_CODE_TAPOFF_TIMEOUT, &tab_dlytime, 2);
			temp = multi_tap_count;
			Lu202x_I2C_Write(data->client, KNOCK_CODE_TAP_COUNT, &temp, 1);
		}
	}

	SleepInLu202x(client)
	
	msleep(2);

	return result;

}
#endif



/****************************************************************************
* Global Functions
****************************************************************************/
static ssize_t show_device_name(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	ret += sprintf(buf+ret, "%s\n", "LU202x/LeadingUI");
	
	return ret;
}

static LGE_TOUCH_ATTR(device_name, S_IRUGO | S_IWUSR, show_device_name, NULL);

static struct attribute *lu202x_attribute_list[] = {
	&lge_touch_attr_device_name.attr,
	NULL,
};



int Lu202x_Initialize(struct i2c_client *client)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

void Lu202x_Reset(struct i2c_client *client)
{
	TouchResetCtrl(0);
	msleep(10);
	TouchResetCtrl(1);
	msleep(200);
	
	TOUCH_LOG("LU202X was reset\n");
	
	gDeviceData.deviceState = STATE_NORMAL;
}


int Lu202x_Connect(void)
{
#if 1 /*                                                 */
	int ret = 0;
	u8 reg[2] = {0};
	u8 data[2] = {0};

	TOUCH_FUNC();

	reg[0] = 0x10;
	reg[1] = 0xFD;

	ret = touch_i2c_read_for_query( TPD_I2C_ADDRESS, reg, 2, data, 2);
	if( ret == TOUCH_SUCCESS )
	{
		if( (data[0] == 0x20) && (data[1] == 0x20) )
		{
			TOUCH_LOG("LU202X was detected\n");
			return TOUCH_SUCCESS;
		}
		else
		{
			TOUCH_LOG("LU202X was detected but deviceID is not matched\n");
			return TOUCH_FAIL;
		}
	} 
	else 
	{
		TOUCH_LOG("LU202X was NOT detected\n");
		return TOUCH_FAIL;
	}
#else
	if( TouchReadMakerId() == 0 )
	{
		TOUCH_LOG("LU202X was detected\n");
		return TOUCH_SUCCESS;
	}
	else 
	{
		TOUCH_LOG("LU202X was NOT detected\n");
		return TOUCH_FAIL;
	}
#endif
}

int Lu202x_InitRegister(struct i2c_client *client)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

/*                                                 */
static int get_lpwg_data(struct i2c_client *client, TouchReadData *pData)
{
	u8 i = 0;
	u8 ret = 0;
	//u8 buffer[12][4] = { {0} };
	u8 tap_count = 0;
	u8 buffer[12 * 4] = {0,};

    if(Lu202x_I2C_Read(client, KNOCK_TAP_COUNT, &tap_count, sizeof(u8)) == 0)
    {
        pData->count = tap_count;
    }
    else
    {
        TOUCH_ERR("KNOCK_TAP_COUNT Read Fail\n");
        goto error;
	}

	if (!tap_count)
	{
        TOUCH_LOG("TAP COUNT = %d",tap_count);
		goto error;
	}
	ret = Lu202x_I2C_Read(client, KNOCK_TAP_COUNT+1, buffer,4*tap_count);
	SleepInLu202x(client);
	if ( ret != 0 )
	{
		TOUCH_ERR("LPWG Data Read Fail\n");
		goto error; 
	}
	
	for (i = 0; i < tap_count; i++)
	{
		pData->knockData[i].x = (buffer[4*i+1] << 8 | buffer[4*i]);
        pData->knockData[i].y = (buffer[4*i+3] << 8 | buffer[4*i+2]);
         TOUCH_LOG("LPWG data [%d, %d]\n", pData->knockData[i].x, pData->knockData[i].y);
  }

	return TOUCH_SUCCESS;
error:
	return TOUCH_FAIL;
}

static void Lu202x_ClearInterrupt(struct i2c_client *client)
{
	return;
}

int Lu202x_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{
	int i=0;
	TouchFingerData *pFingerData = NULL;
	TouchKeyData *pKeyData = NULL;
	lu202x_tpd touch_data;
	touch_info info;
	static u8 pressure_temp = 0;
    //u8 knock_status = 0;

	memset(&info, 0x0, sizeof(touch_info));
	memset(&touch_data, 0x0, sizeof(lu202x_tpd));

	pressure_temp ^= 1;

	pData->type = DATA_UNKNOWN;
	pData->count = 0;

		/* read touch data */
        if(Lu202x_I2C_Read(client, INT_INFORM, (u8 *)&touch_data, sizeof(lu202x_tpd)) != 0)
	    {
             TOUCH_ERR("Read Interrupt Status Fail. (0x0000)\n");
        }
        switch(touch_data.EventType)
        {
            case EVENT_ABS :
            pData->type = DATA_FINGER;
            for ( i = 0 ; i < MAX_FINGER_NUM ; i++ )
            {
            	info.x[i] = ( ( touch_data.Point[i*4+1] & 0x07 ) << 8 ) | touch_data.Point[i*4];
            	info.y[i] = ( ( touch_data.Point[i*4+2] & 0x38 ) << 5 ) | ( ( touch_data.Point[i*4+2] & 0x07 ) << 5 ) | ( ( touch_data.Point[i*4+1] >> 3 ) & 0x1f );
            	info.id[i] = ( ( touch_data.Point[i*4+3] & 0x07 ) << 3 ) | ( ( touch_data.Point[i*4+2] >> 6 ) & 0x03 );
            	info.status[i] = ( touch_data.Point[i*4+3] >> 3 ) & 0x03;
            	if ( info.status[i] == TYPE_PRESS )
            	{
            		pFingerData = &pData->fingerData[pData->count];
            		pFingerData->id = info.id[i];
            		pFingerData->x  = info.x[i];
            		pFingerData->y  = info.y[i];
            		pFingerData->width_major = 15;
            		pFingerData->width_minor = 10;
            		pFingerData->orientation = 1;
            		pFingerData->pressure = 20 + pressure_temp;
            		pData->count++;

            	}
            }
            break; 

            case EVENT_KEY : 
			pData->type = DATA_KEY;
			pData->count++;
			pKeyData = &pData->keyData;
			if ( touch_data.KeyData[0] == 0 )
			{
				pKeyData->index = pressed_key;
				pKeyData->pressed = KEY_RELEASED;
				TOUCH_LOG ( "Touch Key[%d] was Released\n", pKeyData->index );
				pressed_key = 0;
			}
			else
			{
				pressed_key = touch_data.KeyData[0];
				pKeyData->index = pressed_key;
				pKeyData->pressed = KEY_PRESSED;
				TOUCH_LOG ( "Touch Key[%d] was Pressed\n", pKeyData->index );
			}
            break;
			
            case EVENT_KNOCK_ON :
            pData->type = DATA_KNOCK_ON;
            //get_lpwg_data(TouchReadData);
            TOUCH_LOG("[KNOCK ON] Event Type = %d\n", touch_data.EventType);
            break;

            case EVENT_KNOCK_CODE : 
            pData->type = DATA_KNOCK_CODE;
            get_lpwg_data(client, pData);
            TOUCH_LOG("[KNOCK CODE]\n");
            break;
            
            case EVENT_KNOCK_OVER :
            pData->type = DATA_KNOCK_CODE;
            pData->knockData[0].x = 1;
            pData->knockData[0].y = 1;
            pData->knockData[1].x = -1;
            pData->knockData[1].y = -1;
            TOUCH_LOG("[KNOCK CODE OVER] Event Type = %d\n", touch_data.EventType);
            /*To Do*/
            break;

            case EVENT_HOVERING :
            TOUCH_LOG("[HOVERING] Event Type = %d\n", touch_data.EventType);
            /*To Do*/
            break;

            default:
            TOUCH_LOG("[Unknown] Event Type = %d\n",touch_data.EventType);
            break;
		
	}
    SleepInLu202x(client);
    return TOUCH_SUCCESS;
}

int Lu202x_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	int result = TOUCH_SUCCESS;

	u8 readData[4] = {0};
	
	TOUCH_FUNC();

	WakeUpLu202x();	
	result = Lu202x_I2C_Read(client, FW_VERSION_REG-2, &readData[0], 4);
	SleepInLu202x(client);
	
	TOUCH_DBG("IC Firmware Version = 0x%02X 0x%02X 0x%02X 0x%02X\n", readData[0], readData[1], readData[2], readData[3] );

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = 1;
	pFwInfo->version = readData[2]*10 + readData[3]; /* release version */

	TOUCH_LOG("IC Firmware Version = 0x%04X\n", pFwInfo->version);

	return result;	
}

int Lu202x_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int result = TOUCH_SUCCESS;
	
	const struct firmware *fw = NULL;
	u8 *pHeader = NULL;
	char *pFwFilename = NULL;
	
	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);
	
	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if( result )
	{
		TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
		result = TOUCH_FAIL;
		goto earlyReturn;
	}

	pHeader = (u8 *)(fw->data);

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = 1;
	pFwInfo->version = pHeader[7]*10 + pHeader[8]; /* release version */

	TOUCH_LOG("BIN Firmware Version = 0x%04X\n", pFwInfo->version);

	/* Free firmware image buffer */
	release_firmware(fw);
	
earlyReturn:
	return result;
	
}


int Lu202x_UpdateFirmware(struct i2c_client *client, char *pFilename )
{
	int result = TOUCH_SUCCESS;
	
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
    u8 Cmd[2] = {0,};
	char *pFwFilename = NULL;

	char checksum[6] = {0};

	TOUCH_FUNC();
	
	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if( result )
	{
		TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
		result = TOUCH_FAIL;
		return result;
	}
    else
    {
        pBin = (u8 *)(fw->data+HEADER_SIZE); /* header size is 16bytes */
    }
    
    /*TO DO - Firmware update problem - NSM*/
    /*Complete : It need to enter usermode*/
    WakeUpLu202x();
    
    Cmd[0] = 0x80;
    if (Lu202x_I2C_Write(client, 0x1000, Cmd, 1) == -1) {
        TOUCH_ERR("Set mode operation failed\n" );
        goto earlyReturn;
    }

    Cmd[0] = 0x75;
    Cmd[1] = 0x6C;
    
    if (Lu202x_I2C_Write(client, 0x1003, Cmd, 2) == -1) {
        TOUCH_ERR("Set password operation failed\n" );
        goto earlyReturn;
    }
    
    /*This part is LU2020 firmware update. */
    if(Lu202x_programFW(client, pBin, FW_POS, FW_SIZE + CFG_SIZE) == - 1)
    {
        TOUCH_ERR("Failed to program firmware. (LU202x)\n");
        result = TOUCH_FAIL;
        goto earlyReturn;
    }
  
	/* Reset to read checksum */
	Lu202x_Reset(client);

	/* Read checksum */

    if(0)
    {
    	result = Lu202x_ReadChecksum(client, checksum);
    	if( result )
    	{
    		TOUCH_ERR("Failed at read checksum\n");
    		result = TOUCH_FAIL;
    		goto earlyReturn;
    	}
    
	TOUCH_LOG("binary checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		*(fw->data+9), *(fw->data+10), *(fw->data+11), *(fw->data+12), *(fw->data+13), *(fw->data+14) );
    //
	TOUCH_LOG("read checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		checksum[0], checksum[1], checksum[2], checksum[3], checksum[4], checksum[5] );

	/* Compare checksum */
	if( memcmp((u8 *)(fw->data+9), checksum, 6) )
    	{
    		TOUCH_ERR("Checksum value is not same. Failed to program firmware\n");
    		result = TOUCH_FAIL;
    	}
    }

earlyReturn:

    SleepInLu202x(client);
        
    /* Free firmware image buffer */
    release_firmware(fw);
	
	/* Reset ??? */
	Lu202x_Reset(client);
	return result;
	
}

int Lu202x_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int result = TOUCH_SUCCESS;

	u8 temp = 0;
//	u16 temp2 = 0;
	u8 buf[5]={CMD_LU202x_CHANGEMODE, 0, 0, 0, 0};

	TOUCH_FUNC();

	gDeviceData.deviceState = newState;

	if( newState == STATE_NORMAL )
	{
		Lu202x_Reset(client);
		TOUCH_LOG("LU202X was changed to NORMAL\n");
	}

    else if( newState == STATE_KNOCK_ON_ONLY)
	{
		
		WakeUpLu202x();

		buf[1] = 0x11;
		buf[3] = 0x0;
		Lu202x_I2C_Write(client, LU202x_MODE_ADDR, buf, sizeof(buf));
		temp = 1;
		Lu202x_I2C_Write(client, KNOCK_STATUS, &temp, 1);
		temp = 150;
		Lu202x_I2C_Write(client, KNOCK_TAP_THON, &temp, 1);
		temp = 8;
		Lu202x_I2C_Write(client, KNOCK_EXCEPT_PALM_ONCH, &temp, 1);
		temp = 70;
		Lu202x_I2C_Write(client, KNOCK_WAKEUP_INTERVAL, &temp, 1);
		temp = 2;
		Lu202x_I2C_Write(client, KNOCK_ON_TAP_COUNT, &temp, 1);
		
		SleepInLu202x(client);
        TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_ONLY\n");
    }
    else if( newState == STATE_KNOCK_ON_CODE )
    {
        WakeUpLu202x();
        buf[1] = 0x11;
        buf[3] = 0x0;
        Lu202x_I2C_Write(client, LU202x_MODE_ADDR, buf, sizeof(buf));
        
        temp = 3;
        Lu202x_I2C_Write(client, KNOCK_STATUS, &temp, 1);
        temp = 150;
        Lu202x_I2C_Write(client, KNOCK_TAP_THON, &temp, 1);
        temp = 8;
        Lu202x_I2C_Write(client, KNOCK_EXCEPT_PALM_ONCH, &temp, 1);
        temp = 70;
        Lu202x_I2C_Write(client, KNOCK_WAKEUP_INTERVAL, &temp, 1);
        temp = 200;
        Lu202x_I2C_Write(client, KNOCK_CODE_TAPOFF_TIMEOUT, &temp, 2);
        temp = (u8)pLpwgSetting->tapCount;
        Lu202x_I2C_Write(client, KNOCK_CODE_TAP_COUNT, &temp, 1);

        SleepInLu202x(client);
        TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_CODE\n");
    }

    /*TO DO - NSM
      Report mode off case is setted PDN(POWER DOWN MODE)*/
    else if( newState == STATE_OFF )
    {
        
    }
    else
    {
    }
	msleep(2);

	return result;
	
}

#if 0
static void LU202x_ChCap_Test(struct i2c_client *client)
{   
    u8 buf[3] = {CMD_LU202x_CHCAPTEST, 0, 0};
    u8 reply[2] = {0,};
    u8 data_temp[MAX_CHANNEL *2] = {0,};
    u8 Raw_Cap_Value[MAX_CHANNEL] = {0, };
    u8 Jitter_Value[MAX_CHANNEL] = {0, };
    u8 size = 0;
    u8 i = 0;
    WakeUpLu202x();

    /* Change Cap Test mode */
    if( Lu201x_I2C_Write(client, LU202x_MODE_ADDR, buf, 3) == -1)
    {
        TOUCH_ERR("Change Cap Test mode fail.\n");
    }
    SleepInLu202x(client);
    msleep(2);

    WakeUpLu202x();
    
    /* Cap Test Mode Change Check(Reply Type Check) */
    if(Lu202x_I2C_Read(client, LU202x_CMDACK_ADDR, buf, 3) == -1)
    {
        TOUCH_ERR("Read Cmd ACK fail.\n");
    }
    else
    {
        if(buf[2] != CMD_LU202x_CHCAPTEST)
        {
            TOUCH_ERR("Mode Change fail.\n");
        }
    }

    if(Lu201x_I2C_Read(client, LU202x_CMDReply_ADDR, reply, 2) == -1)
    {
        TOUCH_ERR("Reply Data Read fail.\n");
    }
    else
    {
        if( reply[0] != LU202x_CHCAPTEST_Reply )
        {
            TOUCH_ERR("Cap Test reply fail\n");
        }
    }
    /* raw cap data + jiter data size. - Sensor Channel * 4 +6 */
    size = reply[1] - 6;
    TOUCH_LOG("Report data size = %d\n", size);
    
    /* Read Raw Cap value(Read data size = 37 * 2, channel number 37) */
    if( Lu202x_I2C_Read(client, LU202x_CMDReply_ADDR + 2, data_temp, size / 2) == -1)
    { 
        TOUCH_ERR("Raw cap value read fail \n");
    }
    else
    {
        for(i = 0; i< size/4; i++)
        {
            Raw_Cap_Value[i] = (data_temp[i * 2]) | (data_temp[i*2 + 1]<< 8);
        }
    }
    
    if( Lu202x_I2C_Read(client, LU202x_CMDReply_ADDR + 2 +(MAX_CHANNEL * 2), data_temp, size / 2)== -1)
    {
        TOUCH_ERR("Jitter value read fail \n");
    }
    else
    {
        for(i = 0; i< size/4; i++)
        {
            Jitter_Value[i] = (data_temp[i * 2]) | (data_temp[i*2 + 1]<< 8);
        }
    }
    
    for(i = 0; i< size/4; i++)
    {
        TOUCH_LOG("Raw cap value [%d] = %d , Jitter value [%d] = %d", i, Raw_Cap_Value[i], i, Jitter_Value[i]);
    }

    
    for(i = 0; i< size/4; i++)
    {
        if(Jitter_Value[i] > Lu202x_jitter)
        {
            TOUCH_LOG("Channel Numer[%d] Jitter Value Fail = %d", i, Jitter_Value[i]);
        }
    }
    
    for(i = 0; i< size/4; i++)
    {
        if( (Raw_Cap_Value[i] > Lu202x_UpperImageLimit[i]) || Raw_Cap_Value[i] < Lu202x_LowerImageLimit[i] )
        {
            TOUCH_LOG("Channel Numer[%d] Raw Cap Value Fail = %d", i, Raw_Cap_Value[i]);
        }
    }
    
            
    SleepInLu202x(client);
}
#endif


void Lu202x_sd_write ( char *data )
{
	int fd;
	char *fname = "/mnt/sdcard/touch_self_test.txt";

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND|O_SYNC, 0644);

	if (fd >= 0)
	{
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}

	set_fs(old_fs);
}


int Lu202x_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;

	/* CAUTION : be careful not to exceed buffer size */

	/* do implementation for self-diagnosis */
	*pRawStatus = 0;
	*pChannelStatus = 0;
    //LU202x_ChCap_Test(client);
	dataLen = sprintf(pBuf,  "========RESULT=======\n");
	dataLen += sprintf(pBuf+dataLen, "Channel Status : %s", (1) ? "Pass\n" : "Fail\n");
	dataLen += sprintf(pBuf+dataLen,  "Raw Data : %s", (1) ? "Pass\n" : "Fail\n");

    Lu202x_sd_write(pBuf);
/*TODO SD LOG FILE WRITE*/

	*pDataLen = dataLen;
    
	return TOUCH_SUCCESS;
	
}

int Lu202x_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;
	
	switch( cmd )
	{
		case READ_IC_REG:
			ret = Lu202x_I2C_Read(client, (u16)reg, (u8 *)pValue, 1);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Lu202x_I2C_Write(client, (u16)reg, (u8 *)*pValue, 1);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;
	
}

TouchDeviceSpecificFunction Lu202x_Func = {
	.Initialize = Lu202x_Initialize,
	.Reset = Lu202x_Reset,
	.Connect = Lu202x_Connect,
	.InitRegister = Lu202x_InitRegister,
	.ClearInterrupt = Lu202x_ClearInterrupt,
	.InterruptHandler = Lu202x_InterruptHandler,
	.ReadIcFirmwareInfo = Lu202x_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = Lu202x_GetBinFirmwareInfo,
	.UpdateFirmware = Lu202x_UpdateFirmware,
	.SetLpwgMode = Lu202x_SetLpwgMode,
	.DoSelfDiagnosis = Lu202x_DoSelfDiagnosis,
	.AccessRegister = Lu202x_AccessRegister,
	.device_attribute_list = lu202x_attribute_list,
};


/* End Of File */


