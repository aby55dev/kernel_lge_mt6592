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
 *    File  	: lgtp_device_s3320.c
 *    Author(s)   : Branden You < branden.you@lge.com >
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/lgtp_common.h>

#include <linux/input/lgtp_common_driver.h>
#include <linux/input/lgtp_platform_api.h>
#include <linux/input/lgtp_model_config.h>
#include <linux/input/TD4191/touch_synaptics.h>
#include <mach/mt_wdt.h>
#include <mach/wd_api.h>


//#include "./DS5/RefCode_F54.h"


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose				See page
 * $01		RMI Device Control		45
 * $1A		0-D capacitive button sensors	61
 * $05		Image Reporting			68
 * $07		Image Reporting			75
 * $08		BIST				82
 * $09		BIST				87
 * $11		2-D TouchPad sensors		93
 * $19		0-D capacitive button sensors	141
 * $30		GPIO/LEDs			148
 * $31		LEDs				162
 * $34		Flash Memory Management		163
 * $36		Auxiliary ADC			174
 * $54		Test Reporting			176
 */
struct i2c_client *ds4_i2c_client;
static char power_state;
static int get_ic_info(struct synaptics_ts_data *ts);
error_type read_page_description_table(struct i2c_client *client);
extern int mtk_wdt_enable ( enum wk_wdt_en en );
extern bool isRecovery;

int f54_window_crack_check_mode;
int f54_window_crack;
int after_crack_check;
bool touch_irq_mask = 1;
u8 int_mask_cust;

/*static int ts_suspend = 0;
int thermal_status = 0;
extern int touch_thermal_mode;*/
u8 default_finger_amplitude[2] = {0, };
extern int touch_ta_status;
struct synaptics_ts_f12_info {
	bool ctrl_reg_is_present[32];
	bool data_reg_is_present[16];
	u8 ctrl_reg_addr[32];
	u8 data_reg_addr[16];
};

static struct synaptics_ts_f12_info f12_info;
static bool need_scan_pdt = true;

#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x12
#define FLASH_MEMORY_MANAGEMENT			0x34
#define ANALOG_CONTROL				0x54
#define SENSOR_CONTROL				0x55
#define F1A_EXIST                               0x1A

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
/* RMI_DEVICE_CONTROL */
/* Manufacturer ID */
#define MANUFACTURER_ID_REG		(ts->common_fc.dsc.query_base)
/* CUSTOMER_FAMILY QUERY */
#define CUSTOMER_FAMILY_REG		(ts->common_fc.dsc.query_base + 2)
/* FW revision */
#define FW_REVISION_REG			(ts->common_fc.dsc.query_base + 3)
/* Product ID */
#define PRODUCT_ID_REG			(ts->common_fc.dsc.query_base + 11)
#define DEVICE_COMMAND_REG		(ts->common_fc.dsc.command_base)

/* Device Control */
#define DEVICE_CONTROL_REG		(ts->common_fc.dsc.control_base)
/* sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_NORMAL_OP	0x00
/* sleep mode : go to sleep */
#define DEVICE_CONTROL_SLEEP		0x01
/* sleep mode : go to sleep. no-recalibration */
#define DEVICE_CONTROL_SLEEP_NO_RECAL	0x02
#define DEVICE_CONTROL_NOSLEEP		0x04
#define DEVICE_CHARGER_CONNECTED	0x20
#define DEVICE_CONTROL_CONFIGURED	0x80

/* Interrupt Enable 0 */
#define INTERRUPT_ENABLE_REG		(ts->common_fc.dsc.control_base + 1)
/* Doze Interval : unit 10ms */
#define DOZE_INTERVAL_REG               (ts->common_fc.dsc.control_base + 2)
#define DOZE_WAKEUP_THRESHOLD_REG       (ts->common_fc.dsc.control_base + 3)

/* Device Status */
#define DEVICE_STATUS_REG		(ts->common_fc.dsc.data_base)
#define DEVICE_FAILURE_MASK		0x03
#define DEVICE_CRC_ERROR_MASK		0x04
#define DEVICE_STATUS_FLASH_PROG	0x40
#define DEVICE_STATUS_UNCONFIGURED	0x80

/* Interrupt Status */
#define INTERRUPT_STATUS_REG		(ts->common_fc.dsc.data_base + 1)
#define INTERRUPT_MASK_FLASH		0x01
#define INTERRUPT_MASK_ABS0		0x04
#define INTERRUPT_MASK_BUTTON		0x10
#define INTERRUPT_MASK_CUSTOM		0x40

/* TOUCHPAD_SENSORS */
#define FINGER_COMMAND_REG		(ts->finger_fc.dsc.command_base)
#define MOTION_SUPPRESSION		(ts->finger_fc.dsc.control_base + 5)
/* f12_info.ctrl_reg_addr[20] */
#define GLOVED_FINGER_MASK		0x20

/* Finger State */
#define OBJECT_TYPE_AND_STATUS_REG	(ts->finger_fc.dsc.data_base)
#define OBJECT_ATTENTION_REG		(ts->finger_fc.dsc.data_base + 2)
/* Finger Data Register */
#define FINGER_DATA_REG_START		(ts->finger_fc.dsc.data_base)
#define REG_OBJECT_TYPE_AND_STATUS	0
#define REG_X_LSB			1
#define REG_X_MSB			2
#define REG_Y_LSB			3
#define REG_Y_MSB			4
#define REG_Z				5
#define REG_WX				6
#define REG_WY				7

#define MAXIMUM_XY_COORDINATE_REG	(ts->finger_fc.dsc.control_base)

/* ANALOG_CONTROL */
#define ANALOG_COMMAND_REG		(ts->analog_fc.dsc.command_base)
#define ANALOG_CONTROL_REG		(ts->analog_fc.dsc.control_base)
#define THERMAL_UPDATE_INTERVAL_REG     0x2F      /* 1-page */

/* FLASH_MEMORY_MANAGEMENT */
/* Flash Control */
#define FLASH_CONFIG_ID_REG		(ts->flash_fc.dsc.control_base)
#define FLASH_CONTROL_REG		(ts->flash_fc.dsc.data_base + 2)
#define FLASH_STATUS_REG		(ts->flash_fc.dsc.data_base + 3)
#define FLASH_STATUS_MASK		0xFF

/* Page number */
#define COMMON_PAGE			(ts->common_fc.function_page)
#define FINGER_PAGE			(ts->finger_fc.function_page)
#define ANALOG_PAGE			(ts->analog_fc.function_page)
#define FLASH_PAGE			(ts->flash_fc.function_page)
#define SENSOR_PAGE			(ts->sensor_fc.function_page)
#define DEFAULT_PAGE			0x00
#define LPWG_PAGE			0x04

#if 1 //us10_porting_1016
/* Display */
#define DISPLAY_PAGE			0x01
#define DCS_CMD_DATA_REG		0x40 /* 1-page */
#define DCS_CMD_SEND_REG		0x46 /* 1-page */
#define DISPLAY_OFF_VAL			0x28 /* 1-page */
#define DISPLAY_ON_VAL			0x29 /* 1-page */
#define SLEEP_IN_VAL			0x01 /* 1-page */
#define SLEEP_OUT_VAL			0x11 /* 1-page */
#define WRITE_DCS_VAL			0x01 /* 1-page */
#endif

/* Others */
#define LPWG_STATUS_REG			0x00 /* 4-page */
#define LPWG_DATA_REG			0x01 /* 4-page */
#define LPWG_TAPCOUNT_REG		0x31 /* 4-page */
#define LPWG_MIN_INTERTAP_REG		0x32 /* 4-page */
#define LPWG_MAX_INTERTAP_REG		0x33 /* 4-page */
#define LPWG_TOUCH_SLOP_REG		0x34 /* 4-page */
#define LPWG_TAP_DISTANCE_REG		0x35 /* 4-page */
#define LPWG_INTERRUPT_DELAY_REG        0x37 /* 4-page */
#define LPWG_TAPCOUNT_REG2              0x38 /* 4-page */
#define LPWG_MIN_INTERTAP_REG2          0x39 /* 4-page */
#define LPWG_MAX_INTERTAP_REG2          0x3A /* 4-page */
#define LPWG_TOUCH_SLOP_REG2            0x3B /* 4-page */
#define LPWG_TAP_DISTANCE_REG2          0x3C /* 4-page */
#define LPWG_INTERRUPT_DELAY_REG2       0x3E /* 4-page */
#define MISC_HOST_CONTROL_REG           0x3F
#define WAKEUP_GESTURE_ENABLE_REG	0x20 /* f12_info.ctrl_reg_addr[27] */
#define THERMAL_HIGH_FINGER_AMPLITUDE   0x60 /* finger_amplitude(0x80) = 0.5 */

/* LPWG Control Value */
#define REPORT_MODE_CTRL	1
#define TCI_ENABLE_CTRL		2
#define TAP_COUNT_CTRL		3
#define MIN_INTERTAP_CTRL	4
#define MAX_INTERTAP_CTRL	5
#define TOUCH_SLOP_CTRL		6
#define TAP_DISTANCE_CTRL	7
#define INTERRUPT_DELAY_CTRL    8

#define TCI_ENABLE_CTRL2        22
#define TAP_COUNT_CTRL2         23
#define MIN_INTERTAP_CTRL2      24
#define MAX_INTERTAP_CTRL2      25
#define TOUCH_SLOP_CTRL2        26
#define TAP_DISTANCE_CTRL2      27
#define INTERRUPT_DELAY_CTRL2   28

/* Palm / Hover */
#define PALM_TYPE	3
#define HOVER_TYPE	5
#define MAX_PRESSURE	255

#define I2C_DELAY			50
#define UEVENT_DELAY			200
#define REBASE_DELAY			100
#define CAP_DIFF_MAX             500
#define CAP_MIN_MAX_DIFF             1000
#define KNOCKON_DELAY                   68 /* 700ms */

/****************************************************************************
 * Macros
 ****************************************************************************/
/* Get user-finger-data from register.
 */
#define GET_X_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_Y_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_WIDTH_MAJOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_x : _width_y
#define GET_WIDTH_MINOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_y : _width_x

#define GET_ORIENTATION(_width_y, _width_x) \
	((_width_y - _width_x) > 0) ? 0 : 1
#define GET_PRESSURE(_pressure) \
		_pressure

/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
struct synaptics_ts_exp_fhandler {
    struct synaptics_ts_exp_fn *exp_fn;
    bool inserted;
    bool initialized;
};
static struct synaptics_ts_exp_fhandler prox_fhandler;
static struct synaptics_ts_exp_fhandler rmidev_fhandler;
int enable_rmi_dev = 0;

#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
extern struct workqueue_struct *touch_wq;
#endif

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
int synaptics_ts_page_data_read(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
 {
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	 DO_SAFE(touch_i2c_read(client, reg, size, data), error);
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
		 error);
 
	 return 0;
 error:
	 return -1;
 }
 
error_type synaptics_ts_page_data_write(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
 {
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	 DO_SAFE(touch_i2c_write(client, reg, size, data), error);
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
		 error);
 
	 return NO_ERROR;
 error:
	 return ERROR;
 }
 
error_type synaptics_ts_page_data_write_byte(struct i2c_client *client,
	 u8 page, u8 reg, u8 data)
 {
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	 DO_SAFE(touch_i2c_write_byte(client, reg, data), error);
	 DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
		 error);
 
	 return NO_ERROR;
 error:
	 return ERROR;
 }
void get_f12_info(struct synaptics_ts_data *ts)
{
	int retval;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	int i;
	u8 offset;

	if (!ts) {
		TOUCH_ERR_MSG("ts is null\n");
		return;
	}

	/* ctrl_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 5),
			sizeof(query_5.data), query_5.data);

	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_QUERY_05_Control_Presence register\n");
		return;
	}

	f12_info.ctrl_reg_is_present[0] = query_5.ctrl_00_is_present;
	f12_info.ctrl_reg_is_present[1] = query_5.ctrl_01_is_present;
	f12_info.ctrl_reg_is_present[2] = query_5.ctrl_02_is_present;
	f12_info.ctrl_reg_is_present[3] = query_5.ctrl_03_is_present;
	f12_info.ctrl_reg_is_present[4] = query_5.ctrl_04_is_present;
	f12_info.ctrl_reg_is_present[5] = query_5.ctrl_05_is_present;
	f12_info.ctrl_reg_is_present[6] = query_5.ctrl_06_is_present;
	f12_info.ctrl_reg_is_present[7] = query_5.ctrl_07_is_present;
	f12_info.ctrl_reg_is_present[8] = query_5.ctrl_08_is_present;
	f12_info.ctrl_reg_is_present[9] = query_5.ctrl_09_is_present;
	f12_info.ctrl_reg_is_present[10] = query_5.ctrl_10_is_present;
	f12_info.ctrl_reg_is_present[11] = query_5.ctrl_11_is_present;
	f12_info.ctrl_reg_is_present[12] = query_5.ctrl_12_is_present;
	f12_info.ctrl_reg_is_present[13] = query_5.ctrl_13_is_present;
	f12_info.ctrl_reg_is_present[14] = query_5.ctrl_14_is_present;
	f12_info.ctrl_reg_is_present[15] = query_5.ctrl_15_is_present;
	f12_info.ctrl_reg_is_present[16] = query_5.ctrl_16_is_present;
	f12_info.ctrl_reg_is_present[17] = query_5.ctrl_17_is_present;
	f12_info.ctrl_reg_is_present[18] = query_5.ctrl_18_is_present;
	f12_info.ctrl_reg_is_present[19] = query_5.ctrl_19_is_present;
	f12_info.ctrl_reg_is_present[20] = query_5.ctrl_20_is_present;
	f12_info.ctrl_reg_is_present[21] = query_5.ctrl_21_is_present;
	f12_info.ctrl_reg_is_present[22] = query_5.ctrl_22_is_present;
	f12_info.ctrl_reg_is_present[23] = query_5.ctrl_23_is_present;
	f12_info.ctrl_reg_is_present[24] = query_5.ctrl_24_is_present;
	f12_info.ctrl_reg_is_present[25] = query_5.ctrl_25_is_present;
	f12_info.ctrl_reg_is_present[26] = query_5.ctrl_26_is_present;
	f12_info.ctrl_reg_is_present[27] = query_5.ctrl_27_is_present;
	f12_info.ctrl_reg_is_present[28] = query_5.ctrl_28_is_present;
	f12_info.ctrl_reg_is_present[29] = query_5.ctrl_29_is_present;
	f12_info.ctrl_reg_is_present[30] = query_5.ctrl_30_is_present;
	f12_info.ctrl_reg_is_present[31] = query_5.ctrl_31_is_present;

	offset = 0;

	for (i = 0; i < 32; i++) {
		f12_info.ctrl_reg_addr[i] =
			ts->finger_fc.dsc.control_base + offset;

		if (f12_info.ctrl_reg_is_present[i])
			offset++;
	}

	/* data_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 8),
			sizeof(query_8.data), query_8.data);

	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_QUERY_08_Data_Presence register\n");
		return;
	}

	f12_info.data_reg_is_present[0] = query_8.data_00_is_present;
	f12_info.data_reg_is_present[1] = query_8.data_01_is_present;
	f12_info.data_reg_is_present[2] = query_8.data_02_is_present;
	f12_info.data_reg_is_present[3] = query_8.data_03_is_present;
	f12_info.data_reg_is_present[4] = query_8.data_04_is_present;
	f12_info.data_reg_is_present[5] = query_8.data_05_is_present;
	f12_info.data_reg_is_present[6] = query_8.data_06_is_present;
	f12_info.data_reg_is_present[7] = query_8.data_07_is_present;
	f12_info.data_reg_is_present[8] = query_8.data_08_is_present;
	f12_info.data_reg_is_present[9] = query_8.data_09_is_present;
	f12_info.data_reg_is_present[10] = query_8.data_10_is_present;
	f12_info.data_reg_is_present[11] = query_8.data_11_is_present;
	f12_info.data_reg_is_present[12] = query_8.data_12_is_present;
	f12_info.data_reg_is_present[13] = query_8.data_13_is_present;
	f12_info.data_reg_is_present[14] = query_8.data_14_is_present;
	f12_info.data_reg_is_present[15] = query_8.data_15_is_present;

	offset = 0;

	for (i = 0; i < 16; i++) {
		f12_info.data_reg_addr[i] =
			ts->finger_fc.dsc.data_base + offset;

		if (f12_info.data_reg_is_present[i])
			offset++;
	}

	/* print info */
	for (i = 0; i < 32; i++) {
		if (f12_info.ctrl_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.ctrl_reg_addr[%d]=0x%02X\n",
					i, f12_info.ctrl_reg_addr[i]);
	}

	for (i = 0; i < 16; i++) {
		if (f12_info.data_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.data_reg_addr[%d]=0x%02X\n",
					i, f12_info.data_reg_addr[i]);
	}

	return;
}

error_type read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	struct function_descriptor buffer;

	unsigned short address = 0;
	unsigned short page_num = 0;

	LGTC_ENTRY();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->lpwg_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++) {
		DO_SAFE(touch_i2c_write_byte(client,
			PAGE_SELECT_REG, page_num), error);

		for (address = DESCRIPTION_TABLE_START; address > 10;
				address -= sizeof(struct function_descriptor)) {
			DO_SAFE(touch_i2c_read(client, address, sizeof(buffer),
				(unsigned char *)&buffer) < 0, error);

			if (buffer.id == 0)
				break;

			switch (buffer.id) {
			case RMI_DEVICE_CONTROL:
				ts->common_fc.dsc = buffer;
				ts->common_fc.function_page = page_num;
				break;
			case TOUCHPAD_SENSORS:
				ts->finger_fc.dsc = buffer;
				ts->finger_fc.function_page = page_num;
				break;
			/*case LPWG_CONTROL:
				ts->lpwg_fc.dsc = buffer;
				ts->lpwg_fc.function_page = page_num;
				break;*/
			case SENSOR_CONTROL:
				ts->sensor_fc.dsc = buffer;
				ts->sensor_fc.function_page = page_num;
				break;
			case ANALOG_CONTROL:
				ts->analog_fc.dsc = buffer;
				ts->analog_fc.function_page = page_num;
				break;
			case FLASH_MEMORY_MANAGEMENT:
				ts->flash_fc.dsc = buffer;
				ts->flash_fc.function_page = page_num;
			default:
				break;
			}
		}
	}
    
    int_mask_cust = 0x40;

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00), error);
	ERROR_IF(ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0
		|| ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0,
		"page_init_error", error);

	TOUCH_DEBUG(DEBUG_BASE_INFO,
		"common[%dP:0x%02x] finger[%dP:0x%02x] lpwg[%dP:0x%02x] sensor[%dP:0x%02x]"
		"analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
		ts->common_fc.function_page, ts->common_fc.dsc.id,
		ts->finger_fc.function_page, ts->finger_fc.dsc.id,
		ts->lpwg_fc.function_page, ts->lpwg_fc.dsc.id,
		ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
		ts->analog_fc.function_page, ts->analog_fc.dsc.id,
		ts->flash_fc.function_page, ts->flash_fc.dsc.id);
    
    get_f12_info(ts);
    
	LGTC_EXIT();

	return NO_ERROR;
error:
	LGTC_ERR("Fail to read page description\n");
	return ERROR;
}
static error_type check_firmware_status(struct synaptics_ts_data *ts)
{
	u8 device_status = 0;
	u8 flash_status = 0;

	DO_SAFE(touch_i2c_read(ts->client, FLASH_STATUS_REG,
		sizeof(flash_status), &flash_status), error);
	DO_SAFE(touch_i2c_read(ts->client, DEVICE_STATUS_REG,
		sizeof(device_status), &device_status), error);

    ts->fw_info.need_rewrite_firmware = 0;
    
	if ((device_status & DEVICE_STATUS_FLASH_PROG)
			|| (device_status & DEVICE_CRC_ERROR_MASK)
			|| (flash_status & FLASH_STATUS_MASK)) {
		TOUCH_ERR_MSG("FLASH_STATUS[0x%x] DEVICE_STATUS[0x%x]\n",
			(u32)flash_status, (u32)device_status);
        ts->fw_info.need_rewrite_firmware = 1;
	}

	return NO_ERROR;
error:
	return ERROR;
}


/**
 * Knock on
 *
 * Type		Value
 *
 * 1		WakeUp_gesture_only=1 / Normal=0
 * 2		TCI enable=1 / disable=0
 * 3		Tap Count
 * 4		Min InterTap
 * 5		Max InterTap
 * 6		Touch Slop
 * 7		Tap Distance
 * 8		Interrupt Delay
 */
static error_type tci_control(struct synaptics_ts_data *ts, int type, u8 value)
{
	struct i2c_client* client = ts->client;
	u8 buffer[3] = {0};

	switch (type) {
	case REPORT_MODE_CTRL:
		DO_SAFE(touch_i2c_read(ts->client, INTERRUPT_ENABLE_REG,1, buffer), error);
		DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG,
					value ? buffer[0] & ~INTERRUPT_MASK_ABS0
					: buffer[0] | INTERRUPT_MASK_ABS0),
				error);
		if (value) {
			buffer[0] = 0x29;
			buffer[1] = 0x29;
			DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2, buffer), error);
		}
		DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[20],
					3, buffer), error);
		buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[20],
					3, buffer), error);
		break;
	case TCI_ENABLE_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		break;
	case TCI_ENABLE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE,
					LPWG_TAPCOUNT_REG2, 1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG2,
					1, buffer), error);
		break;
	case TAP_COUNT_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG,
					1, buffer), error);
		break;
	case TAP_COUNT_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG2,
					1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
					LPWG_PAGE, LPWG_TAPCOUNT_REG2,
					1, buffer), error);
		break;
	case MIN_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MIN_INTERTAP_REG,
					value), error);
		break;
	case MIN_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MIN_INTERTAP_REG2,
					value), error);
		break;
	case MAX_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MAX_INTERTAP_REG,
					value), error);
		break;
	case MAX_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_MAX_INTERTAP_REG2,
					value), error);
		break;
	case TOUCH_SLOP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TOUCH_SLOP_REG,
					value), error);
		break;
	case TOUCH_SLOP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TOUCH_SLOP_REG2,
					value), error);
		break;
	case TAP_DISTANCE_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TAP_DISTANCE_REG,
					value), error);
		break;
	case TAP_DISTANCE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_TAP_DISTANCE_REG2,
					value), error);
		break;
	case INTERRUPT_DELAY_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG,
					value ?
					(buffer[0] = (KNOCKON_DELAY << 1) | 0x1)
					: (buffer[0] = 0)), error);
		break;
	case INTERRUPT_DELAY_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG2,
					value ?
					(buffer[0] = (KNOCKON_DELAY << 1) | 0x1)
					: (buffer[0] = 0)), error);
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}



static error_type sleep_control(struct synaptics_ts_data *ts, int mode, int recal)
{
	u8 curr = 0;
	u8 next = 0;
    /*
	 * NORMAL == 0 : resume & lpwg state
	 * SLEEP  == 1 : uevent reporting time - sleep
	 * NO_CAL == 2 : proxi near - sleep when recal is not needed
	 */
	DO_SAFE(touch_i2c_read(ts->client, DEVICE_CONTROL_REG, 1, &curr), error);

	next = (curr & 0xFC) | (mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP);
	DO_SAFE(touch_i2c_write_byte(ts->client, DEVICE_CONTROL_REG, next), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : curr = [%x] next[%x]\n", __func__, curr, next);

	return NO_ERROR;
error:
	return ERROR;
}

static void set_lpwg_mode(struct lpwg_control *ctrl, int mode)
{
	ctrl->double_tap_enable =
		(mode & (LPWG_DOUBLE_TAP | LPWG_PASSWORD)) ? 1 : 0;
	ctrl->password_enable = (mode & LPWG_PASSWORD) ? 1 : 0;
	ctrl->signature_enable = (mode & LPWG_SIGNATURE) ? 1 : 0;
	ctrl->lpwg_is_enabled = ctrl->double_tap_enable
		|| ctrl->password_enable || ctrl->signature_enable;
}

static error_type lpwg_control(struct synaptics_ts_data *ts)
{
    set_lpwg_mode(&ts->lpwg_ctrl, ts->reportMode);

	switch (ts->reportMode) {
	case LPWG_SIGNATURE:
		break;
	case T_REPORT_KNOCK_ON_ONLY:                         /* Only TCI-1 */
		tci_control(ts, TCI_ENABLE_CTRL, 1);  /* Tci-1 enable */
		tci_control(ts, TAP_COUNT_CTRL, 2);   /* tap count = 2 */
		tci_control(ts, MIN_INTERTAP_CTRL, 6); /* min inter_tap
							  = 60ms */
		tci_control(ts, MAX_INTERTAP_CTRL, 70); /* max inter_tap
							   = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL, 100); /* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL, 10); /* tap distance
							   = 10mm */
		tci_control(ts, INTERRUPT_DELAY_CTRL, 0); /* interrupt delay
							     = 0ms */
		tci_control(ts, TCI_ENABLE_CTRL2, 0); /* Tci-2 disable */
		tci_control(ts, REPORT_MODE_CTRL, 1); /* wakeup_gesture_only */
		break;
	case T_REPORT_KNOCK_ON_CODE:                           /* TCI-1 and TCI-2 */
		tci_control(ts, TCI_ENABLE_CTRL, 1);  /* Tci-1 enable */
		tci_control(ts, TAP_COUNT_CTRL, 2);   /* tap count = 2 */
		tci_control(ts, MIN_INTERTAP_CTRL, 6); /* min inter_tap
							  = 60ms */
		tci_control(ts, MAX_INTERTAP_CTRL, 70); /* max inter_tap
							   = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL, 100); /* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL, 7); /* tap distance = 7mm */
		tci_control(ts, INTERRUPT_DELAY_CTRL,
				(u8)ts->pw_data.double_tap_check);
		tci_control(ts, TCI_ENABLE_CTRL2, 1); /* Tci-2 ensable */
		tci_control(ts, TAP_COUNT_CTRL2,
				(u8)ts->pw_data.tap_count); /* tap count
							       = user_setting */
		tci_control(ts, MIN_INTERTAP_CTRL2, 6); /* min inter_tap
							   = 60ms */
		tci_control(ts, MAX_INTERTAP_CTRL2, 70); /* max inter_tap
							    = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL2, 100); /* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL2, 255); /* tap distance
							     = MAX */
		tci_control(ts, INTERRUPT_DELAY_CTRL2, 0); /* interrupt delay
							      = 0ms */
		tci_control(ts, REPORT_MODE_CTRL, 1); /* wakeup_gesture_only */
		break;
	default:
		tci_control(ts, TCI_ENABLE_CTRL, 0); /* Tci-1 disable */
		tci_control(ts, TCI_ENABLE_CTRL2, 0); /* tci-2 disable */
		tci_control(ts, REPORT_MODE_CTRL, 0); /* normal */
		break;
	}

	TOUCH_INFO_MSG("%s : lpwg_mode[%d]\n", __func__,  ts->reportMode);
	return NO_ERROR;

}

extern void send_uevent_lpwg(struct i2c_client *client, int type);

static void lpwg_timer_func(struct work_struct *work_timer)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_timer),
		struct synaptics_ts_data, work_timer);

	sleep_control(ts, 0, 1); /* sleep until receive the reply. */
	send_uevent_lpwg(ts->client, LPWG_PASSWORD);
	wake_unlock(&ts->timer_wake_lock);
    TOUCH_DEBUG(DEBUG_LPWG, "u-event timer occur!\n");
	return;
}

static void all_palm_released_func(struct work_struct *work_palm)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_palm),
			struct synaptics_ts_data, work_palm);

	ts->ts_palm_data.all_palm_released = false;
	TOUCH_INFO_MSG("%s: ABS0 event disable time is expired.\n", __func__);

	return;
}

/****************************************************************************
* Global Functions
****************************************************************************/

void synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert)
{
	rmidev_fhandler.inserted = insert;

	if (insert) {
		rmidev_fhandler.exp_fn = kzalloc(sizeof(rmidev_fhandler.exp_fn), GFP_KERNEL);
		rmidev_fhandler.exp_fn = rmidev_fn;
	} else {
		rmidev_fhandler.exp_fn = NULL;
	}

	return;
}
static char *productcode_parse(unsigned char *product)
{
	static char str[128] = {0};
	int len = 0;
	char inch[2] = {0};
	char paneltype = 0;
	char version[2] = {0};

	switch ((product[0] & 0xF0) >> 4) {
	case 0:
		len += snprintf(str + len, sizeof(str)-len, "ELK ");
		break;
	case 1:
		len += snprintf(str + len, sizeof(str)-len, "Suntel ");
		break;
	case 2:
		len += snprintf(str + len, sizeof(str)-len, "Tovis ");
		break;
	case 3:
		len += snprintf(str + len, sizeof(str)-len, "Innotek ");
		break;
	case 4:
		len += snprintf(str + len, sizeof(str)-len, "JDI ");
		break;
	case 5:
		len += snprintf(str + len, sizeof(str)-len, "LGD ");
		break;
	default:
		len += snprintf(str + len, sizeof(str)-len, "Unknown ");
		break;
	}

	len += snprintf(str + len, sizeof(str)-len, "\n");

	switch (product[0] & 0x0F) {
	case 0:
		len += snprintf(str + len, sizeof(str)-len, "0key ");
		break;
	case 2:
		len += snprintf(str + len, sizeof(str)-len, "2Key ");
		break;
	case 3:
		len += snprintf(str + len, sizeof(str)-len, "3Key ");
		break;
	case 4:
		len += snprintf(str + len, sizeof(str)-len, "4Key ");
		break;
	default:
		len += snprintf(str + len, sizeof(str)-len, "Unknown ");
		break;
	}

	len += snprintf(str+len, sizeof(str)-len, "\n");

	switch ((product[1] & 0xF0) >> 4) {
	case 0:
		len += snprintf(str + len, sizeof(str)-len, "Synaptics ");
		break;
	default:
		len += snprintf(str + len, sizeof(str)-len, "Unknown ");
		break;
	}

	len += snprintf(str+len,
			sizeof(str)-len,
			"\n");

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += snprintf(str+len,
			sizeof(str)-len,
			"%d.%d\n",
			inch[0],
			inch[1]);

	paneltype = (product[2] & 0x0F);
	len += snprintf(str+len,
			sizeof(str)-len,
			"PanelType %d\n",
			paneltype);

	version[0] = ((product[3] & 0x80) >> 7);
	version[1] = (product[3] & 0x7F);
	len += snprintf(str+len,
			sizeof(str)-len,
			"version : v%d.%02d\n",
			version[0],
			version[1]);

	return str;
}

/*
 * show_atcmd_fw_ver
 *
 * show only firmware version.
 * It will be used for AT-COMMAND
 */
static ssize_t show_atcmd_fw_ver(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;

	if (ts->fw_info.fw_version[0] > 0x60)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%s\n", ts->fw_info.fw_version);
	else
		ret = snprintf(buf, PAGE_SIZE-ret, "V%d.%02d (0x%X, 0x%X, 0x%X, 0x%X)\n",
				(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0),
				ts->fw_info.fw_version[3] & 0x7F,
				ts->fw_info.fw_version[0],
				ts->fw_info.fw_version[1],
				ts->fw_info.fw_version[2],
				ts->fw_info.fw_version[3]);

	return ret;
}

static ssize_t show_firmware(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int rc = 0;

	read_page_description_table(ts->client);
	rc = get_ic_info(ts);
	if (rc < 0) {
		ret += snprintf(buf+ret,
				PAGE_SIZE,
				"-1\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Read Fail Touch IC Info.\n");
		return ret;
	}
	ret = snprintf(buf+ret,
			PAGE_SIZE-ret,
			"\n======== Firmware Info ========\n");
	if (ts->fw_info.fw_version[0] > 0x60) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"ic_fw_version[%s]\n",
				ts->fw_info.fw_version);
	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
        			"ic_fw_version RAW = %02X %02X %02X %02X\n",
				ts->fw_info.fw_version[0],
				ts->fw_info.fw_version[1],
				ts->fw_info.fw_version[2],
				ts->fw_info.fw_version[3]);
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"=== ic_fw_version info ===\n%s",
				productcode_parse(ts->fw_info.fw_version));
	}
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "IC_product_id[%s]\n",
			ts->fw_info.fw_product_id);
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
			!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3528(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6))) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3621\n\n");
    } else if (!strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else {
		ret += snprintf(buf+ret, PAGE_SIZE - ret,
				"Touch product ID read fail\n\n");
	}

	if (ts->fw_info.fw_image_version[0] > 0x60) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"img_fw_version[%s]\n",
				ts->fw_info.fw_image_version);
	} else {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"img_fw_version RAW = %02X %02X %02X %02X\n",
				ts->fw_info.fw_image_version[0],
				ts->fw_info.fw_image_version[1],
				ts->fw_info.fw_image_version[2],
				ts->fw_info.fw_image_version[3]);
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"=== img_fw_version info ===\n%s",
				productcode_parse(ts->fw_info.fw_image_version));
	}
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Img_product_id[%s]\n",
			ts->fw_info.fw_image_product_id);
	if (!(strncmp(ts->fw_info.fw_image_product_id, "PLG313", 6)) ||
			!(strncmp(ts->fw_info.fw_image_product_id, "PLG352", 6))
			|| !(strncmp(ts->fw_info.fw_image_product_id,
					"PLG391", 6))) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3528(family_id = %d, fw_rev = %d)\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else if (!strncmp(ts->fw_info.fw_image_product_id, "PLG298", 6)) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch IC : s3621\n");
    } else if (!strncmp(ts->fw_info.fw_image_product_id, "PLG463", 6)) {
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Touch IC : td4191(family_id=%d, fw_rev=%d)\n\n",
				ts->fw_info.family,
				ts->fw_info.fw_revision);
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Touch product ID read fail\n");
	}

	return ret;
}

static ssize_t store_tci(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	u32 type = 0, tci_num = 0, value = 0;

	sscanf(buf, "%d %d %d", &type, &tci_num, &value);

	tci_control(ts, type, (u8)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	
	int ret = 0;
	
	return ret;
}

static ssize_t store_reg_ctrl(struct i2c_client *client,
	const char *buf, size_t count)
{
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	int reg = 0;
	int offset = 0;
	int value = 0;

	sscanf(buf, "%s %d %d %d %d ", command, &page, &reg, &offset, &value);

	if (!strcmp(command, "write")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		buffer[offset] = (u8)value;
		synaptics_ts_page_data_write(client, page,
			reg, offset+1, buffer);
	} else if (!strcmp(command, "read")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		TOUCH_DEBUG(DEBUG_BASE_INFO,
			"page[%d] reg[%d] offset[%d] = 0x%x\n",
			page, reg, offset, buffer[offset]);
	} else {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Usage\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Write page reg offset value\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Read page reg offset\n");
	}
	return count;
}

static ssize_t show_object_report(struct i2c_client *client, char *buf)
{/*
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	u8 object_report_enable_reg;
	u8 temp[8];

	int ret = 0;
	int i;

	DO_SAFE(touch_i2c_read(client, OBJECT_REPORT_ENABLE_REG,
		sizeof(object_report_enable_reg), &object_report_enable_reg),
		error);

	for (i = 0; i < 8; i++)
		temp[i] = (object_report_enable_reg >> i) & 0x01;

	ret = sprintf(buf,
		"\n======= read object_report_enable register =======\n");
	ret += sprintf(buf+ret,
		" Addr Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0 HEX\n");
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" 0x%02X %4d %4d %4d %4d %4d %4d %4d %4d 0x%02X\n",
		OBJECT_REPORT_ENABLE_REG, temp[7], temp[6],
		temp[5], temp[4], temp[3], temp[2], temp[1], temp[0],
		object_report_enable_reg);
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" Bit0  : [F]inger -> %7s\n", temp[0] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit1  : [S]tylus -> %7s\n", temp[1] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit2  : [P]alm -> %7s\n", temp[2] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit3  : [U]nclassified Object -> %7s\n",
		temp[3] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit4  : [H]overing Finger -> %7s\n",
		temp[4] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit5  : [G]loved Finger -> %7s\n",
		temp[5] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit6  : [N]arrow Object Swipe -> %7s\n",
		temp[6] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit7  : Hand[E]dge  -> %7s\n",
		temp[7] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		"==================================================\n\n");
error:
	return ret;*/
	return 0;
}

static ssize_t store_object_report(struct i2c_client *client,
	const char *buf, size_t count)
{
     
        return 0;
}

static ssize_t show_use_rmi_dev(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u\n", enable_rmi_dev);

	return ret;
}

static ssize_t store_use_rmi_dev(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	int value = 0;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1) {
		TOUCH_INFO_MSG("Invalid enable_rmi_dev value:%d\n", value);
		return count;
	}

	enable_rmi_dev = value;
	TOUCH_INFO_MSG("enable_rmi_dev:%u\n", enable_rmi_dev);

	if (enable_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			DO_SAFE(rmidev_fhandler.exp_fn->init(ts), error);
			rmidev_fhandler.initialized = true;
		}
	}
	else {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}

	return count;

error:
	TOUCH_ERR_MSG("fail to enable_rmi_dev\n");
	return ERROR;
}

static LGE_TOUCH_ATTR(version, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR, show_object_report, store_object_report);
static LGE_TOUCH_ATTR(enable_rmi_dev, S_IRUGO | S_IWUSR, show_use_rmi_dev, store_use_rmi_dev);

static struct attribute *lge_specific_touch_attribute_list[] = {
	&lge_touch_attr_version.attr,
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_enable_rmi_dev.attr,
	NULL,
};

static int get_tci_data(struct synaptics_ts_data *ts, int count)
{
	struct i2c_client *client = ts->client;
	u8 i = 0;
	u8 buffer[12][4] = {{0} };

	ts->pw_data.data_num = count;

	if (!count)
		return 0;

	DO_SAFE(synaptics_ts_page_data_read(client,LPWG_PAGE, 
        LPWG_DATA_REG, 4 * count,&buffer[0][0]), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
			"%s : knock code coordinates, count = %d\n",
			__func__, count);

	for (i = 0; i < count; i++) {
		ts->pw_data.data[i].x =  GET_X_POSITION(buffer[i][1],
				buffer[i][0]);
		ts->pw_data.data[i].y =  GET_Y_POSITION(buffer[i][3],
				buffer[i][2]);

		if (ts->pdata->role->use_security_mode) {
			if (ts->lpwg_ctrl.password_enable) {
				TOUCH_INFO_MSG("LPWG data xxxx, xxxx\n");
			} else {
				TOUCH_INFO_MSG("LPWG data %d, %d\n",
						ts->pw_data.data[i].x,
						ts->pw_data.data[i].y);
			}
		} else {
			TOUCH_INFO_MSG("LPWG data %d, %d\n",
					ts->pw_data.data[i].x,
					ts->pw_data.data[i].y);
		}
	}

	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : get tci_control failed, count : %d\n",
			__func__, __LINE__, count);
	return -EPERM;
}
/* get_dts_data
 *
 * make platform data
 */
#define GET_PROPERTY_U8(np, string, target)			\
	do {								\
		u32 tmp_val = 0;					\
		if (of_property_read_u32(np, string, &tmp_val) < 0) 	\
		target = 0;					\
		else							\
		target = (u8) tmp_val;		\
	} while (0)
#define GET_PROPERTY_U32(np, string, target)			\
	do {								\
		u32 tmp_val = 0;					\
		if (of_property_read_u32(np, string, &tmp_val) < 0)	\
		target = -1;					\
		else							\
		target = tmp_val;					\
	} while (0)

#define GET_PROPERTY_U32_ARRAY(np, string, target, size)		\
	do {								\
		struct property *prop = of_find_property(np, string, NULL); \
		if (prop && prop->value && prop->length == size) {	\
			int i = 0;					\
			const u8 *iprop = prop->value;			\
			for (i = 0; i < prop->length; i++)		\
			target[i] = (u32)iprop[i];			\
		}							\
	} while (0)

#define GET_PROPERTY_STRING(np, string, target)				\
	do {								\
		const char *tmp_val = np->name;				\
		if (of_property_read_string(np, string, &tmp_val) < 0)	\
		strncpy(target, " ", 1);				\
		else {							\
			int len = strlen(tmp_val);			\
			memcpy(target, tmp_val, len);			\
		}							\
	} while (0)

#define GET_PROPERTY_GPIO(np, string, target)				\
	do {									\
		u32 tmp_val = 0;						\
		if ((tmp_val = of_get_named_gpio(np, string, 0)) < 0) 	\
			target = -1;						\
		else								\
			target = tmp_val;					\
	} while (0)


static struct touch_platform_data *get_dts_data(struct device *dev)
{
	struct touch_platform_data *p_data;
	struct device_node *np;

	ASSIGN(np = dev->of_node, error_mem);
	ASSIGN(p_data = devm_kzalloc(dev,
				sizeof(struct touch_platform_data),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->role = devm_kzalloc(dev,
				sizeof(struct touch_operation_role),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->fw = devm_kzalloc(dev,
				sizeof(struct touch_firmware_module),
				GFP_KERNEL), error_mem);

	ASSIGN(p_data->role->bouncing_filter = devm_kzalloc(dev,
				sizeof(struct bouncing_filter_role),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->role->grip_filter = devm_kzalloc(dev,
				sizeof(struct grip_filter_role),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->role->accuracy_filter = devm_kzalloc(dev,
				sizeof(struct accuracy_filter_role),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->role->jitter_filter = devm_kzalloc(dev,
				sizeof(struct jitter_filter_role),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->role->quickcover_filter = devm_kzalloc(dev,
				sizeof(struct quickcover_filter_role),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->role->ghost_detection = devm_kzalloc(dev,
				sizeof(struct ghost_detection_role),
				GFP_KERNEL), error_mem);
	ASSIGN(p_data->role->crack_detection = devm_kzalloc(dev,
				sizeof(struct crack_detection_role),
				GFP_KERNEL), error_mem);

	//firmware
	p_data->inbuilt_fw_name = "synaptics/lions/PLG463-V0.02-PR1749652-DS5.8.0.0.1056_50057002.img";
    p_data->inbuilt_recovery_fw_name = "synaptics/lions/PLG463-V0.02-PR1749652-DS5.8.0.0.1056_50057002.bin";

	// PANEL
	p_data->panel_spec = "synaptics/lions/lions_limit.txt";

    //F35_MBL_SLAVE_ADDR
    p_data->role->ub_i2c_addr = 0x2C;
	// GPIO
	//p_data->reset_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, NULL);
//	p_data->int_pin = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);
/*
	
	// ROLE
	GET_PROPERTY_U32(np, "report_mode", p_data->role->report_mode);
	GET_PROPERTY_U32(np, "delta_pos_threshold",
			p_data->role->delta_pos_threshold);
	GET_PROPERTY_U32(np, "booting_delay", p_data->role->booting_delay);
	GET_PROPERTY_U32(np, "reset_delay", p_data->role->reset_delay);
	GET_PROPERTY_U32(np, "wake_up_by_touch",
			p_data->role->wake_up_by_touch);
	GET_PROPERTY_U32(np, "use_sleep_mode", p_data->role->use_sleep_mode);
	GET_PROPERTY_U32(np, "use_lpwg_all", p_data->role->use_lpwg_all);
	GET_PROPERTY_U32(np, "use_security_mode",
			p_data->role->use_security_mode);
	GET_PROPERTY_U32(np, "thermal_check", p_data->role->thermal_check);
	GET_PROPERTY_U32(np, "palm_ctrl_mode", p_data->role->palm_ctrl_mode);
	GET_PROPERTY_U32(np, "use_hover_finger", p_data->role->use_hover_finger);
	GET_PROPERTY_U32(np, "use_rmi_dev", p_data->role->use_rmi_dev);
	GET_PROPERTY_U32(np, "mini_os_finger_amplitude",
			p_data->role->mini_os_finger_amplitude);
	GET_PROPERTY_U32(np, "irqflags", p_data->role->irqflags);
	GET_PROPERTY_U32(np, "bouncing.enable",
			p_data->role->bouncing_filter->enable);
	GET_PROPERTY_U32(np, "grip.enable", p_data->role->grip_filter->enable);
	GET_PROPERTY_U32(np, "grip.edge_region",
			p_data->role->grip_filter->edge_region);
	GET_PROPERTY_U32(np, "grip.max_delta",
			p_data->role->grip_filter->max_delta);
	GET_PROPERTY_U32(np, "grip.width_ratio",
			p_data->role->grip_filter->width_ratio);
	GET_PROPERTY_U32(np, "accuracy.enable",
			p_data->role->accuracy_filter->enable);
	GET_PROPERTY_U32(np, "accuracy.min_delta",
			p_data->role->accuracy_filter->min_delta);
	GET_PROPERTY_U32(np, "accuracy.curr_ratio",
			p_data->role->accuracy_filter->curr_ratio);
	GET_PROPERTY_U32(np, "accuracy.min_pressure",
			p_data->role->accuracy_filter->min_pressure);
	GET_PROPERTY_U32(np, "jitter.enable",
			p_data->role->jitter_filter->enable);
	GET_PROPERTY_U32(np, "jitter.curr_ratio",
			p_data->role->jitter_filter->curr_ratio);
	GET_PROPERTY_U32(np, "quickcover.enable",
			p_data->role->quickcover_filter->enable);
	GET_PROPERTY_U32(np, "quickcover.X1",
			p_data->role->quickcover_filter->X1);
	GET_PROPERTY_U32(np, "quickcover.X2",
			p_data->role->quickcover_filter->X2);
	GET_PROPERTY_U32(np, "quickcover.Y1",
			p_data->role->quickcover_filter->Y1);
	GET_PROPERTY_U32(np, "quickcover.Y2",
			p_data->role->quickcover_filter->Y2);
	GET_PROPERTY_U32(np, "crack.enable",
			p_data->role->crack_detection->use_crack_mode);
	GET_PROPERTY_U32(np, "crack.min.cap",
			p_data->role->crack_detection->min_cap_value);

	GET_PROPERTY_U8(np, "ghost_detection_enable",
			p_data->role->ghost_detection->check_enable.ghost_detection_enable);
	GET_PROPERTY_U8(np, "ta_noise_chk",
			p_data->role->ghost_detection->check_enable.ta_noise_chk);
	GET_PROPERTY_U8(np, "incoming_call_chk",
			p_data->role->ghost_detection->check_enable.incoming_call_chk);
	GET_PROPERTY_U8(np, "first_finger_chk",
			p_data->role->ghost_detection->check_enable.first_finger_chk);
	GET_PROPERTY_U8(np, "pressure_zero_chk",
			p_data->role->ghost_detection->check_enable.pressure_zero_chk);
	GET_PROPERTY_U8(np, "ta_debouncing_chk",
			p_data->role->ghost_detection->check_enable.ta_debouncing_chk);
	GET_PROPERTY_U8(np, "press_interval_chk",
			p_data->role->ghost_detection->check_enable.press_interval_chk);
	GET_PROPERTY_U8(np, "diff_fingers_chk",
			p_data->role->ghost_detection->check_enable.diff_fingers_chk);
	GET_PROPERTY_U8(np, "subtraction_finger_chk",
			p_data->role->ghost_detection->check_enable.subtraction_finger_chk);
	GET_PROPERTY_U8(np, "long_press_chk",
			p_data->role->ghost_detection->check_enable.long_press_chk);
	GET_PROPERTY_U8(np, "button_chk",
			p_data->role->ghost_detection->check_enable.button_chk);
	GET_PROPERTY_U8(np, "rebase_repetition_chk",
			p_data->role->ghost_detection->check_enable.rebase_repetition_chk);
	GET_PROPERTY_U32(np, "ghost_detection_chk_cnt",
			p_data->role->ghost_detection->ghost_detection_chk_cnt);
	GET_PROPERTY_U32(np, "jitter_value",
			p_data->role->ghost_detection->jitter_value);
	GET_PROPERTY_U32(np, "first_finger_time",
			p_data->role->ghost_detection->first_finger_time);
	GET_PROPERTY_U32(np, "ta_debouncing_cnt",
			p_data->role->ghost_detection->ta_debouncing_cnt);
	GET_PROPERTY_U32(np, "ta_debouncing_finger_num",
			p_data->role->ghost_detection->ta_debouncing_finger_num);
	GET_PROPERTY_U32(np, "press_interval",
			p_data->role->ghost_detection->press_interval);
	GET_PROPERTY_U32(np, "diff_finger_num",
			p_data->role->ghost_detection->diff_finger_num);
	GET_PROPERTY_U32(np, "subtraction_time",
			p_data->role->ghost_detection->subtraction_time);
	GET_PROPERTY_U32(np, "subtraction_finger_cnt",
			p_data->role->ghost_detection->subtraction_finger_cnt);
	GET_PROPERTY_U32(np, "long_press_chk_time",
			p_data->role->ghost_detection->long_press_chk_time);
	GET_PROPERTY_U32(np, "long_press_cnt",
			p_data->role->ghost_detection->long_press_cnt);
	GET_PROPERTY_U32(np, "button_int_num",
			p_data->role->ghost_detection->button_int_num);
	GET_PROPERTY_U32(np, "button_duration",
			p_data->role->ghost_detection->button_duration);
	GET_PROPERTY_U32(np, "rebase_since_init",
			p_data->role->ghost_detection->rebase_since_init);
	GET_PROPERTY_U32(np, "rebase_since_rebase",
			p_data->role->ghost_detection->rebase_since_rebase);
	// POWER
#if 0
	GET_PROPERTY_U32(np, "use_regulator", p_data->pwr->use_regulator);
	GET_PROPERTY_STRING(np, "vdd", p_data->pwr->vdd);
	GET_PROPERTY_U32(np, "vdd_voltage", p_data->pwr->vdd_voltage);
	GET_PROPERTY_STRING(np, "vio", p_data->pwr->vio);
	GET_PROPERTY_U32(np, "vio_voltage", p_data->pwr->vio_voltage);
#endif
	GET_PROPERTY_U32(np, "vdd_type0", p_data->pwr[0].type);
	GET_PROPERTY_STRING(np, "vdd_name0", p_data->pwr[0].name);
	if(p_data->pwr[0].type == 1) {
		GET_PROPERTY_GPIO(np, "vdd_value0", p_data->pwr[0].value);
	} else {
		GET_PROPERTY_U32(np, "vdd_value0", p_data->pwr[0].value);
	}
	GET_PROPERTY_U32(np, "vdd_type1", p_data->pwr[1].type);
	GET_PROPERTY_STRING(np, "vdd_name1", p_data->pwr[1].name);
	if(p_data->pwr[1].type == 1) {
		GET_PROPERTY_GPIO(np, "vdd_value1", p_data->pwr[1].value);
	} else {
		GET_PROPERTY_U32(np, "vdd_value1", p_data->pwr[1].value);
	}
	GET_PROPERTY_U32(np, "vdd_type2", p_data->pwr[2].type);
	GET_PROPERTY_STRING(np, "vdd_name2", p_data->pwr[2].name);
	if(p_data->pwr[2].type == 1) {
		GET_PROPERTY_GPIO(np, "vdd_value2", p_data->pwr[2].value);
	} else {
		GET_PROPERTY_U32(np, "vdd_value2", p_data->pwr[2].value);
	}

	// FIRMWARE
	GET_PROPERTY_U32(np, "need_upgrade", p_data->fw->need_upgrade);
*/
	return p_data;

error_mem:
	return NULL;
}

static int get_platform_data(struct touch_platform_data **p_data,
		struct i2c_client *client)
{
    
    LGTC_FUN();
    struct touch_platform_data *pdata;
    
	ASSIGN(pdata = devm_kzalloc(&client->dev,
				sizeof(struct touch_platform_data),
				GFP_KERNEL), error);
    ASSIGN(pdata->role = devm_kzalloc(&client->dev,
				sizeof(struct touch_operation_role),
				GFP_KERNEL), error);
      //F35_MBL_SLAVE_ADDR
 //   pdata->role->i2c_addr= 0x20;
    //F35_MBL_SLAVE_ADDR
//    pdata->role->ub_i2c_addr = 0x2C;
    //firmware
	pdata->inbuilt_fw_name = "synaptics/lions/PR1787167-td4191-i2c-mtk-lion-vblank-120hz.img";
    pdata->inbuilt_recovery_fw_name = "synaptics/lions/PLG463-V0.02-PR1749652-DS5.8.0.0.1056_50057002.bin";

	// PANEL
	pdata->panel_spec = "synaptics/lions/lions_limit.txt";
    
	*p_data = pdata;
	return 0;
error:
	return -EPERM;
}
error_type synaptics_ts_fw_recovery(struct i2c_client *client,
		struct touch_fw_info *info)
{
    LGTC_FUN();
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	char path[256];
    isRecovery = true;
	SynaScanPDT(ts);
	if(ts->ubootloader_mode) {
		memcpy(path, ts->pdata->inbuilt_recovery_fw_name, sizeof(path));
		TOUCH_INFO_MSG("FW_ RECOVERY:  force[%d] file[%s]\n",
					info->fw_force_upgrade, path);
		goto firmware;
	}
    isRecovery = false;
	TOUCH_INFO_MSG("Don't need to Recovery Firmware.\n");
	return NO_ERROR;
firmware:
	ts->is_init = 0;
    mtk_wdt_enable ( WK_WDT_DIS );
	DO_SAFE(FirmwareRecovery(ts, path),error);
    mtk_wdt_enable ( WK_WDT_EN);
    isRecovery = false;
error:
    isRecovery = false;
	return ERROR;
}

error_type synaptics_ts_probe(struct i2c_client *client, struct touch_fw_info* fw_info, struct attribute ***attribute_list)
{
	struct synaptics_ts_data *ts;

	LGTC_FUN();

	ASSIGN(ts = devm_kzalloc(&client->dev,
		sizeof(struct synaptics_ts_data), GFP_KERNEL), error);
	set_touch_handle(client, ts);

	ts->client = client;
    
	ds4_i2c_client = client;
//	ts->pdata = lge_ts_data;
//	ts->state = state;

    get_platform_data(&ts->pdata, client);
    TOUCH_INFO_MSG("pdata->inbuilt_fw_name %s\n",ts->pdata->inbuilt_fw_name);
    TOUCH_INFO_MSG("pdata->inbuilt_recovery_fw_name %s\n",ts->pdata->inbuilt_recovery_fw_name);
	ts->touch_fw_info = fw_info;
	*attribute_list = lge_specific_touch_attribute_list;

    ts->is_probed = 0;
	ts->is_init = 0;
	ts->lpwg_ctrl.screen = 1;
	ts->lpwg_ctrl.sensor = 1;
    synaptics_ts_fw_recovery(client,fw_info);
	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
	INIT_DELAYED_WORK(&ts->work_timer, lpwg_timer_func);
	INIT_DELAYED_WORK(&ts->work_palm, all_palm_released_func);
	wake_lock_init(&ts->timer_wake_lock, WAKE_LOCK_SUSPEND, "touch_timer");

	return NO_ERROR;

error:
	TOUCH_ERR_MSG("touch synaptics probe failed\n");

	return ERROR;
}

static int set_doze_param(struct synaptics_ts_data *ts, int value)
{
	u8 buf_array[6] = {0};

	touch_i2c_read(ts->client,
			f12_info.ctrl_reg_addr[27], 6, buf_array);

	/* max active duration */
	if (ts->pw_data.tap_count < 3)
		buf_array[3] = 3;
	else
		buf_array[3] = 3 + ts->pw_data.tap_count;

	buf_array[2] = 0x0C;  /* False Activation Threshold */
	buf_array[4] = 0x01;  /* Timer 1 */
	buf_array[5] = 0x01;  /* Max Active Duration Timeout */

	touch_i2c_write(ts->client, f12_info.ctrl_reg_addr[27],
			6, buf_array);

	DO_SAFE(touch_i2c_write_byte(ts->client,
				DOZE_INTERVAL_REG, 3), error);
	DO_SAFE(touch_i2c_write_byte(ts->client,
				DOZE_WAKEUP_THRESHOLD_REG, 30), error);

	return 0;
error:
	TOUCH_ERR_MSG("%s : failed to set doze interval\n", __func__);
	return -EPERM;
}
static int get_ic_info(struct synaptics_ts_data *ts)
{
	const struct firmware *fw_entry = NULL;
	const u8 *fw = NULL;
	int rc = 0;

	memset(&ts->fw_info, 0, sizeof(struct synaptics_ts_fw_info));

	DO_SAFE(touch_i2c_read(ts->client, PRODUCT_ID_REG,
				sizeof(ts->fw_info.fw_product_id) - 1,
				ts->fw_info.fw_product_id), error);
	DO_SAFE(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
				sizeof(ts->fw_info.fw_version) - 1,
				ts->fw_info.fw_version), error);
	DO_SAFE(touch_i2c_read(ts->client, CUSTOMER_FAMILY_REG, 1,
				&(ts->fw_info.family)), error);
	DO_SAFE(touch_i2c_read(ts->client, FW_REVISION_REG, 1,
				&(ts->fw_info.fw_revision)), error);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "IC FW_PRODUC_ID = %s\n",
			ts->fw_info.fw_product_id);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "IC FW_VERSION = V%d.%02d\n",
			(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0), ts->fw_info.fw_version[3] & 0x7F);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "CUSTOMER_FAMILY_REG = %d\n",
			ts->fw_info.family);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "FW_REVISION_REG = %d\n",
			ts->fw_info.fw_revision);
   
   	if (request_firmware(&fw_entry, ts->touch_fw_info->fw_path, &ts->client->dev) != 0) {
		TOUCH_ERR_MSG("request_firmware() failed %s\n",ts->touch_fw_info->fw_path);
		return ERROR;
	}else{
        ts->fw_flag = TD4191;
	}
    TOUCH_DEBUG(DEBUG_BASE_INFO, "ts->fw_flag = %d\n", ts->fw_flag);
	
	memcpy(ts->fw_info.fw_image_product_id, &fw_entry->data[0x0010], 6);
    memcpy(ts->fw_info.fw_image_version, &fw_entry->data[0xe100], 4);
    TOUCH_ERR_MSG("2222222222222");
	ts->fw_info.fw_start = (unsigned char *)&fw_entry->data[0];
    TOUCH_ERR_MSG("3333333333333333");
	ts->fw_info.fw_size = sizeof(fw_entry);
   TOUCH_ERR_MSG("44444444444444444444444444");
 /*   ts->pdata->inbuilt_fw_name =  "synaptics/lions/PLG463-V0.02-PR1749652-DS5.8.0.0.1056_50057002.img";
	if (!strncmp(ts->fw_info.fw_product_id, "PLG463", 6)) {
		rc = request_firmware(&fw_entry,
				ts->pdata->inbuilt_fw_name,
				&ts->client->dev);
		if (rc != 0) {
			TOUCH_ERR_MSG("request_firmware() failed %d\n", rc);
			goto error;
		}
		ts->fw_flag = TD4191;
	} else {
		TOUCH_ERR_MSG("Touch product ID read error\n");
		goto error;
	}

	TOUCH_DEBUG(DEBUG_BASE_INFO, "ts->fw_flag = %d\n", ts->fw_flag);
	fw = fw_entry->data;

	memcpy(ts->fw_info.fw_image_product_id, &fw[0x0010], 6);
	memcpy(ts->fw_info.fw_image_version, &fw[0xe100], 4);
	ts->fw_info.fw_start = (unsigned char *)&fw[0];
	ts->fw_info.fw_size = sizeof(fw);
*/
    release_firmware(fw_entry);
	return 0;
error:
	TOUCH_ERR_MSG("%s, %d : get_ic_info failed\n", __func__, __LINE__);
	memset(&fw_entry, 0, sizeof(fw_entry));
	return -EIO;
}

static int lpwg_update_all(struct synaptics_ts_data *ts, bool irqctrl)
{
	int sleep_status = 0;
	int lpwg_status = 0;
	bool req_lpwg_param = false;

	TOUCH_TRACE();

	if (ts->lpwg_ctrl.screen) {
		if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 1) {
			if (power_state == POWER_OFF
					|| power_state == POWER_SLEEP)
				ts->is_init = 0;
			if (irqctrl)
				TouchDisableIrq();
		}
		atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
	} else {
		if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
			atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
			if (irqctrl)
				TouchEnableIrq();
			set_doze_param(ts, 3);
		}
	}

	if (ts->lpwg_ctrl.screen) { /* ON(1) */
		sleep_status = 1;
		lpwg_status = 0;
	} else if (!ts->lpwg_ctrl.screen /* OFF(0), CLOSED(0) */
			&& ts->lpwg_ctrl.qcover) {
		sleep_status = 1;
		lpwg_status = 1;
	} else if (!ts->lpwg_ctrl.screen /* OFF(0), OPEN(1), FAR(1) */
			&& !ts->lpwg_ctrl.qcover
			&& ts->lpwg_ctrl.sensor) {
		sleep_status = 1;
		lpwg_status = ts->lpwg_ctrl.lpwg_mode;
	} else if (!ts->lpwg_ctrl.screen /* OFF(0), OPEN(1), NEAR(0) */
			&& !ts->lpwg_ctrl.qcover
			&& !ts->lpwg_ctrl.sensor) {
		if (!after_crack_check) {
			TOUCH_INFO_MSG("%s : Crack check not done... use nonsleep mode to check Crack!!\n",
					__func__);
			sleep_status = 1;
			lpwg_status = ts->lpwg_ctrl.lpwg_mode;
		} else {
			sleep_status = 0;
			req_lpwg_param = true;
		}
	}

	DO_SAFE(sleep_control(ts, sleep_status, 0), error);
	if (req_lpwg_param == false)
		DO_SAFE(lpwg_control(ts), error);

	return NO_ERROR;
error:
	return ERROR;
}

error_type synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();
    
    if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted && prox_fhandler.initialized) {
		prox_fhandler.exp_fn->remove(ts);
		prox_fhandler.initialized = false;
	}

	if (ts->pdata->role->use_rmi_dev && rmidev_fhandler.inserted && rmidev_fhandler.initialized) {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}
    
	kfree(ts);
    wake_lock_destroy(&ts->timer_wake_lock);
	return NO_ERROR;
}

//==========================================================
// Touch IC를 초기화 한다.
// 초기화에 필요한 전원 및 Reset PIN의 컨트롤을 포함한다.
// 함수 수행 이후에는, Normal Mode에서의 정상적인 Touch 동작이 보장되어야 한다.
// Touch IC로 부터 읽어야 하는 정보 ( Firwware 버전 등 ) 를 반환 한다. ( TBD : 반환 Data 정리 필요 )
//==========================================================
error_type synaptics_ts_init(struct i2c_client *client)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};
    int exp_fn_retval;
	u8 motion_suppression_reg_addr;
	int rc = 0;
//	u8 lpwg_mode = ts->lpwg_ctrl.lpwg_mode;
//	int is_suspend = atomic_read(&ts->lpwg_ctrl.is_suspend);
	LGTC_FUN();
    
    if (ts->is_probed == 0) {
         rc = read_page_description_table(ts->client);
         DO_SAFE(check_firmware_status(ts), error);
         if (rc == -EIO)
            return ERROR;
         get_ic_info(ts);
         if (rc == -EINVAL) {
                TOUCH_INFO_MSG("%s : need to rewrite firmware !!",
                    __func__);
                ts->fw_info.need_rewrite_firmware = 1;
         }
         ts->is_probed = 1;
  //       TOUCH_INFO_MSG("%s : lge_get_boot_mode[%d]\n", __func__, lge_get_boot_mode());
    }
  /*
    if (ts->pdata->role->use_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			exp_fn_retval = rmidev_fhandler.exp_fn->init(ts);

			if (exp_fn_retval < 0) {
				TOUCH_INFO_MSG("[Touch RMI_Dev] %s: Failed to init rmi_dev settings\n",
						__func__);
			} else {
				rmidev_fhandler.initialized = true;
			}
		}
	}*/
    DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
				DEVICE_CONTROL_NORMAL_OP
				| DEVICE_CONTROL_CONFIGURED), error);
    DO_SAFE(touch_i2c_read(client, INTERRUPT_ENABLE_REG, 1, &buf), error);
         if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
                !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
                !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
            DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
                        buf | INTERRUPT_MASK_ABS0
                        | int_mask_cust), error);
        } else {
            DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
                        buf | INTERRUPT_MASK_ABS0), error);
        }
   /*     
        if (ts->pdata->role->report_mode == REDUCED_REPORT_MODE
                && !ts->pdata->role->ghost_detection->check_enable.long_press_chk) {
            buf_array[0] = buf_array[1] =
                ts->pdata->role->delta_pos_threshold;
        } else {
            buf_array[0] = buf_array[1] = 0;
            ts->pdata->role->ghost_detection->force_continuous_mode = true;
        }

        */
	motion_suppression_reg_addr = f12_info.ctrl_reg_addr[20];
    DO_SAFE(touch_i2c_write(client, motion_suppression_reg_addr, 2,
				buf_array), error);
	DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[15],
				2, default_finger_amplitude), error);

    
	DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[22],
				1, &buf), error);
	buf_array[0] = buf & 0x03;
    
/*
    if ((ts->pdata->role->palm_ctrl_mode == PALM_REJECT_DRIVER)
			|| (ts->pdata->role->palm_ctrl_mode == PALM_REPORT)) {
		if (buf_array[0] != 0x00) {
			buf &= ~(0x03);
			DO_SAFE(touch_i2c_write_byte(client,
						f12_info.ctrl_reg_addr[22],
						buf), error);
		}
		memset(&ts->ts_palm_data, 0, sizeof(struct palm_data));
	} else {
		if (buf_array[0] != 0x01) {
			buf &= ~(0x02);
			buf |= 0x01;
			DO_SAFE(touch_i2c_write_byte(client,
						f12_info.ctrl_reg_addr[22],
						buf), error);
		}
	}
    */
      /*    if (lge_get_boot_mode() == LGE_BOOT_MODE_QEM_56K) {
            TOUCH_INFO_MSG("mini_os_finger_amplitude = 0x%02X\n",
                    ts->pdata->role->mini_os_finger_amplitude);
            buf_array[0] = ts->pdata->role->mini_os_finger_amplitude;
            buf_array[1] = ts->pdata->role->mini_os_finger_amplitude;
            DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15],
                    2, buf_array), error);
        }*/
    
        if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) ||
                !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) ||
                !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
            if (ts->pdata->role->use_lpwg_all)
                DO_SAFE(lpwg_update_all(ts, 0), error);
            else
                DO_SAFE(lpwg_control(ts), error);
        }
        
    /* It always should be done last. */
        DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf), error);
        ts->is_init = 1;
    
	ts->reportMode = T_REPORT_NORMAL;

	return NO_ERROR;
error:
	LGTC_ERR("Fail to initialise Touch IC\n");
	return ERROR;
}

//==========================================================
// Touch Interrupt가 발생하면 WorkQueue내에서 호출되는 함수
// Interrupt가 발생된 원인을 확인하고 적절한 후속처리를 한다.
// Event처리를 위한 Data도 반환한다. ( TBD : Data구조체 정의 필요 )
// 함수가 호출된 후에는, Interrupt가 해제된 상태여야 한다.
//==========================================================
error_type synaptics_ts_isr(struct i2c_client *client, TouchReadData *pData)
{
	struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	u8  i = 0;
//    u8  finger_index = 0;

	TouchFingerData *pFingerData = NULL;

//	u8 regFingerData[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];

	pData->type = NUM_OF_DATA_TYPE;
	pData->count = 0;
    TOUCH_ERR_MSG("222222222222222222");

	DO_SAFE(touch_i2c_read(client, DEVICE_STATUS_REG,
				sizeof(ts->ts_data.device_status_reg),&ts->ts_data.device_status_reg), error);
	LGTC_DBG("DEVICE_STATUS_REG =%02X\n", ts->ts_data.device_status_reg);
    
    DO_IF((ts->ts_data.device_status_reg & DEVICE_FAILURE_MASK)
                == DEVICE_FAILURE_MASK, error);
    TOUCH_ERR_MSG("111111111111111111111111111");
	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG,
				sizeof(ts->ts_data.interrupt_status_reg),&ts->ts_data.interrupt_status_reg), error);
    LGTC_DBG("INTERRUPT_STATUS_REG =%02X\n", ts->ts_data.interrupt_status_reg);

   TOUCH_ERR_MSG("333333333333333333333");
    if (ts->ts_data.interrupt_status_reg & int_mask_cust) {
        TOUCH_ERR_MSG("55555555555555555555555555555555555555");
        u8 status = 0;
		DO_SAFE(synaptics_ts_page_data_read(client,
					LPWG_PAGE, LPWG_STATUS_REG,1, &status), error);
        if ((status & 0x1)) {   /* TCI-1 Double-Tap */
			TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
					"LPWG Double-Tap mode\n");
			if (ts->lpwg_ctrl.double_tap_enable) {
				get_tci_data(ts, 2);
                pData->type = T_DATA_KNOCK_ON;
			//	send_uevent_lpwg(ts->client, LPWG_DOUBLE_TAP);
			}
		} else if ((status & 0x2)) { /* TCI-2 Multi-Tap */
			TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
					"LPWG Multi-Tap mode\n");
			if (ts->lpwg_ctrl.password_enable) {
                pData->type = T_DATA_KNOCK_CODE;
				get_tci_data(ts, ts->pw_data.tap_count);
                pData->count = ts->pw_data.data_num;
				wake_lock(&ts->timer_wake_lock);
				tci_control(ts, REPORT_MODE_CTRL, 0);
				queue_delayed_work(touch_wq, &ts->work_timer,
						msecs_to_jiffies(UEVENT_DELAY
							- I2C_DELAY));
			}
		} else {
			TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG,
					"LPWG status has problem\n");
		}
		return IGNORE_EVENT;
    } else if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_ABS0) {
        TOUCH_ERR_MSG("44444444444444444444444");
        DO_SAFE(touch_i2c_read(ts->client, FINGER_DATA_REG_START,
					(NUM_OF_EACH_FINGER_DATA_REG * MAX_NUM_OF_FINGERS),
					ts->ts_data.finger.finger_reg[0]),error);
        for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
			if (ts->ts_data.finger.finger_reg[i][0] == F12_FINGER_STATUS
					|| ts->ts_data.finger.finger_reg[i][0] == F12_STYLUS_STATUS
					|| ts->ts_data.finger.finger_reg[i][0] == F12_PALM_STATUS
					|| ts->ts_data.finger.finger_reg[i][0] == F12_GLOVED_FINGER_STATUS) {
					
                pFingerData = &pData->fingerData[pData->count];
                pFingerData->id = i;
                pFingerData->type = ts->ts_data.finger.finger_reg[i][REG_OBJECT_TYPE_AND_STATUS];
                pFingerData->x = GET_X_POSITION(ts->ts_data.finger.finger_reg[i][REG_X_MSB],
							ts->ts_data.finger.finger_reg[i][REG_X_LSB]);   
                pFingerData->y = GET_Y_POSITION(ts->ts_data.finger.finger_reg[i][REG_Y_MSB],
							ts->ts_data.finger.finger_reg[i][REG_Y_LSB]);
                pFingerData->width_major = GET_WIDTH_MAJOR(ts->ts_data.finger.finger_reg[i][REG_WX],
							ts->ts_data.finger.finger_reg[i][REG_WY]);
				pFingerData->width_minor = GET_WIDTH_MINOR(ts->ts_data.finger.finger_reg[i][REG_WX],
							ts->ts_data.finger.finger_reg[i][REG_WY]);
                pFingerData->orientation = GET_ORIENTATION(ts->ts_data.finger.finger_reg[i][REG_WY],
							ts->ts_data.finger.finger_reg[i][REG_WX]);
                pFingerData->pressure = GET_PRESSURE(ts->ts_data.finger.finger_reg[i][REG_Z]);    
                pData->count++;
                
			

				TOUCH_DEBUG(DEBUG_GET_DATA,
						"<%d> type[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d]\n",
						i, pFingerData->type,
						pFingerData->x,
						pFingerData->y,
						pFingerData->width_major,
						pFingerData->width_minor,
						pFingerData->orientation,
						pFingerData->pressure);

			}
        }
        pData->type = T_DATA_FINGER;
    }
error:
	TOUCH_ERR_MSG("%s, %d : get data failed\n", __func__, __LINE__);
	return ERROR;
}


error_type synaptics_ts_ic_ctrl(struct i2c_client *client,
	u8 code, u32 value, u32 *ret)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};

	switch (code) {
	case IC_CTRL_READ:
		DO_SAFE(touch_i2c_read(client, value, 1, &buf), error);
		*ret = (u32)buf;
		break;
	case IC_CTRL_WRITE:
		DO_SAFE(touch_i2c_write_byte(client,
			((value & 0xFF00) >> 8), (value & 0xFF)), error);
		break;
    case IC_CTRL_BASELINE_REBASE:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
					ANALOG_PAGE, ANALOG_COMMAND_REG, value),
				error);
		break;
	case IC_CTRL_REPORT_MODE:
		if (value == REDUCED_REPORT_MODE)
			buf_array[0] = buf_array[1] =
				ts->pdata->role->delta_pos_threshold;
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[20],
					2, buf_array), error);
		break;
	case IC_CTRL_THERMAL:
		switch (value) {
		case THERMAL_LOW:
			buf = 0x00;
			buf_array[0] = default_finger_amplitude[0];
			buf_array[1] = default_finger_amplitude[1];
			break;
		case THERMAL_HIGH:
			buf = 0x04;
			buf_array[0] = buf_array[1] =
				THERMAL_HIGH_FINGER_AMPLITUDE;
			break;
		default:
			TOUCH_ERR_MSG("Invalid current_thermal_mode (%u)\n",
					value);
			goto error;
		}
		TOUCH_INFO_MSG("High Temp Control(0x%02X), Finger Amplitude Threshold(0x%02X), Small Finger Amplitude Threshold(0x%02X)\n",
				buf, buf_array[0], buf_array[1]);
		DO_SAFE(synaptics_ts_page_data_write_byte(client, LPWG_PAGE,
					MISC_HOST_CONTROL_REG, buf), error);
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2,
					buf_array), error);
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}


//==========================================================
// 인자로 전달된 Firmware를 TouchIC에 Write한다.
// Firmware를 Write하기 전에 필요한 사전처리는 함수 호출전 준비되는 것으로 가정한다.
// Write이후의 Verify도 처리한 후 결과를 반환한다.
// Touch가 정상동작하기 위한 후속처리는 고려하지 않는다.
//==========================================================
error_type synaptics_ts_fw_upgrade(struct i2c_client *client)
{
    struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
 
	ts->fw_flag = 0;
	ts->is_probed = 0;
	ts->is_init = 0; /* During upgrading, interrupt will be ignored. */
	need_scan_pdt = true;
	DO_SAFE(FirmwareUpgrade(ts, ts->touch_fw_info->fw_path), error);
	return NO_ERROR;
error:
	return ERROR;    
}


//==========================================================
// 사용될 가능성은 적지만, 혹시나 몰라서 일단 남겨 둠
//==========================================================
error_type synaptics_ts_suspend(struct i2c_client *client)
{
	
    
    struct synaptics_ts_data *ts =
            (struct synaptics_ts_data *)get_touch_handle(client);
    LGTC_ENTRY();
        if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted
                && prox_fhandler.initialized)
            prox_fhandler.exp_fn->suspend(ts);
    
        /*because s3621 doesn't support knock-on*/
        if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6)))
            return NO_ERROR;
    
        if (!atomic_read(&ts->lpwg_ctrl.is_suspend)) {
            DO_SAFE(lpwg_control(ts), error);
            atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
        }
        ts->lpwg_ctrl.screen = 0;
	LGTC_EXIT();
	return NO_ERROR;
error:
	return ERROR;
}

//==========================================================
// 사용될 가능성은 적지만, 혹시나 몰라서 일단 남겨 둠
//==========================================================
error_type synaptics_ts_resume(struct i2c_client *client)
{
	
    
    struct synaptics_ts_data *ts =
            (struct synaptics_ts_data *)get_touch_handle(client);
    LGTC_ENTRY();
        if (ts->pdata->role->use_hover_finger && prox_fhandler.inserted
                && prox_fhandler.initialized)
            prox_fhandler.exp_fn->resume(ts);
    
        cancel_delayed_work_sync(&ts->work_timer);
    
        if (wake_lock_active(&ts->timer_wake_lock))
            wake_unlock(&ts->timer_wake_lock);
        atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
        ts->lpwg_ctrl.screen = 1;
	LGTC_EXIT();

	return NO_ERROR;
}

//==========================================================
// LPGW Command를 처리하는 함수
// 중복된 Command가 오는 것에 대한 방어 처리를 하면 더 좋을 것임.
//==========================================================
error_type synaptics_ts_lpwg(struct i2c_client *client, E_TouchReportMode reportMode, LGTcLpwgSetting  *pLpwgSetting)
{
    struct synaptics_ts_data *ts
		= (struct synaptics_ts_data *)get_touch_handle(client);

	int result = NO_ERROR;

	LGTC_FUN();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LGTcLpwgSetting));

	if( ts->reportMode != reportMode )
	{
		ts->reportMode = reportMode;
		result = lpwg_control(ts);
	}
	
	return result;
}

error_type synaptics_f54_test(struct i2c_client *client,
	char* buf, int* raw_status, int* ch_status)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int full_raw_cap = 0;
	int trx_to_trx = 0;
	int high_resistance = 0;

//	SCAN_PDT(client);

//	full_raw_cap = F54Test('a', 0, buf);
	msleep(30);

//	high_resistance = F54Test('g', 0, buf);
	msleep(50);

//	trx_to_trx = F54Test('f', 0, buf);
	msleep(30);

//	Reset();

	*raw_status = full_raw_cap;
	*ch_status = trx_to_trx && high_resistance;

	synaptics_ts_init(ts->client);
	msleep(30);

	return NO_ERROR;
}

struct touch_device_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe, /* Phone Power ON시 한 번만 수행하는 초기화 코드 */
	.remove		= synaptics_ts_remove, /* Phone Power OFF시 한 번만 수행하는 마무리 코드 */
	.suspend	= synaptics_ts_suspend, /* Suspend 진입에 필요한 처리를 수행하는 코드 */
	.resume		= synaptics_ts_resume, /* Resume시 필요한 처리를 수행하는 코드 */
	.init  		= synaptics_ts_init, /* Touch IC Reset이후 필요한 초기화 코드 */
	.isr  		= synaptics_ts_isr,  /* Interrupt 발생시 필요한 Data를 읽기위한 코드 ( ISR ) */
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.fw_upgrade 	= synaptics_ts_fw_upgrade, //CHECK
	.lpwg		= synaptics_ts_lpwg, /* Touch IC에 Command를 전달하기 위한 코드 ( LPWG / Debugging / ... ) ==> 너무 비대해지지 않도록 분리 검토 필요 */
	.sd			= synaptics_f54_test
};

/* End Of File */


