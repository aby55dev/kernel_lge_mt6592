/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC). 
*
*****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   CMOS sensor header file
 *
 ****************************************************************************/

 #if 0
#else
#ifndef _IMX091MIPI_SENSOR_H
#define _IMX091MIPI_SENSOR_H

#define IMX091MIPI_DEBUG
#define IMX091MIPI_DRIVER_TRACE
//#define IMX091MIPI_TEST_PATTEM
#ifdef IMX091MIPI_DEBUG
//#define SENSORDB printk
#else
//#define SENSORDB(x,...)
#endif

#define IMX091MIPI_FACTORY_START_ADDR 0
#define IMX091MIPI_ENGINEER_START_ADDR 10

//#define MIPI_INTERFACE

 
typedef enum IMX091MIPI_group_enum
{
  IMX091MIPI_PRE_GAIN = 0,
  IMX091MIPI_CMMCLK_CURRENT,
  IMX091MIPI_FRAME_RATE_LIMITATION,
  IMX091MIPI_REGISTER_EDITOR,
  IMX091MIPI_GROUP_TOTAL_NUMS
} IMX091MIPI_FACTORY_GROUP_ENUM;

typedef enum IMX091MIPI_register_index
{
  IMX091MIPI_SENSOR_BASEGAIN = IMX091MIPI_FACTORY_START_ADDR,
  IMX091MIPI_PRE_GAIN_R_INDEX,
  IMX091MIPI_PRE_GAIN_Gr_INDEX,
  IMX091MIPI_PRE_GAIN_Gb_INDEX,
  IMX091MIPI_PRE_GAIN_B_INDEX,
  IMX091MIPI_FACTORY_END_ADDR
} IMX091MIPI_FACTORY_REGISTER_INDEX;

typedef enum IMX091MIPI_engineer_index
{
  IMX091MIPI_CMMCLK_CURRENT_INDEX = IMX091MIPI_ENGINEER_START_ADDR,
  IMX091MIPI_ENGINEER_END
} IMX091MIPI_FACTORY_ENGINEER_INDEX;

typedef struct _sensor_data_struct
{
  SENSOR_REG_STRUCT reg[IMX091MIPI_ENGINEER_END];
  SENSOR_REG_STRUCT cct[IMX091MIPI_FACTORY_END_ADDR];
} sensor_data_struct;
typedef enum {
    IMX091MIPI_SENSOR_MODE_INIT = 0,
    IMX091MIPI_SENSOR_MODE_PREVIEW,
    IMX091MIPI_SENSOR_MODE_VIDEO,
    IMX091MIPI_SENSOR_MODE_CAPTURE
} IMX091MIPI_SENSOR_MODE;


/* SENSOR PREVIEW/CAPTURE VT CLOCK */
//#define IMX091MIPI_PREVIEW_CLK                     69333333  //48100000
//#define IMX091MIPI_CAPTURE_CLK                     69333333  //48100000

//#define IMX091MIPI_COLOR_FORMAT                    SENSOR_OUTPUT_FORMAT_RAW_B //SENSOR_OUTPUT_FORMAT_RAW_R
#define IMX091MIPI_COLOR_FORMAT                    SENSOR_OUTPUT_FORMAT_RAW_R

#define IMX091MIPI_MIN_ANALOG_GAIN				1	/* 1x */
#define IMX091MIPI_MAX_ANALOG_GAIN				8	/* 32x */


/* FRAME RATE UNIT */
#define IMX091MIPI_FPS(x)                          (10 * (x))

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
//#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          2700 /* 9 fps */

//#define IMX091MIPI_DEBUG_SETTING
//#define IMX091_BINNING_SUM//binning: enable  for sum, disable for vertical averag	
#define IMX091MIPI_4LANE
//#define PCLK_259200000
//#define PCLK_254400000
//#define PCLK_249600000
//#define PCLK_244800000
//#define PCLK_240000000
#define PCLK_225600000
//#define PCLK_220800000
//#define PCLK_163200000


//#define IMX091MIPI_PREVIEW_CLK 138666667  //65000000
//#define IMX091MIPI_CAPTURE_CLK 134333333  //117000000  //69333333
//#define IMX091MIPI_DEBUG_SETTING
#ifndef IMX091MIPI_4LANE
#define IMX091MIPI_PREVIEW_CLK 97200000  //65000000
#define IMX091MIPI_CAPTURE_CLK 168000000  //117000000  //69333333
#define IMX091MIPI_ZSD_PRE_CLK 97200000 //117000000
#define IMX091MIPI_VIDEO_CLK   97200000 //117000000
#else
#define IMX091MIPI_PREVIEW_CLK 225600000  //65000000
//#define IMX091MIPI_CAPTURE_CLK 225600000  //225600000  //225600000
//#define IMX091MIPI_ZSD_PRE_CLK 225600000 //117000000
//#define IMX091MIPI_CAPTURE_CLK 220800000  //225600000  //225600000
//#define IMX091MIPI_ZSD_PRE_CLK 220800000 //117000000
//#define IMX091MIPI_CAPTURE_CLK 235200000  //225600000  //225600000
//#define IMX091MIPI_ZSD_PRE_CLK 235200000 //117000000


#if 0//def PCLK_259200000//error
#define IMX091MIPI_CAPTURE_CLK 259200000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 259200000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (5208)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           (3316)//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#ifdef PCLK_259200000
#define IMX091MIPI_CAPTURE_CLK 259200000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 259200000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (5420)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           (3146)//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         (4208-8)//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        (3120-4)//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif

#ifdef PCLK_244800000
#define IMX091MIPI_CAPTURE_CLK 240000000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 240000000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          5220//(4620+500)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3146//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         (4208)//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        (3120)//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#if 0//def PCLK_240000000//ok
#define IMX091MIPI_CAPTURE_CLK 240000000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 240000000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          5580//(4620+500)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3212//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#if 0//def PCLK_240000000//err
#define IMX091MIPI_CAPTURE_CLK 240000000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 240000000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          5020//(4620+500)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3146//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#if 0//def PCLK_240000000
#define IMX091MIPI_CAPTURE_CLK 240000000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 240000000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          5580//(4620+500)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3212//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#ifdef PCLK_240000000
#define IMX091MIPI_CAPTURE_CLK 240000000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 240000000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          5120//(4620+500)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3146//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         (4208-16)//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        (3120-12)//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#ifdef PCLK_225600000//15fps
#define IMX091MIPI_CAPTURE_CLK 225600000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 225600000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (4620)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           (3212)//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         (4208-8)//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        (3120-4)//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif

#if 0//def PCLK_225600000//15fps
#define IMX091MIPI_CAPTURE_CLK 225600000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 225600000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (4620)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           (3212)//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif

#ifdef PCLK_220800000
#define IMX091MIPI_CAPTURE_CLK 220800000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 220800000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (4620)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           (3212)//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#ifdef PCLK_163200000
#define IMX091MIPI_CAPTURE_CLK 163200000  //225600000  //225600000
#define IMX091MIPI_ZSD_PRE_CLK 163200000 //117000000
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (4620)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           (3212)//3256//3212//3146//12//30fps
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
#endif
#define IMX091MIPI_VIDEO_CLK   225600000 //117000000
#endif

#ifndef IMX091MIPI_4LANE
#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          4620  //3055 /* 8 fps */
#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3212//30fps

#define IMX091MIPI_PV_PERIOD_PIXEL_NUMS            4620 //1630 /* 30 fps */
#define IMX091MIPI_PV_PERIOD_LINE_NUMS             1610//1260 //984

#define IMX091MIPI_VIDEO_PERIOD_PIXEL_NUMS         4620 //1630 /* 30 fps */
#define IMX091MIPI_VIDEO_PERIOD_LINE_NUMS          1370//1260 //984

#else
//#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          4620  //3055 /* 8 fps */
//#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3212//30fps
//#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          4774  //3055 /* 8 fps */
//#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3150//12//30fps
//#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (4980+1000)  //3055 /* 8 fps */
//#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3146//12//30fps
//#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          5080//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
//#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3146//12//30fps
#ifdef PCLK_240000000
//#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (4620+500)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
//#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           3212//3256//3212//3146//12//30fps
#endif
#ifdef PCLK_259200000
//#define IMX091MIPI_FULL_PERIOD_PIXEL_NUMS          (5208)//(4620+1000)//(4620+500)//(5080+1000)//(4980+1000)  //3055 /* 8 fps */
//#define IMX091MIPI_FULL_PERIOD_LINE_NUMS           (3316)//3256//3212//3146//12//30fps
#endif

#define IMX091MIPI_PV_PERIOD_PIXEL_NUMS            4620 //1630 /* 30 fps */
#define IMX091MIPI_PV_PERIOD_LINE_NUMS             1606//1260 //984

#define IMX091MIPI_VIDEO_PERIOD_PIXEL_NUMS         4620 //1630 /* 30 fps */
#define IMX091MIPI_VIDEO_PERIOD_LINE_NUMS          1606//1260 //984
#endif


/* SENSOR START/END POSITION */

#ifndef IMX091MIPI_4LANE

#define IMX091MIPI_FULL_X_START                    2
#define IMX091MIPI_FULL_Y_START                    (2+16)	//16=14(Effective OB)+2(Effective Pixel side Ignored area)
#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         (4208 - 208) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        (3120 - 120) /* 1920 */
#define IMX091MIPI_PV_X_START                      2

#define IMX091MIPI_PV_Y_START                      (2+8)	//8=7(Effective OB)+1(Effective Pixel side Ignored area)
#define IMX091MIPI_IMAGE_SENSOR_PV_WIDTH           (2104 - 104)  //    (1280 - 16) /* 1264 */
#define IMX091MIPI_IMAGE_SENSOR_PV_HEIGHT          (1560 - 60)  //(960 - 12) /* 948 */

#define IMX091MIPI_VIDEO_X_START                   2
#define IMX091MIPI_VIDEO_Y_START                   (2+8)
#define IMX091MIPI_IMAGE_SENSOR_VIDEO_WIDTH        (2104 - 104)//(3264 - 64) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_VIDEO_HEIGHT       (1184 - 60)//(2448 - 48) /* 1920 */

#else

#define IMX091MIPI_FULL_X_START                    (0)//
#define IMX091MIPI_FULL_Y_START                    (0)//0//2//for test//(2+4+16)	//16=14(Effective OB)+2(Effective Pixel side Ignored area)
//#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         (4096+16)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
//#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        (3072+12)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
//#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         (4096+16)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
//#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        (3072+12)//3060//3072//3060//3100//(3120 - 120) /* 1920 */
//#define IMX091MIPI_IMAGE_SENSOR_FULL_WIDTH         4208//4096//4208//4096//(4096+16*4)//4080//4096//err//4080//4140//(4208 - 208) /* 2560 */
//#define IMX091MIPI_IMAGE_SENSOR_FULL_HEIGHT        3120//3072//3120//3072//(3072+16*3)//3060//3072//3060//3100//(3120 - 120) /* 1920 */

#define IMX091MIPI_PV_X_START                      (0)
#define IMX091MIPI_PV_Y_START                      (0)	//8=7(Effective OB)+1(Effective Pixel side Ignored area)
#define IMX091MIPI_IMAGE_SENSOR_PV_WIDTH           (2104-8)  //    (1280 - 16) /* 1264 */
#define IMX091MIPI_IMAGE_SENSOR_PV_HEIGHT          (1560-4)  //(960 - 12) /* 948 */


#define IMX091MIPI_VIDEO_X_START                      (0)
#define IMX091MIPI_VIDEO_Y_START                      (0)	//8=7(Effective OB)+1(Effective Pixel side Ignored area)
#define IMX091MIPI_IMAGE_SENSOR_VIDEO_WIDTH           (2104-8)  //    (1280 - 16) /* 1264 */
#define IMX091MIPI_IMAGE_SENSOR_VIDEO_HEIGHT          (1560-4)  //(960 - 12) /* 948 */
#if 0
#define IMX091MIPI_VIDEO_X_START                   (0)
#define IMX091MIPI_VIDEO_Y_START                   (0)
#define IMX091MIPI_IMAGE_SENSOR_VIDEO_WIDTH        (2104-8)//(3264 - 64) /* 2560 */
#define IMX091MIPI_IMAGE_SENSOR_VIDEO_HEIGHT       (1184-8)//(2448 - 48) /* 1920 */
#endif
#endif

#define IMX091MIPI_RGB_OFFSET_X                   (0)
#define IMX091MIPI_RGB_OFFSET_Y                   (0)

/* SENSOR READ/WRITE ID */
#define IMX091MIPI_SLAVE_WRITE_ID_1   (0x6c)
#define IMX091MIPI_SLAVE_WRITE_ID_2   (0x20)

#define IMX091MIPI_WRITE_ID   (0x20)  //(0x6c)
#define IMX091MIPI_READ_ID    (0x21)  //(0x6d)

/* SENSOR ID */
//#define IMX091MIPI_SENSOR_ID						(0x5647)


//added by mandrave
//#define IMX091MIPI_USE_OTP

#if defined(IMX091MIPI_USE_OTP)

struct imx091mipi_otp_struct
{
    kal_uint16 customer_id;
	kal_uint16 module_integrator_id;
	kal_uint16 lens_id;
	kal_uint16 rg_ratio;
	kal_uint16 bg_ratio;
	kal_uint16 user_data[5];
	kal_uint16 lenc[63];

};

#define RG_TYPICAL 0x51
#define BG_TYPICAL 0x57


#endif




/* SENSOR PRIVATE STRUCT */
typedef struct IMX091MIPI_sensor_STRUCT
{
  MSDK_SENSOR_CONFIG_STRUCT cfg_data;
  sensor_data_struct eng; /* engineer mode */
  MSDK_SENSOR_ENG_INFO_STRUCT eng_info;
  kal_uint8 mirror;
  kal_bool pv_mode;
  kal_bool video_mode;
  kal_bool is_zsd;
  kal_bool is_zsd_cap;
  kal_bool is_autofliker;
  kal_uint16 normal_fps; /* video normal mode max fps */
  kal_uint16 night_fps; /* video night mode max fps */
  kal_uint16 shutter;
  kal_uint16 gain;
  kal_uint32 pv_pclk;
  kal_uint32 cap_pclk;
  kal_uint32 video_pclk;
  kal_uint32 pclk;
  kal_uint16 frame_length;
  kal_uint16 line_length;  
  kal_uint16 write_id;
  kal_uint16 read_id;
  kal_uint16 dummy_pixels;
  kal_uint16 dummy_lines;
  kal_uint32 max_exposure_lines;
  
  IMX091MIPI_SENSOR_MODE sensorMode;
} IMX091MIPI_sensor_struct;

//export functions
UINT32 IMX091MIPIOpen(void);
UINT32 IMX091MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 IMX091MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 IMX091MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 IMX091MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 IMX091MIPIClose(void);

#define Sleep(ms) mdelay(ms)

#endif 

#endif
