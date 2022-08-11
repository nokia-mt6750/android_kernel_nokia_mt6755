/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5k3p9mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3p9mipiraw_Sensor.h"

#define PDAFSUPPORT   //for pdaf switch
#define PFX "S5K3P9_camera_sensor"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K3P9_SENSOR_ID,      ////Sensor ID Value: 0x3010//record sensor id defined in Kd_imgsensor.h

	.checksum_value =0xB1F1B3CC,

	.pre = {
        .pclk = 560000000,              //record different mode's pclk
		.linelength = 8496,				//record different mode's linelength
		.framelength = 2196,            //record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2320,		//record different mode's width of grabwindow
		.grabwindow_height = 1744,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,           //30fps
	},
	.cap = {
		.pclk =560000000,
		.linelength = 5088,
		.framelength = 3668,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4640,
		.grabwindow_height = 3488,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
#if 0
	.cap1 = {
		.pclk =560000000,
		.linelength = 6400,
		.framelength = 3643,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4640,
		.grabwindow_height = 3488,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
	},
    .cap2 = {
        .pclk =560000000,
        .linelength = 10240,
        .framelength = 3643,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4640,
        .grabwindow_height = 3488,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 150,
	},
#endif
	.normal_video = {
		.pclk =560000000,
		.linelength = 5088,
		.framelength = 3668,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4640,
		.grabwindow_height = 3488,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},

	.hs_video = {/*slow motion*/
		.pclk = 560000000,
		.linelength = 5088,
		.framelength = 917,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1024,
		.grabwindow_height = 768,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
	},
	.slim_video = {/*VT Call*/
		.pclk = 560000000,
		.linelength = 5088,
		.framelength = 917,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1024,
		.grabwindow_height = 768,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
	},

	.margin = 4,                    //sensor framelength & shutter margin
	.min_shutter = 4,               //min shutter
	.max_frame_length = 0xffff-5,     //max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,       //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame =0, //sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,    //isp gain delay frame for AE cycle
	.ihdr_support = 0,	            //1, support; 0,not support
	.ihdr_le_firstline = 0,         //1,le first ; 0, se first
	.sensor_mode_num = 5,	        //support sensor mode num

	.cap_delay_frame = 3,           //enter capture delay frame num
	.pre_delay_frame = 3,           //enter preview delay frame num
	.video_delay_frame = 3,         //enter video delay frame num
	.hs_video_delay_frame = 3,      //enter high speed video  delay frame num
	.slim_video_delay_frame = 3,    //enter slim video delay frame num

	.isp_driving_current = ISP_DRIVING_2MA,     //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,    //sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2,        //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 0,                //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,                                 //mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20,0x5a,0xff},//record sensor support all write id addr, only supprt 4 must end with 0xff
	.i2c_speed = 400,
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,                   //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,        //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,          //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW, //current scenario id
	.ihdr_mode = KAL_FALSE,             //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,               //record current sensor's i2c write id
};


int chip_id = 0;
// VC_Num, VC_PixelNum, ModeSelect, EXPO_Ratio, ODValue, RG_STATSMODE
// VC0_ID, VC0_DataType, VC0_SIZEH, VC0_SIZE, VC1_ID, VC1_DataType, VC1_SIZEH, VC1_SIZEV
// VC2_ID, VC2_DataType, VC2_SIZEH, VC2_SIZE, VC3_ID, VC3_DataType, VC3_SIZEH, VC3_SIZEV
static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
{// Preview mode setting
 {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
  0x00, 0x2B, 0x0910, 0x06D0, 0x01, 0x00, 0x0000, 0x0000,
  0x01, 0x30, 0x0168, 0x0360, 0x03, 0x00, 0x0000, 0x0000},
  // Video mode setting
 {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
  0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
  0x01, 0x30, 0x0168, 0x0360, 0x03, 0x00, 0x0000, 0x0000},
  // Capture mode setting
 {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
  0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
  0x01, 0x30, 0x0168, 0x0360, 0x03, 0x00, 0x0000, 0x0000}};

/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320, 1744, 0000, 0000, 2320, 1744,	    0,	0, 2320, 1744}, // Preview
	{ 4640, 3488,	  0,	0, 4640, 3488, 4640, 3488, 0000, 0000, 4640, 3488,	    0,	0, 4640, 3488}, // capture
	{ 4640, 3488,	  0,	0, 4640, 3488, 4640, 3488, 0000, 0000, 4640, 3488,	    0,	0, 4640, 3488}, // video
	{ 4640, 3488,   784,  592, 3072, 2304, 1024,   768, 0000, 0000, 1024,  768,     0,  0, 1024,  768}, // hight speed video
	{ 4640, 3488,   784,  592, 3072, 2304, 1024,   768, 0000, 0000, 1024,  768,     0,  0, 1024,  768}, // slim video reuse highspeed
//	{ 4640, 3488,   400,  664, 3840, 2160, 1920,  1080, 0000, 0000, 1920, 1080,     0,  0, 1920, 1080}, // slim video
};


static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 16,
    .i4OffsetY = 16,
    .i4PitchX  = 32,
    .i4PitchY  = 16,
    .i4PairNum  =4,
    .i4SubBlkW  =8,
    .i4SubBlkH  =16,
 		.i4PosL = {{20,19},{28,31},{36,19},{44,31}},
		.i4PosR = {{20,27},{28,23},{36,27},{44,23}},
	.iMirrorFlip = 0,
	.i4BlockNumX = 144,
	.i4BlockNumY = 216,
};

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern bool read_3P8_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);

#if 0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
      iReadReg((u16) addr ,(u8*)&get_byte, imgsensor.i2c_write_id);
      return get_byte;
}

#define write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1,  imgsensor.i2c_write_id)
#endif
#define RWB_ID_OFFSET 0x0F73
#define EEPROM_READ_ID  0xA0
#define EEPROM_WRITE_ID   0xA1

#if 0
static kal_uint16 is_RWB_sensor(void)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(RWB_ID_OFFSET >> 8) , (char)(RWB_ID_OFFSET & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,EEPROM_READ_ID);
	return get_byte;
}
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}
static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	//return; //for test
     write_cmos_sensor(0x0340, imgsensor.frame_length);
     write_cmos_sensor(0x0342, imgsensor.line_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{

    kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		// Extend frame length
            write_cmos_sensor(0x0340, imgsensor.frame_length);

	    }
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length);
        LOG_INF("(else)imgsensor.frame_length = %d\n", imgsensor.frame_length);

	}
	// Update Shutter
    write_cmos_sensor(0x0202, shutter);
	LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}	/*	write_shutter  */
int read_frame_num(void)
{
	kal_uint16 num;
	num = read_cmos_sensor(0x0005);
	printk("songbl the frame num is %d\n",num);
	return num;
        
}

EXPORT_SYMBOL(read_frame_num);
/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;

    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    //gain=1024;//for test
    //return; //for test

    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor(0x0204,reg_gain);
    //write_cmos_sensor_8(0x0204,(reg_gain>>8));
    //write_cmos_sensor_8(0x0205,(reg_gain&0xff));

    return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{

	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp=read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch(image_mirror)
		{

		   case IMAGE_NORMAL:
		   	     write_cmos_sensor_8(0x0101, itemp);
			      break;

		   case IMAGE_V_MIRROR:
			     write_cmos_sensor_8(0x0101, itemp | 0x02);
			     break;

		   case IMAGE_H_MIRROR:
			     write_cmos_sensor_8(0x0101, itemp | 0x01);
			     break;

		   case IMAGE_HV_MIRROR:
			     write_cmos_sensor_8(0x0101, itemp | 0x03);
			     break;
		}
}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif



/*************************************************************************
* FUNCTION
*	check_stremoff
*
* DESCRIPTION
*	waiting function until sensor streaming finish.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void check_stremoff(void)
{
    unsigned int i=0,framecnt=0;
    for(i=0;i<100;i++)
    {
        framecnt = read_cmos_sensor_8(0x0005);
        if(framecnt == 0xFF)
            return;
        write_cmos_sensor_8(0x0100,0x00);
    }
	LOG_INF(" Stream Off Fail!\n");
    return;
}


static void sensor_init(void)
{
	LOG_INF("sensor_init() E\n");
	chip_id = read_cmos_sensor_8(0x0002);
	LOG_INF("chip id:%d E\n", chip_id);
	
	write_cmos_sensor(0x6028, 0x4000);  
	write_cmos_sensor(0x6010, 0x0001); 
	mdelay(3);                         
	write_cmos_sensor(0x6214, 0x7970); 
	write_cmos_sensor(0x6218, 0x7150); 
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x3F4C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x6F12, 0x0548);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0xC405);
	write_cmos_sensor(0x6F12, 0x0549);
	write_cmos_sensor(0x6F12, 0x081A);
	write_cmos_sensor(0x6F12, 0x0349);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0xC805);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD7BB);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x4930);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x6C00);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x40BA);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xC0BA);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF047);
	write_cmos_sensor(0x6F12, 0x1C46);
	write_cmos_sensor(0x6F12, 0x9046);
	write_cmos_sensor(0x6F12, 0x8946);
	write_cmos_sensor(0x6F12, 0x0746);
	write_cmos_sensor(0x6F12, 0xFE48);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x4068);
	write_cmos_sensor(0x6F12, 0x86B2);
	write_cmos_sensor(0x6F12, 0x050C);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x37FC);
	write_cmos_sensor(0x6F12, 0x2346);
	write_cmos_sensor(0x6F12, 0x4246);
	write_cmos_sensor(0x6F12, 0x4946);
	write_cmos_sensor(0x6F12, 0x3846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x36FC);
	write_cmos_sensor(0x6F12, 0xF848);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8B02);
	write_cmos_sensor(0x6F12, 0x88B1);
	write_cmos_sensor(0x6F12, 0x788A);
	write_cmos_sensor(0x6F12, 0x04F1);
	write_cmos_sensor(0x6F12, 0x0054);
	write_cmos_sensor(0x6F12, 0x04EB);
	write_cmos_sensor(0x6F12, 0x8001);
	write_cmos_sensor(0x6F12, 0x09E0);
	write_cmos_sensor(0x6F12, 0x2268);
	write_cmos_sensor(0x6F12, 0xC2F3);
	write_cmos_sensor(0x6F12, 0xC360);
	write_cmos_sensor(0x6F12, 0x90FA);
	write_cmos_sensor(0x6F12, 0xA0F0);
	write_cmos_sensor(0x6F12, 0x22F0);
	write_cmos_sensor(0x6F12, 0x7842);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x5000);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x6F12, 0x8C42);
	write_cmos_sensor(0x6F12, 0xF3D1);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF047);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x14BC);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xFC5F);
	write_cmos_sensor(0x6F12, 0x8346);
	write_cmos_sensor(0x6F12, 0xE748);
	write_cmos_sensor(0x6F12, 0x8A46);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x8068);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x80B2);
	write_cmos_sensor(0x6F12, 0xCDE9);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x0198);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x05FC);
	write_cmos_sensor(0x6F12, 0xABFB);
	write_cmos_sensor(0x6F12, 0x0A10);
	write_cmos_sensor(0x6F12, 0xE24B);
	write_cmos_sensor(0x6F12, 0xE04D);
	write_cmos_sensor(0x6F12, 0xE04A);
	write_cmos_sensor(0x6F12, 0x93F8);
	write_cmos_sensor(0x6F12, 0x9160);
	write_cmos_sensor(0x6F12, 0x05F5);
	write_cmos_sensor(0x6F12, 0xAA69);
	write_cmos_sensor(0x6F12, 0x06FB);
	write_cmos_sensor(0x6F12, 0x0BF6);
	write_cmos_sensor(0x6F12, 0x0023);
	write_cmos_sensor(0x6F12, 0x891B);
	write_cmos_sensor(0x6F12, 0x4D46);
	write_cmos_sensor(0x6F12, 0x60EB);
	write_cmos_sensor(0x6F12, 0x0300);
	write_cmos_sensor(0x6F12, 0x03C5);
	write_cmos_sensor(0x6F12, 0x1D46);
	write_cmos_sensor(0x6F12, 0xEBFB);
	write_cmos_sensor(0x6F12, 0x0A65);
	write_cmos_sensor(0x6F12, 0x02F5);
	write_cmos_sensor(0x6F12, 0xAB67);
	write_cmos_sensor(0x6F12, 0x3A46);
	write_cmos_sensor(0x6F12, 0xD64C);
	write_cmos_sensor(0x6F12, 0x60C2);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0x4835);
	write_cmos_sensor(0x6F12, 0x04F5);
	write_cmos_sensor(0x6F12, 0xA962);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0xA0C4);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0xA144);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0xF858);
	write_cmos_sensor(0x6F12, 0xBCF1);
	write_cmos_sensor(0x6F12, 0x010F);
	write_cmos_sensor(0x6F12, 0x03D0);
	write_cmos_sensor(0x6F12, 0xBCF1);
	write_cmos_sensor(0x6F12, 0x020F);
	write_cmos_sensor(0x6F12, 0x14D0);
	write_cmos_sensor(0x6F12, 0x29E0);
	write_cmos_sensor(0x6F12, 0x08EA);
	write_cmos_sensor(0x6F12, 0x0423);
	write_cmos_sensor(0x6F12, 0x43F0);
	write_cmos_sensor(0x6F12, 0x1103);
	write_cmos_sensor(0x6F12, 0x1380);
	write_cmos_sensor(0x6F12, 0x2346);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xDCFB);
	write_cmos_sensor(0x6F12, 0xC9E9);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x2346);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD4FB);
	write_cmos_sensor(0x6F12, 0xC7E9);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x15E0);
	write_cmos_sensor(0x6F12, 0x08EA);
	write_cmos_sensor(0x6F12, 0x042C);
	write_cmos_sensor(0x6F12, 0x4CF0);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0xA2F8);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x6F12, 0xA1FB);
	write_cmos_sensor(0x6F12, 0x042C);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x04C0);
	write_cmos_sensor(0x6F12, 0x01FB);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x6F12, 0xC9E9);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x6F12, 0xA6FB);
	write_cmos_sensor(0x6F12, 0x0401);
	write_cmos_sensor(0x6F12, 0x05FB);
	write_cmos_sensor(0x6F12, 0x0411);
	write_cmos_sensor(0x6F12, 0x06FB);
	write_cmos_sensor(0x6F12, 0x0311);
	write_cmos_sensor(0x6F12, 0xC7E9);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0xB848);
	write_cmos_sensor(0x6F12, 0xB949);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x4805);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0xB848);
	write_cmos_sensor(0x6F12, 0x0CC8);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x2200);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB5FB);
	write_cmos_sensor(0x6F12, 0xB548);
	write_cmos_sensor(0x6F12, 0x0830);
	write_cmos_sensor(0x6F12, 0x0CC8);
	write_cmos_sensor(0x6F12, 0x48F6);
	write_cmos_sensor(0x6F12, 0x2A00);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xAEFB);
	write_cmos_sensor(0x6F12, 0x5846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB0FB);
	write_cmos_sensor(0x6F12, 0xAD49);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0x68A5);
	write_cmos_sensor(0x6F12, 0xDDE9);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0x02B0);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF05F);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x91BB);
	write_cmos_sensor(0x6F12, 0xA84A);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0xD525);
	write_cmos_sensor(0x6F12, 0x2AB1);
	write_cmos_sensor(0x6F12, 0xA64A);
	write_cmos_sensor(0x6F12, 0xA54B);
	write_cmos_sensor(0x6F12, 0xD2F8);
	write_cmos_sensor(0x6F12, 0x6825);
	write_cmos_sensor(0x6F12, 0xC3F8);
	write_cmos_sensor(0x6F12, 0x3024);
	write_cmos_sensor(0x6F12, 0xA34A);
	write_cmos_sensor(0x6F12, 0xD2F8);
	write_cmos_sensor(0x6F12, 0x3024);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9BBB);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0xA049);
	write_cmos_sensor(0x6F12, 0xA34A);
	write_cmos_sensor(0x6F12, 0xA44B);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3C14);
	write_cmos_sensor(0x6F12, 0x947C);
	write_cmos_sensor(0x6F12, 0x0CB1);
	write_cmos_sensor(0x6F12, 0x908A);
	write_cmos_sensor(0x6F12, 0x1BE0);
	write_cmos_sensor(0x6F12, 0x9B4A);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0xA220);
	write_cmos_sensor(0x6F12, 0xC2F1);
	write_cmos_sensor(0x6F12, 0x0C02);
	write_cmos_sensor(0x6F12, 0xD140);
	write_cmos_sensor(0x6F12, 0x4843);
	write_cmos_sensor(0x6F12, 0x010A);
	write_cmos_sensor(0x6F12, 0x9D48);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x8400);
	write_cmos_sensor(0x6F12, 0x0279);
	write_cmos_sensor(0x6F12, 0x4A43);
	write_cmos_sensor(0x6F12, 0x4179);
	write_cmos_sensor(0x6F12, 0xC088);
	write_cmos_sensor(0x6F12, 0xCA40);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x1210);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8021);
	write_cmos_sensor(0x6F12, 0xB1FB);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x6F12, 0x0911);
	write_cmos_sensor(0x6F12, 0x8842);
	write_cmos_sensor(0x6F12, 0x04D2);
	write_cmos_sensor(0x6F12, 0x4028);
	write_cmos_sensor(0x6F12, 0x00D8);
	write_cmos_sensor(0x6F12, 0x4020);
	write_cmos_sensor(0x6F12, 0x5880);
	write_cmos_sensor(0x6F12, 0x10BD);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0xFBE7);
	write_cmos_sensor(0x6F12, 0x4168);
	write_cmos_sensor(0x6F12, 0x4A7B);
	write_cmos_sensor(0x6F12, 0x9149);
	write_cmos_sensor(0x6F12, 0xA1F8);
	write_cmos_sensor(0x6F12, 0x8223);
	write_cmos_sensor(0x6F12, 0x4268);
	write_cmos_sensor(0x6F12, 0x537B);
	write_cmos_sensor(0x6F12, 0x002B);
	write_cmos_sensor(0x6F12, 0x15D0);
	write_cmos_sensor(0x6F12, 0x01F5);
	write_cmos_sensor(0x6F12, 0x6171);
	write_cmos_sensor(0x6F12, 0x927B);
	write_cmos_sensor(0x6F12, 0x0A80);
	write_cmos_sensor(0x6F12, 0x4068);
	write_cmos_sensor(0x6F12, 0xC07B);
	write_cmos_sensor(0x6F12, 0x4880);
	write_cmos_sensor(0x6F12, 0x8B48);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xC220);
	write_cmos_sensor(0x6F12, 0x8A80);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xC420);
	write_cmos_sensor(0x6F12, 0xCA80);
	write_cmos_sensor(0x6F12, 0x10F8);
	write_cmos_sensor(0x6F12, 0xC72F);
	write_cmos_sensor(0x6F12, 0xC078);
	write_cmos_sensor(0x6F12, 0x5208);
	write_cmos_sensor(0x6F12, 0x4008);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x0881);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xFF4F);
	write_cmos_sensor(0x6F12, 0x8348);
	write_cmos_sensor(0x6F12, 0x83B0);
	write_cmos_sensor(0x6F12, 0x1D46);
	write_cmos_sensor(0x6F12, 0xC079);
	write_cmos_sensor(0x6F12, 0xDDF8);
	write_cmos_sensor(0x6F12, 0x44B0);
	write_cmos_sensor(0x6F12, 0x1646);
	write_cmos_sensor(0x6F12, 0x0F46);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x6ED0);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0xF4A1);
	write_cmos_sensor(0x6F12, 0x0AF1);
	write_cmos_sensor(0x6F12, 0xBA0A);
	write_cmos_sensor(0x6F12, 0xAAF1);
	write_cmos_sensor(0x6F12, 0x1C00);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x0090);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x0480);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x3EFB);
	write_cmos_sensor(0x6F12, 0x0399);
	write_cmos_sensor(0x6F12, 0x109C);
	write_cmos_sensor(0x6F12, 0x0843);
	write_cmos_sensor(0x6F12, 0x04F1);
	write_cmos_sensor(0x6F12, 0x8044);
	write_cmos_sensor(0x6F12, 0x07D0);
	write_cmos_sensor(0x6F12, 0xA780);
	write_cmos_sensor(0x6F12, 0xE680);
	write_cmos_sensor(0x6F12, 0xAAF1);
	write_cmos_sensor(0x6F12, 0x1C00);
	write_cmos_sensor(0x6F12, 0x0188);
	write_cmos_sensor(0x6F12, 0x2181);
	write_cmos_sensor(0x6F12, 0x8088);
	write_cmos_sensor(0x6F12, 0x20E0);
	write_cmos_sensor(0x6F12, 0x6848);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0C10);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xD801);
	write_cmos_sensor(0x6F12, 0x4843);
	write_cmos_sensor(0x6F12, 0x0290);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x2CFB);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0298);
	write_cmos_sensor(0x6F12, 0x01D0);
	write_cmos_sensor(0x6F12, 0x361A);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0744);
	write_cmos_sensor(0x6F12, 0xA780);
	write_cmos_sensor(0x6F12, 0xE680);
	write_cmos_sensor(0x6F12, 0x6048);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xDA61);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8A02);
	write_cmos_sensor(0x6F12, 0x4643);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x21FB);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0xA8EB);
	write_cmos_sensor(0x6F12, 0x0608);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0xB144);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0x0890);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x6081);
	write_cmos_sensor(0x6F12, 0x0398);
	write_cmos_sensor(0x6F12, 0x28B1);
	write_cmos_sensor(0x6F12, 0x5648);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x4F11);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8902);
	write_cmos_sensor(0x6F12, 0x03E0);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0D10);
	write_cmos_sensor(0x6F12, 0x9AF8);
	write_cmos_sensor(0x6F12, 0x0C00);
	write_cmos_sensor(0x6F12, 0x0A01);
	write_cmos_sensor(0x6F12, 0x5149);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x4E11);
	write_cmos_sensor(0x6F12, 0x42EA);
	write_cmos_sensor(0x6F12, 0x8121);
	write_cmos_sensor(0x6F12, 0x41F0);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x6F12, 0xA181);
	write_cmos_sensor(0x6F12, 0x0121);
	write_cmos_sensor(0x6F12, 0xFF22);
	write_cmos_sensor(0x6F12, 0x02EB);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x41EA);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xE081);
	write_cmos_sensor(0x6F12, 0x01A9);
	write_cmos_sensor(0x6F12, 0x6846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xFDFA);
	write_cmos_sensor(0x6F12, 0x9DF8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x9DF8);
	write_cmos_sensor(0x6F12, 0x0410);
	write_cmos_sensor(0x6F12, 0x40EA);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x2082);
	write_cmos_sensor(0x6F12, 0x5F46);
	write_cmos_sensor(0x6F12, 0x3E46);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xEDFA);
	write_cmos_sensor(0x6F12, 0x791E);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0xA889);
	write_cmos_sensor(0x6F12, 0x04D0);
	write_cmos_sensor(0x6F12, 0x4718);
	write_cmos_sensor(0x6F12, 0x46F6);
	write_cmos_sensor(0x6F12, 0xA410);
	write_cmos_sensor(0x6F12, 0x03E0);
	write_cmos_sensor(0x6F12, 0x3FE0);
	write_cmos_sensor(0x6F12, 0x4618);
	write_cmos_sensor(0x6F12, 0x46F6);
	write_cmos_sensor(0x6F12, 0x2410);
	write_cmos_sensor(0x6F12, 0xA880);
	write_cmos_sensor(0x6F12, 0x6782);
	write_cmos_sensor(0x6F12, 0xE682);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xA082);
	write_cmos_sensor(0x6F12, 0xA888);
	write_cmos_sensor(0x6F12, 0x2080);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE2FA);
	write_cmos_sensor(0x6F12, 0x0128);
	write_cmos_sensor(0x6F12, 0x0CD1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE3FA);
	write_cmos_sensor(0x6F12, 0x48B1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE5FA);
	write_cmos_sensor(0x6F12, 0x30B1);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x1340);
	write_cmos_sensor(0x6F12, 0xA081);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x0110);
	write_cmos_sensor(0x6F12, 0xE081);
	write_cmos_sensor(0x6F12, 0x2082);
	write_cmos_sensor(0x6F12, 0x2B6A);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x8320);
	write_cmos_sensor(0x6F12, 0x109A);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xDCFA);
	write_cmos_sensor(0x6F12, 0xE881);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCAFA);
	write_cmos_sensor(0x6F12, 0x0126);
	write_cmos_sensor(0x6F12, 0x0128);
	write_cmos_sensor(0x6F12, 0x12D1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCAFA);
	write_cmos_sensor(0x6F12, 0x78B1);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCCFA);
	write_cmos_sensor(0x6F12, 0x60B1);
	write_cmos_sensor(0x6F12, 0x2680);
	write_cmos_sensor(0x6F12, 0x3048);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x04E0);
	write_cmos_sensor(0x6F12, 0x0288);
	write_cmos_sensor(0x6F12, 0x5208);
	write_cmos_sensor(0x6F12, 0x20F8);
	write_cmos_sensor(0x6F12, 0x022B);
	write_cmos_sensor(0x6F12, 0x491C);
	write_cmos_sensor(0x6F12, 0xEA89);
	write_cmos_sensor(0x6F12, 0xB1EB);
	write_cmos_sensor(0x6F12, 0x420F);
	write_cmos_sensor(0x6F12, 0xF6DB);
	write_cmos_sensor(0x6F12, 0xE989);
	write_cmos_sensor(0x6F12, 0xA889);
	write_cmos_sensor(0x6F12, 0x8142);
	write_cmos_sensor(0x6F12, 0x00D9);
	write_cmos_sensor(0x6F12, 0xE881);
	write_cmos_sensor(0x6F12, 0x2680);
	write_cmos_sensor(0x6F12, 0x07B0);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF08F);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF843);
	write_cmos_sensor(0x6F12, 0x1A48);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x8069);
	write_cmos_sensor(0x6F12, 0x85B2);
	write_cmos_sensor(0x6F12, 0x4FEA);
	write_cmos_sensor(0x6F12, 0x1048);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x6DFA);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB1FA);
	write_cmos_sensor(0x6F12, 0x204F);
	write_cmos_sensor(0x6F12, 0x97F8);
	write_cmos_sensor(0x6F12, 0x7300);
	write_cmos_sensor(0x6F12, 0x30B1);
	write_cmos_sensor(0x6F12, 0x1348);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x8B02);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0x1D49);
	write_cmos_sensor(0x6F12, 0x1B20);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x1C48);
	write_cmos_sensor(0x6F12, 0x0E4E);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0xC046);
	write_cmos_sensor(0x6F12, 0x7088);
	write_cmos_sensor(0x6F12, 0x98B9);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xADF8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0B48);
	write_cmos_sensor(0x6F12, 0x0222);
	write_cmos_sensor(0x6F12, 0x6946);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x0006);
	write_cmos_sensor(0x6F12, 0x2E30);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9AFA);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0xBDF8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x7080);
	write_cmos_sensor(0x6F12, 0x7088);
	write_cmos_sensor(0x6F12, 0x10B9);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8060);
	write_cmos_sensor(0x6F12, 0x7080);
	write_cmos_sensor(0x6F12, 0x97F8);
	write_cmos_sensor(0x6F12, 0x7500);
	write_cmos_sensor(0x6F12, 0x20B3);
	write_cmos_sensor(0x6F12, 0x1DE0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x4900);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8832);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3420);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x21A0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3F40);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3E70);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xA000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2210);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2850);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xF47E);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0FE0);
	write_cmos_sensor(0x6F12, 0x7088);
	write_cmos_sensor(0x6F12, 0x18B1);
	write_cmos_sensor(0x6F12, 0x6043);
	write_cmos_sensor(0x6F12, 0x00F5);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x840A);
	write_cmos_sensor(0x6F12, 0xF648);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8072);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x7C07);
	write_cmos_sensor(0x6F12, 0x9042);
	write_cmos_sensor(0x6F12, 0x01D9);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x1146);
	write_cmos_sensor(0x6F12, 0x8B01);
	write_cmos_sensor(0x6F12, 0xA3F5);
	write_cmos_sensor(0x6F12, 0x8043);
	write_cmos_sensor(0x6F12, 0x9042);
	write_cmos_sensor(0x6F12, 0x01D9);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x1146);
	write_cmos_sensor(0x6F12, 0x01FB);
	write_cmos_sensor(0x6F12, 0x0431);
	write_cmos_sensor(0x6F12, 0xFF23);
	write_cmos_sensor(0x6F12, 0xB3EB);
	write_cmos_sensor(0x6F12, 0x112F);
	write_cmos_sensor(0x6F12, 0x0ED9);
	write_cmos_sensor(0x6F12, 0x9042);
	write_cmos_sensor(0x6F12, 0x01D9);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x1146);
	write_cmos_sensor(0x6F12, 0x8901);
	write_cmos_sensor(0x6F12, 0xA1F5);
	write_cmos_sensor(0x6F12, 0x8041);
	write_cmos_sensor(0x6F12, 0x9042);
	write_cmos_sensor(0x6F12, 0x00D8);
	write_cmos_sensor(0x6F12, 0x1046);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x0410);
	write_cmos_sensor(0x6F12, 0x000A);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0xFF20);
	write_cmos_sensor(0x6F12, 0xE349);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF843);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xEDB9);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0xDF48);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xC169);
	write_cmos_sensor(0x6F12, 0x0C0C);
	write_cmos_sensor(0x6F12, 0x8DB2);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE3F9);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x31FA);
	write_cmos_sensor(0x6F12, 0xDB4A);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0x7400);
	write_cmos_sensor(0x6F12, 0x10B1);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x82F8);
	write_cmos_sensor(0x6F12, 0x7000);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x7040);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD3B9);
	write_cmos_sensor(0x6F12, 0xD549);
	write_cmos_sensor(0x6F12, 0x0A8F);
	write_cmos_sensor(0x6F12, 0x498F);
	write_cmos_sensor(0x6F12, 0x801A);
	write_cmos_sensor(0x6F12, 0xD44A);
	write_cmos_sensor(0x6F12, 0x401E);
	write_cmos_sensor(0x6F12, 0xB2F8);
	write_cmos_sensor(0x6F12, 0xB032);
	write_cmos_sensor(0x6F12, 0x1944);
	write_cmos_sensor(0x6F12, 0x8142);
	write_cmos_sensor(0x6F12, 0x00D9);
	write_cmos_sensor(0x6F12, 0x0846);
	write_cmos_sensor(0x6F12, 0xA2F8);
	write_cmos_sensor(0x6F12, 0xB202);
	write_cmos_sensor(0x6F12, 0x7047);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0646);
	write_cmos_sensor(0x6F12, 0xCA48);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x406A);
	write_cmos_sensor(0x6F12, 0x85B2);
	write_cmos_sensor(0x6F12, 0x040C);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB8F9);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x0AFA);
	write_cmos_sensor(0x6F12, 0xC848);
	write_cmos_sensor(0x6F12, 0xC84F);
	write_cmos_sensor(0x6F12, 0x0068);
	write_cmos_sensor(0x6F12, 0x3B68);
	write_cmos_sensor(0x6F12, 0x418B);
	write_cmos_sensor(0x6F12, 0x090A);
	write_cmos_sensor(0x6F12, 0x83F8);
	write_cmos_sensor(0x6F12, 0x3610);
	write_cmos_sensor(0x6F12, 0xC17E);
	write_cmos_sensor(0x6F12, 0x83F8);
	write_cmos_sensor(0x6F12, 0x3810);
	write_cmos_sensor(0x6F12, 0xC149);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x4C21);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3421);
	write_cmos_sensor(0x6F12, 0x01D0);
	write_cmos_sensor(0x6F12, 0x521C);
	write_cmos_sensor(0x6F12, 0x5208);
	write_cmos_sensor(0x6F12, 0xCE33);
	write_cmos_sensor(0x6F12, 0x160A);
	write_cmos_sensor(0x6F12, 0x1E71);
	write_cmos_sensor(0x6F12, 0x9A71);
	write_cmos_sensor(0x6F12, 0xB1F8);
	write_cmos_sensor(0x6F12, 0x3C21);
	write_cmos_sensor(0x6F12, 0xC2F3);
	write_cmos_sensor(0x6F12, 0x5712);
	write_cmos_sensor(0x6F12, 0x1A70);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x3D21);
	write_cmos_sensor(0x6F12, 0xD200);
	write_cmos_sensor(0x6F12, 0x9A70);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x4D21);
	write_cmos_sensor(0x6F12, 0xCE3B);
	write_cmos_sensor(0x6F12, 0x22B1);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3821);
	write_cmos_sensor(0x6F12, 0x521C);
	write_cmos_sensor(0x6F12, 0x5608);
	write_cmos_sensor(0x6F12, 0x01E0);
	write_cmos_sensor(0x6F12, 0xD1F8);
	write_cmos_sensor(0x6F12, 0x3861);
	write_cmos_sensor(0x6F12, 0x7A68);
	write_cmos_sensor(0x6F12, 0x4FEA);
	write_cmos_sensor(0x6F12, 0x162C);
	write_cmos_sensor(0x6F12, 0x01F5);
	write_cmos_sensor(0x6F12, 0x9071);
	write_cmos_sensor(0x6F12, 0x82F8);
	write_cmos_sensor(0x6F12, 0x16C0);
	write_cmos_sensor(0x6F12, 0x1676);
	write_cmos_sensor(0x6F12, 0xCE8B);
	write_cmos_sensor(0x6F12, 0x00F5);
	write_cmos_sensor(0x6F12, 0xBA70);
	write_cmos_sensor(0x6F12, 0xC6F3);
	write_cmos_sensor(0x6F12, 0x5716);
	write_cmos_sensor(0x6F12, 0x9674);
	write_cmos_sensor(0x6F12, 0xCE7F);
	write_cmos_sensor(0x6F12, 0xF600);
	write_cmos_sensor(0x6F12, 0x1675);
	write_cmos_sensor(0x6F12, 0x0E8C);
	write_cmos_sensor(0x6F12, 0xCF68);
	write_cmos_sensor(0x6F12, 0xF608);
	write_cmos_sensor(0x6F12, 0x7E43);
	write_cmos_sensor(0x6F12, 0x360B);
	write_cmos_sensor(0x6F12, 0x370A);
	write_cmos_sensor(0x6F12, 0x03F8);
	write_cmos_sensor(0x6F12, 0xD67F);
	write_cmos_sensor(0x6F12, 0x7732);
	write_cmos_sensor(0x6F12, 0x9E70);
	write_cmos_sensor(0x6F12, 0x0688);
	write_cmos_sensor(0x6F12, 0x360A);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x276C);
	write_cmos_sensor(0x6F12, 0x4678);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x256C);
	write_cmos_sensor(0x6F12, 0x8688);
	write_cmos_sensor(0x6F12, 0x360A);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x1F6C);
	write_cmos_sensor(0x6F12, 0x4679);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x1D6C);
	write_cmos_sensor(0x6F12, 0x9C4E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x1064);
	write_cmos_sensor(0x6F12, 0xD671);
	write_cmos_sensor(0x6F12, 0x9A4E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x1164);
	write_cmos_sensor(0x6F12, 0x5672);
	write_cmos_sensor(0x6F12, 0x984E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x0B64);
	write_cmos_sensor(0x6F12, 0xD672);
	write_cmos_sensor(0x6F12, 0x964E);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x0964);
	write_cmos_sensor(0x6F12, 0x5673);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x3060);
	write_cmos_sensor(0x6F12, 0xD673);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0xDE00);
	write_cmos_sensor(0x6F12, 0x02F8);
	write_cmos_sensor(0x6F12, 0x1F0F);
	write_cmos_sensor(0x6F12, 0x9148);
	write_cmos_sensor(0x6F12, 0x00F2);
	write_cmos_sensor(0x6F12, 0x7246);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x7204);
	write_cmos_sensor(0x6F12, 0x9074);
	write_cmos_sensor(0x6F12, 0x3078);
	write_cmos_sensor(0x6F12, 0x1075);
	write_cmos_sensor(0x6F12, 0xA522);
	write_cmos_sensor(0x6F12, 0xDA70);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x1871);
	write_cmos_sensor(0x6F12, 0x11F8);
	write_cmos_sensor(0x6F12, 0x7E0C);
	write_cmos_sensor(0x6F12, 0xC0F1);
	write_cmos_sensor(0x6F12, 0x0C01);
	write_cmos_sensor(0x6F12, 0x8948);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x3C04);
	write_cmos_sensor(0x6F12, 0xC840);
	write_cmos_sensor(0x6F12, 0x060A);
	write_cmos_sensor(0x6F12, 0x9E71);
	write_cmos_sensor(0x6F12, 0x1872);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x03F8);
	write_cmos_sensor(0x6F12, 0x2C0C);
	write_cmos_sensor(0x6F12, 0x8448);
	write_cmos_sensor(0x6F12, 0xD0F8);
	write_cmos_sensor(0x6F12, 0x4C04);
	write_cmos_sensor(0x6F12, 0xC840);
	write_cmos_sensor(0x6F12, 0xAA21);
	write_cmos_sensor(0x6F12, 0x03F8);
	write_cmos_sensor(0x6F12, 0x571D);
	write_cmos_sensor(0x6F12, 0x0226);
	write_cmos_sensor(0x6F12, 0x5E70);
	write_cmos_sensor(0x6F12, 0x9A70);
	write_cmos_sensor(0x6F12, 0x3022);
	write_cmos_sensor(0x6F12, 0xDA70);
	write_cmos_sensor(0x6F12, 0x5A22);
	write_cmos_sensor(0x6F12, 0x1A71);
	write_cmos_sensor(0x6F12, 0x060A);
	write_cmos_sensor(0x6F12, 0x5E71);
	write_cmos_sensor(0x6F12, 0x9A71);
	write_cmos_sensor(0x6F12, 0xD871);
	write_cmos_sensor(0x6F12, 0x1972);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x5872);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x13B9);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0746);
	write_cmos_sensor(0x6F12, 0x7148);
	write_cmos_sensor(0x6F12, 0x0C46);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x806A);
	write_cmos_sensor(0x6F12, 0x86B2);
	write_cmos_sensor(0x6F12, 0x050C);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x06F9);
	write_cmos_sensor(0x6F12, 0x2146);
	write_cmos_sensor(0x6F12, 0x3846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5CF9);
	write_cmos_sensor(0x6F12, 0x6D48);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x9702);
	write_cmos_sensor(0x6F12, 0x10B9);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x33F9);
	write_cmos_sensor(0x6F12, 0x20B1);
	write_cmos_sensor(0x6F12, 0x04F1);
	write_cmos_sensor(0x6F12, 0x8044);
	write_cmos_sensor(0x6F12, 0xA08A);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0xA082);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xEFB8);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0746);
	write_cmos_sensor(0x6F12, 0x5F48);
	write_cmos_sensor(0x6F12, 0x0E46);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xC06A);
	write_cmos_sensor(0x6F12, 0x85B2);
	write_cmos_sensor(0x6F12, 0x040C);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE2F8);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x3846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x3DF9);
	write_cmos_sensor(0x6F12, 0x584F);
	write_cmos_sensor(0x6F12, 0x4DF2);
	write_cmos_sensor(0x6F12, 0x0C26);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x8061);
	write_cmos_sensor(0x6F12, 0x3A78);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD5F8);
	write_cmos_sensor(0x6F12, 0x7878);
	write_cmos_sensor(0x6F12, 0xC8B3);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x0071);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCDF8);
	write_cmos_sensor(0x6F12, 0x5648);
	write_cmos_sensor(0x6F12, 0x0088);
	write_cmos_sensor(0x6F12, 0x564B);
	write_cmos_sensor(0x6F12, 0xA3F8);
	write_cmos_sensor(0x6F12, 0x4402);
	write_cmos_sensor(0x6F12, 0x5348);
	write_cmos_sensor(0x6F12, 0x001D);
	write_cmos_sensor(0x6F12, 0x0088);
	write_cmos_sensor(0x6F12, 0xA3F8);
	write_cmos_sensor(0x6F12, 0x4602);
	write_cmos_sensor(0x6F12, 0xB3F8);
	write_cmos_sensor(0x6F12, 0x4402);
	write_cmos_sensor(0x6F12, 0xB3F8);
	write_cmos_sensor(0x6F12, 0x4612);
	write_cmos_sensor(0x6F12, 0x4218);
	write_cmos_sensor(0x6F12, 0x02D0);
	write_cmos_sensor(0x6F12, 0x8002);
	write_cmos_sensor(0x6F12, 0xB0FB);
	write_cmos_sensor(0x6F12, 0xF2F2);
	write_cmos_sensor(0x6F12, 0x91B2);
	write_cmos_sensor(0x6F12, 0x4E4A);
	write_cmos_sensor(0x6F12, 0xA3F8);
	write_cmos_sensor(0x6F12, 0x4812);
	write_cmos_sensor(0x6F12, 0x5088);
	write_cmos_sensor(0x6F12, 0x1288);
	write_cmos_sensor(0x6F12, 0x4A4B);
	write_cmos_sensor(0x6F12, 0xA3F8);
	write_cmos_sensor(0x6F12, 0xA605);
	write_cmos_sensor(0x6F12, 0xA3F8);
	write_cmos_sensor(0x6F12, 0xA825);
	write_cmos_sensor(0x6F12, 0x8018);
	write_cmos_sensor(0x6F12, 0x05D0);
	write_cmos_sensor(0x6F12, 0x9202);
	write_cmos_sensor(0x6F12, 0xB2FB);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x6F12, 0x1A46);
	write_cmos_sensor(0x6F12, 0xA2F8);
	write_cmos_sensor(0x6F12, 0xAA05);
	write_cmos_sensor(0x6F12, 0x4448);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0xAA05);
	write_cmos_sensor(0x6F12, 0x0A18);
	write_cmos_sensor(0x6F12, 0x01FB);
	write_cmos_sensor(0x6F12, 0x1020);
	write_cmos_sensor(0x6F12, 0x40F3);
	write_cmos_sensor(0x6F12, 0x9510);
	write_cmos_sensor(0x6F12, 0x1028);
	write_cmos_sensor(0x6F12, 0x06DC);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x05DA);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x03E0);
	write_cmos_sensor(0x6F12, 0xFFE7);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0xC3E7);
	write_cmos_sensor(0x6F12, 0x1020);
	write_cmos_sensor(0x6F12, 0x3D49);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x8CB8);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x7B71);
	write_cmos_sensor(0x6F12, 0x3748);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE9F8);
	write_cmos_sensor(0x6F12, 0x2C4C);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x2371);
	write_cmos_sensor(0x6F12, 0x6060);
	write_cmos_sensor(0x6F12, 0x3448);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE1F8);
	write_cmos_sensor(0x6F12, 0xA060);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x2D60);
	write_cmos_sensor(0x6F12, 0x3249);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xC861);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x8351);
	write_cmos_sensor(0x6F12, 0x3148);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD6F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x1141);
	write_cmos_sensor(0x6F12, 0x6061);
	write_cmos_sensor(0x6F12, 0x2E48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xCFF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x0931);
	write_cmos_sensor(0x6F12, 0xA061);
	write_cmos_sensor(0x6F12, 0x2C48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xC8F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xE121);
	write_cmos_sensor(0x6F12, 0xE061);
	write_cmos_sensor(0x6F12, 0x2948);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xC1F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x4F61);
	write_cmos_sensor(0x6F12, 0x2062);
	write_cmos_sensor(0x6F12, 0x2748);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBAF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x0761);
	write_cmos_sensor(0x6F12, 0xE060);
	write_cmos_sensor(0x6F12, 0x2448);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB3F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xEF21);
	write_cmos_sensor(0x6F12, 0x2061);
	write_cmos_sensor(0x6F12, 0x2248);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xACF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x9911);
	write_cmos_sensor(0x6F12, 0x6062);
	write_cmos_sensor(0x6F12, 0x1F48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA5F8);
	write_cmos_sensor(0x6F12, 0xA062);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x2146);
	write_cmos_sensor(0x6F12, 0x0246);
	write_cmos_sensor(0x6F12, 0x4880);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x6911);
	write_cmos_sensor(0x6F12, 0x1B48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9BF8);
	write_cmos_sensor(0x6F12, 0x0949);
	write_cmos_sensor(0x6F12, 0xE062);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0xF170);
	write_cmos_sensor(0x6F12, 0x0968);
	write_cmos_sensor(0x6F12, 0x4883);
	write_cmos_sensor(0x6F12, 0x10BD);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0FE0);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xF474);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x4900);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2850);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0E20);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x08D0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x36E0);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x9404);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x38C0);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xD214);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xA410);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xDE1F);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x5F3B);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0850);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xD719);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x27FF);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x39E3);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x32CF);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x1E3B);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xEC45);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x67B9);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xE62B);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x2265);
	write_cmos_sensor(0x6F12, 0x4AF2);
	write_cmos_sensor(0x6F12, 0x2B1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4DF6);
	write_cmos_sensor(0x6F12, 0x1F6C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0x655C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F6);
	write_cmos_sensor(0x6F12, 0x433C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F6);
	write_cmos_sensor(0x6F12, 0xE36C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x46F2);
	write_cmos_sensor(0x6F12, 0x7B1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0xD90C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0x791C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0x811C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F2);
	write_cmos_sensor(0x6F12, 0xB50C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F6);
	write_cmos_sensor(0x6F12, 0xE90C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F6);
	write_cmos_sensor(0x6F12, 0x155C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F6);
	write_cmos_sensor(0x6F12, 0x1D5C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4DF2);
	write_cmos_sensor(0x6F12, 0xC95C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x42F2);
	write_cmos_sensor(0x6F12, 0xFF7C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x48F2);
	write_cmos_sensor(0x6F12, 0x712C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F6);
	write_cmos_sensor(0x6F12, 0xE31C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x46F2);
	write_cmos_sensor(0x6F12, 0xB97C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4EF2);
	write_cmos_sensor(0x6F12, 0x2B6C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x42F2);
	write_cmos_sensor(0x6F12, 0x652C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF2);
	write_cmos_sensor(0x6F12, 0x2D1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x16F0);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x16F2);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x16FA);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x16FC);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x1708);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x170A);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x1712);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x1714);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x1716);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x1722);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x1724);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x172C);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x172E);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x1736);
	write_cmos_sensor(0x6F12, 0x1500);
	write_cmos_sensor(0x602A, 0x1738);
	write_cmos_sensor(0x6F12, 0x1500);
	write_cmos_sensor(0x602A, 0x1740);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x1742);
	write_cmos_sensor(0x6F12, 0x152A);
	write_cmos_sensor(0x602A, 0x16BE);
	write_cmos_sensor(0x6F12, 0x1515);
	write_cmos_sensor(0x6F12, 0x1515);
	write_cmos_sensor(0x602A, 0x16C8);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x602A, 0x16D6);
	write_cmos_sensor(0x6F12, 0x0015);
	write_cmos_sensor(0x6F12, 0x0015);
	write_cmos_sensor(0x602A, 0x16E0);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x6F12, 0x2929);
	write_cmos_sensor(0x602A, 0x19B8);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x2224);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0DF8);
	write_cmos_sensor(0x6F12, 0x1001);
	write_cmos_sensor(0x602A, 0x1EDA);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x602A, 0x16A0);
	write_cmos_sensor(0x6F12, 0x3D09);
	write_cmos_sensor(0x602A, 0x10A8);
	write_cmos_sensor(0x6F12, 0x000E);
	write_cmos_sensor(0x602A, 0x1198);
	write_cmos_sensor(0x6F12, 0x002B);
	write_cmos_sensor(0x602A, 0x1002);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x0F70);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x002F);
	write_cmos_sensor(0x602A, 0x0F76);
	write_cmos_sensor(0x6F12, 0x0030);
	write_cmos_sensor(0x602A, 0x0F7A);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x6F12, 0x0009);
	write_cmos_sensor(0x6F12, 0xF46E);
	write_cmos_sensor(0x602A, 0x1698);
	write_cmos_sensor(0x6F12, 0x0D05);
	write_cmos_sensor(0x602A, 0x20A0);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0203);
	write_cmos_sensor(0x602A, 0x4900);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0FEA, 0x1440);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0xF44A, 0x0007);
	write_cmos_sensor(0xF456, 0x000A);
	write_cmos_sensor(0xF46A, 0xBFA0);
	write_cmos_sensor(0x0D80, 0x1388);
	write_cmos_sensor(0x0A02, 0x007D);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x25B8);
	write_cmos_sensor(0x6F12, 0x0403);
	write_cmos_sensor(0x6F12, 0x040B);
	write_cmos_sensor(0x6F12, 0x040F);
	write_cmos_sensor(0x6F12, 0x0407);
	write_cmos_sensor(0x6F12, 0x0403);
	write_cmos_sensor(0x6F12, 0x040B);
	write_cmos_sensor(0x6F12, 0x040F);
	write_cmos_sensor(0x6F12, 0x0407);
	write_cmos_sensor(0x602A, 0x25B0);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x6F12, 0x1004);
	write_cmos_sensor(0x602A, 0x25E8);
	write_cmos_sensor(0x6F12, 0x4444);
	write_cmos_sensor(0x6F12, 0x4444);
	write_cmos_sensor(0x602A, 0x25B6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x17F8);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x602A, 0x17FC);
	write_cmos_sensor(0x6F12, 0x1101);
	write_cmos_sensor(0x6F12, 0x1105);
	write_cmos_sensor(0x6F12, 0x1109);
	write_cmos_sensor(0x6F12, 0x110D);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x6F12, 0x1115);
	write_cmos_sensor(0x6F12, 0x1119);
	write_cmos_sensor(0x6F12, 0x111D);
	write_cmos_sensor(0x6F12, 0x1121);
	write_cmos_sensor(0x6F12, 0x1125);
	write_cmos_sensor(0x6F12, 0x1129);
	write_cmos_sensor(0x6F12, 0x112D);
	write_cmos_sensor(0x6F12, 0x1131);
	write_cmos_sensor(0x6F12, 0x1135);
	write_cmos_sensor(0x6F12, 0x1139);
	write_cmos_sensor(0x6F12, 0x113D);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x25F0);
	write_cmos_sensor(0x6F12, 0x0410);
	write_cmos_sensor(0x602A, 0x25F6);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x602A, 0x2666);
	write_cmos_sensor(0x6F12, 0x0000);  
	
}	/*	sensor_init  */


static void preview_setting(void)
{
    LOG_INF("preview_setting() E\n");
	//stream off
  write_cmos_sensor_8(0x0100,0x00);
  check_stremoff();

	write_cmos_sensor(0x6028, 0x4000); 
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1B0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009F);
	write_cmos_sensor(0x6F12, 0x0007);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x000A);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0055);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38CD);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0C60);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x6F12, 0x0202);
	write_cmos_sensor(0x6028, 0x4000);
	//write_cmos_sensor(0x0344, 0x0008);
	write_cmos_sensor(0x0344, 0x0000);
	//write_cmos_sensor(0x0344, 0x0010);
	write_cmos_sensor(0x0346, 0x0008);
	//write_cmos_sensor(0x0348, 0x1227);
	write_cmos_sensor(0x0348, 0x122F);
	write_cmos_sensor(0x034A, 0x0DA7);
	write_cmos_sensor(0x034C, 0x0910);
	write_cmos_sensor(0x034E, 0x06D0);
	write_cmos_sensor(0x0350, 0x0004);// modified
	write_cmos_sensor(0x0900, 0x0112);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0003);
	write_cmos_sensor(0x0404, 0x2000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0301);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0085);
	//write_cmos_sensor(0x0310, 0x007A);
	write_cmos_sensor(0x0312, 0x0001);
	//write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x0894);
	//write_cmos_sensor(0x0340, 0x0E55);
	write_cmos_sensor(0x0342, 0x2130);
	//write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	//write_cmos_sensor(0x0D00, 0x0101);
	//write_cmos_sensor(0x0D02, 0x0101);
	//write_cmos_sensor(0x0D04, 0x0100);
	//write_cmos_sensor(0x0D06, 0x0120);
	//write_cmos_sensor(0x0D08, 0x0360);
	mdelay(10);
	write_cmos_sensor(0x0100, 0x0100);
	mdelay(10);

}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture_setting() E! currefps:%d\n",currefps);

	//stream off
  write_cmos_sensor_8(0x0100,0x00);
  check_stremoff();
	
	write_cmos_sensor(0x6028, 0x4000); 
	write_cmos_sensor(0x6214, 0x7970); 
	write_cmos_sensor(0x6218, 0x7150); 
	write_cmos_sensor(0x6028, 0x2000); 
	write_cmos_sensor(0x602A, 0x0ED6); 
	write_cmos_sensor(0x6F12, 0x0100); 
	write_cmos_sensor(0x602A, 0x1CF0); 
	write_cmos_sensor(0x6F12, 0x0100); 
	write_cmos_sensor(0x602A, 0x0E58); 
	write_cmos_sensor(0x6F12, 0x0040); 
	write_cmos_sensor(0x602A, 0x1694); 
	write_cmos_sensor(0x6F12, 0x1B0F); 
	write_cmos_sensor(0x602A, 0x16AA); 
	write_cmos_sensor(0x6F12, 0x009F); 
	write_cmos_sensor(0x6F12, 0x0007); 
	write_cmos_sensor(0x602A, 0x1098); 
	write_cmos_sensor(0x6F12, 0x000A); 
	write_cmos_sensor(0x602A, 0x2690); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0055); 
	write_cmos_sensor(0x602A, 0x16A8); 
	write_cmos_sensor(0x6F12, 0x38CD); 
	write_cmos_sensor(0x602A, 0x108C); 
	write_cmos_sensor(0x6F12, 0x0003); 
	write_cmos_sensor(0x602A, 0x10CC); 
	write_cmos_sensor(0x6F12, 0x0008); 
	write_cmos_sensor(0x602A, 0x10D0); 
	write_cmos_sensor(0x6F12, 0x000F); 
	write_cmos_sensor(0x602A, 0x0F50); 
	write_cmos_sensor(0x6F12, 0x0100); 
	write_cmos_sensor(0x602A, 0x0C60); 
	write_cmos_sensor(0x6F12, 0x0001); 
	write_cmos_sensor(0x6F12, 0x0202); 
	write_cmos_sensor(0x6028, 0x4000); 
	write_cmos_sensor(0x0344, 0x0000); 
	write_cmos_sensor(0x0346, 0x0008); 
	write_cmos_sensor(0x0348, 0x122F); 
	write_cmos_sensor(0x034A, 0x0DA7); 
	write_cmos_sensor(0x034C, 0x1220); 
	write_cmos_sensor(0x034E, 0x0DA0); 
	write_cmos_sensor(0x0350, 0x0008); 
	write_cmos_sensor(0x0900, 0x0011); 
	write_cmos_sensor(0x0380, 0x0001); 
	write_cmos_sensor(0x0382, 0x0001); 
	write_cmos_sensor(0x0384, 0x0001); 
	write_cmos_sensor(0x0386, 0x0001); 
	write_cmos_sensor(0x0404, 0x1000); 
	write_cmos_sensor(0x0402, 0x1010); 
	write_cmos_sensor(0x0400, 0x1010); 
	write_cmos_sensor(0x0114, 0x0301); 
	write_cmos_sensor(0x0110, 0x1002); 
	write_cmos_sensor(0x0136, 0x1800); 
	write_cmos_sensor(0x0300, 0x0007); 
	write_cmos_sensor(0x0302, 0x0001); 
	write_cmos_sensor(0x0304, 0x0006); 
	write_cmos_sensor(0x0306, 0x00F5); 
	write_cmos_sensor(0x0308, 0x0008); 
	write_cmos_sensor(0x030A, 0x0001); 
	write_cmos_sensor(0x030C, 0x0000); 
	write_cmos_sensor(0x030E, 0x0004); 
	write_cmos_sensor(0x0310, 0x007A); 
	write_cmos_sensor(0x0312, 0x0000); 
	write_cmos_sensor(0x0340, 0x0E54); 
	write_cmos_sensor(0x0342, 0x13E0); 
	write_cmos_sensor(0x0202, 0x0100); 
	write_cmos_sensor(0x0200, 0x0100); 
	write_cmos_sensor(0x021E, 0x0000);	
	/* for pdaf */
#ifdef  PDAFSUPPORT
	write_cmos_sensor(0x0D00, 0x0101); 
	write_cmos_sensor(0x0D02, 0x0101); 
#else
	write_cmos_sensor(0x0D00, 0x0100);// without vc mode 
	write_cmos_sensor(0x0D02, 0x0001);
#endif
	write_cmos_sensor(0x0D04, 0x0102); 
	write_cmos_sensor(0x0D06, 0x0120); 
	write_cmos_sensor(0x0D08, 0x0360);
	mdelay(10);
	write_cmos_sensor(0x0100, 0x0100); 
	mdelay(10);


}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting() E! currefps:%d\n",currefps);

	//stream off
    write_cmos_sensor_8(0x0100,0x00);
    check_stremoff();           

	write_cmos_sensor(0x6028, 0x4000); 
	write_cmos_sensor(0x6214, 0x7970); 
	write_cmos_sensor(0x6218, 0x7150); 
	write_cmos_sensor(0x6028, 0x2000); 
	write_cmos_sensor(0x602A, 0x0ED6); 
	write_cmos_sensor(0x6F12, 0x0100); 
	write_cmos_sensor(0x602A, 0x1CF0); 
	write_cmos_sensor(0x6F12, 0x0100); 
	write_cmos_sensor(0x602A, 0x0E58); 
	write_cmos_sensor(0x6F12, 0x0040); 
	write_cmos_sensor(0x602A, 0x1694); 
	write_cmos_sensor(0x6F12, 0x1B0F); 
	write_cmos_sensor(0x602A, 0x16AA); 
	write_cmos_sensor(0x6F12, 0x009F); 
	write_cmos_sensor(0x6F12, 0x0007); 
	write_cmos_sensor(0x602A, 0x1098); 
	write_cmos_sensor(0x6F12, 0x000A); 
	write_cmos_sensor(0x602A, 0x2690); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0055); 
	write_cmos_sensor(0x602A, 0x16A8); 
	write_cmos_sensor(0x6F12, 0x38CD); 
	write_cmos_sensor(0x602A, 0x108C); 
	write_cmos_sensor(0x6F12, 0x0003); 
	write_cmos_sensor(0x602A, 0x10CC); 
	write_cmos_sensor(0x6F12, 0x0008); 
	write_cmos_sensor(0x602A, 0x10D0); 
	write_cmos_sensor(0x6F12, 0x000F); 
	write_cmos_sensor(0x602A, 0x0F50); 
	write_cmos_sensor(0x6F12, 0x0100); 
	write_cmos_sensor(0x602A, 0x0C60); 
	write_cmos_sensor(0x6F12, 0x0001); 
	write_cmos_sensor(0x6F12, 0x0202); 
	write_cmos_sensor(0x6028, 0x4000); 
	write_cmos_sensor(0x0344, 0x0000); 
	write_cmos_sensor(0x0346, 0x0008); 
	write_cmos_sensor(0x0348, 0x122F); 
	write_cmos_sensor(0x034A, 0x0DA7); 
	write_cmos_sensor(0x034C, 0x1220); 
	write_cmos_sensor(0x034E, 0x0DA0); 
	write_cmos_sensor(0x0350, 0x0008); 
	write_cmos_sensor(0x0900, 0x0011); 
	write_cmos_sensor(0x0380, 0x0001); 
	write_cmos_sensor(0x0382, 0x0001); 
	write_cmos_sensor(0x0384, 0x0001); 
	write_cmos_sensor(0x0386, 0x0001); 
	write_cmos_sensor(0x0404, 0x1000); 
	write_cmos_sensor(0x0402, 0x1010); 
	write_cmos_sensor(0x0400, 0x1010); 
	write_cmos_sensor(0x0114, 0x0301); 
	write_cmos_sensor(0x0110, 0x1002); 
	write_cmos_sensor(0x0136, 0x1800); 
	write_cmos_sensor(0x0300, 0x0007); 
	write_cmos_sensor(0x0302, 0x0001); 
	write_cmos_sensor(0x0304, 0x0006); 
	write_cmos_sensor(0x0306, 0x00F5); 
	write_cmos_sensor(0x0308, 0x0008); 
	write_cmos_sensor(0x030A, 0x0001); 
	write_cmos_sensor(0x030C, 0x0000); 
	write_cmos_sensor(0x030E, 0x0004); 
	write_cmos_sensor(0x0310, 0x007A); 
	write_cmos_sensor(0x0312, 0x0000); 
	write_cmos_sensor(0x0340, 0x0E54); 
	write_cmos_sensor(0x0342, 0x13E0); 
	write_cmos_sensor(0x0202, 0x0100); 
	write_cmos_sensor(0x0200, 0x0100); 
	write_cmos_sensor(0x021E, 0x0000);	

	write_cmos_sensor(0x0100, 0x0100);
}


static void hs_video_setting(void)
{
	LOG_INF("hs_video_setting() E\n");
    //720p 120fps
    write_cmos_sensor_8(0x0100,0x00);
    check_stremoff();
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1B0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009F);
	write_cmos_sensor(0x6F12, 0x0007);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x000A);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0055);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38CD);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0C60);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x6F12, 0x0202);
	write_cmos_sensor(0x602A, 0x1758);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0300);
	write_cmos_sensor(0x0346, 0x0258);
	write_cmos_sensor(0x0348, 0x0F2F);
	write_cmos_sensor(0x034A, 0x0B57);
	write_cmos_sensor(0x034C, 0x0400);
	write_cmos_sensor(0x034E, 0x0300);
	write_cmos_sensor(0x0350, 0x0008);
	write_cmos_sensor(0x0900, 0x0113);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0005);
	write_cmos_sensor(0x0404, 0x3000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x0077);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x0395);
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0000);
	write_cmos_sensor(0x0D02, 0x0001);
	write_cmos_sensor(0x0D04, 0x0102);
	write_cmos_sensor(0x0D06, 0x0000);
	write_cmos_sensor(0x0D08, 0x0000);
	write_cmos_sensor(0x0100, 0x0100);
}

static void slim_video_setting(void)
{
	LOG_INF("slim_video_setting() E\n");
    //1080p 60fps
  write_cmos_sensor_8(0x0100,0x00);
    check_stremoff();               
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7970);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0ED6);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1CF0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0E58);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x602A, 0x1694);
	write_cmos_sensor(0x6F12, 0x1B0F);
	write_cmos_sensor(0x602A, 0x16AA);
	write_cmos_sensor(0x6F12, 0x009F);
	write_cmos_sensor(0x6F12, 0x0007);
	write_cmos_sensor(0x602A, 0x1098);
	write_cmos_sensor(0x6F12, 0x000A);
	write_cmos_sensor(0x602A, 0x2690);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0055);
	write_cmos_sensor(0x602A, 0x16A8);
	write_cmos_sensor(0x6F12, 0x38CD);
	write_cmos_sensor(0x602A, 0x108C);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x10CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x10D0);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x0F50);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x0C60);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x6F12, 0x0202);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0190);
	write_cmos_sensor(0x0346, 0x02A0);
	write_cmos_sensor(0x0348, 0x109F);
	write_cmos_sensor(0x034A, 0x0B0F);
	write_cmos_sensor(0x034C, 0x0780);
	write_cmos_sensor(0x034E, 0x0438);
	write_cmos_sensor(0x0350, 0x0004);
	write_cmos_sensor(0x0900, 0x0112);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0003);
	write_cmos_sensor(0x0404, 0x2000);
	write_cmos_sensor(0x0402, 0x1010);
	write_cmos_sensor(0x0400, 0x1010);
	write_cmos_sensor(0x0114, 0x0301);
	write_cmos_sensor(0x0110, 0x1002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0007);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00F5);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x030E, 0x0004);
	write_cmos_sensor(0x0310, 0x007A);
	write_cmos_sensor(0x0312, 0x0000);
	write_cmos_sensor(0x0340, 0x0729);
	write_cmos_sensor(0x0342, 0x13E0);
	write_cmos_sensor(0x0202, 0x0100);
	write_cmos_sensor(0x0200, 0x0100);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x0D00, 0x0101);
	write_cmos_sensor(0x0D02, 0x0101);
	write_cmos_sensor(0x0D04, 0x0100);
	write_cmos_sensor(0x0D06, 0x00F0);
	write_cmos_sensor(0x0D08, 0x021C);
	write_cmos_sensor(0x0100, 0x0100);



}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
			LOG_INF("read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",read_cmos_sensor_8(0x0000),read_cmos_sensor_8(0x0001),read_cmos_sensor(0x0000));
			if (*sensor_id ==imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);

				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
        LOG_INF("%s",__FUNCTION__);
	LOG_INF("PLATFORM:MT6757,MIPI 4LANE\n");
	LOG_INF("preview 2320*1744@30fps,1464Mbps/lane video 4640*3488@30fps,1464Mbps/lane; capture 4640*3488@30fps,1464Mbps/lane\n");
        kdSetI2CSpeed(imgsensor_info.i2c_speed);
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
            sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("preview E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("capture E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}  else if(imgsensor.current_fps == imgsensor_info.cap2.max_framerate){
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("normal_video E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

        normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("hs_video E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("slim_video E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("get_resolution E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("get_info -> scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	//sensor_info->PDAF_Support = 0; //change pdaf support mode to pdaf VC mode
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;
#ifdef PDAFSUPPORT
        sensor_info->PDAF_Support = 2;
#else
        sensor_info->PDAF_Support = 0;
#endif


	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	////LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			 if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			 if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
                frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
             if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
            break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			 if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			 if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			 if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x0600, 0x0002);  //0 : Normal, 1 : Solid Color, 2 : Color Bar, 3 : Shade Color Bar, 4 : PN9
	} else {
		write_cmos_sensor(0x0600,0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
        unsigned long long *feature_data=(unsigned long long *) feature_para;

        SET_PD_BLOCK_INFO_T *PDAFinfo;
	SENSOR_VC_INFO_STRUCT *pvcinfo;
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
            //night_mode((BOOL) *feature_data); no need to implement this mode
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
                     imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
						break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            //ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
        case SENSOR_FEATURE_SET_AWB_GAIN:
            break;
        case SENSOR_FEATURE_SET_HDR_SHUTTER:
            LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
            //ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1));
            break;
	/******************** PDAF start   <<< *********/
	case SENSOR_FEATURE_GET_PDAF_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT16)*feature_data);
	    PDAFinfo = (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
	    switch (*feature_data) {
		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: //full
		    	memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T)); //need to check
			    break;
		    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		    //case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //2x2 binning
			  //  memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T)); //need to check
			   // break;
		    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		    case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //2x2 binning
		    case MSDK_SCENARIO_ID_SLIM_VIDEO:
		    default:
			    break;
	    }
	    break;

	case SENSOR_FEATURE_GET_VC_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
	    pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
	    switch (*feature_data_32) {
		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			    memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
			    break;
		    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			    memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
			    break;
		    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		    default:
			    memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
			    break;
	    }
	    break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
	    LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT16)*feature_data);
	    //PDAF capacity enable or not
	    switch (*feature_data) {
		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			    break;
		    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // video & capture use same setting
			    break;
		    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			    break;
		    case MSDK_SCENARIO_ID_SLIM_VIDEO:
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; //need to check
			    break;
		    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			    break;
		    default:
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			    break;
	    }
	    break;
	case SENSOR_FEATURE_GET_PDAF_DATA:	//get cal data from eeprom
	    LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
	    S5K3P9_read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
	    LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA success\n");
	    break;
	case SENSOR_FEATURE_SET_PDAF:
	    LOG_INF("PDAF mode :%d\n", *feature_data_16);
	    imgsensor.pdaf_mode= *feature_data_16;
	    break;
	    /******************** PDAF END   <<< *********/

	default:
	    break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K3P9_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}
