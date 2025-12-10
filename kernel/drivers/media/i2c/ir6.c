//DOTHINKEY 2.2 sensor ini format(Start from 2014.11.06)
//IMX322_1079_113_34.ini
//www.dothinkey.com
//Preview Type    : 0:DVP Raw 10 bit; 1:Raw 8 bit; 2:YUV422; 3:RAW16
//Preview Type    : 4:RGB565; 5:Pixart SPI; 6:MIPI 10bit; 7:MIPI 12bit; 8: MTK SPI
//port            : 0:MIPI; 1:Parallel; 2:MTK; 3:SPI; 4:TEST; 5: HISPI; 6 : Z2P/Z4P
//I2C Mode        : 0:Normal 8Addr,8Data;  1:Samsung 8 Addr,8Data; 2:Micron 8 Addr,16Data
//I2C Mode        : 3:Stmicro 16Addr,8Data;4:Micron2 16 Addr,16Data
//Out Format      : 0:YCbYCr/RG_GB; 1:YCrYCb/GR_BG; 2:CbYCrY/GB_RG; 3:CrYCbY/BG_GR
//MCLK Speed      : in KHZ,like this:24M,value is 24000
//pin             : BIT0 pwdn; BIT1:reset
//avdd            : in mV, like this:2.8V, value is 2800
//dovdd           : in mV, like this:2.8V, value is 2800
//dvdd            : in mV, like this:2.8V, value is 2800
//Quick_w         : Quick Preview width
//Quick_h         : Quick Preview height
//[Quick_ParaList]: Quick Preview resolution sensor settings

// 头文件
#define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"
#include <linux/time.h>

// 宏定义：MIPI 频率，芯片 ID，寄存器地址等
#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x06)

#ifndef V4L2_CID_DIGITAL_GAIN  //数字增益
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define MIPI_FREQ_518M			518400000
#define MIPI_FREQ_259M			259200000
#define MIPI_FREQ_567M			283500000

#define IR6_4LANES			4

#define IR6_MAX_PIXEL_RATE		(MIPI_FREQ_518M / 12 * 2 * IR6_4LANES)	//829440000
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define IR6_XVCLK_FREQ_27M		27000000

#define CHIP_ID				0xc0
#define IR6_REG_CHIP_ID		0x0083

#define IR6_SLEEP_MODE		0x0140
#define IR6_MODE_SW_STANDBY		BIT(0)//1
#define IR6_MODE_STREAMING		0x0

//ADC增益控制信号,默认一倍增益。用于非DOL2模式和DOL2模式的前一帧。
#define IR6_ADC_GAIN_REG_H			0x002c
#define IR6_ADC_GAIN_REG_L			0x002b
#define IR6_ADC_VFT_REG				0x0030

#define IR6_ADC_GAIN_REG_H_VALUE			0x00
#define IR6_ADC_GAIN_REG_L_VALUE			0x00
#define IR6_ADC_VFT_REG_VALUE				0x1f

//ADC增益控制信号,默认一倍增益。用于DOL2模式的后一帧。
#define IR6_ADC_DOL2_GAIN_REG_H		0x002e
#define IR6_ADC_DOL2_GAIN_REG_L  	0x002d
#define IR6_ADC_DOL2_VFT_REG		0x0031

#define IR6_ADC_DOL2_GAIN_REG_H_VALUE		0x00
#define IR6_ADC_DOL2_GAIN_REG_L_VALUE  		0x00
#define IR6_ADC_DOL2_VFT_REG_VALUE			0x1f


#define IR6_DIG_GAIN_MIN			0x00
#define IR6_DIG_GAIN_MAX			0x40
#define IR6_DIG_GAIN_STEP			1
#define IR6_DIG_GAIN_DEFAULT		0x3f

//dol2模式下第一帧数字增益
#define IR6_DIG_GAIN_EN					0x01d7
#define IR6_DIG_GAIN_COARSE_DOL1		0x01d8
#define IR6_DIG_GAIN_FINE_L_DOL1		0x01d9
#define IR6_DIG_GAIN_FINE_H_DOL1		0x01da

#define IR6_DIG_GAIN_COARSE_DOL1_VALUE		0x00
#define IR6_DIG_GAIN_FINE_L_DOL1_VALUE		0x00
#define IR6_DIG_GAIN_FINE_H_DOL1_VALUE		0x00

//dol2模式下第二帧的数字增益
#define IR6_DIG_GAIN_COARSE_DOL2		0x01db
#define IR6_DIG_GAIN_FINE_L_DOL2		0x01dc
#define IR6_DIG_GAIN_FINE_H_DOL2		0x01dd

#define IR6_DIG_GAIN_COARSE_DOL2_VALUE		0x00
#define IR6_DIG_GAIN_FINE_L_DOL2_VALUE		0x00
#define IR6_DIG_GAIN_FINE_H_DOL2_VALUE		0x00

#define IR6_FETCH_GAIN_H(VAL)	(((VAL) >> 8) & 0x07)
#define IR6_FETCH_GAIN_L(VAL)	((VAL) & 0xFF)
#define IR6_DIG_GAIN_L(VAL)		((VAL) & 0x3F)
#define IR6_DIG_GAIN_FINE_H(VAL)		(((VAL)>>8) & 0x03)
#define IR6_DIG_GAIN_FINE_L(VAL)		((VAL) & 0xFF)
#define IR6_DCG_SWITCH(VAL)		((VAL) & 0x01)

#define IR6_EXPT_LINE_L     0x008f
#define IR6_EXPT_LINE_M 	0x0090
#define IR6_EXPT_LINE_H 	0x0091

#define IR6_REG_A_L			0x0092
#define IR6_REG_A_M			0x0093
#define IR6_REG_A_H			0x0094

#define IR6_REG_B_L			0x0095
#define IR6_REG_B_M			0x0096
#define IR6_REG_B_H			0x0097

#define IR6_REG_C_L   		0x0098
#define IR6_REG_C_M 		0x0099
#define IR6_REG_C_H 		0x009a

#define IR6_VSYNC_ITV_L		0x01e3
#define IR6_VSYNC_ITV_H		0x01e4

#define IR6_REQ_SEL			0x00a5
#define IR6_REQ_SYS			0x00a6

#define	IR6_EXPOSURE_MIN		4
#define	IR6_EXPOSURE_STEP		1
#define IR6_VTS_MAX				0xffff

#define IR6_ADC_GAIN_MIN			0x01
#define IR6_ADC_GAIN_MAX			0x2000
#define IR6_ADC_GAIN_STEP			1
#define IR6_ADC_GAIN_DEFAULT		0x01

#define IR6_FETCH_EXPT_H(VAL)		(((VAL) >> 16) & 0x0F)
#define IR6_FETCH_EXPT_M(VAL)		(((VAL) >> 8) & 0xFF)
#define IR6_FETCH_EXPT_L(VAL)		((VAL) & 0xFF)

#define IR6_FLIP_REG_ROW		0x00ce//1:行镜像使能v
#define IR6_FLIP_REG_COL		0x00cf//1:列镜像使能h

#define IR6_ROW_BIT_MASK		BIT(0)//置1
#define IR6_COL_BIT_MASK		BIT(0)

#define REG_NULL			0xFFFF

#define IR6_REG_VALUE_08BIT		1
#define IR6_REG_VALUE_16BIT		2
#define IR6_REG_VALUE_24BIT		3

#define IR6_GROUP_HOLD_REG		 	0x00a4//0x3001//
#define IR6_GROUP_HOLD_START		0x01
#define IR6_GROUP_HOLD_END			0x00


/* Basic Readout Lines. Number of necessary readout lines in sensor */
#define BRL_ALL				1300u
#define BRL_BINNING			1280u
/* Readout timing setting of SEF1(DOL2): RHS1 < 2 * BRL and should be 4n + 1 */
#define RHS1_MAX_X2(VAL)		(((VAL) * 2 - 1) / 4 * 4 + 1)
#define SHR1_MIN_X2			9u

#define IR6_EXP_ENABLE			0X00a5
#define IR6_EXP_SYS				0X00a6
#define IR6_EXP_EXT				0X00a7

#define IR6_DCG_GAIN_REG_0		0x009b
#define IR6_DCG_GAIN_REG_1		0x009c
#define IR6_DCG_GAIN_REG_2		0x009d
#define IR6_DCG_GAIN_REG_3		0x009e

#define IR6_HCG_GAIN_VALUE_0 	0x00
#define IR6_HCG_GAIN_VALUE_1	0x00
#define IR6_HCG_GAIN_VALUE_2	0x00
#define IR6_HCG_GAIN_VALUE_3 	0x00

#define IR6_LCG_GAIN_VALUE_0 	0x01
#define IR6_LCG_GAIN_VALUE_1 	0x01
#define IR6_LCG_GAIN_VALUE_2 	0x01
#define IR6_LCG_GAIN_VALUE_3 	0x01

#define IR6_BLC_EN			 0x00e2//暗电流水平矫正
#define IR6_BLC_STAT_EN		 0x00e3//BLC统计开关
#define IR6_BLC_FILTER		 0x00e4//BLC中值滤波
#define IR6_BLC_ADJ_MOD 	 0x00e5//模块矫正方式

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define IR6_NAME			"ir6"

static const char * const ir6_supply_names[] = {
	"dvdd",		/* Digital core power */
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
};

#define IR6_NUM_SUPPLIES ARRAY_SIZE(ir6_supply_names)

enum ir6_max_pad {
	PAD0, /* link to isp */
	PAD1, /* link to csi wr0 | hdr x2:L x3:M */
	PAD2, /* link to csi wr1 | hdr      x3:L */
	PAD3, /* link to csi wr2 | hdr x2:M x3:S */
	PAD_MAX,
};

struct regval {
	u16 addr;
	u8 val;
};

struct regval_1 {
	u16 addr_1;
	u32 val_1;
};

// 传感器工作模式（分辨率、帧率等）
struct ir6_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 mipi_freq_idx;
	u32 bpp;
	const struct regval *global_reg_list;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

// CIF 核心数据结构 GPIO I2C V4L2
struct ir6 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*power_gpio;
	struct gpio_desc	*exp_gpio;
	struct gpio_desc	*frame_gpio;
	struct gpio_desc	*trig_gpio;
	struct regulator_bulk_data supplies[IR6_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_a_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*dcg_handoff;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	bool			is_thunderboot;
	bool			is_thunderboot_ng;
	bool			is_first_streamoff;
	const struct ir6_mode *cur_mode;
	u32			module_index;
	u32			cfg_num;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			has_init_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	u32 		dcg_ratio;
	bool			long_hcg;
	bool			middle_hcg;
	bool			short_hcg;
};

#define to_ir6(sd) container_of(sd, struct ir6, subdev)

static __maybe_unused const struct regval ir6_global_12bit_1936x1280_regs[] = {
    {0x0140,0x01},
	{REG_NULL, 0x00},
};

static const struct regval TC001_ir6_MIPI4Lane_60fps12Bit_MCLK518p4M[] = {
    
    {0x007f,0x00},
    {0x0082,0x04},
    {0x0083,0xC0},
    {0x0084,0x02},
    {0x0086,0x01},
    {0x0089,0x05},
    {0x008b,0x05},
    {0x01df,0x00},
    {0x01e0,0x00},
    {0x01e1,0x8f},
    {0x01e2,0x07},
    {0x00ac,0x00},
    {0x00ad,0x05},
    {0x01e5,0x00},
    {0x01e6,0x00},
    {0x01e7,0x01},
    {0x01e8,0x01},
    {0x00a5,0x00},
    {0x00a7,0x00},
    {0x00a4,0x00},
    {0x00a9,0x01},
    {0x00aa,0x00},
    {0x00ab,0x2d},
    {0x008f,0x08},
    {0x0090,0x00},
    {0x0091,0x00},
	//黑电流水平矫正
	{0x00e2,0x01},
	{0x00e3,0x01},
	{0x00e4,0x01},
	{0x00e5,0x01},
    {0x014f,0x00},
    {0x0150,0x00},
	//数字增益
	{0x01d7,0x01},
	{0x01d8,0x01},
	{0x01d9,0x00},
	{0x01da,0x00},
	//ADC增益
	{0x002c,0x00},
	{0x002b,0x00},
	{0x0030,0x1f},

    {0x01d6,0x00},
    {0x0058,0x01},
    {0x0059,0x0f},
    {0x005a,0x01},
    {0x005b,0x03},
    {0x1201,0xf0},
    {0x1202,0x70},
    {0x1203,0x10},
    {0x1204,0x10},
    {0x1070,0x02},
    {0x1205,0x00},
    {0x1208,0x01},
    {0x1000,0x08},
    {0x1001,0x00},
    {0x1070,0x12},
    {0x1070,0x02},
    {0x1024,0x90},
    {0x1025,0x07},
    {0x1026,0x00},
    {0x1027,0x05},
    {0x1040,0x8d},
    {0x1020,0x2a},
    {0x1042,0x0f},
    {0x1028,0x90},
    {0x1029,0x07},
    {0x102a,0x00},
    {0x102b,0x05},
    {0x102c,0x90},
    {0x102d,0x07},
    {0x102e,0x00},
    {0x102f,0x05},
    {0x1030,0x90},
    {0x1031,0x07},
    {0x1032,0x00},
    {0x1033,0x05},
    {0x01e3,0x1e},
    {0x00a4,0x01},
    {0x1040,0x8c},
    {0x005b,0x03},
    {0x005d,0x0f},
    {0x008f,0xc8},
    {0x0090,0x00},
    {0x0091,0x00},
    {0x0042,0xaa},
	//HCG LCG切换
    {0x009b,0x00},
    {0x009c,0x00},
    {0x009d,0x00},
    {0x009e,0x00},
	
	{0x0065,0x05},
	{0x0261,0x0f},
	{0x0262,0x81},

    {0x00a5,0x01},
    {0x00a6,0x01},
    {0x00a6,0x00},
	{REG_NULL, 0x00},
};

static const struct regval TC002_ir6_MIPI4Lane_30fps12BitDol2_MCLK518p4M[] = {
    {0x007f,0x00},
    {0x0082,0x04},
    {0x0083,0xC0},
    {0x0084,0x02},
    {0x0086,0x01},
    {0x0089,0x05},
    {0x008b,0x05},
    {0x01df,0x00},
    {0x01e0,0x00},
    {0x01e1,0x8f},
    {0x01e2,0x07},
    {0x00ac,0x00},
    {0x00ad,0x05},
    {0x01e5,0x01},
    {0x01e6,0x00},
    {0x01e7,0x01},
    {0x01e8,0x01},
    {0x00a5,0x00},
    {0x00a7,0x00},
    {0x00a4,0x01},
    {0x00a9,0x03},
    {0x00aa,0x00},

	{0x00ab,0x2d},
    {0x008f,0xfe},
    {0x0090,0x00},
    {0x0091,0x00},
    {0x0092,0xcc},
    {0x0093,0x08},
    {0x0094,0x00},
 	//黑电流水平矫正
	{0x00e2,0x01},
	{0x00e3,0x01},
	{0x00e4,0x01},
	{0x00e5,0x01},

    {0x014f,0x00},
    {0x0150,0x00},
    {0x01d7,0x00},
	//数字增益
	{0x01d7,0x01},
	{0x01d8,0x01},
	{0x01d9,0x00},
	{0x01da,0x00},
	{0x01db,0x01},
	{0x01dc,0x00},
	{0x01dd,0x00},
	//ADC增益
	{0x002c,0x00},
	{0x002b,0x00},
	{0x0030,0x1f},
	{0x002e,0x00},
	{0x002d,0x00},
	{0x0031,0x1f},
	//图像调优
	{0x0065,0x05},
	{0x0261,0x0f},
	{0x0262,0x81},

    {0x01d6,0x00},
    {0x0058,0x01},
    {0x0059,0x0f},
    {0x005a,0x01},
    {0x005b,0x03},
    {0x1201,0xf0},
    {0x1202,0x70},
    {0x1203,0x10},
    {0x1204,0x10},
    {0x1070,0x02},
    {0x1205,0x00},
    {0x1208,0x01},
    {0x1000,0x08},
    {0x1001,0x00},
    {0x1070,0x12},
    {0x1070,0x02},
    {0x1024,0x90},
    {0x1025,0x07},
    {0x1026,0x00},
    {0x1027,0x05},
    {0x1040,0x8c},
    {0x1020,0x2a},
    {0x1042,0x0f},
    {0x1028,0x90},
    {0x1029,0x07},
    {0x102a,0x00},
    {0x102b,0x05},
    {0x102c,0x90},
    {0x102d,0x07},
    {0x102e,0x00},
    {0x102f,0x05},
    {0x1030,0x90},
    {0x1031,0x07},
    {0x1032,0x00},
    {0x1033,0x05},
    {0x01e3,0x1e},
    {0x00a4,0x01},
    {0x1040,0x8c},
    {0x005b,0x03},
    {0x005d,0x07},
    {0x008f,0xc8},
    {0x0090,0x00},
    {0x0091,0x00},
    {0x0092,0xcc},
    {0x0093,0x08},
    {0x0094,0x00},
    {0x0095,0x01},
    {0x0096,0x00},
    {0x0097,0x00},
    {0x005d,0x0f},

	{0x0042,0xaa},
	{0x009b,0x01},
	{0x009c,0x01},
	{0x009d,0x01},
	{0x009e,0x01},
	{0x00a5,0x01},
	{0x00a6,0x01},
	{0x00a6,0x00},/**/
	{REG_NULL, 0x00},
};

// 传感器的工作模式
static const struct ir6_mode supported_modes[] = {
	// 工作模式：分辨率、帧率
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB12_1X12,
		.width = 1936,						
		.height = 1280,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0514- 0x0029,//曝光时间 1259=1300-40-1
		.hts_def = 0x0110 * IR6_4LANES * 2,//有效图像宽度+hblank 240 1936
		.vts_def = 0x0514,////有效图像宽度+vblank  1280+20
		.global_reg_list = ir6_global_12bit_1936x1280_regs,
		.reg_list = TC001_ir6_MIPI4Lane_60fps12Bit_MCLK518p4M,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1,
		.bpp = 12,
	},
    {
		.bus_fmt = MEDIA_BUS_FMT_SRGGB12_1X12,
		.width = 1936,
		.height = 1280,
		.max_fps = {
			.numerator = 10000,
			.denominator = 250000,
		},
		.exp_def = 0x0514- 0x0029,
		.hts_def = 0x0110 * IR6_4LANES * 2 ,
		.vts_def = 0x0514 ,
		.global_reg_list = ir6_global_12bit_1936x1280_regs,
		.reg_list = TC002_ir6_MIPI4Lane_30fps12BitDol2_MCLK518p4M,
		.hdr_mode = HDR_X2,
		.mipi_freq_idx = 1,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
	},
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_518M,
	MIPI_FREQ_259M,
	MIPI_FREQ_567M,
};

/* Write registers up to 4 at a time */
static int ir6_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;
	//printk("ir6_write_reg\n");
	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ir6_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;
	//printk("ir6_write_array\n");
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret = ir6_write_reg(client, regs[i].addr,
				       IR6_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int ir6_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;
	//printk("ir6_read_reg\n");
	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	//Write register address 
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	//Read data from register 
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

// 计算传感器模式和目标分辨率距离
static int ir6_get_reso_dist(const struct ir6_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{	
	//printk("ir6_get_reso_dist\n");
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

// 根据目标分辨率选择最佳传感器模式
static const struct ir6_mode *
ir6_find_best_fit(struct ir6 *ir6, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;
	//printk("ir6_find_best_fit\n");
	for (i = 0; i < ir6->cfg_num; i++) {
		dist = ir6_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
			supported_modes[i].bus_fmt == framefmt->code) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int __ir6_power_on(struct ir6 *ir6);

// 切换传感器工作模式
static void ir6_change_mode(struct ir6 *ir6, const struct ir6_mode *mode)
{	//printk("ir6_change_mode\n");
	if (ir6->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
		ir6->is_thunderboot = false;
		ir6->is_thunderboot_ng = true;
		__ir6_power_on(ir6);
	}
	ir6->cur_mode = mode;
	ir6->cur_vts = ir6->cur_mode->vts_def;
	dev_dbg(&ir6->client->dev, "set fmt: cur_mode: %dx%d, hdr: %d\n",
		mode->width, mode->height, mode->hdr_mode);
}

// 设置图像格式
static int ir6_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{	
	struct ir6 *ir6 = to_ir6(sd);
	const struct ir6_mode *mode;
	s64 h_blank, vblank_def, vblank_min;
	u64 pixel_rate = 0;
	//printk("ir6_set_fmt\n");
	mutex_lock(&ir6->mutex);

	mode = ir6_find_best_fit(ir6, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ir6->mutex);
		return -ENOTTY;
#endif
	} else {
		ir6_change_mode(ir6, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ir6->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		/* VMAX >= (PIX_VWIDTH / 2) + 46 = height + 46 */
		vblank_min = (mode->height + 46) - mode->height;
		__v4l2_ctrl_modify_range(ir6->vblank, vblank_min,
					 IR6_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(ir6->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * IR6_4LANES;
		__v4l2_ctrl_s_ctrl_int64(ir6->pixel_rate,
					 pixel_rate);
	}

	mutex_unlock(&ir6->mutex);

	return 0;
}

// 获取当前传感器分辨率和图像格式
static int ir6_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ir6 *ir6 = to_ir6(sd);
	const struct ir6_mode *mode = ir6->cur_mode;
	//printk("ir6_get_fmt\n");
	mutex_lock(&ir6->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ir6->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&ir6->mutex);

	return 0;
}

static int ir6_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ir6 *ir6 = to_ir6(sd);
	//printk("ir6_enum_mbus_code\n");
	if (code->index != 0)
		return -EINVAL;
	code->code = ir6->cur_mode->bus_fmt;

	return 0;
}

// 枚举传感器支持的分辨率和图像格式
static int ir6_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ir6 *ir6 = to_ir6(sd);
	//printk("ir6_enum_frame_sizes\n");
	if (fse->index >= ir6->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

// 获取当前模式的帧间隔（帧率）
static int ir6_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ir6 *ir6 = to_ir6(sd);
	const struct ir6_mode *mode = ir6->cur_mode;
	//printk("ir6_g_frame_interval\n");
	mutex_lock(&ir6->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&ir6->mutex);

	return 0;
}

// 获取 Media 总线的配置
static int ir6_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct ir6 *ir6 = to_ir6(sd);
	const struct ir6_mode *mode = ir6->cur_mode;
	u32 val = 0;
	//printk("ir6_g_mbus_config\n");
	val = 1 << (IR6_4LANES - 1) |
	      V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;
	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

// 获取模块信息：sensor 名称等
static void ir6_get_module_inf(struct ir6 *ir6,
				  struct rkmodule_inf *inf)
{	//printk("ir6_get_module_inf\n");
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IR6_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ir6->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ir6->len_name, sizeof(inf->base.lens));
}

// 设置 HDR 自动曝光参数
static int ir6_set_hdrae(struct ir6 *ir6,
			    struct preisp_hdrae_exp_s *ae)
{
	struct i2c_client *client = ir6->client;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;
	u32 l_dcg_mode, m_dcg_mode, s_dcg_mode;

	u8 l_adc_vft=31,l_adc_h =0,l_adc_l =0,shr2_l=0;
	u32 shr1_l=0,shr3_l=0;

	u8 s_adc_vft=31,s_adc_h =0,s_adc_l =0,shr2_s=0;
	u32 shr1_s=0,shr3_s=0;

	int ret = 0;
	int  l_shr0,s_shr0,l_shr1,s_shr1;
	u32 cur_cg;
	u32 val = 0;
	printk("ir6_set_hdrae\n");

	if (!ir6->has_init_exp && !ir6->streaming) {
		ir6->init_hdrae_exp = *ae;
		ir6->has_init_exp = true;
		dev_dbg(&ir6->client->dev, "ir6 is not streaming, save hdr ae!\n");
		return ret;
	}
	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;
	l_dcg_mode = ae->long_cg_mode;
	m_dcg_mode = ae->middle_cg_mode;
	s_dcg_mode = ae->short_cg_mode;
	//printk("938_sethdrae:l_exp_time=%d,m_exp_time=%d,s_exp_time=%d\n l_a_gain =%d,m_a_gain =%d,s_a_gain =%d\n l_dcg_mode=%d,m_dcg_mode=%d,s_dcg_mode: %d\n",l_exp_time,m_exp_time,s_exp_time,l_a_gain,m_a_gain,s_a_gain,l_dcg_mode,m_dcg_mode, s_dcg_mode);

	dev_dbg(&client->dev,
		"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_a_gain, m_a_gain, s_a_gain);

	if (ir6->cur_mode->hdr_mode == HDR_X2) {
		l_a_gain = m_a_gain;
		l_exp_time = m_exp_time;
		l_dcg_mode = m_dcg_mode;
	}

	cur_cg=l_dcg_mode;
	ret = ir6_read_reg(client,
		IR6_DCG_GAIN_REG_0,
		IR6_REG_VALUE_08BIT,
		&val);
		//printk("961——value: %d\n", val);
	if(cur_cg ==val){
		if(cur_cg == GAIN_MODE_HCG){
			val = 0;
		}
		else if(cur_cg == GAIN_MODE_LCG){
			val = 1;
		}
		
		//printk("dcgout: %d\n", val);
		
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_0,
			IR6_REG_VALUE_08BIT,
			val);
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_1,
			IR6_REG_VALUE_08BIT,
			val);
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_2,
			IR6_REG_VALUE_08BIT,
			val);
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_3,
			IR6_REG_VALUE_08BIT,
			val);
	}

	shr1_l=l_a_gain ;
	if(shr1_l<=512){
		// printk("990_sethdrae:l_ana_gain: %d\n", shr1_l/16);
		shr2_l=0;
		shr3_l=0;
		if(shr1_l==16){
			l_adc_vft = 31;
			l_adc_h = 0;
			l_adc_l = (shr1_l-16)*16 ;
		}
		else if(shr1_l>16&&shr1_l<=32){
			l_adc_vft = 43;
			l_adc_h = 0;
			l_adc_l = shr1_l-16 ;
		}
		else if(shr1_l>32&&shr1_l<=64){
			l_adc_vft = 48;
			l_adc_h = 0;
			l_adc_l = shr1_l-16 ;
		}
		else if(shr1_l>64&&shr1_l<=96){
			l_adc_vft = 49;
			l_adc_h = 0;
			l_adc_l = shr1_l-16 ;
		}
		else if(shr1_l>96&&shr1_l<=256){
			l_adc_vft = 50;
			l_adc_h = 0;
			if(shr1_l<=128){
				l_adc_l = shr1_l-16 ;
			}
			else{
				l_adc_l = 128 + (shr1_l/2-16);
			}
		}
		else if(shr1_l>256&&shr1_l<=512){
			l_adc_vft = 45;
			if(shr1_l<=272){
				l_adc_h = 0;
				l_adc_l = 128 + (shr1_l/2-16);
			}
			else{
				l_adc_h = 1;
				l_adc_l = shr1_l/4-16 ;
			}
		}
	}
	else if(shr1_l>512&&shr1_l<=2528){
		//printk("1041_sethdrae:l_dig_gain: %d\n", (shr1_l-480)/32);
		shr2_l = (shr1_l -480)/ 32-1;
		shr3_l = (shr1_l - 480 - (shr2_l+1) * 32) * 1024 / 32;
		l_adc_vft = 45;
		l_adc_h = 1;
		l_adc_l = 112;
	}
	
	ret = ir6_write_reg(ir6->client,IR6_ADC_VFT_REG,
					IR6_REG_VALUE_08BIT,
					IR6_FETCH_GAIN_L(l_adc_vft));
	ret |= ir6_write_reg(ir6->client, IR6_ADC_GAIN_REG_H,
			       IR6_REG_VALUE_08BIT,
			       IR6_FETCH_GAIN_L(l_adc_h));
	ret |= ir6_write_reg(ir6->client, IR6_ADC_GAIN_REG_L,
			       IR6_REG_VALUE_08BIT,
			       IR6_FETCH_GAIN_L(l_adc_l));
	
	ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_COARSE_DOL1,
			       IR6_REG_VALUE_08BIT,
			       IR6_DIG_GAIN_L(shr2_l));
	ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_FINE_L_DOL1,
			       IR6_REG_VALUE_08BIT,
			       IR6_DIG_GAIN_FINE_L(shr3_l));
	ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_FINE_H_DOL1,
			       IR6_REG_VALUE_08BIT,
			       IR6_DIG_GAIN_FINE_H(shr3_l));

	shr1_s=s_a_gain ;
	if(shr1_s<=512){
		//printk("1075_sethdrae:s_ana_gain: %d\n", shr1_s/16);
		shr2_s=0;
		shr3_s=0;
		if(shr1_s==16){
			s_adc_vft = 31;
			s_adc_h = 0;
			s_adc_l = (shr1_s-16)*16 ;
		}
		else if(shr1_s>16&&shr1_s<=32){
			s_adc_vft = 43;
			s_adc_h = 0;
			s_adc_l = shr1_s-16 ;
		}
		else if(shr1_s>32&&shr1_s<=64){
			s_adc_vft = 48;
			s_adc_h = 0;
			s_adc_l = shr1_s-16 ;
		}
		else if(shr1_s>64&&shr1_s<=96){
			s_adc_vft = 49;
			s_adc_h = 0;
			s_adc_l = shr1_s-16 ;
		}
		else if(shr1_s>96&&shr1_s<=256){
			s_adc_vft = 50;
			s_adc_h = 0;
			if(shr1_s<=128){
				s_adc_l = shr1_s-16 ;
			}
			else{
				s_adc_l = 128 + (shr1_s/2-16);
			}
		}
		else if(shr1_s>256&&shr1_s<=512){
			s_adc_vft = 45;
			if(shr1_s<=272){
				s_adc_h = 0;
				s_adc_l = 128 + (shr1_s/2-16);
			}
			else{
				s_adc_h = 1;
				s_adc_l = shr1_s/4-16 ;
			}
		}
	}
	else if(shr1_s>512&&shr1_s<=2528){
		//printk("1125_sethdrae:s_dig_gain: %d\n", (shr1_s-480)/32);
		shr2_s = (shr1_s - 480) / 32-1;
		shr3_s = (shr1_s - 480 -(shr2_s+1) * 32) * 1024 / 32;
		s_adc_vft = 45;
		s_adc_h = 1;
		s_adc_l = 112;
	}

	ret = ir6_write_reg(ir6->client,IR6_ADC_DOL2_VFT_REG,
					IR6_REG_VALUE_08BIT,
					IR6_FETCH_GAIN_L(s_adc_vft));
	ret |= ir6_write_reg(ir6->client, IR6_ADC_DOL2_GAIN_REG_H,
			       IR6_REG_VALUE_08BIT,
			       IR6_FETCH_GAIN_L(s_adc_h));
	ret |= ir6_write_reg(ir6->client, IR6_ADC_DOL2_GAIN_REG_L,
			       IR6_REG_VALUE_08BIT,
			       IR6_FETCH_GAIN_L(s_adc_l));

	ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_COARSE_DOL2,
			       IR6_REG_VALUE_08BIT,
			       IR6_DIG_GAIN_L(shr2_s));
	ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_FINE_L_DOL2,
			       IR6_REG_VALUE_08BIT,
			       IR6_DIG_GAIN_FINE_L(shr3_s));
	ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_FINE_H_DOL2,
			       IR6_REG_VALUE_08BIT,
			       IR6_DIG_GAIN_FINE_H(shr3_s));
	
	l_shr0 = l_exp_time;
	s_shr0 = s_exp_time+1;
	s_shr1 = 201-s_shr0;
	// ret |= ir6_write_reg(client,
	// 	IR6_VSYNC_ITV_L,
	// 	IR6_REG_VALUE_08BIT,
	// 	IR6_FETCH_EXPT_L(s_shr0));

	ret |= ir6_write_reg(client,
		IR6_EXPT_LINE_L,
		IR6_REG_VALUE_08BIT,
		IR6_FETCH_EXPT_L(l_shr0));
	ret |= ir6_write_reg(client,
		IR6_EXPT_LINE_M,
		IR6_REG_VALUE_08BIT,
		IR6_FETCH_EXPT_M(l_shr0));
	ret |= ir6_write_reg(client,
		IR6_EXPT_LINE_H,
		IR6_REG_VALUE_08BIT,
		IR6_FETCH_EXPT_H(l_shr0));
	// if(s_shr0<201){
	// 	s_shr1 = 201-s_shr0;
	// 	ret |= ir6_write_reg(client,
	// 		IR6_REG_B_L,
	// 		IR6_REG_VALUE_08BIT,
	// 		IR6_FETCH_EXPT_L(s_shr1));
	// 	ret |= ir6_write_reg(client,
	// 		IR6_REG_B_M,
	// 		IR6_REG_VALUE_08BIT,
	// 		IR6_FETCH_EXPT_M(s_shr1));
	// 	ret |= ir6_write_reg(client,
	// 		IR6_REG_B_H,
	// 		IR6_REG_VALUE_08BIT,
	// 		IR6_FETCH_EXPT_H(s_shr1));
	// }
	// ret |= ir6_write_reg(client,
	// 	IR6_REG_C_L,
	// 	IR6_REG_VALUE_08BIT,
	// 	IR6_FETCH_EXPT_L(s_shr0));
	// ret |= ir6_write_reg(client,
	// 	IR6_REG_C_M,
	// 	IR6_REG_VALUE_08BIT,
	// 	IR6_FETCH_EXPT_M(s_shr0));
	// ret |= ir6_write_reg(client,
	// 	IR6_REG_C_H,
	// 	IR6_REG_VALUE_08BIT,
	// 	IR6_FETCH_EXPT_H(s_shr0));
	
	l_shr1=1300-l_shr0;
	ret |= ir6_write_reg(client,
		IR6_REG_A_L,
		IR6_REG_VALUE_08BIT,
		IR6_FETCH_EXPT_L(l_shr1));
	ret |= ir6_write_reg(client,
		IR6_REG_A_M,
		IR6_REG_VALUE_08BIT,
		IR6_FETCH_EXPT_M(l_shr1));
	ret |= ir6_write_reg(client,
		IR6_REG_A_H,
		IR6_REG_VALUE_08BIT,
		IR6_FETCH_EXPT_H(l_shr1));
	//printk("1285:l_a_time: %d,s_a_time: %d\n", l_shr0 , s_shr0);

	
	// ret |= ir6_write_reg(client,
	// 	IR6_REQ_SEL,
	// 	IR6_REG_VALUE_08BIT,
	// 	IR6_FETCH_EXPT_L(1));
	// ret |= ir6_write_reg(client,
	// 	IR6_REQ_SYS,
	// 	IR6_REG_VALUE_08BIT,
	// 	IR6_FETCH_EXPT_L(1));
	// ret |= ir6_write_reg(client,
	// 	IR6_REQ_SYS,
	// 	IR6_REG_VALUE_08BIT,
	// 	IR6_FETCH_EXPT_L(0));
	return ret;
}

// 传感器增益转换（配置寄存器）
static int ir6_set_conversion_gain(struct ir6 *ir6, u32 *cg)
{
	int ret = 0;
	struct i2c_client *client = ir6->client;
	u32 cur_cg = *cg;
	u32 val = 0;
	//printk("ir6_set_conversion_gain\n");
	dev_dbg(&ir6->client->dev, "set conversion gain %d\n", cur_cg);
	if (ir6->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
		ir6->is_thunderboot = false;
		ir6->is_thunderboot_ng = true;
		__ir6_power_on(ir6);
	}

	ret = ir6_read_reg(client,
		IR6_DCG_GAIN_REG_0,
		IR6_REG_VALUE_08BIT,
		&val);
	if(cur_cg ==val){

		if(cur_cg == GAIN_MODE_HCG){
			val = 0;
		}
		else if(cur_cg == GAIN_MODE_LCG){
			val = 1;
		}
		//printk("dcgout: %d\n", val);
		
	
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_0,
			IR6_REG_VALUE_08BIT,
			val);
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_1,
			IR6_REG_VALUE_08BIT,
			val);
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_2,
			IR6_REG_VALUE_08BIT,
			val);
		ret |= ir6_write_reg(client,
			IR6_DCG_GAIN_REG_3,
			IR6_REG_VALUE_08BIT,
			val);
	}

	return ret;
}

#ifdef USED_SYS_DEBUG
//ag: echo 0 >  /sys/devices/platform/ff510000.i2c/i2c-1/1-0036-1/cam_s_cg
static ssize_t set_conversion_gain_status(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ir6 *ir6 = to_ir6(sd);
	int status = 0;
	int ret = 0;
	//printk("set_conversion_gain_status\n");
	ret = kstrtoint(buf, 0, &status);
	if (!ret && status >= 0 && status < 2)
		ir6_set_conversion_gain(ir6, &status);
	else
		dev_err(dev, "input 0 for LCG, 1 for HCG, cur %d\n", status);
	return count;
}

static struct device_attribute attributes[] = {
	__ATTR(cam_s_cg, S_IWUSR, NULL, set_conversion_gain_status),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;
	//printk("add_sysfs_interfaces\n");
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0 ; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}
#endif

// V4l2 子设备 ioctl 接口
static long ir6_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{	
	struct ir6 *ir6 = to_ir6(sd);
	struct rkmodule_hdr_cfg *hdr;
	//struct rkmodule_dcg_ratio *dcg;os04a10
	u32 i, h, w, stream;
	long ret = 0;
	const struct ir6_mode *mode;
	u64 pixel_rate = 0;
	//printk("ir6_ioctl:%d\n",cmd);
	switch (cmd) {
	// HDR 自动曝光
	case PREISP_CMD_SET_HDRAE_EXP:
		if (ir6->cur_mode->hdr_mode == HDR_X2)
			ret = ir6_set_hdrae(ir6, arg);
		//else if (ir6->cur_mode->hdr_mode == HDR_X3)
		//	ret = ir6_set_hdrae_3frame(ir6, arg);
		//printk("PREISP_CMD_SET_HDRAE_EXP\n");
		break;
	// 获取模块信息
	case RKMODULE_GET_MODULE_INFO:
		ir6_get_module_inf(ir6, (struct rkmodule_inf *)arg);
		//printk("RKMODULE_GET_MODULE_INFO\n");
		break;
	// 获取 HDR 配置
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = ir6->cur_mode->hdr_mode;
		//printk("RKMODULE_GET_HDR_CFG:%d\n",hdr->hdr_mode);
		break;
	// 设置 HDR 配置
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = ir6->cur_mode->width;
		h = ir6->cur_mode->height;
		//printk("RKMODULE_SET_HDR_CFG\n");
		for (i = 0; i < ir6->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
					//printk("i：%d\n",i);
				ir6_change_mode(ir6, &supported_modes[i]);
				break;
			}
		}
		if (i == ir6->cfg_num) {
			dev_err(&ir6->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mode = ir6->cur_mode;
			if (ir6->streaming) {

				ret |= ir6_write_array(ir6->client, ir6->cur_mode->reg_list);
				if (ret)
					return ret;
			}
			w = mode->hts_def - ir6->cur_mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(ir6->hblank, 10, 2000, 1, 100);
			__v4l2_ctrl_modify_range(ir6->vblank, h,
				IR6_VTS_MAX - mode->height,
				1, h);
			__v4l2_ctrl_s_ctrl(ir6->link_freq, mode->mipi_freq_idx);
			pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * IR6_4LANES;
			__v4l2_ctrl_s_ctrl_int64(ir6->pixel_rate,
						 pixel_rate);
		}
		break;
	// 设置转换增益
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = ir6_set_conversion_gain(ir6, (u32 *)arg);
		//printk("RKMODULE_SET_CONVERSION_GAIN\n");
		break;//os04a10
	// 快速启动或停止视频流
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);
		//printk("RKMODULE_SET_QUICK_STREAM\n");
		if (stream)
			ret = ir6_write_reg(ir6->client, IR6_SLEEP_MODE,
				IR6_REG_VALUE_08BIT, IR6_MODE_STREAMING);
		else
			ret = ir6_write_reg(ir6->client, IR6_SLEEP_MODE,
				IR6_REG_VALUE_08BIT, IR6_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_SONY_BRL:
		//printk("RKMODULE_GET_SONY_BRL\n");
		if (ir6->cur_mode->width == 1936 && ir6->cur_mode->height == 1280)
			*((u32 *)arg) = BRL_ALL;
		else
			*((u32 *)arg) = BRL_BINNING;
		//printk("1\n");
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ir6_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;

	long ret;
	u32  stream;
	u32 brl = 0;
	//printk("ir6_compat_ioctl32\n");
	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ir6_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree(inf);
				return -EFAULT;
			}
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -EFAULT;
		}
		ret = ir6_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ir6_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr))) {
				kfree(hdr);
				return -EFAULT;
			}
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdr, up, sizeof(*hdr))) {
			kfree(hdr);
			return -EFAULT;
		}
		ret = ir6_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdrae, up, sizeof(*hdrae))) {
			kfree(hdrae);
			return -EFAULT;
		}
		ret = ir6_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN://os04a10
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = ir6_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = ir6_ioctl(sd, cmd, &stream);
		break;
	case RKMODULE_GET_DCG_RATIO://os04a10
		dcg = kzalloc(sizeof(*dcg), GFP_KERNEL);
		if (!dcg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ir6_ioctl(sd, cmd, dcg);
		if (!ret)
			ret = copy_to_user(up, dcg, sizeof(*dcg));
		kfree(dcg);
		break;
	case RKMODULE_GET_SONY_BRL:
		ret = ir6_ioctl(sd, cmd, &brl);
		if (!ret) {
			if (copy_to_user(up, &brl, sizeof(u32)))
				return -EFAULT;
		}
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

// 初始化传感器的转换增益
static int ir6_init_conversion_gain(struct ir6 *ir6)
{
	int ret = 0;
	struct i2c_client *client = ir6->client;
	u32 val = 0;
	//printk("ir6_init_conversion_gain\n");

	ret = ir6_read_reg(client,
		IR6_DCG_GAIN_REG_0,
		IR6_REG_VALUE_08BIT,
		&val);
	val |= 0x01;
	ret |= ir6_write_reg(client,
		IR6_DCG_GAIN_REG_0,
		IR6_REG_VALUE_08BIT,
		val);
	ret |= ir6_write_reg(client,
		IR6_DCG_GAIN_REG_1,
		IR6_REG_VALUE_08BIT,
		val);
	ret |= ir6_write_reg(client,
		IR6_DCG_GAIN_REG_2,
		IR6_REG_VALUE_08BIT,
		val);
	ret |= ir6_write_reg(client,
		IR6_DCG_GAIN_REG_3,
		IR6_REG_VALUE_08BIT,
		val);
	ir6->long_hcg = false;
	ir6->middle_hcg = false;
	ir6->short_hcg = false;
	return ret;
}

// 开启视频流
static int __ir6_start_stream(struct ir6 *ir6)
{
	int ret;
	if (!ir6->is_thunderboot) {
		ret = ir6_write_array(ir6->client, ir6->cur_mode->reg_list);
		if (ret)
			return ret;
		//printk("1111111773");
	}
//os04a10
	ret = ir6_init_conversion_gain(ir6);
	if (ret)
		return ret;
	//printk("__ir6_start_stream\n");

	// In case these controls are set before streaming 
	ret = __v4l2_ctrl_handler_setup(&ir6->ctrl_handler);
	if (ret)
		return ret;
	if (ir6->has_init_exp && ir6->cur_mode->hdr_mode != NO_HDR) {
		ret = ir6_ioctl(&ir6->subdev, PREISP_CMD_SET_HDRAE_EXP,
			&ir6->init_hdrae_exp);
		if (ret) {
			dev_err(&ir6->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}
	return ir6_write_reg(ir6->client, IR6_SLEEP_MODE,
				IR6_REG_VALUE_08BIT, 0);
}

static int __ir6_stop_stream(struct ir6 *ir6)
{//printk("__ir6_stop_stream\n");
	ir6->has_init_exp = false;
	if (ir6->is_thunderboot)
		ir6->is_first_streamoff = true;
	return ir6_write_reg(ir6->client, IR6_SLEEP_MODE,
				IR6_REG_VALUE_08BIT, 1);
}

// 控制传感器的视频流
static int ir6_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ir6 *ir6 = to_ir6(sd);
	struct i2c_client *client = ir6->client;
	int ret = 0;

	dev_dbg(&ir6->client->dev, "s_stream: %d. %dx%d, hdr: %d, bpp: %d\n",
	       on, ir6->cur_mode->width, ir6->cur_mode->height,
	       ir6->cur_mode->hdr_mode, ir6->cur_mode->bpp);
	//printk("ir6_s_stream\n");
	mutex_lock(&ir6->mutex);
	on = !!on;
	if (on == ir6->streaming)
		goto unlock_and_return;

	if (on) {
		if (ir6->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			ir6->is_thunderboot = false;
			__ir6_power_on(ir6);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ir6_start_stream(ir6);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ir6_stop_stream(ir6);
		pm_runtime_put(&client->dev);
	}

	ir6->streaming = on;

unlock_and_return:
	mutex_unlock(&ir6->mutex);

	return ret;
}

static int ir6_runtime_suspend(struct device *dev);
static int ir6_runtime_resume(struct device *dev);
static int ir6_s_power(struct v4l2_subdev *sd, int on)
{
	struct ir6 *ir6 = to_ir6(sd);
	struct i2c_client *client = ir6->client;
	int ret = 0;

	//printk("ir6_s_power\n");
	mutex_lock(&ir6->mutex);

	if (ir6->power_on == !!on)
		goto unlock_and_return;
	
	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		if(gpiod_get_value(ir6->power_gpio)==0){
			ir6_runtime_resume(&client->dev);
		}
		ir6->power_on = true;

	} else {
		pm_runtime_put(&client->dev);
		if(gpiod_get_value(ir6->power_gpio)==1){
			ir6_runtime_suspend(&client->dev);
		}
		ir6->power_on = false;
	}
	
unlock_and_return:
	mutex_unlock(&ir6->mutex);
		
	return ret;
}
//os04a10
 //Calculate the delay in us by clock rate and clock cycles 
static inline u32 ir6_cal_delay(u32 cycles)
{	//printk("ir6_cal_delay\n");
	return DIV_ROUND_UP(cycles, IR6_XVCLK_FREQ_27M / 1000 / 1000);
}

int __ir6_power_on(struct ir6 *ir6)
{
	int ret;
	struct device *dev = &ir6->client->dev;
	printk("__ir6_power_on\n");
	if (ir6->is_thunderboot)
		return 0;

	if (!IS_ERR_OR_NULL(ir6->pins_default)) {
		ret = pinctrl_select_state(ir6->pinctrl,
					   ir6->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = regulator_bulk_enable(IR6_NUM_SUPPLIES, ir6->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto err_pinctrl;
	}
	if (!IS_ERR(ir6->power_gpio))
		gpiod_direction_output(ir6->power_gpio, 1);
	/* At least 500ns between power raising and XCLR */
	/* fix power on timing if insmod this ko */
	usleep_range(10 * 1000, 20 * 1000);
	if (!IS_ERR(ir6->reset_gpio))
		gpiod_direction_output(ir6->reset_gpio, 1);

	/* At least 1us between XCLR and clk */
	/* fix power on timing if insmod this ko */
	usleep_range(10 * 1000, 20 * 1000);
	ret = clk_set_rate(ir6->xvclk, IR6_XVCLK_FREQ_27M);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate\n");
	if (clk_get_rate(ir6->xvclk) != IR6_XVCLK_FREQ_27M)
		dev_warn(dev, "xvclk mismatched\n");
	ret = clk_prepare_enable(ir6->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		goto err_clk;
	}

	mdelay(2);
	return 0;

err_clk:
	if (!IS_ERR(ir6->reset_gpio))
		gpiod_direction_output(ir6->reset_gpio, 1);
	regulator_bulk_disable(IR6_NUM_SUPPLIES, ir6->supplies);

err_pinctrl:
	if (!IS_ERR_OR_NULL(ir6->pins_sleep))
		pinctrl_select_state(ir6->pinctrl, ir6->pins_sleep);

	return ret;
}

static void __ir6_power_off(struct ir6 *ir6)
{
	int ret;
	struct device *dev = &ir6->client->dev;
	//printk("__ir6_power_off\n");
	if (ir6->is_thunderboot) {
		if (ir6->is_first_streamoff) {
			ir6->is_thunderboot = false;
			ir6->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(ir6->reset_gpio))
		gpiod_direction_output(ir6->reset_gpio, 0);
	clk_disable_unprepare(ir6->xvclk);
	if (!IS_ERR_OR_NULL(ir6->pins_sleep)) {
		ret = pinctrl_select_state(ir6->pinctrl,
					   ir6->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(ir6->power_gpio))
		gpiod_direction_output(ir6->power_gpio, 0);
	regulator_bulk_disable(IR6_NUM_SUPPLIES, ir6->supplies);
}

static int ir6_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ir6 *ir6 = to_ir6(sd);
	//printk("ir6_runtime_resume\n");
	return __ir6_power_on(ir6);
}

static int ir6_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ir6 *ir6 = to_ir6(sd);
	//printk("ir6_runtime_suspend\n");
	__ir6_power_off(ir6);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ir6_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ir6 *ir6 = to_ir6(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ir6_mode *def_mode = &supported_modes[0];
	//printk("ir6_open\n");
	mutex_lock(&ir6->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ir6->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ir6_enum_frame_interval(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ir6 *ir6 = to_ir6(sd);
	//printk("ir6_enum_frame_interval\n");
	if (fie->index >= ir6->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH_3840 3840
#define DST_HEIGHT_2160 2160
#define DST_WIDTH_1920 1920
#define DST_HEIGHT_1080 1080

/*
 * The resolution of the driver configuration needs to be exactly
 * the same as the current output resolution of the sensor,
 * the input width of the isp needs to be 16 aligned,
 * the input height of the isp needs to be 8 aligned.
 * Can be cropped to standard resolution by this function,
 * otherwise it will crop out strange resolution according
 * to the alignment rules.
 */
static int ir6_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct ir6 *ir6 = to_ir6(sd);
	//printk("ir6_get_selection\n");
	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		if (ir6->cur_mode->width == 3864) {
			sel->r.left = CROP_START(ir6->cur_mode->width, DST_WIDTH_3840);
			sel->r.width = DST_WIDTH_3840;
			sel->r.top = CROP_START(ir6->cur_mode->height, DST_HEIGHT_2160);
			sel->r.height = DST_HEIGHT_2160;
		} else if (ir6->cur_mode->width == 1936) {
			sel->r.left = CROP_START(ir6->cur_mode->width, DST_WIDTH_1920);
			sel->r.width = DST_WIDTH_1920;
			sel->r.top = CROP_START(ir6->cur_mode->height, DST_HEIGHT_1080);
			sel->r.height = DST_HEIGHT_1080;
		} else {
			sel->r.left = CROP_START(ir6->cur_mode->width, ir6->cur_mode->width);
			sel->r.width = ir6->cur_mode->width;
			sel->r.top = CROP_START(ir6->cur_mode->height, ir6->cur_mode->height);
			sel->r.height = ir6->cur_mode->height;
		}
		return 0;
	}
	return -EINVAL;
}

static const struct dev_pm_ops ir6_pm_ops = {
	SET_RUNTIME_PM_OPS(ir6_runtime_suspend,
			   ir6_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ir6_internal_ops = {
	.open = ir6_open,
};
#endif

static const struct v4l2_subdev_core_ops ir6_core_ops = {
	.s_power = ir6_s_power,
	.ioctl = ir6_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ir6_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ir6_video_ops = {
	.s_stream = ir6_s_stream,
	.g_frame_interval = ir6_g_frame_interval,
	.g_mbus_config = ir6_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ir6_pad_ops = {
	.enum_mbus_code = ir6_enum_mbus_code,
	.enum_frame_size = ir6_enum_frame_sizes,
	.enum_frame_interval = ir6_enum_frame_interval,
	.get_fmt = ir6_get_fmt,
	.set_fmt = ir6_set_fmt,
	.get_selection = ir6_get_selection,
};

static const struct v4l2_subdev_ops ir6_subdev_ops = {
	.core	= &ir6_core_ops,
	.video	= &ir6_video_ops,
	.pad	= &ir6_pad_ops,
};

static int ir6_set_ctrl(struct v4l2_ctrl *ctrl)
{

	struct timeval tv;
	struct ir6 *ir6 = container_of(ctrl->handler,
					     struct ir6, ctrl_handler);
	struct i2c_client *client = ir6->client;
	s64 max;
	u32 vts = 0, val;
	u8 adc_vft=31,adc_h =0,adc_l =0,shr2=0;
	int ret = 0;
	u32 shr0 = 0,shr1=0,shr3=0;
	// printk("ir6_set_ctrl\n");
	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		if (ir6->cur_mode->hdr_mode == NO_HDR) {
			/* Update max exposure while meeting expected vblanking */
			max = ir6->cur_mode->height + ctrl->val - 41;
			printk("V4L2_CID_VBLANK:%d\n\n\n",ctrl->val);
			__v4l2_ctrl_modify_range(ir6->exposure,
					 ir6->exposure->minimum, max,
					 ir6->exposure->step,
					 ir6->exposure->default_value);
		}
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BAND_STOP_FILTER:
		printk("V4L2_CID_BAND_STOP_FILTER\n\n\n");
		break;
	case V4L2_CID_EXPOSURE:
		if (ir6->cur_mode->hdr_mode != NO_HDR)
			return ret;
		shr0 = ctrl->val;
		printk("V4L2_CID_EXPOSURE:%d\n\n",shr0);
		ret = ir6_write_reg(ir6->client, IR6_EXPT_LINE_L,
				       IR6_REG_VALUE_08BIT,
				       IR6_FETCH_EXPT_L(shr0));
		ret |= ir6_write_reg(ir6->client, IR6_EXPT_LINE_M,
				       IR6_REG_VALUE_08BIT,
				       IR6_FETCH_EXPT_M(shr0));
		ret |= ir6_write_reg(ir6->client, IR6_EXPT_LINE_H,
				       IR6_REG_VALUE_08BIT,
				       IR6_FETCH_EXPT_H(shr0));
		dev_dbg(&client->dev, "set exposure(shr0) %d = cur_vts(%d) - val(%d)\n",
			shr0, ir6->cur_vts, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (ir6->cur_mode->hdr_mode != NO_HDR)
			return ret;
		shr1=ctrl->val ;
		if(shr1<=512){
			shr2=0;
			shr3=0;
			if(shr1==16){
				adc_vft = 31;
				adc_h = 0;
				adc_l = (shr1-16)*16 ;
			}
			else if(shr1>16&&shr1<=32){
				adc_vft = 43;
				adc_h = 0;
				adc_l = shr1-16 ;
			}
			else if(shr1>32&&shr1<=64){
				adc_vft = 48;
				adc_h = 0;
				adc_l = shr1-16 ;
			}
			else if(shr1>64&&shr1<=96){
				adc_vft = 49;
				adc_h = 0;
				adc_l = shr1-16 ;
			}
			else if(shr1>96&&shr1<=256){
				adc_vft = 50;
				adc_h = 0;
				if(shr1<=128){
					adc_l = shr1-16 ;
				}
				else{
					adc_l = 128 + (shr1/2-16);
				}
			}
			else if(shr1>256&&shr1<=512){
				adc_vft = 45;
				if(shr1<=272){
					adc_h = 0;
					adc_l = 128 + (shr1/2-16);
				}
				else{
					adc_h = 1;
					adc_l = shr1/4-16 ;
				}
			}
		}
		else if(shr1>512&&shr1<=2528){
			shr2 = (shr1 - 480)/32-1;
			shr3 = (shr1 - 480 - (shr2+1) * 32) * 1024 / 32;
			adc_vft = 45;
			adc_h = 1;
			adc_l = 112;
		}
		
		ret = ir6_write_reg(ir6->client,IR6_ADC_VFT_REG,
						IR6_REG_VALUE_08BIT,
						IR6_FETCH_GAIN_L(adc_vft));
		ret |= ir6_write_reg(ir6->client, IR6_ADC_GAIN_REG_H,
				       IR6_REG_VALUE_08BIT,
				       IR6_FETCH_GAIN_L(adc_h));
		ret |= ir6_write_reg(ir6->client, IR6_ADC_GAIN_REG_L,
				       IR6_REG_VALUE_08BIT,
				       IR6_FETCH_GAIN_L(adc_l));
		
		ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_COARSE_DOL1,
				       IR6_REG_VALUE_08BIT,
				       IR6_DIG_GAIN_L(shr2));
		ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_FINE_H_DOL1,
				       IR6_REG_VALUE_08BIT,
				       IR6_DIG_GAIN_FINE_H(shr3));
		ret |= ir6_write_reg(ir6->client, IR6_DIG_GAIN_FINE_L_DOL1,
				       IR6_REG_VALUE_08BIT,
				       IR6_DIG_GAIN_FINE_L(shr3));
		
		dev_dbg(&client->dev, "set analog gain 0x%x\n",
			ctrl->val);
		break;
		
	case V4L2_CID_AUDIO_VOLUME:
		if (ir6->cur_mode->hdr_mode != NO_HDR)
			return ret;
		// ir6_read_array(ir6->client,read_TC001_ir6_MIPI4Lane_60fps12Bit_MCLK518p4M);
		do_gettimeofday(&tv);
	    if (!IS_ERR(ir6->exp_gpio))
			gpiod_direction_output(ir6->exp_gpio, 1);
		mdelay(1);
		gpiod_direction_output(ir6->exp_gpio, 0);

		mdelay(ctrl->val-1);
		do_gettimeofday(&tv);

		gpiod_direction_output(ir6->trig_gpio, 0);
		if (!IS_ERR(ir6->frame_gpio))
		 	gpiod_direction_output(ir6->frame_gpio, 1);
		mdelay(1);
		gpiod_direction_output(ir6->frame_gpio, 0);
		   
		dev_dbg(&client->dev, "set exp_enable value 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_AUDIO_BALANCE:
	
		val=ctrl->val;
		// //printk("trig_reg:%d",val);
		gpiod_direction_output(ir6->trig_gpio, val);
		break;
	case V4L2_CID_GAIN:
		if (ir6->cur_mode->hdr_mode != NO_HDR)
			return ret;
		// //printk("dcg_number=%d",ctrl->val);
		ret = ir6_write_reg(ir6->client, IR6_DCG_GAIN_REG_0,
				       IR6_REG_VALUE_08BIT,
				       IR6_DCG_SWITCH(ctrl->val));
		ret |= ir6_write_reg(ir6->client, IR6_DCG_GAIN_REG_1,
				       IR6_REG_VALUE_08BIT,
				       IR6_DCG_SWITCH(ctrl->val));
		ret |= ir6_write_reg(ir6->client, IR6_DCG_GAIN_REG_2,
				       IR6_REG_VALUE_08BIT,
				       IR6_DCG_SWITCH(ctrl->val));
		ret |= ir6_write_reg(ir6->client, IR6_DCG_GAIN_REG_3,
				       IR6_REG_VALUE_08BIT,
				       IR6_DCG_SWITCH(ctrl->val));
		ret |= ir6_read_reg(client,
		IR6_DCG_GAIN_REG_0,
		IR6_REG_VALUE_08BIT,
		&val);
		// //printk("dcg_addr_reg=%d",val);
		dev_dbg(&client->dev, "set dcg value 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + ir6->cur_mode->height;
		/*
		 * vts of hdr mode is double to correct T-line calculation.
		 * Restore before write to reg.
		 */
		if (ir6->cur_mode->hdr_mode == HDR_X2) {
			vts = (vts + 3) / 4 * 4;
			ir6->cur_vts = vts;
			vts /= 2;
		} else if (ir6->cur_mode->hdr_mode == HDR_X3) {
			vts = (vts + 11) / 12 * 12;
			ir6->cur_vts = vts;
			vts /= 4;
		} else {
			ir6->cur_vts = vts;
		}
		dev_dbg(&client->dev, "set vblank 0x%x vts %d\n",
			ctrl->val, vts);
		break;
	case V4L2_CID_HFLIP:
		ret = ir6_read_reg(ir6->client, IR6_FLIP_REG_COL,
				      IR6_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IR6_COL_BIT_MASK;
		else
			val &= ~IR6_COL_BIT_MASK;
		ret = ir6_write_reg(ir6->client, IR6_FLIP_REG_COL,
				       IR6_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = ir6_read_reg(ir6->client, IR6_FLIP_REG_ROW,
				      IR6_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IR6_ROW_BIT_MASK;
		else
			val &= ~IR6_ROW_BIT_MASK;
		ret = ir6_write_reg(ir6->client, IR6_FLIP_REG_ROW,
				       IR6_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ir6_ctrl_ops = {
	.s_ctrl = ir6_set_ctrl,
};

static int ir6_initialize_controls(struct ir6 *ir6)
{
	const struct ir6_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u64 pixel_rate;
	u32 h_blank;
	int ret;
	//printk("ir6_initialize_controls\n");
	handler = &ir6->ctrl_handler;
	mode = ir6->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &ir6->mutex;

	ir6->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_items) - 1, 0,
				link_freq_items);
	__v4l2_ctrl_s_ctrl(ir6->link_freq, mode->mipi_freq_idx);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * IR6_4LANES;
	ir6->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
		V4L2_CID_PIXEL_RATE, 0, IR6_MAX_PIXEL_RATE,
		1, pixel_rate);

	h_blank = mode->hts_def - mode->width;
	ir6->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				10, 2000, 1, 200);
	if (ir6->hblank)
		ir6->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ir6->vblank = v4l2_ctrl_new_std(handler, &ir6_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				IR6_VTS_MAX - mode->height,
				1, vblank_def);
	ir6->cur_vts = mode->vts_def;

	exposure_max = mode->vts_def-41;
	ir6->exposure = v4l2_ctrl_new_std(handler, &ir6_ctrl_ops,
				V4L2_CID_EXPOSURE, IR6_EXPOSURE_MIN,
				exposure_max, IR6_EXPOSURE_STEP,
				25);

	ir6->anal_a_gain = v4l2_ctrl_new_std(handler, &ir6_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, IR6_ADC_GAIN_MIN,
				IR6_ADC_GAIN_MAX, IR6_ADC_GAIN_STEP,
				IR6_ADC_GAIN_DEFAULT);

	v4l2_ctrl_new_std(handler, &ir6_ctrl_ops,V4L2_CID_GAIN, 0 , 1, 1 , 1);
	v4l2_ctrl_new_std(handler, &ir6_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &ir6_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &ir6_ctrl_ops, V4L2_CID_AUDIO_VOLUME, 0, 1800, 1, 2);
	v4l2_ctrl_new_std(handler, &ir6_ctrl_ops, V4L2_CID_AUDIO_BALANCE, 0, 1, 1, 0);
	// ir6->digi_gain=v4l2_ctrl_new_std(handler, &ir6_ctrl_ops, V4L2_CID_BAND_STOP_FILTER, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&ir6->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ir6->subdev.ctrl_handler = handler;
	ir6->has_init_exp = false;
	//os04a10
	ir6->long_hcg = false;
	ir6->middle_hcg = false;
	ir6->short_hcg = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ir6_check_sensor_id(struct ir6 *ir6,
				  struct i2c_client *client)
{
	struct device *dev = &ir6->client->dev;
	u32 id = 0;
	int ret;
	//printk("ir6_check_sensor_id\n");
	if (ir6->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = ir6_read_reg(client, IR6_REG_CHIP_ID,
			      IR6_REG_VALUE_08BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected ir6 id %06x\n", CHIP_ID);

	return 0;
}

static int ir6_configure_regulators(struct ir6 *ir6)
{
	unsigned int i;
	//printk("ir6_configure_regulators\n");
	for (i = 0; i < IR6_NUM_SUPPLIES; i++)
		ir6->supplies[i].supply = ir6_supply_names[i];

	return devm_regulator_bulk_get(&ir6->client->dev,
				       IR6_NUM_SUPPLIES,
				       ir6->supplies);
}

static int ir6_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ir6 *ir6;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;
	printk("ir6_probe=======================\n");
	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ir6 = devm_kzalloc(dev, sizeof(*ir6), GFP_KERNEL);
	if (!ir6)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ir6->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ir6->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ir6->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ir6->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}
	ir6->client = client;
	ir6->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < ir6->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			ir6->cur_mode = &supported_modes[i];
			break;
		}
	}

	ir6->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	ir6->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ir6->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ir6->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ir6->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");
	ir6->power_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
	if (IS_ERR(ir6->power_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");
	
	ir6->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ir6->pinctrl)) {
		ir6->pins_default =
			pinctrl_lookup_state(ir6->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ir6->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		ir6->pins_sleep =
			pinctrl_lookup_state(ir6->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ir6->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}

	ret = ir6_configure_regulators(ir6);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ir6->mutex);

	sd = &ir6->subdev;
	v4l2_i2c_subdev_init(sd, client, &ir6_subdev_ops);
	ret = ir6_initialize_controls(ir6);
	if (ret)
		goto err_destroy_mutex;

	ret = __ir6_power_on(ir6);
	if (ret)
		goto err_free_handler;

	ret = ir6_check_sensor_id(ir6, client);
	if (ret)
		goto err_power_off;

	//ret = ir6_get_dcg_ratio(ir6);//os04a10

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ir6_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ir6->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ir6->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ir6->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ir6->module_index, facing,
		 IR6_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ir6_power_off(ir6);
err_free_handler:
	v4l2_ctrl_handler_free(&ir6->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ir6->mutex);

	return ret;
}

static int ir6_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ir6 *ir6 = to_ir6(sd);
	//printk("ir6_remove\n");
	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ir6->ctrl_handler);
	mutex_destroy(&ir6->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ir6_power_off(ir6);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ir6_of_match[] = {
	{ .compatible = "uhdc,ir6" },
	{},
};
MODULE_DEVICE_TABLE(of, ir6_of_match);
#endif

static const struct i2c_device_id ir6_match_id[] = {
	{ "uhdc,ir6", 0 },
	{ },
};

static struct i2c_driver ir6_i2c_driver = {
	.driver = {
		.name = IR6_NAME,//IR6
		.pm = &ir6_pm_ops,
		.of_match_table = of_match_ptr(ir6_of_match),
	},
	.probe		= &ir6_probe,//与dts设备树匹配
	.remove		= &ir6_remove,//取消绑定
	.id_table	= ir6_match_id,//"uhdc,ir6"
};

static int __init sensor_mod_init(void)
{	//printk("sensor_mod_init\n");
	return i2c_add_driver(&ir6_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{	//printk("sensor_mod_exit\n");
	i2c_del_driver(&ir6_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("uhdc ir6 sensor driver");
MODULE_LICENSE("GPL v2");
