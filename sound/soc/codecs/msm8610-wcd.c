// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8610 (MSM8x10) internal audio codec
 *
 * The codec consists of two parts: the analog part with its own small
 * digital core ("Helicon"), which is controlled over I2C at two slave
 * addresses, and the digital part with the decimators and interpolators
 * ("Dino") inside the LPASS, which is memory mapped. They are connected
 * by a PDM interface, while the digital part is connected to the LPASS
 * MI2S interfaces.
 *
 * Both parts are presented as one register map: 0x000-0x0ff and 0x100-0x1ff
 * are the two I2C pages of the analog part, 0x400-0x5ff (stride 4) is the
 * digital part.
 *
 * Based on the downstream msm8x10-wcd driver,
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

/* Analog part, page 0 */
#define CHIP_CTL			0x000
#define CHIP_ID_BYTE_0			0x004
#define CDC_RST_CTL			0x020
#define CDC_TOP_CLK_CTL			0x022
#define CDC_ANA_CLK_CTL			0x023
#define CDC_DIG_CLK_CTL			0x024
#define CDC_CONN_HPHR_DAC_CTL		0x042
#define CDC_CONN_LO_DAC_CTL		0x043

/* Analog part, page 1 */
#define BIAS_CENTRAL_BG_CTL		0x101
#define BIAS_CURR_CTL_2			0x104
#define MICB_CFILT_1_VAL		0x129
#define MICB_1_CTL			0x12b
#define MICB_1_INT_RBIAS		0x12c
#define TX_COM_BIAS			0x14c
#define TX_1_EN				0x153
#define TX_2_EN				0x154
#define TX_1_2_TEST_CTL			0x158
#define TX_3_EN				0x15d
#define CP_EN				0x192
#define CP_STATIC			0x194
#define RX_COM_OCP_COUNT		0x1a0
#define RX_COM_BIAS			0x1a2
#define RX_HPH_BIAS_PA			0x1a6
#define RX_HPH_OCP_CTL			0x1aa
#define RX_HPH_CNP_EN			0x1ab
#define RX_HPH_L_GAIN			0x1ae
#define RX_HPH_L_TEST			0x1af
#define RX_HPH_L_DAC_CTL		0x1b1
#define RX_HPH_R_GAIN			0x1b4
#define RX_HPH_R_TEST			0x1b5
#define RX_HPH_R_DAC_CTL		0x1b7
#define RX_EAR_BIAS_PA			0x1ba
#define RX_EAR_EN			0x1bc
#define RX_EAR_GAIN			0x1bd
#define RX_EAR_CMBUFF			0x1be
#define RX_LINE_BIAS_PA			0x1c6
#define RX_LINE_CNP_EN			0x1ca
#define RX_LINE_1_GAIN			0x1cd
#define RX_LINE_1_DAC_CTL		0x1cf
#define SPKR_DRV_EN			0x1df
#define SPKR_DRV_DAC_CTL		0x1e1

/* Digital part */
#define DIG_BASE			0x400
#define CDC_CLK_RX_RESET_CTL		0x400
#define CDC_CLK_TX_RESET_B1_CTL		0x404
#define CDC_CLK_DMIC_B1_CTL		0x408
#define CDC_CLK_RX_I2S_CTL		0x40c
#define CDC_CLK_TX_I2S_CTL		0x410
#define  I2S_CTL_MODE_16		BIT(5)
#define  I2S_CTL_FS_RATE_MASK		GENMASK(2, 0)
#define CDC_CLK_OTHR_RESET_B1_CTL	0x414
#define CDC_CLK_TX_CLK_EN_B1_CTL	0x418
#define CDC_CLK_OTHR_CTL		0x41c
#define CDC_CLK_RX_B1_CTL		0x420
#define CDC_CLK_MCLK_CTL		0x424
#define CDC_CLK_PDM_CTL			0x428
#define CDC_CLK_SD_CTL			0x42c
#define CDC_RX1_B4_CTL			0x44c
#define CDC_RX1_B5_CTL			0x450
#define CDC_RX1_B6_CTL			0x454
#define CDC_RX1_VOL_CTL_B2_CTL		0x45c
#define CDC_RX2_B4_CTL			0x46c
#define CDC_RX2_B5_CTL			0x470
#define CDC_RX2_B6_CTL			0x474
#define CDC_RX2_VOL_CTL_B2_CTL		0x47c
#define CDC_RX3_B4_CTL			0x48c
#define CDC_RX3_B5_CTL			0x490
#define CDC_RX3_B6_CTL			0x494
#define CDC_RX3_VOL_CTL_B2_CTL		0x49c
#define CDC_CLSG_FREQ_THRESH_B1_CTL	0x4a0
#define CDC_CLSG_FREQ_THRESH_B2_CTL	0x4a4
#define CDC_CLSG_FREQ_THRESH_B3_CTL	0x4a8
#define CDC_CLSG_FREQ_THRESH_B4_CTL	0x4ac
#define CDC_CLSG_GAIN_THRESH_CTL	0x4b0
#define CDC_CLSG_CTL			0x4bc
#define CDC_TX1_VOL_CTL_GAIN		0x4c4
#define CDC_TX1_VOL_CTL_CFG		0x4c8
#define CDC_TX1_MUX_CTL			0x4cc
#define CDC_TX1_DMIC_CTL		0x4d4
#define CDC_TX2_VOL_CTL_GAIN		0x4e4
#define CDC_TX2_MUX_CTL			0x4ec
#define CDC_TX2_DMIC_CTL		0x4f4
#define CDC_IIR1_GAIN_B1_CTL		0x500
#define CDC_IIR1_GAIN_B2_CTL		0x504
#define CDC_IIR1_GAIN_B3_CTL		0x508
#define CDC_IIR1_GAIN_B4_CTL		0x50c
#define CDC_IIR1_CTL			0x520
#define CDC_CONN_RX1_B1_CTL		0x580
#define CDC_CONN_RX1_B2_CTL		0x584
#define CDC_CONN_RX1_B3_CTL		0x588
#define CDC_CONN_RX2_B1_CTL		0x58c
#define CDC_CONN_RX2_B3_CTL		0x594
#define CDC_CONN_RX3_B1_CTL		0x598
#define CDC_CONN_TX_B1_CTL		0x5a4
#define CDC_CONN_EQ1_B1_CTL		0x5a8
#define DIG_MAX_REGISTER		0x5f0

#define MSM8610_WCD_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000)
#define MSM8610_WCD_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

#define MSM8610_WCD_MCLK_RATE	9600000

static const char * const msm8610_wcd_supply_names[] = {
	"vdda-h",
	"vdd-px",
	"vdd-1p2",
};

struct msm8610_wcd {
	struct device *dev;
	struct i2c_client *client[2];
	struct regmap *digital;
	struct regmap *regmap;
	struct clk *mclk;
};

/* Readable registers of the analog part, as documented downstream */
static const struct regmap_range msm8610_wcd_analog_ranges[] = {
	regmap_reg_range(0x000, 0x008),
	regmap_reg_range(0x00c, 0x00c),
	regmap_reg_range(0x010, 0x010),
	regmap_reg_range(0x012, 0x012),
	regmap_reg_range(0x014, 0x014),
	regmap_reg_range(0x018, 0x019),
	regmap_reg_range(0x020, 0x020),
	regmap_reg_range(0x022, 0x024),
	regmap_reg_range(0x030, 0x039),
	regmap_reg_range(0x040, 0x046),
	regmap_reg_range(0x048, 0x049),
	regmap_reg_range(0x050, 0x050),
	regmap_reg_range(0x058, 0x058),
	regmap_reg_range(0x060, 0x060),
	regmap_reg_range(0x070, 0x073),
	regmap_reg_range(0x080, 0x080),
	regmap_reg_range(0x082, 0x082),
	regmap_reg_range(0x084, 0x084),
	regmap_reg_range(0x090, 0x090),
	regmap_reg_range(0x094, 0x096),
	regmap_reg_range(0x098, 0x09a),
	regmap_reg_range(0x09c, 0x09e),
	regmap_reg_range(0x0a4, 0x0a6),
	regmap_reg_range(0x0a8, 0x0aa),
	regmap_reg_range(0x0c0, 0x0df),
	regmap_reg_range(0x100, 0x105),
	regmap_reg_range(0x128, 0x12d),
	regmap_reg_range(0x14a, 0x14c),
	regmap_reg_range(0x14e, 0x15e),
	regmap_reg_range(0x171, 0x171),
	regmap_reg_range(0x175, 0x175),
	regmap_reg_range(0x192, 0x198),
	regmap_reg_range(0x19b, 0x19c),
	regmap_reg_range(0x19e, 0x1a2),
	regmap_reg_range(0x1a4, 0x1c3),
	regmap_reg_range(0x1c5, 0x1d0),
	regmap_reg_range(0x1dd, 0x1dd),
	regmap_reg_range(0x1df, 0x1ec),
	regmap_reg_range(0x1fa, 0x1fe),
};

/* Readable registers of the digital part, at a stride of 4 */
static const struct regmap_range msm8610_wcd_digital_ranges[] = {
	regmap_reg_range(0x400, 0x42c),
	regmap_reg_range(0x440, 0x4d4),
	regmap_reg_range(0x4e0, 0x4f4),
	regmap_reg_range(0x500, 0x52c),
	regmap_reg_range(0x540, 0x56c),
	regmap_reg_range(0x580, 0x59c),
	regmap_reg_range(0x5a4, 0x5c8),
	regmap_reg_range(0x5d0, 0x5d0),
	regmap_reg_range(0x5d8, 0x5d8),
	regmap_reg_range(0x5e0, 0x5f0),
};

static bool msm8610_wcd_readable_reg(struct device *dev, unsigned int reg)
{
	if (reg >= DIG_BASE)
		return !(reg & 3) &&
		       regmap_reg_in_ranges(reg, msm8610_wcd_digital_ranges,
					    ARRAY_SIZE(msm8610_wcd_digital_ranges));

	return regmap_reg_in_ranges(reg, msm8610_wcd_analog_ranges,
				    ARRAY_SIZE(msm8610_wcd_analog_ranges));
}

static int msm8610_wcd_reg_read(void *context, unsigned int reg,
				unsigned int *val)
{
	struct msm8610_wcd *wcd = context;
	int ret;

	if (reg >= DIG_BASE)
		return regmap_read(wcd->digital, reg - DIG_BASE, val);

	ret = i2c_smbus_read_byte_data(wcd->client[reg >> 8], reg & 0xff);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

static int msm8610_wcd_reg_write(void *context, unsigned int reg,
				 unsigned int val)
{
	struct msm8610_wcd *wcd = context;

	if (reg >= DIG_BASE)
		return regmap_write(wcd->digital, reg - DIG_BASE, val);

	return i2c_smbus_write_byte_data(wcd->client[reg >> 8], reg & 0xff,
					 val);
}

static const struct regmap_config msm8610_wcd_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = DIG_MAX_REGISTER,
	.readable_reg = msm8610_wcd_readable_reg,
	.reg_read = msm8610_wcd_reg_read,
	.reg_write = msm8610_wcd_reg_write,
	.cache_type = REGCACHE_NONE,
};

static const DECLARE_TLV_DB_SCALE(digital_gain, -8400, 100, -8400);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);

static const char * const ear_pa_gain_text[] = { "POS_6_DB", "POS_2_DB" };
static const unsigned int ear_pa_gain_values[] = { 0x0, 0x4 };
static SOC_VALUE_ENUM_SINGLE_DECL(ear_pa_gain_enum, RX_EAR_GAIN, 5, 0x7,
				  ear_pa_gain_text, ear_pa_gain_values);

static const char * const cf_text[] = { "4Hz", "75Hz", "150Hz" };
static SOC_ENUM_SINGLE_DECL(tx1_hpf_cutoff_enum, CDC_TX1_MUX_CTL, 4, cf_text);
static SOC_ENUM_SINGLE_DECL(tx2_hpf_cutoff_enum, CDC_TX2_MUX_CTL, 4, cf_text);
static SOC_ENUM_SINGLE_DECL(rx1_dcb_cutoff_enum, CDC_RX1_B4_CTL, 0, cf_text);
static SOC_ENUM_SINGLE_DECL(rx2_dcb_cutoff_enum, CDC_RX2_B4_CTL, 0, cf_text);
static SOC_ENUM_SINGLE_DECL(rx3_dcb_cutoff_enum, CDC_RX3_B4_CTL, 0, cf_text);

static const struct snd_kcontrol_new msm8610_wcd_snd_controls[] = {
	SOC_ENUM("EAR PA Gain", ear_pa_gain_enum),
	SOC_SINGLE_TLV("LINEOUT Volume", RX_LINE_1_GAIN, 0, 12, 1, line_gain),
	SOC_SINGLE_TLV("HPHL Volume", RX_HPH_L_GAIN, 0, 12, 1, line_gain),
	SOC_SINGLE_TLV("HPHR Volume", RX_HPH_R_GAIN, 0, 12, 1, line_gain),

	SOC_SINGLE_S8_TLV("RX1 Digital Volume", CDC_RX1_VOL_CTL_B2_CTL,
			  -84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX2 Digital Volume", CDC_RX2_VOL_CTL_B2_CTL,
			  -84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX3 Digital Volume", CDC_RX3_VOL_CTL_B2_CTL,
			  -84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("TX1 Digital Volume", CDC_TX1_VOL_CTL_GAIN,
			  -84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("TX2 Digital Volume", CDC_TX2_VOL_CTL_GAIN,
			  -84, 40, digital_gain),

	SOC_SINGLE_TLV("ADC1 Volume", TX_1_EN, 2, 19, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", TX_2_EN, 2, 19, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", TX_3_EN, 2, 19, 0, analog_gain),

	SOC_SINGLE_S8_TLV("IIR1 INP1 Volume", CDC_IIR1_GAIN_B1_CTL,
			  -84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP2 Volume", CDC_IIR1_GAIN_B2_CTL,
			  -84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP3 Volume", CDC_IIR1_GAIN_B3_CTL,
			  -84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP4 Volume", CDC_IIR1_GAIN_B4_CTL,
			  -84, 40, digital_gain),
	SOC_SINGLE("IIR1 Band1 Switch", CDC_IIR1_CTL, 0, 1, 0),
	SOC_SINGLE("IIR1 Band2 Switch", CDC_IIR1_CTL, 1, 1, 0),
	SOC_SINGLE("IIR1 Band3 Switch", CDC_IIR1_CTL, 2, 1, 0),
	SOC_SINGLE("IIR1 Band4 Switch", CDC_IIR1_CTL, 3, 1, 0),
	SOC_SINGLE("IIR1 Band5 Switch", CDC_IIR1_CTL, 4, 1, 0),

	SOC_ENUM("TX1 HPF Cutoff", tx1_hpf_cutoff_enum),
	SOC_ENUM("TX2 HPF Cutoff", tx2_hpf_cutoff_enum),

	SOC_ENUM("RX1 DCB Cutoff", rx1_dcb_cutoff_enum),
	SOC_ENUM("RX2 DCB Cutoff", rx2_dcb_cutoff_enum),
	SOC_ENUM("RX3 DCB Cutoff", rx3_dcb_cutoff_enum),
	SOC_SINGLE("RX1 DCB Switch", CDC_RX1_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX2 DCB Switch", CDC_RX2_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX3 DCB Switch", CDC_RX3_B5_CTL, 2, 1, 0),
};

static const char * const rx_mix1_text[] = {
	"ZERO", "IIR1", "IIR2", "RX1", "RX2", "RX3"
};

static const char * const rx_mix2_text[] = { "ZERO", "IIR1", "IIR2" };

static const char * const dec_mux_text[] = {
	"ZERO", "ADC1", "ADC2", "DMIC1", "DMIC2"
};

static const char * const adc2_mux_text[] = { "ZERO", "INP2", "INP3" };

static const char * const iir_inp1_text[] = {
	"ZERO", "DEC1", "DEC2", "RX1", "RX2", "RX3"
};

/*
 * There is only one bit to select RX2 (0) or RX3 (1); "ZERO" keeps the
 * lineout from being powered up along with the headphones.
 */
static const char * const rdac4_text[] = { "ZERO", "RX3", "RX2" };

static const char * const rdac3_text[] = { "RX1", "RX2" };

static SOC_ENUM_SINGLE_DECL(rx1_mix1_inp1_enum, CDC_CONN_RX1_B1_CTL, 0,
			    rx_mix1_text);
static SOC_ENUM_SINGLE_DECL(rx1_mix1_inp2_enum, CDC_CONN_RX1_B1_CTL, 3,
			    rx_mix1_text);
static SOC_ENUM_SINGLE_DECL(rx1_mix1_inp3_enum, CDC_CONN_RX1_B2_CTL, 0,
			    rx_mix1_text);
static SOC_ENUM_SINGLE_DECL(rx2_mix1_inp1_enum, CDC_CONN_RX2_B1_CTL, 0,
			    rx_mix1_text);
static SOC_ENUM_SINGLE_DECL(rx2_mix1_inp2_enum, CDC_CONN_RX2_B1_CTL, 3,
			    rx_mix1_text);
static SOC_ENUM_SINGLE_DECL(rx3_mix1_inp1_enum, CDC_CONN_RX3_B1_CTL, 0,
			    rx_mix1_text);
static SOC_ENUM_SINGLE_DECL(rx3_mix1_inp2_enum, CDC_CONN_RX3_B1_CTL, 3,
			    rx_mix1_text);
static SOC_ENUM_SINGLE_DECL(rx1_mix2_inp1_enum, CDC_CONN_RX1_B3_CTL, 0,
			    rx_mix2_text);
static SOC_ENUM_SINGLE_DECL(rx2_mix2_inp1_enum, CDC_CONN_RX2_B3_CTL, 0,
			    rx_mix2_text);
static SOC_ENUM_SINGLE_DECL(dec1_mux_enum, CDC_CONN_TX_B1_CTL, 0,
			    dec_mux_text);
static SOC_ENUM_SINGLE_DECL(dec2_mux_enum, CDC_CONN_TX_B1_CTL, 3,
			    dec_mux_text);
static SOC_ENUM_SINGLE_DECL(iir1_inp1_mux_enum, CDC_CONN_EQ1_B1_CTL, 0,
			    iir_inp1_text);
static SOC_ENUM_SINGLE_DECL(rdac4_enum, CDC_CONN_LO_DAC_CTL, 0, rdac4_text);
static SOC_ENUM_SINGLE_DECL(rdac3_enum, CDC_CONN_HPHR_DAC_CTL, 0, rdac3_text);
static SOC_ENUM_SINGLE_VIRT_DECL(adc2_enum, adc2_mux_text);

static const struct snd_kcontrol_new rx1_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx1_mix1_inp1_enum);
static const struct snd_kcontrol_new rx1_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP2 Mux", rx1_mix1_inp2_enum);
static const struct snd_kcontrol_new rx1_mix1_inp3_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP3 Mux", rx1_mix1_inp3_enum);
static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_enum);
static const struct snd_kcontrol_new rx2_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP2 Mux", rx2_mix1_inp2_enum);
static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp1_enum);
static const struct snd_kcontrol_new rx3_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP2 Mux", rx3_mix1_inp2_enum);
static const struct snd_kcontrol_new rx1_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX2 INP1 Mux", rx1_mix2_inp1_enum);
static const struct snd_kcontrol_new rx2_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX2 INP1 Mux", rx2_mix2_inp1_enum);
static const struct snd_kcontrol_new iir1_inp1_mux =
	SOC_DAPM_ENUM("IIR1 INP1 Mux", iir1_inp1_mux_enum);
static const struct snd_kcontrol_new rdac4_mux =
	SOC_DAPM_ENUM("RDAC4 MUX Mux", rdac4_enum);
static const struct snd_kcontrol_new rdac3_mux =
	SOC_DAPM_ENUM("RDAC3 MUX Mux", rdac3_enum);
static const struct snd_kcontrol_new adc2_mux =
	SOC_DAPM_ENUM("ADC2 MUX Mux", adc2_enum);

/* The decimator takes PDM data from the analog part or from a DMIC */
static int msm8610_wcd_put_dec_enum(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_dapm_kcontrol_to_component(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int item = ucontrol->value.enumerated.item[0];
	unsigned int tx_mux_ctl_reg;
	bool dmic;

	if (item >= e->items)
		return -EINVAL;

	/* DEC1 is at shift 0, DEC2 at shift 3 */
	tx_mux_ctl_reg = e->shift_l ? CDC_TX2_MUX_CTL : CDC_TX1_MUX_CTL;
	dmic = item == 3 || item == 4;
	snd_soc_component_update_bits(component, tx_mux_ctl_reg, BIT(0), dmic);

	return snd_soc_dapm_put_enum_double(kcontrol, ucontrol);
}

#define MSM8610_WCD_DEC_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_dapm_get_enum_double, \
	.put = msm8610_wcd_put_dec_enum, \
	.private_value = (unsigned long)&xenum }

static const struct snd_kcontrol_new dec1_mux =
	MSM8610_WCD_DEC_ENUM("DEC1 MUX Mux", dec1_mux_enum);
static const struct snd_kcontrol_new dec2_mux =
	MSM8610_WCD_DEC_ENUM("DEC2 MUX Mux", dec2_mux_enum);

static const struct snd_kcontrol_new dac1_switch[] = {
	SOC_DAPM_SINGLE("Switch", RX_EAR_EN, 5, 1, 0)
};

static const struct snd_kcontrol_new hphl_switch[] = {
	SOC_DAPM_SINGLE("Switch", RX_HPH_L_DAC_CTL, 6, 1, 0)
};

static const struct snd_kcontrol_new spkr_switch[] = {
	SOC_DAPM_SINGLE("Switch", SPKR_DRV_DAC_CTL, 2, 1, 0)
};

/* Bandgap reference and the clock block of the analog part */
static int msm8610_wcd_mclk_event(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Bandgap in slow mode, with precharge */
		snd_soc_component_update_bits(component, BIAS_CENTRAL_BG_CTL,
					      0x80, 0x80);
		snd_soc_component_update_bits(component, BIAS_CENTRAL_BG_CTL,
					      0x04, 0x04);
		snd_soc_component_update_bits(component, BIAS_CENTRAL_BG_CTL,
					      0x01, 0x01);
		usleep_range(1000, 1100);
		snd_soc_component_update_bits(component, BIAS_CENTRAL_BG_CTL,
					      0x80, 0x00);

		/* Clock the analog part from MCLK over the PDM interface */
		snd_soc_component_update_bits(component, CDC_CLK_PDM_CTL,
					      0x03, 0x03);
		snd_soc_component_update_bits(component, CDC_TOP_CLK_CTL,
					      0x0f, 0x0d);
		snd_soc_component_update_bits(component, CDC_CLK_MCLK_CTL,
					      0x01, 0x01);
		usleep_range(50, 100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, CDC_CLK_PDM_CTL,
					      0x03, 0x00);
		snd_soc_component_update_bits(component, BIAS_CENTRAL_BG_CTL,
					      0x03, 0x00);
		usleep_range(100, 200);
		break;
	}

	return 0;
}

static int msm8610_wcd_cp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Charge pump clock */
		snd_soc_component_update_bits(component, CDC_CLK_OTHR_CTL,
					      0x01, 0x01);
		snd_soc_component_update_bits(component, CDC_CLSG_CTL,
					      0x08, 0x08);
		usleep_range(200, 300);
		snd_soc_component_update_bits(component, CP_STATIC, 0x10, 0x00);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component,
					      CDC_CLK_OTHR_RESET_B1_CTL,
					      0x01, 0x01);
		usleep_range(20, 100);
		snd_soc_component_update_bits(component, CP_STATIC, 0x08, 0x08);
		snd_soc_component_update_bits(component, CP_STATIC, 0x10, 0x10);
		snd_soc_component_update_bits(component, CDC_CLSG_CTL,
					      0x08, 0x00);
		snd_soc_component_update_bits(component, CDC_CLK_OTHR_CTL,
					      0x01, 0x00);
		snd_soc_component_update_bits(component,
					      CDC_CLK_OTHR_RESET_B1_CTL,
					      0x01, 0x00);
		snd_soc_component_update_bits(component, CP_STATIC, 0x08, 0x00);
		break;
	}

	return 0;
}

static int msm8610_wcd_adc_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	unsigned int init_bit = w->reg == TX_1_EN ? BIT(7) : BIT(6);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, CDC_ANA_CLK_CTL,
					      0x20, 0x20);
		snd_soc_component_update_bits(component, TX_1_2_TEST_CTL,
					      init_bit, init_bit);
		usleep_range(1000, 1100);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, TX_1_2_TEST_CTL,
					      init_bit, 0);
		usleep_range(1000, 1100);
		break;
	}

	return 0;
}

static int msm8610_wcd_micbias_event(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	unsigned int rbias = 0;

	if (strstr(w->name, "Internal1"))
		rbias = 0x80;
	else if (strstr(w->name, "Internal2"))
		rbias = 0x10;
	else if (strstr(w->name, "Internal3"))
		rbias = 0x02;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Always pull up the TX2 front end to the micbias */
		snd_soc_component_update_bits(component, MICB_1_INT_RBIAS,
					      rbias | 0x24, rbias | 0x24);
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(20000, 20100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, MICB_1_INT_RBIAS,
					      rbias | 0x24, 0);
		break;
	}

	return 0;
}

static int msm8610_wcd_dec_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	unsigned int tx_vol_ctl_reg = CDC_TX1_VOL_CTL_CFG + 32 * w->shift;
	unsigned int tx_mux_ctl_reg = CDC_TX1_MUX_CTL + 32 * w->shift;
	unsigned int tx_gain_reg = CDC_TX1_VOL_CTL_GAIN + 32 * w->shift;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Mute while the decimator settles */
		snd_soc_component_update_bits(component, tx_vol_ctl_reg,
					      0x01, 0x01);
		snd_soc_component_update_bits(component,
					      CDC_CLK_TX_RESET_B1_CTL,
					      BIT(w->shift), BIT(w->shift));
		snd_soc_component_update_bits(component,
					      CDC_CLK_TX_RESET_B1_CTL,
					      BIT(w->shift), 0);
		/* Enable the high pass filter */
		snd_soc_component_update_bits(component, tx_mux_ctl_reg,
					      0x08, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, tx_vol_ctl_reg,
					      0x01, 0x00);
		/* The gain only takes effect once the decimator runs */
		snd_soc_component_write(component, tx_gain_reg,
				snd_soc_component_read(component, tx_gain_reg));
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, tx_vol_ctl_reg,
					      0x01, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, tx_mux_ctl_reg,
					      0x08, 0x08);
		break;
	}

	return 0;
}

static int msm8610_wcd_interpolator_event(struct snd_soc_dapm_widget *w,
					  struct snd_kcontrol *kcontrol,
					  int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	unsigned int rx_gain_reg = CDC_RX1_VOL_CTL_B2_CTL + 32 * w->shift;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, CDC_CLK_RX_RESET_CTL,
					      BIT(w->shift), BIT(w->shift));
		snd_soc_component_update_bits(component, CDC_CLK_RX_RESET_CTL,
					      BIT(w->shift), 0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* The gain only takes effect once the interpolator runs */
		snd_soc_component_write(component, rx_gain_reg,
				snd_soc_component_read(component, rx_gain_reg));
		break;
	}

	return 0;
}

static int msm8610_wcd_dac_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, w->reg, 0x40, 0x40);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, w->reg, 0x40, 0x00);
		break;
	}

	return 0;
}

static int msm8610_wcd_hph_pa_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	case SND_SOC_DAPM_POST_PMD:
		usleep_range(10000, 10100);
		break;
	}

	return 0;
}

static int msm8610_wcd_lineout_pa_event(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, RX_LINE_1_GAIN,
					      0x40, 0x40);
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(16000, 16100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RX_LINE_1_GAIN,
					      0x40, 0x00);
		break;
	}

	return 0;
}

static int msm8610_wcd_ear_pa_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	case SND_SOC_DAPM_POST_PMD:
		msleep(20);
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget msm8610_wcd_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("MCLK", SND_SOC_NOPM, 0, 0, msm8610_wcd_mclk_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("RX_I2S_CLK", CDC_CLK_RX_I2S_CTL, 4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("TX_I2S_CLK", CDC_CLK_TX_I2S_CTL, 4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("CDC_CONN", CDC_CLK_OTHR_CTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX_BIAS", RX_COM_BIAS, 7, 0, NULL, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("vdd-cp", 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("vdd-micbias", 0, 0),
	SND_SOC_DAPM_SUPPLY("CP", CP_EN, 0, 0, msm8610_wcd_cp_event,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* RX */
	SND_SOC_DAPM_AIF_IN("I2S RX1", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S RX2", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S RX3", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
			 &rx1_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
			 &rx1_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP3", SND_SOC_NOPM, 0, 0,
			 &rx1_mix1_inp3_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", SND_SOC_NOPM, 0, 0,
			 &rx2_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP2", SND_SOC_NOPM, 0, 0,
			 &rx2_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP1", SND_SOC_NOPM, 0, 0,
			 &rx3_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP2", SND_SOC_NOPM, 0, 0,
			 &rx3_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX2 INP1", SND_SOC_NOPM, 0, 0,
			 &rx1_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX2 INP1", SND_SOC_NOPM, 0, 0,
			 &rx2_mix2_inp1_mux),

	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER_E("RX1 MIX2", CDC_CLK_RX_B1_CTL, 0, 0, NULL, 0,
			     msm8610_wcd_interpolator_event,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX2 MIX2", CDC_CLK_RX_B1_CTL, 1, 0, NULL, 0,
			     msm8610_wcd_interpolator_event,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX3 MIX1", CDC_CLK_RX_B1_CTL, 2, 0, NULL, 0,
			     msm8610_wcd_interpolator_event,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_SUPPLY("RX1 CLK", CDC_DIG_CLK_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX2 CLK", CDC_DIG_CLK_CTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("RX3 CLK", CDC_DIG_CLK_CTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX1 CHAIN", CDC_RX1_B6_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 CHAIN", CDC_RX2_B6_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX3 CHAIN", CDC_RX3_B6_CTL, 5, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RDAC3 MUX", SND_SOC_NOPM, 0, 0, &rdac3_mux),
	SND_SOC_DAPM_MUX("RDAC4 MUX", SND_SOC_NOPM, 0, 0, &rdac4_mux),

	SND_SOC_DAPM_SUPPLY("HPHR CLK", CDC_ANA_CLK_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("HPHL CLK", CDC_ANA_CLK_CTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("EAR CLK", CDC_ANA_CLK_CTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("LINEOUT CLK", CDC_ANA_CLK_CTL, 3, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("SPK CLK", CDC_ANA_CLK_CTL, 4, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("DAC1", RX_EAR_EN, 6, 0, dac1_switch,
			   ARRAY_SIZE(dac1_switch)),
	SND_SOC_DAPM_PGA_E("EAR PA", RX_EAR_EN, 4, 0, NULL, 0,
			   msm8610_wcd_ear_pa_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("HPHL DAC", RX_HPH_L_DAC_CTL, 7, 0, hphl_switch,
			   ARRAY_SIZE(hphl_switch)),
	SND_SOC_DAPM_DAC_E("HPHR DAC", NULL, RX_HPH_R_DAC_CTL, 7, 0,
			   msm8610_wcd_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHL", RX_HPH_CNP_EN, 5, 0, NULL, 0,
			   msm8610_wcd_hph_pa_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHR", RX_HPH_CNP_EN, 4, 0, NULL, 0,
			   msm8610_wcd_hph_pa_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("LINEOUT DAC", NULL, RX_LINE_1_DAC_CTL, 7, 0,
			   msm8610_wcd_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT PA", RX_LINE_CNP_EN, 0, 0, NULL, 0,
			   msm8610_wcd_lineout_pa_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("SPK DAC", SND_SOC_NOPM, 0, 0, spkr_switch,
			   ARRAY_SIZE(spkr_switch)),
	SND_SOC_DAPM_PGA("SPK PA", SPKR_DRV_EN, 7, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("EAR"),
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_OUTPUT("LINEOUT"),
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),

	/* TX */
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_INPUT("AMIC3"),

	SND_SOC_DAPM_SUPPLY("INT_LDO_H", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal1", MICB_1_CTL, 7, 0,
			    msm8610_wcd_micbias_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal2", MICB_1_CTL, 7, 0,
			    msm8610_wcd_micbias_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS Internal3", MICB_1_CTL, 7, 0,
			    msm8610_wcd_micbias_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS External", MICB_1_CTL, 7, 0,
			    msm8610_wcd_micbias_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("ADC1", NULL, TX_1_EN, 7, 0, msm8610_wcd_adc_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_ADC_E("ADC2_INP2", NULL, TX_2_EN, 7, 0,
			   msm8610_wcd_adc_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_ADC_E("ADC2_INP3", NULL, TX_3_EN, 7, 0,
			   msm8610_wcd_adc_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER("ADC2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX("ADC2 MUX", SND_SOC_NOPM, 0, 0, &adc2_mux),

	SND_SOC_DAPM_ADC("DMIC1", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("DMIC2", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("DMIC_CLK", CDC_CLK_DMIC_B1_CTL, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX_E("DEC1 MUX", CDC_CLK_TX_CLK_EN_B1_CTL, 0, 0,
			   &dec1_mux, msm8610_wcd_dec_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("DEC2 MUX", CDC_CLK_TX_CLK_EN_B1_CTL, 1, 0,
			   &dec2_mux, msm8610_wcd_dec_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT("I2S TX1", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S TX2", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),

	/* Sidetone */
	SND_SOC_DAPM_MUX("IIR1 INP1 MUX", SND_SOC_NOPM, 0, 0, &iir1_inp1_mux),
	SND_SOC_DAPM_PGA("IIR1", CDC_CLK_SD_CTL, 0, 0, NULL, 0),
};

static const struct snd_soc_dapm_route msm8610_wcd_audio_map[] = {
	{ "RX_I2S_CLK", NULL, "MCLK" },
	{ "TX_I2S_CLK", NULL, "MCLK" },
	{ "RX_BIAS", NULL, "MCLK" },
	{ "INT_LDO_H", NULL, "MCLK" },
	{ "RX_I2S_CLK", NULL, "CDC_CONN" },

	{ "I2S RX1", NULL, "RX_I2S_CLK" },
	{ "I2S RX2", NULL, "RX_I2S_CLK" },
	{ "I2S RX3", NULL, "RX_I2S_CLK" },
	{ "I2S TX1", NULL, "TX_I2S_CLK" },
	{ "I2S TX2", NULL, "TX_I2S_CLK" },
	{ "I2S TX1", NULL, "DEC1 MUX" },
	{ "I2S TX2", NULL, "DEC2 MUX" },

	/* Earpiece */
	{ "EAR", NULL, "EAR PA" },
	{ "EAR PA", NULL, "DAC1" },
	{ "DAC1", NULL, "CP" },
	{ "DAC1", NULL, "EAR CLK" },
	{ "DAC1", "Switch", "RX1 CHAIN" },

	/* Headphones */
	{ "HEADPHONE", NULL, "HPHL" },
	{ "HEADPHONE", NULL, "HPHR" },
	{ "HPHL", NULL, "HPHL DAC" },
	{ "HPHR", NULL, "HPHR DAC" },
	{ "HPHL DAC", NULL, "CP" },
	{ "HPHR DAC", NULL, "CP" },
	{ "HPHL DAC", NULL, "HPHL CLK" },
	{ "HPHR DAC", NULL, "HPHR CLK" },
	{ "HPHL DAC", "Switch", "RX1 CHAIN" },
	{ "HPHR DAC", NULL, "RDAC3 MUX" },
	{ "RDAC3 MUX", "RX1", "RX1 CHAIN" },
	{ "RDAC3 MUX", "RX2", "RX2 CHAIN" },

	/* Lineout */
	{ "LINEOUT", NULL, "LINEOUT PA" },
	{ "LINEOUT PA", NULL, "CP" },
	{ "LINEOUT PA", NULL, "LINEOUT DAC" },
	{ "LINEOUT DAC", NULL, "LINEOUT CLK" },
	{ "LINEOUT DAC", NULL, "RDAC4 MUX" },
	{ "RDAC4 MUX", "RX2", "RX2 CHAIN" },
	{ "RDAC4 MUX", "RX3", "RX3 CHAIN" },

	/* Speaker driver */
	{ "SPK_OUT", NULL, "SPK PA" },
	{ "SPK PA", NULL, "SPK DAC" },
	{ "SPK DAC", NULL, "CP" },
	{ "SPK DAC", NULL, "SPK CLK" },
	{ "SPK DAC", "Switch", "RX3 CHAIN" },

	{ "CP", NULL, "vdd-cp" },
	{ "CP", NULL, "RX_BIAS" },

	{ "RX1 CHAIN", NULL, "RX1 CLK" },
	{ "RX2 CHAIN", NULL, "RX2 CLK" },
	{ "RX3 CHAIN", NULL, "RX3 CLK" },
	{ "RX1 CHAIN", NULL, "RX1 MIX2" },
	{ "RX2 CHAIN", NULL, "RX2 MIX2" },
	{ "RX3 CHAIN", NULL, "RX3 MIX1" },

	{ "RX1 MIX1", NULL, "RX1 MIX1 INP1" },
	{ "RX1 MIX1", NULL, "RX1 MIX1 INP2" },
	{ "RX1 MIX1", NULL, "RX1 MIX1 INP3" },
	{ "RX2 MIX1", NULL, "RX2 MIX1 INP1" },
	{ "RX2 MIX1", NULL, "RX2 MIX1 INP2" },
	{ "RX3 MIX1", NULL, "RX3 MIX1 INP1" },
	{ "RX3 MIX1", NULL, "RX3 MIX1 INP2" },
	{ "RX1 MIX2", NULL, "RX1 MIX1" },
	{ "RX1 MIX2", NULL, "RX1 MIX2 INP1" },
	{ "RX2 MIX2", NULL, "RX2 MIX1" },
	{ "RX2 MIX2", NULL, "RX2 MIX2 INP1" },

	{ "RX1 MIX1 INP1", "RX1", "I2S RX1" },
	{ "RX1 MIX1 INP1", "RX2", "I2S RX2" },
	{ "RX1 MIX1 INP1", "RX3", "I2S RX3" },
	{ "RX1 MIX1 INP1", "IIR1", "IIR1" },
	{ "RX1 MIX1 INP2", "RX1", "I2S RX1" },
	{ "RX1 MIX1 INP2", "RX2", "I2S RX2" },
	{ "RX1 MIX1 INP2", "RX3", "I2S RX3" },
	{ "RX1 MIX1 INP2", "IIR1", "IIR1" },
	{ "RX1 MIX1 INP3", "RX1", "I2S RX1" },
	{ "RX1 MIX1 INP3", "RX2", "I2S RX2" },
	{ "RX1 MIX1 INP3", "RX3", "I2S RX3" },

	{ "RX2 MIX1 INP1", "RX1", "I2S RX1" },
	{ "RX2 MIX1 INP1", "RX2", "I2S RX2" },
	{ "RX2 MIX1 INP1", "RX3", "I2S RX3" },
	{ "RX2 MIX1 INP1", "IIR1", "IIR1" },
	{ "RX2 MIX1 INP2", "RX1", "I2S RX1" },
	{ "RX2 MIX1 INP2", "RX2", "I2S RX2" },
	{ "RX2 MIX1 INP2", "RX3", "I2S RX3" },
	{ "RX2 MIX1 INP2", "IIR1", "IIR1" },

	{ "RX3 MIX1 INP1", "RX1", "I2S RX1" },
	{ "RX3 MIX1 INP1", "RX2", "I2S RX2" },
	{ "RX3 MIX1 INP1", "RX3", "I2S RX3" },
	{ "RX3 MIX1 INP1", "IIR1", "IIR1" },
	{ "RX3 MIX1 INP2", "RX1", "I2S RX1" },
	{ "RX3 MIX1 INP2", "RX2", "I2S RX2" },
	{ "RX3 MIX1 INP2", "RX3", "I2S RX3" },
	{ "RX3 MIX1 INP2", "IIR1", "IIR1" },

	{ "RX1 MIX2 INP1", "IIR1", "IIR1" },
	{ "RX2 MIX2 INP1", "IIR1", "IIR1" },

	/* Decimators */
	{ "DEC1 MUX", "ADC1", "ADC1" },
	{ "DEC1 MUX", "ADC2", "ADC2" },
	{ "DEC1 MUX", "DMIC1", "DMIC1" },
	{ "DEC1 MUX", "DMIC2", "DMIC2" },
	{ "DEC1 MUX", NULL, "CDC_CONN" },
	{ "DEC2 MUX", "ADC1", "ADC1" },
	{ "DEC2 MUX", "ADC2", "ADC2" },
	{ "DEC2 MUX", "DMIC1", "DMIC1" },
	{ "DEC2 MUX", "DMIC2", "DMIC2" },
	{ "DEC2 MUX", NULL, "CDC_CONN" },

	{ "DMIC1", NULL, "DMIC_CLK" },
	{ "DMIC2", NULL, "DMIC_CLK" },

	{ "ADC2", NULL, "ADC2 MUX" },
	{ "ADC2 MUX", "INP2", "ADC2_INP2" },
	{ "ADC2 MUX", "INP3", "ADC2_INP3" },

	{ "ADC1", NULL, "AMIC1" },
	{ "ADC2_INP2", NULL, "AMIC2" },
	{ "ADC2_INP3", NULL, "AMIC3" },

	{ "MIC BIAS Internal1", NULL, "INT_LDO_H" },
	{ "MIC BIAS Internal2", NULL, "INT_LDO_H" },
	{ "MIC BIAS Internal3", NULL, "INT_LDO_H" },
	{ "MIC BIAS External", NULL, "INT_LDO_H" },
	{ "MIC BIAS Internal1", NULL, "vdd-micbias" },
	{ "MIC BIAS Internal2", NULL, "vdd-micbias" },
	{ "MIC BIAS Internal3", NULL, "vdd-micbias" },
	{ "MIC BIAS External", NULL, "vdd-micbias" },

	/* Sidetone */
	{ "IIR1", NULL, "IIR1 INP1 MUX" },
	{ "IIR1 INP1 MUX", "DEC1", "DEC1 MUX" },
	{ "IIR1 INP1 MUX", "DEC2", "DEC2 MUX" },
};

static int msm8610_wcd_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	unsigned int reg, rate, mode;

	switch (params_rate(params)) {
	case 8000:
		rate = 0;
		break;
	case 16000:
		rate = 1;
		break;
	case 32000:
		rate = 2;
		break;
	case 48000:
		rate = 3;
		break;
	default:
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		mode = I2S_CTL_MODE_16;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		mode = 0;
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = CDC_CLK_RX_I2S_CTL;
	else
		reg = CDC_CLK_TX_I2S_CTL;

	snd_soc_component_update_bits(component, reg,
				      I2S_CTL_MODE_16 | I2S_CTL_FS_RATE_MASK,
				      mode | rate);

	return 0;
}

static const struct snd_soc_dai_ops msm8610_wcd_dai_ops = {
	.hw_params = msm8610_wcd_hw_params,
};

static struct snd_soc_dai_driver msm8610_wcd_dai[] = {
	{
		.name = "msm8610_wcd_i2s_rx1",
		.id = 0,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = MSM8610_WCD_RATES,
			.formats = MSM8610_WCD_FORMATS,
			.channels_min = 1,
			.channels_max = 3,
		},
		.ops = &msm8610_wcd_dai_ops,
	},
	{
		.name = "msm8610_wcd_i2s_tx1",
		.id = 1,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = MSM8610_WCD_RATES,
			.formats = MSM8610_WCD_FORMATS,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &msm8610_wcd_dai_ops,
	},
};

static const struct reg_sequence msm8610_wcd_reg_init[] = {
	/* 9.6 MHz MCLK */
	{ CHIP_CTL, 0x00 },
	{ RX_EAR_CMBUFF, 0x05 },
	{ CDC_RX1_B5_CTL, 0x78 },
	{ CDC_RX2_B5_CTL, 0x78 },
	{ CDC_RX3_B5_CTL, 0x78 },
	{ CDC_RX1_B6_CTL, 0xa0 },
	{ CDC_RX2_B6_CTL, 0xa0 },
	{ CDC_RX3_B6_CTL, 0x80 },
	/* Reduce the HPH, EAR and LINE DAC bias to 70% */
	{ RX_HPH_BIAS_PA, 0x7a },
	{ RX_EAR_BIAS_PA, 0x76 },
	{ RX_LINE_BIAS_PA, 0x78 },
	/* Disable the internal biasing path, which can leak */
	{ BIAS_CURR_CTL_2, 0x04 },
	/* Pull down to reduce leakage */
	{ MICB_1_CTL, 0x82 },
	{ TX_COM_BIAS, 0xe0 },
	{ TX_1_EN, 0x32 },
	{ TX_2_EN, 0x32 },
	{ TX_3_EN, 0x30 },
	/* Class G tuning for 16 ohm headphones */
	{ CDC_CLSG_FREQ_THRESH_B1_CTL, 0x05 },
	{ CDC_CLSG_FREQ_THRESH_B2_CTL, 0x0c },
	{ CDC_CLSG_FREQ_THRESH_B3_CTL, 0x1a },
	{ CDC_CLSG_FREQ_THRESH_B4_CTL, 0x47 },
	{ CDC_CLSG_GAIN_THRESH_CTL, 0x23 },
	/* Keep TXD_CLK_EN set to reduce leakage */
	{ CDC_DIG_CLK_CTL, 0x10 },
	/* Do not gate MCLK to the MBHC */
	{ CDC_ANA_CLK_CTL, 0x20 },
};

static int msm8610_wcd_component_probe(struct snd_soc_component *component)
{
	struct msm8610_wcd *wcd = snd_soc_component_get_drvdata(component);
	int ret;

	/* Reset the codec core */
	snd_soc_component_write(component, CDC_RST_CTL, 0x02);
	snd_soc_component_write(component, CHIP_CTL, 0x00);
	usleep_range(5000, 5100);
	snd_soc_component_write(component, CDC_RST_CTL, 0x03);

	/* 350 mA current limit, 4096 wait and run cycles */
	snd_soc_component_update_bits(component, RX_HPH_OCP_CTL, 0xe1, 0x61);
	snd_soc_component_update_bits(component, RX_COM_OCP_COUNT, 0xff, 0xff);
	snd_soc_component_update_bits(component, RX_HPH_L_TEST, 0x01, 0x01);
	snd_soc_component_update_bits(component, RX_HPH_R_TEST, 0x01, 0x01);
	/* Use the gain from the gain registers */
	snd_soc_component_update_bits(component, RX_HPH_L_GAIN, 0x20, 0x20);
	snd_soc_component_update_bits(component, RX_HPH_R_GAIN, 0x20, 0x20);
	snd_soc_component_update_bits(component, RX_LINE_1_GAIN, 0x20, 0x20);
	snd_soc_component_update_bits(component, CDC_TX1_MUX_CTL, 0x08, 0x00);
	snd_soc_component_update_bits(component, CDC_TX2_MUX_CTL, 0x08, 0x00);
	/* DMIC clock: 3.2 MHz from the 9.6 MHz MCLK */
	snd_soc_component_update_bits(component, CDC_TX1_DMIC_CTL, 0x07, 0x01);
	snd_soc_component_update_bits(component, CDC_TX2_DMIC_CTL, 0x07, 0x01);
	snd_soc_component_update_bits(component, CDC_CLK_DMIC_B1_CTL,
				      0xee, 0x22);
	/* Disable REF_EN of the speaker driver DAC */
	snd_soc_component_update_bits(component, SPKR_DRV_DAC_CTL, 0x04, 0x00);

	ret = regmap_multi_reg_write(wcd->regmap, msm8610_wcd_reg_init,
				     ARRAY_SIZE(msm8610_wcd_reg_init));
	if (ret)
		return ret;

	/* Micbias CFILT voltage: 2.7 V with the LDOH at 3.0 V */
	snd_soc_component_update_bits(component, MICB_CFILT_1_VAL, 0xfc,
				      38 << 2);

	return 0;
}

static const struct snd_soc_component_driver msm8610_wcd_component = {
	.probe = msm8610_wcd_component_probe,
	.controls = msm8610_wcd_snd_controls,
	.num_controls = ARRAY_SIZE(msm8610_wcd_snd_controls),
	.dapm_widgets = msm8610_wcd_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(msm8610_wcd_dapm_widgets),
	.dapm_routes = msm8610_wcd_audio_map,
	.num_dapm_routes = ARRAY_SIZE(msm8610_wcd_audio_map),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
};

static void msm8610_wcd_unregister_page(void *data)
{
	i2c_unregister_device(data);
}

static int msm8610_wcd_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct msm8610_wcd *wcd;
	unsigned int id;
	int ret;

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return -ENOMEM;

	wcd->dev = dev;
	wcd->client[0] = client;
	wcd->client[1] = i2c_new_ancillary_device(client, "analog", 0x77);
	if (IS_ERR(wcd->client[1]))
		return dev_err_probe(dev, PTR_ERR(wcd->client[1]),
				     "failed to register analog page\n");

	ret = devm_add_action_or_reset(dev, msm8610_wcd_unregister_page,
				       wcd->client[1]);
	if (ret)
		return ret;

	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, "ahb"));
	if (ret)
		return dev_err_probe(dev, ret, "failed to get ahb clock\n");

	wcd->digital = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "qcom,digital-codec");
	if (IS_ERR(wcd->digital))
		return dev_err_probe(dev, PTR_ERR(wcd->digital),
				     "failed to get digital codec\n");

	ret = devm_regulator_bulk_get_enable(dev,
					     ARRAY_SIZE(msm8610_wcd_supply_names),
					     msm8610_wcd_supply_names);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	wcd->mclk = devm_clk_get_enabled(dev, "mclk");
	if (IS_ERR(wcd->mclk))
		return dev_err_probe(dev, PTR_ERR(wcd->mclk),
				     "failed to get mclk\n");

	ret = clk_set_rate(wcd->mclk, MSM8610_WCD_MCLK_RATE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set mclk rate\n");

	wcd->regmap = devm_regmap_init(dev, NULL, wcd,
				       &msm8610_wcd_regmap_config);
	if (IS_ERR(wcd->regmap))
		return PTR_ERR(wcd->regmap);

	ret = regmap_read(wcd->regmap, CHIP_ID_BYTE_0 + 2, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip id\n");
	dev_dbg(dev, "chip id byte 2: %#x\n", id);

	dev_set_drvdata(dev, wcd);

	return devm_snd_soc_register_component(dev, &msm8610_wcd_component,
					       msm8610_wcd_dai,
					       ARRAY_SIZE(msm8610_wcd_dai));
}

static const struct of_device_id msm8610_wcd_of_match[] = {
	{ .compatible = "qcom,msm8610-wcd" },
	{ }
};
MODULE_DEVICE_TABLE(of, msm8610_wcd_of_match);

static struct i2c_driver msm8610_wcd_driver = {
	.driver = {
		.name = "msm8610-wcd",
		.of_match_table = msm8610_wcd_of_match,
	},
	.probe = msm8610_wcd_probe,
};
module_i2c_driver(msm8610_wcd_driver);

MODULE_DESCRIPTION("Qualcomm MSM8610 internal audio codec driver");
MODULE_LICENSE("GPL");
