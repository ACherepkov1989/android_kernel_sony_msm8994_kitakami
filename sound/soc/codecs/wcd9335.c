/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/wcd9xxx_registers.h>
#include <linux/mfd/wcd9335/registers.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "wcd9335.h"

#define TASHA_RX_PORT_START_NUMBER  16

#define WCD9335_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
		       SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
		       SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define TASHA_FORMATS_S16_S24_LE (SNDRV_PCM_FMTBIT_S16_LE | \
				  SNDRV_PCM_FORMAT_S24_LE)

#define TASHA_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

#define TASHA_SLIM_CLOSE_TIMEOUT 1000
#define TASHA_SLIM_IRQ_OVERFLOW (1 << 0)
#define TASHA_SLIM_IRQ_UNDERFLOW (1 << 1)
#define TASHA_SLIM_IRQ_PORT_CLOSED (1 << 2)
#define TASHA_MCLK_CLK_12P288MHZ 12288000
#define TASHA_MCLK_CLK_9P6MHZ 9600000

#define TASHA_SLIM_PGD_PORT_INT_TX_EN0 (TASHA_SLIM_PGD_PORT_INT_EN0 + 2)

#define TASHA_NUM_INTERPOLATORS 9

enum {
	AIF1_PB = 0,
	AIF1_CAP,
	AIF2_PB,
	AIF2_CAP,
	AIF3_PB,
	AIF3_CAP,
	NUM_CODEC_DAIS,
};

enum {
	INTn_1_MIX_INP_SEL_ZERO = 0,
	INTn_1_MIX_INP_SEL_DEC0,
	INTn_1_MIX_INP_SEL_DEC1,
	INTn_1_MIX_INP_SEL_IIR0,
	INTn_1_MIX_INP_SEL_IIR1,
	INTn_1_MIX_INP_SEL_RX0,
	INTn_1_MIX_INP_SEL_RX1,
	INTn_1_MIX_INP_SEL_RX2,
	INTn_1_MIX_INP_SEL_RX3,
	INTn_1_MIX_INP_SEL_RX4,
	INTn_1_MIX_INP_SEL_RX5,
	INTn_1_MIX_INP_SEL_RX6,
	INTn_1_MIX_INP_SEL_RX7,

};

static const struct wcd9xxx_ch tasha_rx_chs[TASHA_RX_MAX] = {
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER, 0),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 1, 1),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 2, 2),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 3, 3),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 4, 4),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 5, 5),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 6, 6),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 7, 7),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 8, 8),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 9, 9),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 10, 10),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 11, 11),
	WCD9XXX_CH(TASHA_RX_PORT_START_NUMBER + 12, 12),
};

static const struct wcd9xxx_ch tasha_tx_chs[TASHA_TX_MAX] = {
	WCD9XXX_CH(0, 0),
	WCD9XXX_CH(1, 1),
	WCD9XXX_CH(2, 2),
	WCD9XXX_CH(3, 3),
	WCD9XXX_CH(4, 4),
	WCD9XXX_CH(5, 5),
	WCD9XXX_CH(6, 6),
	WCD9XXX_CH(7, 7),
	WCD9XXX_CH(8, 8),
	WCD9XXX_CH(9, 9),
	WCD9XXX_CH(10, 10),
	WCD9XXX_CH(11, 11),
	WCD9XXX_CH(12, 12),
	WCD9XXX_CH(13, 13),
	WCD9XXX_CH(14, 14),
	WCD9XXX_CH(15, 15),
};

static const u32 vport_check_table[NUM_CODEC_DAIS] = {
	0,					/* AIF1_PB */
	(1 << AIF2_CAP) | (1 << AIF3_CAP),	/* AIF1_CAP */
	0,					/* AIF2_PB */
	(1 << AIF1_CAP) | (1 << AIF3_CAP),	/* AIF2_CAP */
	0,					/* AIF3_PB */
	(1 << AIF1_CAP) | (1 << AIF2_CAP),	/* AIF3_CAP */
};


/* Codec supports 2 IIR filters */
enum {
	IIR0 = 0,
	IIR1,
	IIR_MAX,
};

/* Each IIR has 5 Filter Stages */
enum {
	BAND1 = 0,
	BAND2,
	BAND3,
	BAND4,
	BAND5,
	BAND_MAX,
};

enum {
	COMPANDER_1, /* HPH_L */
	COMPANDER_2, /* HPH_R */
	COMPANDER_3, /* LO1_DIFF */
	COMPANDER_4, /* LO2_DIFF */
	COMPANDER_5, /* LO3_SE */
	COMPANDER_6, /* LO4_SE */
	COMPANDER_7, /* SWR SPK CH1 */
	COMPANDER_8, /* SWR SPK CH2 */
	COMPANDER_MAX,
};

static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 1, 0);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);

static struct snd_soc_dai_driver tasha_dai[];

/* Hold instance to soundwire platform device */
struct tasha_swr_ctrl_data {
	struct platform_device *swr_pdev;
	struct ida swr_ida;
};

struct wcd_swr_ctrl_platform_data {
	void *handle; /* holds codec private data */
	int (*read)(void *handle, int reg);
	int (*write)(void *handle, int reg, int val);
	int (*clk)(void *handle, bool enable);
	int (*handle_irq)(void *handle,
			  irqreturn_t (*swrm_irq_handler)(int irq,
							  void *data),
			  void *swrm_handle,
			  int action);
};

struct tasha_priv {
	struct device *dev;
	struct wcd9xxx *wcd9xxx;

	struct snd_soc_codec *codec;
	u32 adc_count;
	u32 rx_bias_count;
	s32 dmic_1_2_clk_cnt;
	s32 dmic_3_4_clk_cnt;
	s32 dmic_5_6_clk_cnt;
	s32 ldo_h_users;
	s32 micb_2_users;

	u32 anc_slot;
	bool anc_func;

	/*track tasha interface type*/
	u8 intf_type;

	/* num of slim ports required */
	struct wcd9xxx_codec_dai_data  dai[NUM_CODEC_DAIS];

	/* SoundWire data structure */
	struct tasha_swr_ctrl_data *swr_ctrl_data;
	int nr;

	/*compander*/
	int comp_enabled[COMPANDER_MAX];

	/* Maintain the status of AUX PGA */
	int aux_pga_cnt;
	u8 aux_l_gain;
	u8 aux_r_gain;

	bool spkr_pa_widget_on;
	struct regulator *spkdrv_reg;
	struct regulator *spkdrv2_reg;

	bool mbhc_started;

	struct afe_param_cdc_slimbus_slave_cfg slimbus_slave_cfg;

	/*
	 * list used to save/restore registers at start and
	 * end of impedance measurement
	 */
	struct list_head reg_save_restore;

	u32 ana_rx_supplies;
	/* Multiplication factor used for impedance detection */
	int zdet_gain_mul_fact;

	/* to track the status */
	unsigned long status_mask;

	struct work_struct swr_add_devices_work;
	struct wcd_swr_ctrl_platform_data swr_plat_data;

	/* Port values for Rx and Tx codec_dai */
	unsigned int rx_port_value;
	unsigned int tx_port_value;

	/* Tasha Interpolator Mode Select for EAR, HPH_L and HPH_R */
	u32 rx_int_mode[3];

	struct clk *wcd_ext_clk;
	struct mutex swr_read_lock;
	struct mutex swr_write_lock;
};

static int tasha_get_iir_enable_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;
	u16 iir_reg = WCD9335_CDC_SIDETONE_IIR0_IIR_CTL + 16 * iir_idx;

	ucontrol->value.integer.value[0] = (snd_soc_read(codec, iir_reg) &
					    (1 << band_idx)) != 0;

	dev_dbg(codec->dev, "%s: IIR #%d band #%d enable %d\n", __func__,
		iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[0]);
	return 0;
}

/* virtual port entries */
static int slim_tx_mixer_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist =
					dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = tasha_p->tx_port_value;
	return 0;
}

static int slim_tx_mixer_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist =
					dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(codec);
	struct wcd9xxx *core = dev_get_drvdata(codec->dev->parent);
	struct snd_soc_dapm_update *update = NULL;
	struct soc_multi_mixer_control *mixer =
		((struct soc_multi_mixer_control *)kcontrol->private_value);
	u32 dai_id = widget->shift;
	u32 port_id = mixer->shift;
	u32 enable = ucontrol->value.integer.value[0];
	u32 vtable = vport_check_table[dai_id];


	dev_dbg(codec->dev, "%s: wname %s cname %s value %u shift %d item %ld\n",
		  __func__,
		widget->name, ucontrol->id.name, tasha_p->tx_port_value,
		widget->shift, ucontrol->value.integer.value[0]);

	mutex_lock(&codec->mutex);

	if (tasha_p->intf_type != WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
		if (dai_id != AIF1_CAP) {
			dev_err(codec->dev, "%s: invalid AIF for I2C mode\n",
				__func__);
			mutex_unlock(&codec->mutex);
			return -EINVAL;
		}
	}
	switch (dai_id) {
	case AIF1_CAP:
	case AIF2_CAP:
	case AIF3_CAP:
		/* only add to the list if value not set
		 */
		if (enable && !(tasha_p->tx_port_value & 1 << port_id)) {

			if (tasha_p->intf_type ==
				WCD9XXX_INTERFACE_TYPE_SLIMBUS)
				vtable = vport_check_table[dai_id];

			if (wcd9xxx_tx_vport_validation(
					vtable,
					port_id,
					tasha_p->dai, NUM_CODEC_DAIS)) {
				dev_dbg(codec->dev, "%s: TX%u is used by other virtual port\n",
					__func__, port_id);
				mutex_unlock(&codec->mutex);
				return 0;
			}
			tasha_p->tx_port_value |= 1 << port_id;
			list_add_tail(&core->tx_chs[port_id].list,
			      &tasha_p->dai[dai_id].wcd9xxx_ch_list
					      );
		} else if (!enable && (tasha_p->tx_port_value &
					1 << port_id)) {
			tasha_p->tx_port_value &= ~(1 << port_id);
			list_del_init(&core->tx_chs[port_id].list);
		} else {
			if (enable)
				dev_dbg(codec->dev, "%s: TX%u port is used by\n"
					"this virtual port\n",
					__func__, port_id);
			else
				dev_dbg(codec->dev, "%s: TX%u port is not used by\n"
					"this virtual port\n",
					__func__, port_id);
			/* avoid update power function */
			mutex_unlock(&codec->mutex);
			return 0;
		}
		break;
	default:
		pr_err("Unknown AIF %d\n", dai_id);
		mutex_unlock(&codec->mutex);
		return -EINVAL;
	}
	pr_debug("%s: name %s sname %s updated value %u shift %d\n", __func__,
		widget->name, widget->sname, tasha_p->tx_port_value,
		widget->shift);

	mutex_unlock(&codec->mutex);
	snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, enable, update);

	return 0;
}

static int slim_rx_mux_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist =
					dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = tasha_p->rx_port_value;
	return 0;
}

static const char *const slim_rx_mux_text[] = {
	"ZERO", "AIF1_PB", "AIF2_PB", "AIF3_PB"
};

static int slim_rx_mux_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist =
					dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(codec);
	struct wcd9xxx *core = dev_get_drvdata(codec->dev->parent);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;
	u32 port_id = widget->shift;

	pr_debug("%s: wname %s cname %s value %u shift %d item %ld\n", __func__,
		widget->name, ucontrol->id.name, tasha_p->rx_port_value,
		widget->shift, ucontrol->value.integer.value[0]);

	tasha_p->rx_port_value = ucontrol->value.enumerated.item[0];

	mutex_lock(&codec->mutex);

	if (tasha_p->intf_type != WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
		if (tasha_p->rx_port_value > 2) {
			dev_err(codec->dev, "%s: invalid AIF for I2C mode\n",
				__func__);
			goto err;
		}
	}
	/* value need to match the Virtual port and AIF number
	 */
	switch (tasha_p->rx_port_value) {
	case 0:
		list_del_init(&core->rx_chs[port_id].list);
		break;
	case 1:
		if (wcd9xxx_rx_vport_validation(port_id +
			TASHA_RX_PORT_START_NUMBER,
			&tasha_p->dai[AIF1_PB].wcd9xxx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &tasha_p->dai[AIF1_PB].wcd9xxx_ch_list);
		break;
	case 2:
		if (wcd9xxx_rx_vport_validation(port_id +
			TASHA_RX_PORT_START_NUMBER,
			&tasha_p->dai[AIF2_PB].wcd9xxx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &tasha_p->dai[AIF2_PB].wcd9xxx_ch_list);
		break;
	case 3:
		if (wcd9xxx_rx_vport_validation(port_id +
			TASHA_RX_PORT_START_NUMBER,
			&tasha_p->dai[AIF3_PB].wcd9xxx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &tasha_p->dai[AIF3_PB].wcd9xxx_ch_list);
		break;
	default:
		pr_err("Unknown AIF %d\n", tasha_p->rx_port_value);
		goto err;
	}
rtn:
	mutex_unlock(&codec->mutex);
	snd_soc_dapm_mux_update_power(widget->dapm, kcontrol,
					tasha_p->rx_port_value, e, update);

	return 0;
err:
	mutex_unlock(&codec->mutex);
	return -EINVAL;
}

static const struct soc_enum slim_rx_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slim_rx_mux_text), slim_rx_mux_text);

static const struct snd_kcontrol_new slim_rx_mux[TASHA_RX_MAX] = {
	SOC_DAPM_ENUM_EXT("SLIM RX0 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX1 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX2 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX3 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX4 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX5 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX6 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX7 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
};

static const struct snd_kcontrol_new aif1_cap_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX0", SND_SOC_NOPM, TASHA_TX0, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, TASHA_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, TASHA_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, TASHA_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, TASHA_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, TASHA_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, TASHA_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, TASHA_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, TASHA_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, TASHA_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, TASHA_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static const struct snd_kcontrol_new aif2_cap_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX0", SND_SOC_NOPM, TASHA_TX0, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, TASHA_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, TASHA_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, TASHA_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, TASHA_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, TASHA_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, TASHA_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, TASHA_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, TASHA_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, TASHA_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, TASHA_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static const struct snd_kcontrol_new aif3_cap_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX0", SND_SOC_NOPM, TASHA_TX0, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, TASHA_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, TASHA_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, TASHA_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, TASHA_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, TASHA_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, TASHA_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, TASHA_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, TASHA_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, TASHA_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, TASHA_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static int tasha_put_iir_enable_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;
	int value = ucontrol->value.integer.value[0];
	u16 iir_reg = WCD9335_CDC_SIDETONE_IIR0_IIR_CTL + 16 * iir_idx;

	/* Mask first 5 bits, 6-8 are reserved */
	snd_soc_update_bits(codec, iir_reg, (1 << band_idx),
			    (value << band_idx));

	pr_debug("%s: IIR #%d band #%d enable %d\n", __func__,
		iir_idx, band_idx,
		((snd_soc_read(codec, iir_reg) &
		(1 << band_idx)) != 0));
	return 0;
}

static uint32_t get_iir_band_coeff(struct snd_soc_codec *codec,
				int iir_idx, int band_idx,
				int coeff_idx)
{
	uint32_t value = 0;

	/* Address does not automatically update if reading */
	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B1_CTL + 16 * iir_idx),
		((band_idx * BAND_MAX + coeff_idx)
		* sizeof(uint32_t)) & 0x7F);

	value |= snd_soc_read(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL + 16 * iir_idx));

	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B1_CTL + 16 * iir_idx),
		((band_idx * BAND_MAX + coeff_idx)
		* sizeof(uint32_t) + 1) & 0x7F);

	value |= (snd_soc_read(codec,
			       (WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL +
				16 * iir_idx)) << 8);

	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B1_CTL + 16 * iir_idx),
		((band_idx * BAND_MAX + coeff_idx)
		* sizeof(uint32_t) + 2) & 0x7F);

	value |= (snd_soc_read(codec,
			       (WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL +
				16 * iir_idx)) << 16);

	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B1_CTL + 16 * iir_idx),
		((band_idx * BAND_MAX + coeff_idx)
		* sizeof(uint32_t) + 3) & 0x7F);

	/* Mask bits top 2 bits since they are reserved */
	value |= ((snd_soc_read(codec,
				(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL +
				 16 * iir_idx)) & 0x3F) << 24);

	return value;
}

static int tasha_get_iir_band_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;

	ucontrol->value.integer.value[0] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 0);
	ucontrol->value.integer.value[1] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 1);
	ucontrol->value.integer.value[2] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 2);
	ucontrol->value.integer.value[3] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 3);
	ucontrol->value.integer.value[4] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 4);

	pr_debug("%s: IIR #%d band #%d b0 = 0x%x\n"
		"%s: IIR #%d band #%d b1 = 0x%x\n"
		"%s: IIR #%d band #%d b2 = 0x%x\n"
		"%s: IIR #%d band #%d a1 = 0x%x\n"
		"%s: IIR #%d band #%d a2 = 0x%x\n",
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[0],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[1],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[2],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[3],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[4]);
	return 0;
}

static void set_iir_band_coeff(struct snd_soc_codec *codec,
				int iir_idx, int band_idx,
				uint32_t value)
{
	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL + 16 * iir_idx),
		(value & 0xFF));

	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL + 16 * iir_idx),
		(value >> 8) & 0xFF);

	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL + 16 * iir_idx),
		(value >> 16) & 0xFF);

	/* Mask top 2 bits, 7-8 are reserved */
	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL + 16 * iir_idx),
		(value >> 24) & 0x3F);
}

static void tasha_codec_enable_int_port(struct wcd9xxx_codec_dai_data *dai,
					struct snd_soc_codec *codec)
{
	struct wcd9xxx_ch *ch;
	int port_num = 0;
	unsigned short reg = 0;
	u8 val = 0;
	struct tasha_priv *tasha_p;

	if (!dai || !codec) {
		pr_err("%s: Invalid params\n", __func__);
		return;
	}

	tasha_p = snd_soc_codec_get_drvdata(codec);
	list_for_each_entry(ch, &dai->wcd9xxx_ch_list, list) {
		if (ch->port >= TASHA_RX_PORT_START_NUMBER) {
			port_num = ch->port - TASHA_RX_PORT_START_NUMBER;
			reg = TASHA_SLIM_PGD_PORT_INT_EN0 + (port_num / 8);
			val = wcd9xxx_interface_reg_read(tasha_p->wcd9xxx,
				reg);
			if (!(val & (1 << (port_num % 8)))) {
				val |= (1 << (port_num % 8));
				wcd9xxx_interface_reg_write(
					tasha_p->wcd9xxx, reg, val);
				val = wcd9xxx_interface_reg_read(
					tasha_p->wcd9xxx, reg);
			}
		} else {
			port_num = ch->port;
			reg = TASHA_SLIM_PGD_PORT_INT_TX_EN0 + (port_num / 8);
			val = wcd9xxx_interface_reg_read(tasha_p->wcd9xxx,
				reg);
			if (!(val & (1 << (port_num % 8)))) {
				val |= (1 << (port_num % 8));
				wcd9xxx_interface_reg_write(tasha_p->wcd9xxx,
					reg, val);
				val = wcd9xxx_interface_reg_read(
					tasha_p->wcd9xxx, reg);
			}
		}
	}
}

static int tasha_codec_enable_slim_chmask(struct wcd9xxx_codec_dai_data *dai,
					  bool up)
{
	int ret = 0;
	struct wcd9xxx_ch *ch;

	if (up) {
		list_for_each_entry(ch, &dai->wcd9xxx_ch_list, list) {
			ret = wcd9xxx_get_slave_port(ch->ch_num);
			if (ret < 0) {
				pr_err("%s: Invalid slave port ID: %d\n",
				       __func__, ret);
				ret = -EINVAL;
			} else {
				set_bit(ret, &dai->ch_mask);
			}
		}
	} else {
		ret = wait_event_timeout(dai->dai_wait, (dai->ch_mask == 0),
					 msecs_to_jiffies(
						TASHA_SLIM_CLOSE_TIMEOUT));
		if (!ret) {
			pr_err("%s: Slim close tx/rx wait timeout\n", __func__);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
		}
	}
	return ret;
}

static int tasha_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct wcd9xxx *core;
	struct snd_soc_codec *codec = w->codec;
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	struct wcd9xxx_codec_dai_data *dai;

	core = dev_get_drvdata(codec->dev->parent);

	dev_dbg(codec->dev, "%s: event called! codec name %s num_dai %d\n"
		"stream name %s event %d\n",
		__func__, w->codec->name, w->codec->num_dai, w->sname, event);

	/* Execute the callback only if interface type is slimbus */
	if (tasha_p->intf_type != WCD9XXX_INTERFACE_TYPE_SLIMBUS)
		return 0;

	dai = &tasha_p->dai[w->shift];
	dev_dbg(codec->dev, "%s: w->name %s w->shift %d event %d\n",
		 __func__, w->name, w->shift, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		tasha_codec_enable_int_port(dai, codec);
		(void) tasha_codec_enable_slim_chmask(dai, true);
		ret = wcd9xxx_cfg_slim_sch_rx(core, &dai->wcd9xxx_ch_list,
					      dai->rate, dai->bit_width,
					      &dai->grph);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = wcd9xxx_close_slim_sch_rx(core, &dai->wcd9xxx_ch_list,
						dai->grph);
		ret = tasha_codec_enable_slim_chmask(dai, false);
		if (ret < 0) {
			ret = wcd9xxx_disconnect_port(core,
						      &dai->wcd9xxx_ch_list,
						      dai->grph);
			dev_dbg(codec->dev, "%s: Disconnect RX port, ret = %d\n",
				 __func__, ret);
		}

		break;
	}
	return ret;
}

static int tasha_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct wcd9xxx *core;
	struct snd_soc_codec *codec = w->codec;
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(codec);
	u32  ret = 0;
	struct wcd9xxx_codec_dai_data *dai;

	core = dev_get_drvdata(codec->dev->parent);

	pr_debug("%s: event called! codec name %s num_dai %d stream name %s\n",
		__func__, w->codec->name, w->codec->num_dai, w->sname);

	/* Execute the callback only if interface type is slimbus */
	if (tasha_p->intf_type != WCD9XXX_INTERFACE_TYPE_SLIMBUS)
		return 0;

	pr_debug("%s(): w->name %s event %d w->shift %d\n",
		__func__, w->name, event, w->shift);

	dai = &tasha_p->dai[w->shift];
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		tasha_codec_enable_int_port(dai, codec);
		(void) tasha_codec_enable_slim_chmask(dai, true);
		ret = wcd9xxx_cfg_slim_sch_tx(core, &dai->wcd9xxx_ch_list,
					      dai->rate, dai->bit_width,
					      &dai->grph);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = wcd9xxx_close_slim_sch_tx(core, &dai->wcd9xxx_ch_list,
						dai->grph);
		ret = tasha_codec_enable_slim_chmask(dai, false);
		if (ret < 0) {
			ret = wcd9xxx_disconnect_port(core,
						      &dai->wcd9xxx_ch_list,
						      dai->grph);
			pr_debug("%s: Disconnect RX port, ret = %d\n",
				 __func__, ret);
		}

		break;
	}
	return ret;
}

static int tasha_put_iir_band_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;

	/* Mask top bit it is reserved */
	/* Updates addr automatically for each B2 write */
	snd_soc_write(codec,
		(WCD9335_CDC_SIDETONE_IIR0_IIR_COEF_B1_CTL + 16 * iir_idx),
		(band_idx * BAND_MAX * sizeof(uint32_t)) & 0x7F);

	set_iir_band_coeff(codec, iir_idx, band_idx,
				ucontrol->value.integer.value[0]);
	set_iir_band_coeff(codec, iir_idx, band_idx,
				ucontrol->value.integer.value[1]);
	set_iir_band_coeff(codec, iir_idx, band_idx,
				ucontrol->value.integer.value[2]);
	set_iir_band_coeff(codec, iir_idx, band_idx,
				ucontrol->value.integer.value[3]);
	set_iir_band_coeff(codec, iir_idx, band_idx,
				ucontrol->value.integer.value[4]);

	pr_debug("%s: IIR #%d band #%d b0 = 0x%x\n"
		"%s: IIR #%d band #%d b1 = 0x%x\n"
		"%s: IIR #%d band #%d b2 = 0x%x\n"
		"%s: IIR #%d band #%d a1 = 0x%x\n"
		"%s: IIR #%d band #%d a2 = 0x%x\n",
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 0),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 1),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 2),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 3),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 4));
	return 0;
}

static int tasha_get_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int comp = ((struct soc_multi_mixer_control *)
		    kcontrol->private_value)->shift;
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = tasha->comp_enabled[comp];
	return 0;
}

static int tasha_set_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);
	int comp = ((struct soc_multi_mixer_control *)
		    kcontrol->private_value)->shift;
	int value = ucontrol->value.integer.value[0];

	pr_debug("%s: Compander %d enable current %d, new %d\n",
		 __func__, comp, tasha->comp_enabled[comp], value);
	tasha->comp_enabled[comp] = value;

	/* Any specific register configuration for compander */
	switch (comp) {
	case COMPANDER_1:
		/* Set Gain Source Select based on compander enable/disable */
		snd_soc_update_bits(codec, WCD9335_HPH_L_EN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_2:
		snd_soc_update_bits(codec, WCD9335_HPH_R_EN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_3:
		break;
	case COMPANDER_4:
		break;
	case COMPANDER_5:
		snd_soc_update_bits(codec, WCD9335_SE_LO_LO3_GAIN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_6:
		snd_soc_update_bits(codec, WCD9335_SE_LO_LO4_GAIN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_7:
		break;
	case COMPANDER_8:
		break;
	};
	return 0;
}

static int tasha_int_mode_select_reg(struct snd_soc_dapm_widget *w)
{
	if (strnstr(w->name, "RX INT0", sizeof("RX INT0")))
		return WCD9335_CDC_RX0_RX_PATH_SEC0;
	else if (strnstr(w->name, "RX INT1", sizeof("RX INT1")))
		return WCD9335_CDC_RX1_RX_PATH_SEC0;
	else if (strnstr(w->name, "RX INT2", sizeof("RX INT2")))
		return WCD9335_CDC_RX2_RX_PATH_SEC0;
	else {
		dev_err(w->codec->dev, "%s: Cannot find mode select reg for %s\n",
			__func__, w->name);
		return -EINVAL;
	}
}

static int tasha_codec_enable_rx_supplies(struct snd_soc_codec *codec,
					  int event)
{
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s:ana rx supplies: 0x%x\n", __func__,
		tasha->ana_rx_supplies);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (!tasha->ana_rx_supplies) {
			snd_soc_update_bits(codec, WCD9335_ANA_RX_SUPPLIES,
					    0x80, 0x80);
			snd_soc_update_bits(codec, WCD9335_ANA_RX_SUPPLIES,
					    0x40, 0x40);
		} else
			tasha->ana_rx_supplies++;
		break;
	case SND_SOC_DAPM_POST_PMD:
		tasha->ana_rx_supplies--;

		if (!tasha->ana_rx_supplies) {
			snd_soc_update_bits(codec, WCD9335_ANA_RX_SUPPLIES,
					    0x40, 0x00);
			snd_soc_update_bits(codec, WCD9335_ANA_RX_SUPPLIES,
					    0x80, 0x00);
		}
		break;
	};

	return 0;
}

static int tasha_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		tasha->rx_bias_count++;
		if (tasha->rx_bias_count == 1)
			snd_soc_update_bits(codec, WCD9335_ANA_RX_SUPPLIES,
					    0x01, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tasha->rx_bias_count--;
		if (!tasha->rx_bias_count)
			snd_soc_update_bits(codec, WCD9335_ANA_RX_SUPPLIES,
					    0x01, 0x00);
		break;
	};
	dev_dbg(codec->dev, "%s: Current RX BIAS user count: %d\n", __func__,
		tasha->rx_bias_count);

	return 0;
}

static int tasha_codec_enable_ear_pa(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	dev_dbg(codec->dev, "%s %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		tasha_codec_enable_rx_supplies(codec, event);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, WCD9335_CDC_RX0_RX_PATH_CTL,
				    0x10, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tasha_codec_enable_rx_supplies(codec, event);
		break;
	};

	return 0;
}

static int tasha_codec_ear_dac_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int mode;

	dev_dbg(codec->dev, "%s %s %d\n", __func__, w->name, event);

	mode = snd_soc_read(codec, WCD9335_CDC_RX0_RX_PATH_SEC0) & 0x03;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (mode == 0x1) {
			/* Class-H config */
		} else if (mode == 0x0) {
			/* Class-AB config */
			snd_soc_update_bits(codec, WCD9335_ANA_RX_SUPPLIES,
					    0x02, 0x02);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		break;
	case SND_SOC_DAPM_PRE_PMD:
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	};

	return 0;
}

static int tasha_codec_enable_interpolator(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u16 gain_reg;

	dev_dbg(codec->dev, "%s %d %s\n", __func__, event, w->name);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Reset if needed */
		/* Enable Digital Mute */
		snd_soc_update_bits(codec, w->reg, 0x10, 0x10);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* apply gain after int clk is enabled */
		if (w->reg == WCD9335_CDC_RX0_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX0_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX1_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX1_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX2_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX2_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX3_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX3_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX4_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX4_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX5_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX5_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX6_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX6_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX7_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX7_RX_VOL_CTL;
		else if (w->reg == WCD9335_CDC_RX8_RX_PATH_CTL)
			gain_reg = WCD9335_CDC_RX8_RX_VOL_CTL;
		else {
			dev_err(codec->dev, "%s: No gain register avail for %s\n",
				__func__, w->name);
			return 0;
		}
		snd_soc_write(codec, gain_reg, snd_soc_read(codec, gain_reg));
		break;
	};

	return 0;
}

static int tasha_interpolator_mode_select(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int dem_inp_sel_reg;
	u8 dem_inp_sel_val;
	struct snd_soc_codec *codec = w->codec;
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);

	dem_inp_sel_reg = tasha_int_mode_select_reg(w);
	if (dem_inp_sel_reg < 0)
		return -EINVAL;

	if (w->shift > 2) {
		dev_err(codec->dev, "%s: cannot set mode for: %s\n",
			__func__, w->name);
		return -EINVAL;
	}
	dem_inp_sel_val = tasha->rx_int_mode[w->shift];
	if (dem_inp_sel_val > 1) {
		dev_err(codec->dev, "%s: Unexpected dem inp value 0x%x for widget:%s\n",
			__func__, dem_inp_sel_val, w->name);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, dem_inp_sel_reg, 0x03,
				    dem_inp_sel_val);
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	};

	return 0;
}

static int tasha_codec_iir_mux_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	dev_dbg(codec->dev, "%s: event = %d\n", __func__, event);

	return 0;
}

#define  TX_HPF_CUT_OFF_FREQ_MASK	0x60
#define  CF_MIN_3DB_4HZ			0x0
#define  CF_MIN_3DB_75HZ		0x1
#define  CF_MIN_3DB_150HZ		0x2

static int tasha_codec_enable_dec(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	unsigned int decimator;
	char *dec_adc_mux_name = NULL;
	char *widget_name = NULL;
	char *wname;
	int ret = 0;
	u16 tx_vol_ctl_reg, tx_hpf_ctl_reg;
	char *dec;

	dev_dbg(codec->dev, "%s %d\n", __func__, event);

	widget_name = kstrndup(w->name, 15, GFP_KERNEL);
	if (!widget_name)
		return -ENOMEM;

	wname = widget_name;
	dec_adc_mux_name = strsep(&widget_name, " ");
	if (!dec_adc_mux_name) {
		dev_err(codec->dev, "%s: Invalid decimator = %s\n",
			__func__, w->name);
		ret =  -EINVAL;
		goto out;
	}
	dec_adc_mux_name = widget_name;

	dec = strpbrk(dec_adc_mux_name, "012345678");
	if (!dec) {
		dev_err(codec->dev, "%s: decimator index not found\n",
			__func__);
		ret =  -EINVAL;
		goto out;
	}

	ret = kstrtouint(dec, 10, &decimator);
	if (ret < 0) {
		dev_err(codec->dev, "%s: Invalid decimator = %s\n",
			__func__, wname);
		ret =  -EINVAL;
		goto out;
	}

	dev_dbg(codec->dev, "%s(): widget = %s decimator = %u\n", __func__,
			w->name, decimator);

	tx_vol_ctl_reg = WCD9335_CDC_TX0_TX_PATH_CTL + 16 * decimator;
	tx_hpf_ctl_reg = WCD9335_CDC_TX0_TX_PATH_CFG0 + 16 * decimator;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enable TX PGA Mute */
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x10, 0x10);
		snd_soc_update_bits(codec, tx_hpf_ctl_reg, 0x01, 0x01);
		/* 75Hz hpf cutoff */
		snd_soc_update_bits(codec, tx_hpf_ctl_reg,
				    TX_HPF_CUT_OFF_FREQ_MASK,
				    CF_MIN_3DB_75HZ << 5);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* Remove Mute */
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x10, 0x00);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x10, 0x10);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, tx_hpf_ctl_reg,
				    TX_HPF_CUT_OFF_FREQ_MASK,
				    CF_MIN_3DB_4HZ << 5);
		snd_soc_update_bits(codec, tx_hpf_ctl_reg, 0x01, 0x00);
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x10, 0x00);
		break;
	};
out:
	kfree(wname);
	return ret;
}

static int tasha_codec_enable_ana_bias(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, WCD9335_ANA_BIAS, 0x40, 0x40);
		snd_soc_update_bits(codec, WCD9335_ANA_BIAS, 0x80, 0x80);
		/* delay require for the bias to be stable */
		usleep_range(20, 25);
		snd_soc_update_bits(codec, WCD9335_ANA_BIAS, 0x40, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, WCD9335_ANA_BIAS, 0x80, 0x00);
		break;
	};
	return 0;
}

static int tasha_codec_enable_ana_clk(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enable Analog clk top clock buffer */
		snd_soc_update_bits(codec, WCD9335_ANA_CLK_TOP, 0x80, 0x80);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, WCD9335_ANA_CLK_TOP, 0x80, 0x00);
		break;
	};

	return 0;
}

static int tasha_codec_enable_dmic(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}

static int tasha_codec_enable_micbias(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u16 micb_ctl;

	micb_ctl = w->reg;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* VOUT_CTL = 24
		 * micbias = 1V + VOUT_CTL * 50mV
		 */
		snd_soc_update_bits(codec, micb_ctl, 0x3F, 0x18);
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	};

	return 0;
}

/* Cutoff frequency for high pass filter */
static const char * const cf_text[] = {
	"CF_NEG_3DB_4HZ", "CF_NEG_3DB_75HZ", "CF_NEG_3DB_150HZ"
};

static const char * const rx_cf_text[] = {
	"CF_NEG_3DB_4HZ", "CF_NEG_3DB_75HZ", "CF_NEG_3DB_150HZ",
	"CF_NEG_3DB_0P48HZ"
};

static const struct soc_enum cf_dec0_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX0_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX1_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec2_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX2_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec3_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX3_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec4_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX4_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec5_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX5_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec6_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX6_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec7_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX7_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec8_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX8_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_int0_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX0_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int1_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX1_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int2_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX2_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int3_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX3_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int4_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX4_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int5_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX5_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int6_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX6_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int7_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX7_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct soc_enum cf_int8_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX8_RX_PATH_CFG2, 0, 4, rx_cf_text);

static const struct snd_soc_dapm_route audio_map[] = {
	/* SLIMBUS Connections */
	{"AIF1 CAP", NULL, "AIF1_CAP Mixer"},
	{"AIF2 CAP", NULL, "AIF2_CAP Mixer"},
	{"AIF3 CAP", NULL, "AIF3_CAP Mixer"},

	/* SLIM_MIXER("AIF1_CAP Mixer"),*/
	{"AIF1_CAP Mixer", "SLIM TX0", "SLIM TX0 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX1", "SLIM TX1 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX2", "SLIM TX2 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX3", "SLIM TX3 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX4", "SLIM TX4 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX5", "SLIM TX5 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX6", "SLIM TX6 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX7", "SLIM TX7 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX8", "SLIM TX8 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX9", "SLIM TX9 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX10", "SLIM TX10 MUX"},
	/* SLIM_MIXER("AIF2_CAP Mixer"),*/
	{"AIF2_CAP Mixer", "SLIM TX0", "SLIM TX0 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX1", "SLIM TX1 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX2", "SLIM TX2 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX3", "SLIM TX3 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX4", "SLIM TX4 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX5", "SLIM TX5 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX6", "SLIM TX6 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX7", "SLIM TX7 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX8", "SLIM TX8 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX9", "SLIM TX9 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX10", "SLIM TX10 MUX"},
	/* SLIM_MIXER("AIF3_CAP Mixer"),*/
	{"AIF3_CAP Mixer", "SLIM TX0", "SLIM TX0 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX1", "SLIM TX1 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX2", "SLIM TX2 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX3", "SLIM TX3 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX4", "SLIM TX4 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX5", "SLIM TX5 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX6", "SLIM TX6 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX7", "SLIM TX7 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX8", "SLIM TX8 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX9", "SLIM TX9 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX10", "SLIM TX10 MUX"},

	{"SLIM TX0 MUX", "DEC0", "ADC MUX0"},
	{"SLIM TX0 MUX", "RX_MIX_TX0", "RX MIX TX0 MUX"},

	{"SLIM TX1 MUX", "DEC1", "ADC MUX1"},
	{"SLIM TX1 MUX", "RX_MIX_TX1", "RX MIX TX1 MUX"},

	{"SLIM TX2 MUX", "DEC2", "ADC MUX2"},
	{"SLIM TX2 MUX", "RX_MIX_TX2", "RX MIX TX2 MUX"},

	{"SLIM TX3 MUX", "DEC3", "ADC MUX3"},
	{"SLIM TX3 MUX", "RX_MIX_TX3", "RX MIX TX3 MUX"},

	{"SLIM TX4 MUX", "DEC4", "ADC MUX4"},
	{"SLIM TX4 MUX", "RX_MIX_TX4", "RX MIX TX4 MUX"},

	{"SLIM TX5 MUX", "DEC5", "ADC MUX5"},
	{"SLIM TX5 MUX", "RX_MIX_TX5", "RX MIX TX5 MUX"},

	{"SLIM TX6 MUX", "DEC6", "ADC MUX6"},
	{"SLIM TX6 MUX", "RX_MIX_TX6", "RX MIX TX6 MUX"},

	{"SLIM TX7 MUX", "DEC7", "ADC MUX7"},
	{"SLIM TX7 MUX", "RX_MIX_TX7", "RX MIX TX7 MUX"},

	{"SLIM TX8 MUX", "DEC8", "ADC MUX8"},
	{"SLIM TX8 MUX", "RX_MIX_TX8", "RX MIX TX8 MUX"},

	{"SLIM TX9 MUX", "DEC7", "ADC MUX7"},
	{"SLIM TX10 MUX", "DEC6", "ADC MUX6"},

	{"RX MIX TX0 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX0 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX1 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX1 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX2 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX2 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX3 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX3 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX4 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX4 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX5 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX5 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX6 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX6 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX7 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX7 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"RX MIX TX8 MUX", "RX_MIX0", "RX INT0_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX1", "RX INT1_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX2", "RX INT2_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX3", "RX INT3_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX4", "RX INT4_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX5", "RX INT5_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX6", "RX INT6_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX7", "RX INT7_1 MIX1"},
	{"RX MIX TX8 MUX", "RX_MIX8", "RX INT8_1 MIX1"},

	{"ADC MUX0", "DMIC", "DMIC MUX0"},
	{"ADC MUX0", "AMIC", "AMIC MUX0"},
	{"ADC MUX0", NULL, "ANA_CLK_TOP"},
	{"ADC MUX1", "DMIC", "DMIC MUX1"},
	{"ADC MUX1", "AMIC", "AMIC MUX1"},
	{"ADC MUX1", NULL, "ANA_CLK_TOP"},
	{"ADC MUX2", "DMIC", "DMIC MUX2"},
	{"ADC MUX2", "AMIC", "AMIC MUX2"},
	{"ADC MUX2", NULL, "ANA_CLK_TOP"},
	{"ADC MUX3", "DMIC", "DMIC MUX3"},
	{"ADC MUX3", "AMIC", "AMIC MUX3"},
	{"ADC MUX3", NULL, "ANA_CLK_TOP"},
	{"ADC MUX4", "DMIC", "DMIC MUX4"},
	{"ADC MUX4", "AMIC", "AMIC MUX4"},
	{"ADC MUX4", NULL, "ANA_CLK_TOP"},
	{"ADC MUX5", "DMIC", "DMIC MUX5"},
	{"ADC MUX5", "AMIC", "AMIC MUX5"},
	{"ADC MUX5", NULL, "ANA_CLK_TOP"},
	{"ADC MUX6", "DMIC", "DMIC MUX6"},
	{"ADC MUX6", "AMIC", "AMIC MUX6"},
	{"ADC MUX6", NULL, "ANA_CLK_TOP"},
	{"ADC MUX7", "DMIC", "DMIC MUX7"},
	{"ADC MUX7", "AMIC", "AMIC MUX7"},
	{"ADC MUX7", NULL, "ANA_CLK_TOP"},
	{"ADC MUX8", "DMIC", "DMIC MUX8"},
	{"ADC MUX8", "AMIC", "AMIC MUX8"},
	{"ADC MUX8", NULL, "ANA_CLK_TOP"},

	{"DMIC MUX0", "DMIC0", "DMIC0"},
	{"DMIC MUX0", "DMIC1", "DMIC1"},
	{"DMIC MUX0", "DMIC2", "DMIC2"},
	{"DMIC MUX0", "DMIC3", "DMIC3"},
	{"DMIC MUX0", "DMIC4", "DMIC4"},
	{"DMIC MUX0", "DMIC5", "DMIC5"},
	{"AMIC MUX0", "ADC1", "ADC1"},
	{"AMIC MUX0", "ADC2", "ADC2"},
	{"AMIC MUX0", "ADC3", "ADC3"},
	{"AMIC MUX0", "ADC4", "ADC4"},
	{"AMIC MUX0", "ADC5", "ADC5"},
	{"AMIC MUX0", "ADC6", "ADC6"},

	{"DMIC MUX1", "DMIC0", "DMIC0"},
	{"DMIC MUX1", "DMIC1", "DMIC1"},
	{"DMIC MUX1", "DMIC2", "DMIC2"},
	{"DMIC MUX1", "DMIC3", "DMIC3"},
	{"DMIC MUX1", "DMIC4", "DMIC4"},
	{"DMIC MUX1", "DMIC5", "DMIC5"},
	{"AMIC MUX1", "ADC1", "ADC1"},
	{"AMIC MUX1", "ADC2", "ADC2"},
	{"AMIC MUX1", "ADC3", "ADC3"},
	{"AMIC MUX1", "ADC4", "ADC4"},
	{"AMIC MUX1", "ADC5", "ADC5"},
	{"AMIC MUX1", "ADC6", "ADC6"},

	{"DMIC MUX2", "DMIC0", "DMIC0"},
	{"DMIC MUX2", "DMIC1", "DMIC1"},
	{"DMIC MUX2", "DMIC2", "DMIC2"},
	{"DMIC MUX2", "DMIC3", "DMIC3"},
	{"DMIC MUX2", "DMIC4", "DMIC4"},
	{"DMIC MUX2", "DMIC5", "DMIC5"},
	{"AMIC MUX2", "ADC1", "ADC1"},
	{"AMIC MUX2", "ADC2", "ADC2"},
	{"AMIC MUX2", "ADC3", "ADC3"},
	{"AMIC MUX2", "ADC4", "ADC4"},
	{"AMIC MUX2", "ADC5", "ADC5"},
	{"AMIC MUX2", "ADC6", "ADC6"},

	{"DMIC MUX3", "DMIC0", "DMIC0"},
	{"DMIC MUX3", "DMIC1", "DMIC1"},
	{"DMIC MUX3", "DMIC2", "DMIC2"},
	{"DMIC MUX3", "DMIC3", "DMIC3"},
	{"DMIC MUX3", "DMIC4", "DMIC4"},
	{"DMIC MUX3", "DMIC5", "DMIC5"},
	{"AMIC MUX3", "ADC1", "ADC1"},
	{"AMIC MUX3", "ADC2", "ADC2"},
	{"AMIC MUX3", "ADC3", "ADC3"},
	{"AMIC MUX3", "ADC4", "ADC4"},
	{"AMIC MUX3", "ADC5", "ADC5"},
	{"AMIC MUX3", "ADC6", "ADC6"},

	{"DMIC MUX4", "DMIC0", "DMIC0"},
	{"DMIC MUX4", "DMIC1", "DMIC1"},
	{"DMIC MUX4", "DMIC2", "DMIC2"},
	{"DMIC MUX4", "DMIC3", "DMIC3"},
	{"DMIC MUX4", "DMIC4", "DMIC4"},
	{"DMIC MUX4", "DMIC5", "DMIC5"},
	{"AMIC MUX4", "ADC1", "ADC1"},
	{"AMIC MUX4", "ADC2", "ADC2"},
	{"AMIC MUX4", "ADC3", "ADC3"},
	{"AMIC MUX4", "ADC4", "ADC4"},
	{"AMIC MUX4", "ADC5", "ADC5"},
	{"AMIC MUX4", "ADC6", "ADC6"},

	{"DMIC MUX5", "DMIC0", "DMIC0"},
	{"DMIC MUX5", "DMIC1", "DMIC1"},
	{"DMIC MUX5", "DMIC2", "DMIC2"},
	{"DMIC MUX5", "DMIC3", "DMIC3"},
	{"DMIC MUX5", "DMIC4", "DMIC4"},
	{"DMIC MUX5", "DMIC5", "DMIC5"},
	{"AMIC MUX5", "ADC1", "ADC1"},
	{"AMIC MUX5", "ADC2", "ADC2"},
	{"AMIC MUX5", "ADC3", "ADC3"},
	{"AMIC MUX5", "ADC4", "ADC4"},
	{"AMIC MUX5", "ADC5", "ADC5"},
	{"AMIC MUX5", "ADC6", "ADC6"},

	{"DMIC MUX6", "DMIC0", "DMIC0"},
	{"DMIC MUX6", "DMIC1", "DMIC1"},
	{"DMIC MUX6", "DMIC2", "DMIC2"},
	{"DMIC MUX6", "DMIC3", "DMIC3"},
	{"DMIC MUX6", "DMIC4", "DMIC4"},
	{"DMIC MUX6", "DMIC5", "DMIC5"},
	{"AMIC MUX6", "ADC1", "ADC1"},
	{"AMIC MUX6", "ADC2", "ADC2"},
	{"AMIC MUX6", "ADC3", "ADC3"},
	{"AMIC MUX6", "ADC4", "ADC4"},
	{"AMIC MUX6", "ADC5", "ADC5"},
	{"AMIC MUX6", "ADC6", "ADC6"},

	{"DMIC MUX7", "DMIC0", "DMIC0"},
	{"DMIC MUX7", "DMIC1", "DMIC1"},
	{"DMIC MUX7", "DMIC2", "DMIC2"},
	{"DMIC MUX7", "DMIC3", "DMIC3"},
	{"DMIC MUX7", "DMIC4", "DMIC4"},
	{"DMIC MUX7", "DMIC5", "DMIC5"},
	{"AMIC MUX7", "ADC1", "ADC1"},
	{"AMIC MUX7", "ADC2", "ADC2"},
	{"AMIC MUX7", "ADC3", "ADC3"},
	{"AMIC MUX7", "ADC4", "ADC4"},
	{"AMIC MUX7", "ADC5", "ADC5"},
	{"AMIC MUX7", "ADC6", "ADC6"},

	{"DMIC MUX8", "DMIC0", "DMIC0"},
	{"DMIC MUX8", "DMIC1", "DMIC1"},
	{"DMIC MUX8", "DMIC2", "DMIC2"},
	{"DMIC MUX8", "DMIC3", "DMIC3"},
	{"DMIC MUX8", "DMIC4", "DMIC4"},
	{"DMIC MUX8", "DMIC5", "DMIC5"},
	{"AMIC MUX8", "ADC1", "ADC1"},
	{"AMIC MUX8", "ADC2", "ADC2"},
	{"AMIC MUX8", "ADC3", "ADC3"},
	{"AMIC MUX8", "ADC4", "ADC4"},
	{"AMIC MUX8", "ADC5", "ADC5"},
	{"AMIC MUX8", "ADC6", "ADC6"},

	/* ADC Connections */
	{"ADC1", NULL, "AMIC1"},
	{"ADC2", NULL, "AMIC2"},
	{"ADC3", NULL, "AMIC3"},
	{"ADC4", NULL, "AMIC4"},
	{"ADC5", NULL, "AMIC5"},
	{"ADC6", NULL, "AMIC6"},

	{"MIC BIAS1 External", NULL, "ANA_BIAS"},
	{"MIC BIAS2 External", NULL, "ANA_BIAS"},
	{"MIC BIAS3 External", NULL, "ANA_BIAS"},
	{"MIC BIAS4 External", NULL, "ANA_BIAS"},

	{"RX INT0_1 MIX1", NULL, "RX INT0_1 MIX1 INP0"},
	{"RX INT0_1 MIX1", NULL, "RX INT0_1 MIX1 INP1"},
	{"RX INT0_1 MIX1", NULL, "RX INT0_1 MIX1 INP2"},
	{"RX INT1_1 MIX1", NULL, "RX INT1_1 MIX1 INP0"},
	{"RX INT1_1 MIX1", NULL, "RX INT1_1 MIX1 INP1"},
	{"RX INT1_1 MIX1", NULL, "RX INT1_1 MIX1 INP2"},
	{"RX INT2_1 MIX1", NULL, "RX INT2_1 MIX1 INP0"},
	{"RX INT2_1 MIX1", NULL, "RX INT2_1 MIX1 INP1"},
	{"RX INT2_1 MIX1", NULL, "RX INT2_1 MIX1 INP2"},
	{"RX INT3_1 MIX1", NULL, "RX INT3_1 MIX1 INP0"},
	{"RX INT3_1 MIX1", NULL, "RX INT3_1 MIX1 INP1"},
	{"RX INT3_1 MIX1", NULL, "RX INT3_1 MIX1 INP2"},
	{"RX INT4_1 MIX1", NULL, "RX INT4_1 MIX1 INP0"},
	{"RX INT4_1 MIX1", NULL, "RX INT4_1 MIX1 INP1"},
	{"RX INT4_1 MIX1", NULL, "RX INT4_1 MIX1 INP2"},
	{"RX INT5_1 MIX1", NULL, "RX INT5_1 MIX1 INP0"},
	{"RX INT5_1 MIX1", NULL, "RX INT5_1 MIX1 INP1"},
	{"RX INT5_1 MIX1", NULL, "RX INT5_1 MIX1 INP2"},
	{"RX INT6_1 MIX1", NULL, "RX INT6_1 MIX1 INP0"},
	{"RX INT6_1 MIX1", NULL, "RX INT6_1 MIX1 INP1"},
	{"RX INT6_1 MIX1", NULL, "RX INT6_1 MIX1 INP2"},
	{"RX INT7_1 MIX1", NULL, "RX INT7_1 MIX1 INP0"},
	{"RX INT7_1 MIX1", NULL, "RX INT7_1 MIX1 INP1"},
	{"RX INT7_1 MIX1", NULL, "RX INT7_1 MIX1 INP2"},
	{"RX INT8_1 MIX1", NULL, "RX INT8_1 MIX1 INP0"},
	{"RX INT8_1 MIX1", NULL, "RX INT8_1 MIX1 INP1"},
	{"RX INT8_1 MIX1", NULL, "RX INT8_1 MIX1 INP2"},
	{"RX INT0 MIX2", NULL, "RX INT0_1 MIX1"},
	{"RX INT0 MIX2", NULL, "RX INT0 MIX2 INP"},
	{"RX INT1 MIX2", NULL, "RX INT1_1 MIX1"},
	{"RX INT1 MIX2", NULL, "RX INT1 MIX2 INP"},
	{"RX INT2 MIX2", NULL, "RX INT2_1 MIX1"},
	{"RX INT2 MIX2", NULL, "RX INT2 MIX2 INP"},
	{"RX INT3 MIX2", NULL, "RX INT3_1 MIX1"},
	{"RX INT3 MIX2", NULL, "RX INT3 MIX2 INP"},
	{"RX INT4 MIX2", NULL, "RX INT4_1 MIX1"},
	{"RX INT4 MIX2", NULL, "RX INT4 MIX2 INP"},
	{"RX INT7 MIX2", NULL, "RX INT7_1 MIX1"},
	{"RX INT7 MIX2", NULL, "RX INT7 MIX2 INP"},

	{"RX INT1_1 MIX1", NULL, "COMP1_CLK"},
	{"RX INT2_1 MIX1", NULL, "COMP2_CLK"},
	{"RX INT3_1 MIX1", NULL, "COMP3_CLK"},
	{"RX INT4_1 MIX1", NULL, "COMP4_CLK"},
	{"RX INT5_1 MIX1", NULL, "COMP5_CLK"},
	{"RX INT6_1 MIX1", NULL, "COMP6_CLK"},
	{"RX INT7_1 MIX1", NULL, "COMP7_CLK"},
	{"RX INT8_1 MIX1", NULL, "COMP8_CLK"},

	{"RX INT0 MODE MUX", "CLS_H", "RX INT0 MIX2"},
	{"RX INT0 MODE MUX", "CLS_AB", "RX INT0 MIX2"},
	{"RX INT0 INTERP", NULL, "RX INT0 MODE MUX"},
	{"RX INT0 DAC", NULL, "RX INT0 INTERP"},
	{"RX INT0 DAC", NULL, "RX_BIAS"},
	{"EAR PA", NULL, "RX INT0 DAC"},
	{"EAR", NULL, "EAR PA"},

	{"RX INT7 CHAIN", NULL, "RX INT7 MIX2"},
	{"RX INT7 CHAIN", NULL, "RX_BIAS"},
	{"SPK1 OUT", NULL, "RX INT7 CHAIN"},

	{"RX INT8 CHAIN", NULL, "RX INT8_1 MIX1"},
	{"RX INT8 CHAIN", NULL, "RX_BIAS"},
	{"SPK2 OUT", NULL, "RX INT8 CHAIN"},

	/* SLIM_MUX("AIF1_PB", "AIF1 PB"),*/
	{"SLIM RX0 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX1 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX2 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX3 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX4 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX5 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX6 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX7 MUX", "AIF1_PB", "AIF1 PB"},
	/* SLIM_MUX("AIF2_PB", "AIF2 PB"),*/
	{"SLIM RX0 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX1 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX2 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX3 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX4 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX5 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX6 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX7 MUX", "AIF2_PB", "AIF2 PB"},
	/* SLIM_MUX("AIF3_PB", "AIF3 PB"),*/
	{"SLIM RX0 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX1 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX2 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX3 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX4 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX5 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX6 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX7 MUX", "AIF3_PB", "AIF3 PB"},

	{"SLIM RX0", NULL, "SLIM RX0 MUX"},
	{"SLIM RX1", NULL, "SLIM RX1 MUX"},
	{"SLIM RX2", NULL, "SLIM RX2 MUX"},
	{"SLIM RX3", NULL, "SLIM RX3 MUX"},
	{"SLIM RX4", NULL, "SLIM RX4 MUX"},
	{"SLIM RX5", NULL, "SLIM RX5 MUX"},
	{"SLIM RX6", NULL, "SLIM RX6 MUX"},
	{"SLIM RX7", NULL, "SLIM RX7 MUX"},

	{"RX INT0_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT0_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT0_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT0_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT0_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT0_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT0_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT0_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT0_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT0_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT0_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT0_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT0_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT0_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT0_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT0_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT0_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT0_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT0_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT0_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT0_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT0_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT0_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT0_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT1_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT1_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT1_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT1_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT1_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT1_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT1_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT1_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT1_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT1_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT1_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT1_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT1_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT1_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT1_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT1_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT1_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT1_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT1_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT1_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT1_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT1_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT1_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT1_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT2_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT2_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT2_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT2_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT2_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT2_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT2_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT2_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT2_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT2_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT2_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT2_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT2_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT2_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT2_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT2_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT2_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT2_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT2_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT2_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT2_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT2_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT2_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT2_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT3_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT3_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT3_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT3_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT3_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT3_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT3_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT3_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT3_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT3_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT3_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT3_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT3_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT3_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT3_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT3_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT3_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT3_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT3_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT3_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT3_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT3_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT3_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT3_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT4_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT4_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT4_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT4_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT4_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT4_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT4_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT4_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT4_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT4_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT4_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT4_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT4_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT4_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT4_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT4_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT4_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT4_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT4_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT4_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT4_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT4_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT4_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT4_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT5_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT5_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT5_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT5_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT5_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT5_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT5_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT5_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT5_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT5_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT5_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT5_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT5_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT5_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT5_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT5_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT5_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT5_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT5_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT5_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT5_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT5_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT5_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT5_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT6_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT6_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT6_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT6_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT6_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT6_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT6_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT6_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT6_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT6_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT6_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT6_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT6_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT6_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT6_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT6_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT6_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT6_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT6_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT6_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT6_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT6_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT6_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT6_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT7_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT7_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT7_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT7_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT7_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT7_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT7_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT7_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT7_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT7_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT7_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT7_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT7_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT7_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT7_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT7_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT7_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT7_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT7_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT7_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT7_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT7_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT7_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT7_1 MIX1 INP2", "RX7", "SLIM RX7"},

	{"RX INT8_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT8_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT8_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT8_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT8_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT8_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT8_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT8_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT8_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT8_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT8_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT8_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT8_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT8_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT8_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT8_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT8_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT8_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT8_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT8_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT8_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT8_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT8_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT8_1 MIX1 INP2", "RX7", "SLIM RX7"},

	/* SRC0, SRC1 inputs to Sidetone RX Mixer
	 * on RX0, RX1, RX2, RX3, RX4 and RX7 chains
	 */
	{"IIR0", NULL, "IIR0 INP0 MUX"},
	{"IIR0 INP0 MUX", "DEC0", "ADC MUX0"},
	{"IIR0 INP0 MUX", "DEC1", "ADC MUX1"},
	{"IIR0 INP0 MUX", "DEC2", "ADC MUX2"},
	{"IIR0 INP0 MUX", "DEC3", "ADC MUX3"},
	{"IIR0 INP0 MUX", "DEC4", "ADC MUX4"},
	{"IIR0 INP0 MUX", "DEC5", "ADC MUX5"},
	{"IIR0 INP0 MUX", "DEC6", "ADC MUX6"},
	{"IIR0 INP0 MUX", "DEC7", "ADC MUX7"},
	{"IIR0 INP0 MUX", "DEC8", "ADC MUX8"},
	{"IIR0 INP0 MUX", "RX0", "SLIM RX0"},
	{"IIR0 INP0 MUX", "RX1", "SLIM RX1"},
	{"IIR0 INP0 MUX", "RX2", "SLIM RX2"},
	{"IIR0 INP0 MUX", "RX3", "SLIM RX3"},
	{"IIR0 INP0 MUX", "RX4", "SLIM RX4"},
	{"IIR0 INP0 MUX", "RX5", "SLIM RX5"},
	{"IIR0 INP0 MUX", "RX6", "SLIM RX6"},
	{"IIR0 INP0 MUX", "RX7", "SLIM RX7"},
	{"IIR0", NULL, "IIR0 INP1 MUX"},
	{"IIR0 INP1 MUX", "DEC0", "ADC MUX0"},
	{"IIR0 INP1 MUX", "DEC1", "ADC MUX1"},
	{"IIR0 INP1 MUX", "DEC2", "ADC MUX2"},
	{"IIR0 INP1 MUX", "DEC3", "ADC MUX3"},
	{"IIR0 INP1 MUX", "DEC4", "ADC MUX4"},
	{"IIR0 INP1 MUX", "DEC5", "ADC MUX5"},
	{"IIR0 INP1 MUX", "DEC6", "ADC MUX6"},
	{"IIR0 INP1 MUX", "DEC7", "ADC MUX7"},
	{"IIR0 INP1 MUX", "DEC8", "ADC MUX8"},
	{"IIR0 INP1 MUX", "RX0", "SLIM RX0"},
	{"IIR0 INP1 MUX", "RX1", "SLIM RX1"},
	{"IIR0 INP1 MUX", "RX2", "SLIM RX2"},
	{"IIR0 INP1 MUX", "RX3", "SLIM RX3"},
	{"IIR0 INP1 MUX", "RX4", "SLIM RX4"},
	{"IIR0 INP1 MUX", "RX5", "SLIM RX5"},
	{"IIR0 INP1 MUX", "RX6", "SLIM RX6"},
	{"IIR0 INP1 MUX", "RX7", "SLIM RX7"},
	{"IIR0", NULL, "IIR0 INP2 MUX"},
	{"IIR0 INP2 MUX", "DEC0", "ADC MUX0"},
	{"IIR0 INP2 MUX", "DEC1", "ADC MUX1"},
	{"IIR0 INP2 MUX", "DEC2", "ADC MUX2"},
	{"IIR0 INP2 MUX", "DEC3", "ADC MUX3"},
	{"IIR0 INP2 MUX", "DEC4", "ADC MUX4"},
	{"IIR0 INP2 MUX", "DEC5", "ADC MUX5"},
	{"IIR0 INP2 MUX", "DEC6", "ADC MUX6"},
	{"IIR0 INP2 MUX", "DEC7", "ADC MUX7"},
	{"IIR0 INP2 MUX", "DEC8", "ADC MUX8"},
	{"IIR0 INP2 MUX", "RX0", "SLIM RX0"},
	{"IIR0 INP2 MUX", "RX1", "SLIM RX1"},
	{"IIR0 INP2 MUX", "RX2", "SLIM RX2"},
	{"IIR0 INP2 MUX", "RX3", "SLIM RX3"},
	{"IIR0 INP2 MUX", "RX4", "SLIM RX4"},
	{"IIR0 INP2 MUX", "RX5", "SLIM RX5"},
	{"IIR0 INP2 MUX", "RX6", "SLIM RX6"},
	{"IIR0 INP2 MUX", "RX7", "SLIM RX7"},
	{"IIR0", NULL, "IIR0 INP3 MUX"},
	{"IIR0 INP3 MUX", "DEC0", "ADC MUX0"},
	{"IIR0 INP3 MUX", "DEC1", "ADC MUX1"},
	{"IIR0 INP3 MUX", "DEC2", "ADC MUX2"},
	{"IIR0 INP3 MUX", "DEC3", "ADC MUX3"},
	{"IIR0 INP3 MUX", "DEC4", "ADC MUX4"},
	{"IIR0 INP3 MUX", "DEC5", "ADC MUX5"},
	{"IIR0 INP3 MUX", "DEC6", "ADC MUX6"},
	{"IIR0 INP3 MUX", "DEC7", "ADC MUX7"},
	{"IIR0 INP3 MUX", "DEC8", "ADC MUX8"},
	{"IIR0 INP3 MUX", "RX0", "SLIM RX0"},
	{"IIR0 INP3 MUX", "RX1", "SLIM RX1"},
	{"IIR0 INP3 MUX", "RX2", "SLIM RX2"},
	{"IIR0 INP3 MUX", "RX3", "SLIM RX3"},
	{"IIR0 INP3 MUX", "RX4", "SLIM RX4"},
	{"IIR0 INP3 MUX", "RX5", "SLIM RX5"},
	{"IIR0 INP3 MUX", "RX6", "SLIM RX6"},
	{"IIR0 INP3 MUX", "RX7", "SLIM RX7"},

	{"IIR1", NULL, "IIR1 INP0 MUX"},
	{"IIR1 INP0 MUX", "DEC0", "ADC MUX0"},
	{"IIR1 INP0 MUX", "DEC1", "ADC MUX1"},
	{"IIR1 INP0 MUX", "DEC2", "ADC MUX2"},
	{"IIR1 INP0 MUX", "DEC3", "ADC MUX3"},
	{"IIR1 INP0 MUX", "DEC4", "ADC MUX4"},
	{"IIR1 INP0 MUX", "DEC5", "ADC MUX5"},
	{"IIR1 INP0 MUX", "DEC6", "ADC MUX6"},
	{"IIR1 INP0 MUX", "DEC7", "ADC MUX7"},
	{"IIR1 INP0 MUX", "DEC8", "ADC MUX8"},
	{"IIR1 INP0 MUX", "RX0", "SLIM RX0"},
	{"IIR1 INP0 MUX", "RX1", "SLIM RX1"},
	{"IIR1 INP0 MUX", "RX2", "SLIM RX2"},
	{"IIR1 INP0 MUX", "RX3", "SLIM RX3"},
	{"IIR1 INP0 MUX", "RX4", "SLIM RX4"},
	{"IIR1 INP0 MUX", "RX5", "SLIM RX5"},
	{"IIR1 INP0 MUX", "RX6", "SLIM RX6"},
	{"IIR1 INP0 MUX", "RX7", "SLIM RX7"},
	{"IIR1", NULL, "IIR1 INP1 MUX"},
	{"IIR1 INP1 MUX", "DEC0", "ADC MUX0"},
	{"IIR1 INP1 MUX", "DEC1", "ADC MUX1"},
	{"IIR1 INP1 MUX", "DEC2", "ADC MUX2"},
	{"IIR1 INP1 MUX", "DEC3", "ADC MUX3"},
	{"IIR1 INP1 MUX", "DEC4", "ADC MUX4"},
	{"IIR1 INP1 MUX", "DEC5", "ADC MUX5"},
	{"IIR1 INP1 MUX", "DEC6", "ADC MUX6"},
	{"IIR1 INP1 MUX", "DEC7", "ADC MUX7"},
	{"IIR1 INP1 MUX", "DEC8", "ADC MUX8"},
	{"IIR1 INP1 MUX", "RX0", "SLIM RX0"},
	{"IIR1 INP1 MUX", "RX1", "SLIM RX1"},
	{"IIR1 INP1 MUX", "RX2", "SLIM RX2"},
	{"IIR1 INP1 MUX", "RX3", "SLIM RX3"},
	{"IIR1 INP1 MUX", "RX4", "SLIM RX4"},
	{"IIR1 INP1 MUX", "RX5", "SLIM RX5"},
	{"IIR1 INP1 MUX", "RX6", "SLIM RX6"},
	{"IIR1 INP1 MUX", "RX7", "SLIM RX7"},
	{"IIR1", NULL, "IIR1 INP2 MUX"},
	{"IIR1 INP2 MUX", "DEC0", "ADC MUX0"},
	{"IIR1 INP2 MUX", "DEC1", "ADC MUX1"},
	{"IIR1 INP2 MUX", "DEC2", "ADC MUX2"},
	{"IIR1 INP2 MUX", "DEC3", "ADC MUX3"},
	{"IIR1 INP2 MUX", "DEC4", "ADC MUX4"},
	{"IIR1 INP2 MUX", "DEC5", "ADC MUX5"},
	{"IIR1 INP2 MUX", "DEC6", "ADC MUX6"},
	{"IIR1 INP2 MUX", "DEC7", "ADC MUX7"},
	{"IIR1 INP2 MUX", "DEC8", "ADC MUX8"},
	{"IIR1 INP2 MUX", "RX0", "SLIM RX0"},
	{"IIR1 INP2 MUX", "RX1", "SLIM RX1"},
	{"IIR1 INP2 MUX", "RX2", "SLIM RX2"},
	{"IIR1 INP2 MUX", "RX3", "SLIM RX3"},
	{"IIR1 INP2 MUX", "RX4", "SLIM RX4"},
	{"IIR1 INP2 MUX", "RX5", "SLIM RX5"},
	{"IIR1 INP2 MUX", "RX6", "SLIM RX6"},
	{"IIR1 INP2 MUX", "RX7", "SLIM RX7"},
	{"IIR1", NULL, "IIR1 INP3 MUX"},
	{"IIR1 INP3 MUX", "DEC0", "ADC MUX0"},
	{"IIR1 INP3 MUX", "DEC1", "ADC MUX1"},
	{"IIR1 INP3 MUX", "DEC2", "ADC MUX2"},
	{"IIR1 INP3 MUX", "DEC3", "ADC MUX3"},
	{"IIR1 INP3 MUX", "DEC4", "ADC MUX4"},
	{"IIR1 INP3 MUX", "DEC5", "ADC MUX5"},
	{"IIR1 INP3 MUX", "DEC6", "ADC MUX6"},
	{"IIR1 INP3 MUX", "DEC7", "ADC MUX7"},
	{"IIR1 INP3 MUX", "DEC8", "ADC MUX8"},
	{"IIR1 INP3 MUX", "RX0", "SLIM RX0"},
	{"IIR1 INP3 MUX", "RX1", "SLIM RX1"},
	{"IIR1 INP3 MUX", "RX2", "SLIM RX2"},
	{"IIR1 INP3 MUX", "RX3", "SLIM RX3"},
	{"IIR1 INP3 MUX", "RX4", "SLIM RX4"},
	{"IIR1 INP3 MUX", "RX5", "SLIM RX5"},
	{"IIR1 INP3 MUX", "RX6", "SLIM RX6"},
	{"IIR1 INP3 MUX", "RX7", "SLIM RX7"},

	{"SRC0", NULL, "IIR0"},
	{"SRC1", NULL, "IIR1"},
	{"RX INT0 MIX2 INP", "SRC0", "SRC0"},
	{"RX INT0 MIX2 INP", "SRC1", "SRC1"},
	{"RX INT1 MIX2 INP", "SRC0", "SRC0"},
	{"RX INT1 MIX2 INP", "SRC1", "SRC1"},
	{"RX INT2 MIX2 INP", "SRC0", "SRC0"},
	{"RX INT2 MIX2 INP", "SRC1", "SRC1"},
	{"RX INT3 MIX2 INP", "SRC0", "SRC0"},
	{"RX INT3 MIX2 INP", "SRC1", "SRC1"},
	{"RX INT4 MIX2 INP", "SRC0", "SRC0"},
	{"RX INT4 MIX2 INP", "SRC1", "SRC1"},
	{"RX INT7 MIX2 INP", "SRC0", "SRC0"},
	{"RX INT7 MIX2 INP", "SRC1", "SRC1"},
};

static const struct snd_kcontrol_new tasha_snd_controls[] = {
	SOC_SINGLE_SX_TLV("RX0 Digital Volume", WCD9335_CDC_RX0_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX1 Digital Volume", WCD9335_CDC_RX1_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX2 Digital Volume", WCD9335_CDC_RX2_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX3 Digital Volume", WCD9335_CDC_RX3_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX4 Digital Volume", WCD9335_CDC_RX4_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX5 Digital Volume", WCD9335_CDC_RX5_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX6 Digital Volume", WCD9335_CDC_RX6_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX7 Digital Volume", WCD9335_CDC_RX7_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX8 Digital Volume", WCD9335_CDC_RX8_RX_VOL_CTL,
		0, -84, 40, digital_gain),

	SOC_SINGLE_SX_TLV("DEC0 Volume", WCD9335_CDC_TX0_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC1 Volume", WCD9335_CDC_TX1_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC2 Volume", WCD9335_CDC_TX2_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC3 Volume", WCD9335_CDC_TX3_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC4 Volume", WCD9335_CDC_TX4_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC5 Volume", WCD9335_CDC_TX5_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC6 Volume", WCD9335_CDC_TX6_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC7 Volume", WCD9335_CDC_TX7_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC8 Volume", WCD9335_CDC_TX8_TX_VOL_CTL, 0,
					  -84, 40, digital_gain),

	SOC_SINGLE_SX_TLV("IIR0 INP1 Volume",
			  WCD9335_CDC_SIDETONE_IIR0_IIR_GAIN_B1_CTL, 0, -84,
			  40, digital_gain),
	SOC_SINGLE_SX_TLV("IIR0 INP2 Volume",
			  WCD9335_CDC_SIDETONE_IIR0_IIR_GAIN_B2_CTL, 0, -84,
			  40, digital_gain),
	SOC_SINGLE_SX_TLV("IIR0 INP3 Volume",
			  WCD9335_CDC_SIDETONE_IIR0_IIR_GAIN_B3_CTL, 0, -84,
			  40, digital_gain),
	SOC_SINGLE_SX_TLV("IIR0 INP4 Volume",
			  WCD9335_CDC_SIDETONE_IIR0_IIR_GAIN_B4_CTL, 0, -84,
			  40, digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP1 Volume",
			  WCD9335_CDC_SIDETONE_IIR1_IIR_GAIN_B1_CTL, 0, -84,
			  40, digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP2 Volume",
			  WCD9335_CDC_SIDETONE_IIR1_IIR_GAIN_B2_CTL, 0, -84,
			  40, digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP3 Volume",
			  WCD9335_CDC_SIDETONE_IIR1_IIR_GAIN_B2_CTL, 0, -84,
			  40, digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP4 Volume",
			  WCD9335_CDC_SIDETONE_IIR1_IIR_GAIN_B2_CTL, 0, -84,
			  40, digital_gain),

	SOC_ENUM("TX0 HPF cut off", cf_dec0_enum),
	SOC_ENUM("TX1 HPF cut off", cf_dec1_enum),
	SOC_ENUM("TX2 HPF cut off", cf_dec2_enum),
	SOC_ENUM("TX3 HPF cut off", cf_dec3_enum),
	SOC_ENUM("TX4 HPF cut off", cf_dec4_enum),
	SOC_ENUM("TX5 HPF cut off", cf_dec5_enum),
	SOC_ENUM("TX6 HPF cut off", cf_dec6_enum),
	SOC_ENUM("TX7 HPF cut off", cf_dec7_enum),
	SOC_ENUM("TX8 HPF cut off", cf_dec8_enum),

	SOC_ENUM("RX INT0_1 HPF cut off", cf_int0_1_enum),
	SOC_ENUM("RX INT1_1 HPF cut off", cf_int1_1_enum),
	SOC_ENUM("RX INT2_1 HPF cut off", cf_int2_1_enum),
	SOC_ENUM("RX INT3_1 HPF cut off", cf_int3_1_enum),
	SOC_ENUM("RX INT4_1 HPF cut off", cf_int4_1_enum),
	SOC_ENUM("RX INT5_1 HPF cut off", cf_int5_1_enum),
	SOC_ENUM("RX INT6_1 HPF cut off", cf_int6_1_enum),
	SOC_ENUM("RX INT7_1 HPF cut off", cf_int7_1_enum),
	SOC_ENUM("RX INT8_1 HPF cut off", cf_int8_1_enum),

	SOC_SINGLE_EXT("IIR0 Enable Band1", IIR0, BAND1, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR0 Enable Band2", IIR0, BAND2, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR0 Enable Band3", IIR0, BAND3, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR0 Enable Band4", IIR0, BAND4, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR0 Enable Band5", IIR0, BAND5, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band1", IIR1, BAND1, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band2", IIR1, BAND2, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band3", IIR1, BAND3, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band4", IIR1, BAND4, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band5", IIR1, BAND5, 1, 0,
	tasha_get_iir_enable_audio_mixer, tasha_put_iir_enable_audio_mixer),

	SOC_SINGLE_MULTI_EXT("IIR0 Band1", IIR0, BAND1, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR0 Band2", IIR0, BAND2, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR0 Band3", IIR0, BAND3, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR0 Band4", IIR0, BAND4, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR0 Band5", IIR0, BAND5, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band1", IIR1, BAND1, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band2", IIR1, BAND2, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band3", IIR1, BAND3, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band4", IIR1, BAND4, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band5", IIR1, BAND5, 255, 0, 5,
	tasha_get_iir_band_audio_mixer, tasha_put_iir_band_audio_mixer),

	SOC_SINGLE_EXT("COMP1 Switch", SND_SOC_NOPM, COMPANDER_1, 1, 0,
		       tasha_get_compander, tasha_set_compander),
	SOC_SINGLE_EXT("COMP2 Switch", SND_SOC_NOPM, COMPANDER_2, 1, 0,
		       tasha_get_compander, tasha_set_compander),
	SOC_SINGLE_EXT("COMP3 Switch", SND_SOC_NOPM, COMPANDER_3, 1, 0,
		       tasha_get_compander, tasha_set_compander),
	SOC_SINGLE_EXT("COMP4 Switch", SND_SOC_NOPM, COMPANDER_4, 1, 0,
		       tasha_get_compander, tasha_set_compander),
	SOC_SINGLE_EXT("COMP5 Switch", SND_SOC_NOPM, COMPANDER_5, 1, 0,
		       tasha_get_compander, tasha_set_compander),
	SOC_SINGLE_EXT("COMP6 Switch", SND_SOC_NOPM, COMPANDER_6, 1, 0,
		       tasha_get_compander, tasha_set_compander),
	SOC_SINGLE_EXT("COMP7 Switch", SND_SOC_NOPM, COMPANDER_7, 1, 0,
		       tasha_get_compander, tasha_set_compander),
	SOC_SINGLE_EXT("COMP8 Switch", SND_SOC_NOPM, COMPANDER_8, 1, 0,
		       tasha_get_compander, tasha_set_compander),
};

static int tasha_rx_int_mode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int tasha_rx_int_mode_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist =
					dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(codec);
	u32 mode_val;
	short rx_int_num;

	mode_val = ucontrol->value.enumerated.item[0];
	rx_int_num = widget->shift;

	dev_dbg(codec->dev, "%s: wname: %s, mode: %d, rx_int: %d\n",
		__func__, widget->name, mode_val, rx_int_num);

	if (rx_int_num < 0 || rx_int_num > 2) {
		dev_err(codec->dev, "mode select is not supported for:%s\n",
			widget->name);
		return -EINVAL;
	}

	tasha_p->rx_int_mode[rx_int_num] = mode_val;
	return 0;
}

static int tasha_ear_pa_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ear_pa_gain = snd_soc_read(codec, WCD9335_ANA_EAR);

	ear_pa_gain = (ear_pa_gain & 0x70) >> 4;

	ucontrol->value.integer.value[0] = ear_pa_gain;

	dev_dbg(codec->dev, "%s: ear_pa_gain = 0x%x\n", __func__,
		ear_pa_gain);

	return 0;
}

static int tasha_ear_pa_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	dev_dbg(codec->dev, "%s: ucontrol->value.integer.value[0]  = %ld\n",
			__func__, ucontrol->value.integer.value[0]);

	ear_pa_gain =  ucontrol->value.integer.value[0] << 4;

	snd_soc_update_bits(codec, WCD9335_ANA_EAR, 0x70, ear_pa_gain);
	return 0;
}

static int tasha_config_compander(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);
	const int comp = w->shift;
	u16 comp_ctl0_reg, rx_path_cfg0_reg;

	pr_debug("%s: %s event %d compander %d, enabled %d", __func__,
		 w->name, event, comp, tasha->comp_enabled[comp]);

	if (!tasha->comp_enabled[comp])
		return 0;

	comp_ctl0_reg = WCD9335_CDC_COMPANDER1_CTL0 + (comp * 8);
	rx_path_cfg0_reg = WCD9335_CDC_RX1_RX_PATH_CFG0 + (comp * 20);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enable Compander Clock */
		snd_soc_update_bits(codec, comp_ctl0_reg, 0x01, 0x01);
		snd_soc_update_bits(codec, comp_ctl0_reg, 0x02, 0x02);
		snd_soc_update_bits(codec, comp_ctl0_reg, 0x02, 0x00);
		snd_soc_update_bits(codec, rx_path_cfg0_reg, 0x02, 0x02);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, rx_path_cfg0_reg, 0x02, 0x00);
		snd_soc_update_bits(codec, comp_ctl0_reg, 0x02, 0x02);
		snd_soc_update_bits(codec, comp_ctl0_reg, 0x02, 0x00);
		snd_soc_update_bits(codec, comp_ctl0_reg, 0x01, 0x00);
		break;
	};

	return 0;
}

static const char * const tasha_ear_pa_gain_text[] = {
	"G_6_DB", "G_4P5_DB", "G_3_DB", "G_1P5_DB",
	"G_0_DB", "G_M2P5_DB", "UNDEFINED", "G_M12_DB"
};

static const struct soc_enum tasha_ear_pa_gain_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tasha_ear_pa_gain_text),
			tasha_ear_pa_gain_text);

static const struct snd_kcontrol_new tasha_analog_gain_controls[] = {
	SOC_ENUM_EXT("EAR PA Gain", tasha_ear_pa_gain_enum,
		tasha_ear_pa_gain_get, tasha_ear_pa_gain_put),

	SOC_SINGLE_TLV("HPHL Volume", WCD9335_HPH_L_EN, 0, 20, 1,
		line_gain),
	SOC_SINGLE_TLV("HPHR Volume", WCD9335_HPH_R_EN, 0, 20, 1,
		line_gain),

	SOC_SINGLE_TLV("ADC1 Volume", WCD9335_ANA_AMIC1, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", WCD9335_ANA_AMIC2, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", WCD9335_ANA_AMIC3, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC4 Volume", WCD9335_ANA_AMIC4, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC5 Volume", WCD9335_ANA_AMIC5, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC6 Volume", WCD9335_ANA_AMIC6, 0, 20, 0,
			analog_gain),
};

static const char * const rx_prim_mix_text[] = {
	"ZERO", "DEC0", "DEC1", "IIR0", "IIR1", "RX0", "RX1", "RX2",
	"RX3", "RX4", "RX5", "RX6", "RX7"
};

static const char * const rx_sidetone_mix_text[] = {
	"ZERO", "SRC0", "SRC1", "SRC_SUM"
};

static const char * const sb_tx0_mux_text[] = {
	"ZERO", "RX_MIX_TX0", "DEC0", "DEC0_192"
};

static const char * const sb_tx1_mux_text[] = {
	"ZERO", "RX_MIX_TX1", "DEC1", "DEC1_192"
};

static const char * const sb_tx2_mux_text[] = {
	"ZERO", "RX_MIX_TX2", "DEC2", "DEC2_192"
};

static const char * const sb_tx3_mux_text[] = {
	"ZERO", "RX_MIX_TX3", "DEC3", "DEC3_192"
};

static const char * const sb_tx4_mux_text[] = {
	"ZERO", "RX_MIX_TX4", "DEC4", "DEC4_192"
};

static const char * const sb_tx5_mux_text[] = {
	"ZERO", "RX_MIX_TX5", "DEC5", "DEC5_192"
};

static const char * const sb_tx6_mux_text[] = {
	"ZERO", "RX_MIX_TX6", "DEC6", "DEC6_192"
};

static const char * const sb_tx7_mux_text[] = {
	"ZERO", "RX_MIX_TX7", "DEC7", "DEC7_192"
};

static const char * const sb_tx8_mux_text[] = {
	"ZERO", "RX_MIX_TX8", "DEC8", "DEC8_192"
};

static const char * const sb_tx9_mux_text[] = {
	"ZERO", "DEC7", "DEC7_192"
};

static const char * const sb_tx10_mux_text[] = {
	"ZERO", "DEC6", "DEC6_192"
};

static const char * const iir_inp_mux_text[] = {
	"ZERO", "DEC0", "DEC1", "DEC2", "DEC3", "DEC4", "DEC5", "DEC6",
	"DEC7", "DEC8",	"RX0", "RX1", "RX2", "RX3", "RX4", "RX5", "RX6", "RX7"
};

static const char * const rx_int_mode_mux_text[] = {
	"CLS_AB", "CLS_H",
};

static const char * const rx_int0_interp_mux_text[] = {
	"ZERO", "RX INT0 MODE MUX",
};

static const char * const adc_mux_text[] = {
	"DMIC", "AMIC", "ANC_FB_TUNE1", "ANC_FB_TUNE2"
};

static const char * const dmic_mux_text[] = {
	"ZERO", "DMIC0", "DMIC1", "DMIC2", "DMIC3", "DMIC4", "DMIC5",
	"SMIC0", "SMIC1", "SMIC2", "SMIC3"
};

static const char * const dmic_mux_alt_text[] = {
	"ZERO", "DMIC0", "DMIC1", "DMIC2", "DMIC3", "DMIC4", "DMIC5",
};

static const char * const amic_mux_text[] = {
	"ZERO", "ADC1", "ADC2", "ADC3", "ADC4", "ADC5", "ADC6"
};

static const char * const rx_echo_mux_text[] = {
	"ZERO", "RX_MIX0", "RX_MIX1", "RX_MIX2", "RX_MIX3", "RX_MIX4",
	"RX_MIX5", "RX_MIX6", "RX_MIX7", "RX_MIX8", "RX_MIX_VBAT5",
	"RX_MIX_VBAT6",	"RX_MIX_VBAT7", "RX_MIX_VBAT8"
};

static const struct soc_enum rx_int0_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int0_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int0_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int1_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int1_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int1_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int2_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT2_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int2_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT2_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int2_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT2_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int3_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT3_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int3_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT3_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int3_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT3_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int4_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT4_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int4_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT4_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int4_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT4_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int5_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT5_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int5_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT5_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int5_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT5_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int6_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT6_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int6_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT6_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int6_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT6_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int7_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT7_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int7_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT7_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int7_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT7_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int8_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT8_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int8_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT8_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int8_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT8_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int0_sidetone_mix_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SIDETONE_SRC_CFG0, 0, 4,
			rx_sidetone_mix_text);

static const struct soc_enum rx_int1_sidetone_mix_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SIDETONE_SRC_CFG0, 2, 4,
			rx_sidetone_mix_text);

static const struct soc_enum rx_int2_sidetone_mix_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SIDETONE_SRC_CFG0, 4, 4,
			rx_sidetone_mix_text);

static const struct soc_enum rx_int3_sidetone_mix_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SIDETONE_SRC_CFG0, 6, 4,
			rx_sidetone_mix_text);

static const struct soc_enum rx_int4_sidetone_mix_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SIDETONE_SRC_CFG1, 0, 4,
			rx_sidetone_mix_text);

static const struct soc_enum rx_int7_sidetone_mix_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SIDETONE_SRC_CFG1, 2, 4,
			rx_sidetone_mix_text);

static const struct soc_enum tx_adc_mux0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX0_CFG1, 0, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX1_CFG1, 0, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX2_CFG1, 0, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux3_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX3_CFG1, 0, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux4_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX4_CFG0, 6, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux5_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX5_CFG0, 6, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux6_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX6_CFG0, 6, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux7_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX7_CFG0, 6, 4,
			adc_mux_text);

static const struct soc_enum tx_adc_mux8_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX8_CFG0, 6, 4,
			adc_mux_text);

static const struct soc_enum tx_dmic_mux0_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX0_CFG0, 3, 11,
			dmic_mux_text);

static const struct soc_enum tx_dmic_mux1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX1_CFG0, 3, 11,
			dmic_mux_text);

static const struct soc_enum tx_dmic_mux2_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX2_CFG0, 3, 11,
			dmic_mux_text);

static const struct soc_enum tx_dmic_mux3_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX3_CFG0, 3, 11,
			dmic_mux_text);

static const struct soc_enum tx_dmic_mux4_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX4_CFG0, 3, 7,
			dmic_mux_alt_text);

static const struct soc_enum tx_dmic_mux5_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX5_CFG0, 3, 7,
			dmic_mux_alt_text);

static const struct soc_enum tx_dmic_mux6_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX6_CFG0, 3, 7,
			dmic_mux_alt_text);

static const struct soc_enum tx_dmic_mux7_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX7_CFG0, 3, 7,
			dmic_mux_alt_text);

static const struct soc_enum tx_dmic_mux8_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX8_CFG0, 3, 7,
			dmic_mux_alt_text);

static const struct soc_enum tx_amic_mux0_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX0_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX1_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux2_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX2_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux3_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX3_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux4_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX4_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux5_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX5_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux6_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX6_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux7_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX7_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum tx_amic_mux8_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_TX_INP_MUX_ADC_MUX8_CFG0, 0, 7,
			amic_mux_text);

static const struct soc_enum sb_tx0_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG0, 0, 4,
			sb_tx0_mux_text);

static const struct soc_enum sb_tx1_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG0, 2, 4,
			sb_tx1_mux_text);

static const struct soc_enum sb_tx2_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG0, 4, 4,
			sb_tx2_mux_text);

static const struct soc_enum sb_tx3_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG0, 6, 4,
			sb_tx3_mux_text);

static const struct soc_enum sb_tx4_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG1, 0, 4,
			sb_tx4_mux_text);

static const struct soc_enum sb_tx5_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG1, 2, 4,
			sb_tx5_mux_text);

static const struct soc_enum sb_tx6_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG1, 4, 4,
			sb_tx6_mux_text);

static const struct soc_enum sb_tx7_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG1, 6, 4,
			sb_tx7_mux_text);

static const struct soc_enum sb_tx8_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG2, 0, 4,
			sb_tx8_mux_text);

static const struct soc_enum sb_tx9_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG2, 2, 3,
			sb_tx9_mux_text);

static const struct soc_enum sb_tx10_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_IF_ROUTER_TX_MUX_CFG2, 4, 3,
			sb_tx10_mux_text);

static const struct soc_enum rx_mix_tx0_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG0, 0, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx1_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG0, 4, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx2_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG1, 0, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx3_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG1, 4, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx4_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG2, 0, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx5_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG2, 4, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx6_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG3, 0, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx7_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG3, 4, 14,
			rx_echo_mux_text);

static const struct soc_enum rx_mix_tx8_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_MIX_CFG4, 0, 14,
			rx_echo_mux_text);

static const struct soc_enum iir0_inp0_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR0_MIX_CFG0, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum iir0_inp1_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR0_MIX_CFG1, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum iir0_inp2_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR0_MIX_CFG2, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum iir0_inp3_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR0_MIX_CFG3, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum iir1_inp0_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR1_MIX_CFG0, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum iir1_inp1_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR1_MIX_CFG1, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum iir1_inp2_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR1_MIX_CFG2, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum iir1_inp3_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_SIDETONE_IIR_INP_MUX_IIR1_MIX_CFG3, 0, 18,
			iir_inp_mux_text);

static const struct soc_enum rx_int0_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX0_RX_PATH_CTL, 5, 2,
			rx_int0_interp_mux_text);

static const struct snd_kcontrol_new rx_int0_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT0_1 MIX1 INP0 Mux", rx_int0_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int0_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT0_1 MIX1 INP1 Mux", rx_int0_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int0_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT0_1 MIX1 INP2 Mux", rx_int0_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int1_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT1_1 MIX1 INP0 Mux", rx_int1_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int1_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT1_1 MIX1 INP1 Mux", rx_int1_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int1_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT1_1 MIX1 INP2 Mux", rx_int1_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int2_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT2_1 MIX1 INP0 Mux", rx_int2_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int2_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT2_1 MIX1 INP1 Mux", rx_int2_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int2_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT2_1 MIX1 INP2 Mux", rx_int2_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int3_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT3_1 MIX1 INP0 Mux", rx_int3_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int3_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT3_1 MIX1 INP1 Mux", rx_int3_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int3_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT3_1 MIX1 INP2 Mux", rx_int3_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int4_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT4_1 MIX1 INP0 Mux", rx_int4_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int4_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT4_1 MIX1 INP1 Mux", rx_int4_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int4_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT4_1 MIX1 INP2 Mux", rx_int4_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int5_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT5_1 MIX1 INP0 Mux", rx_int5_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int5_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT5_1 MIX1 INP1 Mux", rx_int5_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int5_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT5_1 MIX1 INP2 Mux", rx_int5_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int6_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT6_1 MIX1 INP0 Mux", rx_int6_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int6_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT6_1 MIX1 INP1 Mux", rx_int6_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int6_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT6_1 MIX1 INP2 Mux", rx_int6_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int7_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT7_1 MIX1 INP0 Mux", rx_int7_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int7_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT7_1 MIX1 INP1 Mux", rx_int7_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int7_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT7_1 MIX1 INP2 Mux", rx_int7_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int8_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT8_1 MIX1 INP0 Mux", rx_int8_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int8_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT8_1 MIX1 INP1 Mux", rx_int8_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int8_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT8_1 MIX1 INP2 Mux", rx_int8_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int0_mix2_inp_mux =
	SOC_DAPM_ENUM("RX INT0 MIX2 INP Mux", rx_int0_sidetone_mix_chain_enum);

static const struct snd_kcontrol_new rx_int1_mix2_inp_mux =
	SOC_DAPM_ENUM("RX INT1 MIX2 INP Mux", rx_int1_sidetone_mix_chain_enum);

static const struct snd_kcontrol_new rx_int2_mix2_inp_mux =
	SOC_DAPM_ENUM("RX INT2 MIX2 INP Mux", rx_int2_sidetone_mix_chain_enum);

static const struct snd_kcontrol_new rx_int3_mix2_inp_mux =
	SOC_DAPM_ENUM("RX INT3 MIX2 INP Mux", rx_int3_sidetone_mix_chain_enum);

static const struct snd_kcontrol_new rx_int4_mix2_inp_mux =
	SOC_DAPM_ENUM("RX INT4 MIX2 INP Mux", rx_int4_sidetone_mix_chain_enum);

static const struct snd_kcontrol_new rx_int7_mix2_inp_mux =
	SOC_DAPM_ENUM("RX INT7 MIX2 INP Mux", rx_int7_sidetone_mix_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux0 =
	SOC_DAPM_ENUM("ADC MUX0 Mux", tx_adc_mux0_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux1 =
	SOC_DAPM_ENUM("ADC MUX1 Mux", tx_adc_mux1_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux2 =
	SOC_DAPM_ENUM("ADC MUX2 Mux", tx_adc_mux2_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux3 =
	SOC_DAPM_ENUM("ADC MUX3 Mux", tx_adc_mux3_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux4 =
	SOC_DAPM_ENUM("ADC MUX4 Mux", tx_adc_mux4_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux5 =
	SOC_DAPM_ENUM("ADC MUX5 Mux", tx_adc_mux5_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux6 =
	SOC_DAPM_ENUM("ADC MUX6 Mux", tx_adc_mux6_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux7 =
	SOC_DAPM_ENUM("ADC MUX7 Mux", tx_adc_mux7_chain_enum);

static const struct snd_kcontrol_new tx_adc_mux8 =
	SOC_DAPM_ENUM("ADC MUX8 Mux", tx_adc_mux8_chain_enum);

static const struct snd_kcontrol_new tx_dmic_mux0 =
	SOC_DAPM_ENUM("DMIC MUX0 Mux", tx_dmic_mux0_enum);

static const struct snd_kcontrol_new tx_dmic_mux1 =
	SOC_DAPM_ENUM("DMIC MUX1 Mux", tx_dmic_mux1_enum);

static const struct snd_kcontrol_new tx_dmic_mux2 =
	SOC_DAPM_ENUM("DMIC MUX2 Mux", tx_dmic_mux2_enum);

static const struct snd_kcontrol_new tx_dmic_mux3 =
	SOC_DAPM_ENUM("DMIC MUX3 Mux", tx_dmic_mux3_enum);

static const struct snd_kcontrol_new tx_dmic_mux4 =
	SOC_DAPM_ENUM("DMIC MUX4 Mux", tx_dmic_mux4_enum);

static const struct snd_kcontrol_new tx_dmic_mux5 =
	SOC_DAPM_ENUM("DMIC MUX5 Mux", tx_dmic_mux5_enum);

static const struct snd_kcontrol_new tx_dmic_mux6 =
	SOC_DAPM_ENUM("DMIC MUX6 Mux", tx_dmic_mux6_enum);

static const struct snd_kcontrol_new tx_dmic_mux7 =
	SOC_DAPM_ENUM("DMIC MUX7 Mux", tx_dmic_mux7_enum);

static const struct snd_kcontrol_new tx_dmic_mux8 =
	SOC_DAPM_ENUM("DMIC MUX8 Mux", tx_dmic_mux8_enum);

static const struct snd_kcontrol_new tx_amic_mux0 =
	SOC_DAPM_ENUM("AMIC MUX0 Mux", tx_amic_mux0_enum);

static const struct snd_kcontrol_new tx_amic_mux1 =
	SOC_DAPM_ENUM("AMIC MUX1 Mux", tx_amic_mux1_enum);

static const struct snd_kcontrol_new tx_amic_mux2 =
	SOC_DAPM_ENUM("AMIC MUX2 Mux", tx_amic_mux2_enum);

static const struct snd_kcontrol_new tx_amic_mux3 =
	SOC_DAPM_ENUM("AMIC MUX3 Mux", tx_amic_mux3_enum);

static const struct snd_kcontrol_new tx_amic_mux4 =
	SOC_DAPM_ENUM("AMIC MUX4 Mux", tx_amic_mux4_enum);

static const struct snd_kcontrol_new tx_amic_mux5 =
	SOC_DAPM_ENUM("AMIC MUX5 Mux", tx_amic_mux5_enum);

static const struct snd_kcontrol_new tx_amic_mux6 =
	SOC_DAPM_ENUM("AMIC MUX6 Mux", tx_amic_mux6_enum);

static const struct snd_kcontrol_new tx_amic_mux7 =
	SOC_DAPM_ENUM("AMIC MUX7 Mux", tx_amic_mux7_enum);

static const struct snd_kcontrol_new tx_amic_mux8 =
	SOC_DAPM_ENUM("AMIC MUX8 Mux", tx_amic_mux8_enum);

static const struct snd_kcontrol_new sb_tx0_mux =
	SOC_DAPM_ENUM("SLIM TX0 MUX Mux", sb_tx0_mux_enum);

static const struct snd_kcontrol_new sb_tx1_mux =
	SOC_DAPM_ENUM("SLIM TX1 MUX Mux", sb_tx1_mux_enum);

static const struct snd_kcontrol_new sb_tx2_mux =
	SOC_DAPM_ENUM("SLIM TX2 MUX Mux", sb_tx2_mux_enum);

static const struct snd_kcontrol_new sb_tx3_mux =
	SOC_DAPM_ENUM("SLIM TX3 MUX Mux", sb_tx3_mux_enum);

static const struct snd_kcontrol_new sb_tx4_mux =
	SOC_DAPM_ENUM("SLIM TX4 MUX Mux", sb_tx4_mux_enum);

static const struct snd_kcontrol_new sb_tx5_mux =
	SOC_DAPM_ENUM("SLIM TX5 MUX Mux", sb_tx5_mux_enum);

static const struct snd_kcontrol_new sb_tx6_mux =
	SOC_DAPM_ENUM("SLIM TX6 MUX Mux", sb_tx6_mux_enum);

static const struct snd_kcontrol_new sb_tx7_mux =
	SOC_DAPM_ENUM("SLIM TX7 MUX Mux", sb_tx7_mux_enum);

static const struct snd_kcontrol_new sb_tx8_mux =
	SOC_DAPM_ENUM("SLIM TX8 MUX Mux", sb_tx8_mux_enum);

static const struct snd_kcontrol_new sb_tx9_mux =
	SOC_DAPM_ENUM("SLIM TX9 MUX Mux", sb_tx9_mux_enum);

static const struct snd_kcontrol_new sb_tx10_mux =
	SOC_DAPM_ENUM("SLIM TX10 MUX Mux", sb_tx10_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx0_mux =
	SOC_DAPM_ENUM("RX MIX TX0 MUX Mux", rx_mix_tx0_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx1_mux =
	SOC_DAPM_ENUM("RX MIX TX1 MUX Mux", rx_mix_tx1_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx2_mux =
	SOC_DAPM_ENUM("RX MIX TX2 MUX Mux", rx_mix_tx2_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx3_mux =
	SOC_DAPM_ENUM("RX MIX TX3 MUX Mux", rx_mix_tx3_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx4_mux =
	SOC_DAPM_ENUM("RX MIX TX4 MUX Mux", rx_mix_tx4_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx5_mux =
	SOC_DAPM_ENUM("RX MIX TX5 MUX Mux", rx_mix_tx5_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx6_mux =
	SOC_DAPM_ENUM("RX MIX TX6 MUX Mux", rx_mix_tx6_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx7_mux =
	SOC_DAPM_ENUM("RX MIX TX7 MUX Mux", rx_mix_tx7_mux_enum);

static const struct snd_kcontrol_new rx_mix_tx8_mux =
	SOC_DAPM_ENUM("RX MIX TX8 MUX Mux", rx_mix_tx8_mux_enum);

static const struct snd_kcontrol_new iir0_inp0_mux =
	SOC_DAPM_ENUM("IIR0 INP0 Mux", iir0_inp0_mux_enum);

static const struct snd_kcontrol_new iir0_inp1_mux =
	SOC_DAPM_ENUM("IIR0 INP1 Mux", iir0_inp1_mux_enum);

static const struct snd_kcontrol_new iir0_inp2_mux =
	SOC_DAPM_ENUM("IIR0 INP2 Mux", iir0_inp2_mux_enum);

static const struct snd_kcontrol_new iir0_inp3_mux =
	SOC_DAPM_ENUM("IIR0 INP3 Mux", iir0_inp3_mux_enum);

static const struct snd_kcontrol_new iir1_inp0_mux =
	SOC_DAPM_ENUM("IIR1 INP0 Mux", iir1_inp0_mux_enum);

static const struct snd_kcontrol_new iir1_inp1_mux =
	SOC_DAPM_ENUM("IIR1 INP1 Mux", iir1_inp1_mux_enum);

static const struct snd_kcontrol_new iir1_inp2_mux =
	SOC_DAPM_ENUM("IIR1 INP2 Mux", iir1_inp2_mux_enum);

static const struct snd_kcontrol_new iir1_inp3_mux =
	SOC_DAPM_ENUM("IIR1 INP3 Mux", iir1_inp3_mux_enum);

static const struct soc_enum rx_int_mode_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rx_int_mode_mux_text),
			    rx_int_mode_mux_text);

static const struct snd_kcontrol_new rx_int_mode_mux[] = {
	SOC_DAPM_ENUM_EXT("RX INT0 MODE MUX Mux", rx_int_mode_mux_enum,
			tasha_rx_int_mode_get, tasha_rx_int_mode_put),
	SOC_DAPM_ENUM_EXT("RX INT1 MODE MUX Mux", rx_int_mode_mux_enum,
			tasha_rx_int_mode_get, tasha_rx_int_mode_put),
	SOC_DAPM_ENUM_EXT("RX INT2 MODE MUX Mux", rx_int_mode_mux_enum,
			tasha_rx_int_mode_get, tasha_rx_int_mode_put),
};

static const struct snd_kcontrol_new rx_int0_interp_mux =
	SOC_DAPM_ENUM("RX INT0 INTERP Mux", rx_int0_interp_mux_enum);

static const struct snd_soc_dapm_widget tasha_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("EAR"),
	SND_SOC_DAPM_AIF_IN_E("AIF1 PB", "AIF1 Playback", 0, SND_SOC_NOPM,
				AIF1_PB, 0, tasha_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("AIF2 PB", "AIF2 Playback", 0, SND_SOC_NOPM,
				AIF2_PB, 0, tasha_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("AIF3 PB", "AIF3 Playback", 0, SND_SOC_NOPM,
				AIF3_PB, 0, tasha_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("SLIM RX0 MUX", SND_SOC_NOPM, TASHA_RX0, 0,
				&slim_rx_mux[TASHA_RX0]),
	SND_SOC_DAPM_MUX("SLIM RX1 MUX", SND_SOC_NOPM, TASHA_RX1, 0,
				&slim_rx_mux[TASHA_RX1]),
	SND_SOC_DAPM_MUX("SLIM RX2 MUX", SND_SOC_NOPM, TASHA_RX2, 0,
				&slim_rx_mux[TASHA_RX2]),
	SND_SOC_DAPM_MUX("SLIM RX3 MUX", SND_SOC_NOPM, TASHA_RX3, 0,
				&slim_rx_mux[TASHA_RX3]),
	SND_SOC_DAPM_MUX("SLIM RX4 MUX", SND_SOC_NOPM, TASHA_RX4, 0,
				&slim_rx_mux[TASHA_RX4]),
	SND_SOC_DAPM_MUX("SLIM RX5 MUX", SND_SOC_NOPM, TASHA_RX5, 0,
				&slim_rx_mux[TASHA_RX5]),
	SND_SOC_DAPM_MUX("SLIM RX6 MUX", SND_SOC_NOPM, TASHA_RX6, 0,
				&slim_rx_mux[TASHA_RX6]),
	SND_SOC_DAPM_MUX("SLIM RX7 MUX", SND_SOC_NOPM, TASHA_RX7, 0,
				&slim_rx_mux[TASHA_RX7]),

	SND_SOC_DAPM_MIXER("SLIM RX0", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX3", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX4", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX5", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX6", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX7", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX INT0_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int0_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT0_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int0_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT0_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int0_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT1_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int1_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT1_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int1_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT1_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int1_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT2_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int2_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT2_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int2_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT2_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int2_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT3_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int3_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT3_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int3_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT3_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int3_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT4_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int4_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT4_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int4_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT4_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int4_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT5_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int5_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT5_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int5_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT5_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int5_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT6_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int6_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT6_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int6_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT6_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int6_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT7_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int7_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT7_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int7_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT7_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int7_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT8_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int8_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT8_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int8_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT8_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int8_1_mix_inp2_mux),

	SND_SOC_DAPM_MIXER("RX INT0_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT1_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT2_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT3_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT4_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT5_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT6_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT7_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT8_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("RX INT0 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT1 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT2 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT3 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT4 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER_E("RX INT7 MIX2", WCD9335_CDC_RX7_RX_PATH_CTL,
		5, 0, NULL, 0, tasha_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER("RX INT7 CHAIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT8 CHAIN", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX INT0 MIX2 INP", SND_SOC_NOPM, 0, 0,
		&rx_int0_mix2_inp_mux),
	SND_SOC_DAPM_MUX("RX INT1 MIX2 INP", SND_SOC_NOPM, 0, 0,
		&rx_int1_mix2_inp_mux),
	SND_SOC_DAPM_MUX("RX INT2 MIX2 INP", SND_SOC_NOPM, 0, 0,
		&rx_int2_mix2_inp_mux),
	SND_SOC_DAPM_MUX("RX INT3 MIX2 INP", SND_SOC_NOPM, 0, 0,
		&rx_int3_mix2_inp_mux),
	SND_SOC_DAPM_MUX("RX INT4 MIX2 INP", SND_SOC_NOPM, 0, 0,
		&rx_int4_mix2_inp_mux),
	SND_SOC_DAPM_MUX("RX INT7 MIX2 INP", SND_SOC_NOPM, 0, 0,
		&rx_int7_mix2_inp_mux),

	SND_SOC_DAPM_MUX("SLIM TX0 MUX", SND_SOC_NOPM, TASHA_TX0, 0,
		&sb_tx0_mux),
	SND_SOC_DAPM_MUX("SLIM TX1 MUX", SND_SOC_NOPM, TASHA_TX1, 0,
		&sb_tx1_mux),
	SND_SOC_DAPM_MUX("SLIM TX2 MUX", SND_SOC_NOPM, TASHA_TX2, 0,
		&sb_tx2_mux),
	SND_SOC_DAPM_MUX("SLIM TX3 MUX", SND_SOC_NOPM, TASHA_TX3, 0,
		&sb_tx3_mux),
	SND_SOC_DAPM_MUX("SLIM TX4 MUX", SND_SOC_NOPM, TASHA_TX4, 0,
		&sb_tx4_mux),
	SND_SOC_DAPM_MUX("SLIM TX5 MUX", SND_SOC_NOPM, TASHA_TX5, 0,
		&sb_tx5_mux),
	SND_SOC_DAPM_MUX("SLIM TX6 MUX", SND_SOC_NOPM, TASHA_TX6, 0,
		&sb_tx6_mux),
	SND_SOC_DAPM_MUX("SLIM TX7 MUX", SND_SOC_NOPM, TASHA_TX7, 0,
		&sb_tx7_mux),
	SND_SOC_DAPM_MUX("SLIM TX8 MUX", SND_SOC_NOPM, TASHA_TX8, 0,
		&sb_tx8_mux),
	SND_SOC_DAPM_MUX("SLIM TX9 MUX", SND_SOC_NOPM, TASHA_TX9, 0,
		&sb_tx9_mux),
	SND_SOC_DAPM_MUX("SLIM TX10 MUX", SND_SOC_NOPM, TASHA_TX10, 0,
		&sb_tx10_mux),

	SND_SOC_DAPM_MUX_E("ADC MUX0", WCD9335_CDC_TX0_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux0, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX1", WCD9335_CDC_TX1_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux1, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX2", WCD9335_CDC_TX2_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux2, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX3", WCD9335_CDC_TX3_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux3, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX4", WCD9335_CDC_TX4_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux4, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX5", WCD9335_CDC_TX5_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux5, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX6", WCD9335_CDC_TX6_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux6, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX7", WCD9335_CDC_TX7_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux7, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("ADC MUX8", WCD9335_CDC_TX8_TX_PATH_CTL, 5, 0,
			   &tx_adc_mux8, tasha_codec_enable_dec,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("DMIC MUX0", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux0),
	SND_SOC_DAPM_MUX("DMIC MUX1", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux1),
	SND_SOC_DAPM_MUX("DMIC MUX2", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux2),
	SND_SOC_DAPM_MUX("DMIC MUX3", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux3),
	SND_SOC_DAPM_MUX("DMIC MUX4", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux4),
	SND_SOC_DAPM_MUX("DMIC MUX5", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux5),
	SND_SOC_DAPM_MUX("DMIC MUX6", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux6),
	SND_SOC_DAPM_MUX("DMIC MUX7", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux7),
	SND_SOC_DAPM_MUX("DMIC MUX8", SND_SOC_NOPM, 0, 0,
		&tx_dmic_mux8),

	SND_SOC_DAPM_MUX("AMIC MUX0", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux0),
	SND_SOC_DAPM_MUX("AMIC MUX1", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux1),
	SND_SOC_DAPM_MUX("AMIC MUX2", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux2),
	SND_SOC_DAPM_MUX("AMIC MUX3", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux3),
	SND_SOC_DAPM_MUX("AMIC MUX4", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux4),
	SND_SOC_DAPM_MUX("AMIC MUX5", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux5),
	SND_SOC_DAPM_MUX("AMIC MUX6", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux6),
	SND_SOC_DAPM_MUX("AMIC MUX7", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux7),
	SND_SOC_DAPM_MUX("AMIC MUX8", SND_SOC_NOPM, 0, 0,
		&tx_amic_mux8),

	SND_SOC_DAPM_ADC("ADC1", NULL, WCD9335_ANA_AMIC1, 7, 0),
	SND_SOC_DAPM_ADC("ADC2", NULL, WCD9335_ANA_AMIC2, 7, 0),
	SND_SOC_DAPM_ADC("ADC3", NULL, WCD9335_ANA_AMIC3, 7, 0),
	SND_SOC_DAPM_ADC("ADC4", NULL, WCD9335_ANA_AMIC4, 7, 0),
	SND_SOC_DAPM_ADC("ADC5", NULL, WCD9335_ANA_AMIC5, 7, 0),
	SND_SOC_DAPM_ADC("ADC6", NULL, WCD9335_ANA_AMIC6, 7, 0),

	SND_SOC_DAPM_SUPPLY("ANA_BIAS", SND_SOC_NOPM, 7, 0,
			    tasha_codec_enable_ana_bias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("ANA_CLK_TOP", WCD9335_ANA_CLK_TOP, 6, 0,
			    tasha_codec_enable_ana_clk,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("COMP1_CLK", SND_SOC_NOPM, COMPANDER_1, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("COMP2_CLK", SND_SOC_NOPM, COMPANDER_2, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("COMP3_CLK", SND_SOC_NOPM, COMPANDER_3, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("COMP4_CLK", SND_SOC_NOPM, COMPANDER_4, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("COMP5_CLK", SND_SOC_NOPM, COMPANDER_5, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("COMP6_CLK", SND_SOC_NOPM, COMPANDER_6, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("COMP7_CLK", SND_SOC_NOPM, COMPANDER_7, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("COMP8_CLK", SND_SOC_NOPM, COMPANDER_8, 0,
			    tasha_config_compander, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS1 External", WCD9335_ANA_MICB1, 7, 0,
			       tasha_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS2 External", WCD9335_ANA_MICB2, 7, 0,
			       tasha_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS3 External", WCD9335_ANA_MICB3, 7, 0,
			       tasha_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS4 External", WCD9335_ANA_MICB4, 7, 0,
			       tasha_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_INPUT("AMIC3"),
	SND_SOC_DAPM_INPUT("AMIC4"),
	SND_SOC_DAPM_INPUT("AMIC5"),
	SND_SOC_DAPM_INPUT("AMIC6"),

	SND_SOC_DAPM_AIF_OUT_E("AIF1 CAP", "AIF1 Capture", 0, SND_SOC_NOPM,
		AIF1_CAP, 0, tasha_codec_enable_slimtx,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("AIF2 CAP", "AIF2 Capture", 0, SND_SOC_NOPM,
		AIF2_CAP, 0, tasha_codec_enable_slimtx,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("AIF3 CAP", "AIF3 Capture", 0, SND_SOC_NOPM,
		AIF3_CAP, 0, tasha_codec_enable_slimtx,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("AIF1_CAP Mixer", SND_SOC_NOPM, AIF1_CAP, 0,
		aif1_cap_mixer, ARRAY_SIZE(aif1_cap_mixer)),

	SND_SOC_DAPM_MIXER("AIF2_CAP Mixer", SND_SOC_NOPM, AIF2_CAP, 0,
		aif2_cap_mixer, ARRAY_SIZE(aif2_cap_mixer)),

	SND_SOC_DAPM_MIXER("AIF3_CAP Mixer", SND_SOC_NOPM, AIF3_CAP, 0,
		aif3_cap_mixer, ARRAY_SIZE(aif3_cap_mixer)),

	/* Digital Mic Inputs */
	SND_SOC_DAPM_ADC_E("DMIC0", NULL, SND_SOC_NOPM, 0, 0,
		tasha_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 0, 0,
		tasha_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 0, 0,
		tasha_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC3", NULL, SND_SOC_NOPM, 0, 0,
		tasha_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC4", NULL, SND_SOC_NOPM, 0, 0,
		tasha_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC5", NULL, SND_SOC_NOPM, 0, 0,
		tasha_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR0 INP0 MUX", SND_SOC_NOPM, 0, 0,
		&iir0_inp0_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR0 INP1 MUX", SND_SOC_NOPM, 0, 0,
		&iir0_inp1_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR0 INP2 MUX", SND_SOC_NOPM, 0, 0,
		&iir0_inp2_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR0 INP3 MUX", SND_SOC_NOPM, 0, 0,
		&iir0_inp3_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR1 INP0 MUX", SND_SOC_NOPM, 0, 0,
		&iir1_inp0_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR1 INP1 MUX", SND_SOC_NOPM, 0, 0,
		&iir1_inp1_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR1 INP2 MUX", SND_SOC_NOPM, 0, 0,
		&iir1_inp2_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("IIR1 INP3 MUX", SND_SOC_NOPM, 0, 0,
		&iir1_inp3_mux, tasha_codec_iir_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("IIR0", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("IIR1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SRC0", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SRC1", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX MIX TX0 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx0_mux),
	SND_SOC_DAPM_MUX("RX MIX TX1 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx1_mux),
	SND_SOC_DAPM_MUX("RX MIX TX2 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx2_mux),
	SND_SOC_DAPM_MUX("RX MIX TX3 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx3_mux),
	SND_SOC_DAPM_MUX("RX MIX TX4 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx4_mux),
	SND_SOC_DAPM_MUX("RX MIX TX5 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx5_mux),
	SND_SOC_DAPM_MUX("RX MIX TX6 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx6_mux),
	SND_SOC_DAPM_MUX("RX MIX TX7 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx7_mux),
	SND_SOC_DAPM_MUX("RX MIX TX8 MUX", SND_SOC_NOPM, 0, 0,
		&rx_mix_tx8_mux),

	SND_SOC_DAPM_VIRT_MUX_E("RX INT0 MODE MUX",
		SND_SOC_NOPM, 0, 0, &rx_int_mode_mux[0],
		tasha_interpolator_mode_select,
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT0 INTERP", WCD9335_CDC_RX0_RX_PATH_CTL,
		5, 0, &rx_int0_interp_mux, tasha_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_DAC_E("RX INT0 DAC", NULL, SND_SOC_NOPM,
		0, 0, tasha_codec_ear_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("EAR PA", WCD9335_ANA_EAR, 7, 0, NULL, 0,
			   tasha_codec_enable_ear_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0,
		tasha_codec_enable_rx_bias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("SPK1 OUT"),
	SND_SOC_DAPM_OUTPUT("SPK2 OUT"),
};

static int tasha_get_channel_map(struct snd_soc_dai *dai,
				 unsigned int *tx_num, unsigned int *tx_slot,
				 unsigned int *rx_num, unsigned int *rx_slot)
{
	struct tasha_priv *tasha_p = snd_soc_codec_get_drvdata(dai->codec);
	u32 i = 0;
	struct wcd9xxx_ch *ch;

	switch (dai->id) {
	case AIF1_PB:
	case AIF2_PB:
	case AIF3_PB:
		if (!rx_slot || !rx_num) {
			pr_err("%s: Invalid rx_slot %p or rx_num %p\n",
				 __func__, rx_slot, rx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &tasha_p->dai[dai->id].wcd9xxx_ch_list,
				    list) {
			pr_debug("%s: slot_num %u ch->ch_num %d\n",
				 __func__, i, ch->ch_num);
			rx_slot[i++] = ch->ch_num;
		}
		pr_debug("%s: rx_num %d\n", __func__, i);
		*rx_num = i;
		break;
	case AIF1_CAP:
	case AIF2_CAP:
	case AIF3_CAP:
		if (!tx_slot || !tx_num) {
			pr_err("%s: Invalid tx_slot %p or tx_num %p\n",
				 __func__, tx_slot, tx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &tasha_p->dai[dai->id].wcd9xxx_ch_list,
				    list) {
			pr_debug("%s: slot_num %u ch->ch_num %d\n",
				 __func__, i,  ch->ch_num);
			tx_slot[i++] = ch->ch_num;
		}
		pr_debug("%s: tx_num %d\n", __func__, i);
		*tx_num = i;
		break;

	default:
		pr_err("%s: Invalid DAI ID %x\n", __func__, dai->id);
		break;
	}

	return 0;
}

static int tasha_set_channel_map(struct snd_soc_dai *dai,
				 unsigned int tx_num, unsigned int *tx_slot,
				 unsigned int rx_num, unsigned int *rx_slot)
{
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(dai->codec);
	struct wcd9xxx *core = dev_get_drvdata(dai->codec->dev->parent);
	if (!tx_slot || !rx_slot) {
		pr_err("%s: Invalid tx_slot=%p, rx_slot=%p\n",
			__func__, tx_slot, rx_slot);
		return -EINVAL;
	}
	pr_debug("%s(): dai_name = %s DAI-ID %x tx_ch %d rx_ch %d\n"
		 "tasha->intf_type %d\n",
		 __func__, dai->name, dai->id, tx_num, rx_num,
		 tasha->intf_type);

	if (tasha->intf_type == WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
		wcd9xxx_init_slimslave(core, core->slim->laddr,
					   tx_num, tx_slot, rx_num, rx_slot);
	}
	return 0;
}

static int tasha_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pr_debug("%s(): substream = %s  stream = %d\n" , __func__,
		 substream->name, substream->stream);

	return 0;
}

static void tasha_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pr_debug("%s(): substream = %s  stream = %d\n" , __func__,
		 substream->name, substream->stream);
}

static int tasha_set_interpolator_rate(struct snd_soc_dai *dai,
				       u8 int_prim_fs_rate_reg_val,
				       u32 sample_rate)
{
	u8 int_1_mix1_inp;
	u32 j;
	u16 int_mux_cfg0, int_mux_cfg1;
	u16 int_fs_reg;
	u8 int_mux_cfg0_val, int_mux_cfg1_val;
	struct snd_soc_codec *codec = dai->codec;
	struct wcd9xxx_ch *ch;
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);

	list_for_each_entry(ch, &tasha->dai[dai->id].wcd9xxx_ch_list, list) {
		int_1_mix1_inp = ch->port + INTn_1_MIX_INP_SEL_RX0 -
				  TASHA_RX_PORT_START_NUMBER;
		if ((int_1_mix1_inp < INTn_1_MIX_INP_SEL_RX0) ||
		   (int_1_mix1_inp > INTn_1_MIX_INP_SEL_RX7)) {
			pr_err("%s: Invalid RX%u port, Dai ID is %d\n",
				__func__,
				(ch->port - TASHA_RX_PORT_START_NUMBER),
				dai->id);
			return -EINVAL;
		}

		int_mux_cfg0 = WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG0;

		/* Loop through all interpolator MUX inputs and find out
		 * to which interpolator input, the slim rx port
		 * is connected
		 */
		for (j = 0; j < TASHA_NUM_INTERPOLATORS; j++) {
			int_mux_cfg1 = int_mux_cfg0 + 1;

			int_mux_cfg0_val = snd_soc_read(codec, int_mux_cfg0);
			int_mux_cfg1_val = snd_soc_read(codec, int_mux_cfg1);

			if (((int_mux_cfg0_val & 0x0F) == int_1_mix1_inp) ||
			(((int_mux_cfg0_val >> 4) & 0x0F) ==
			 int_1_mix1_inp) ||
			(((int_mux_cfg1_val >> 4) & 0x0F) ==
			 int_1_mix1_inp)) {
				int_fs_reg = WCD9335_CDC_RX0_RX_PATH_CTL +
					     20 * j;
				pr_debug("%s: AIF_PB DAI(%d) connected to INT%u_1\n",
					  __func__, dai->id, j);
				pr_debug("%s: set INT%u_1 sample rate to %u\n",
					__func__, j, sample_rate);
				if ((j == 0) && (sample_rate == 44100)) {
					pr_info("%s: Cannot set 44.1KHz on INT0\n",
						__func__);
				} else
					snd_soc_update_bits(codec, int_fs_reg,
						0x0F, int_prim_fs_rate_reg_val);
			}
			int_mux_cfg0 += 2;
		}
	}

	return 0;
}

static int tasha_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(dai->codec);
	int ret;
	int int_prim_fs_rate_val = -EINVAL;

	pr_err("%s: dai_name = %s DAI-ID %x rate %d num_ch %d\n", __func__,
		 dai->name, dai->id, params_rate(params),
		 params_channels(params));

	switch (params_rate(params)) {
	case 8000:
		int_prim_fs_rate_val = 0x00;
		break;
	case 16000:
		int_prim_fs_rate_val = 0x01;
		break;
	case 32000:
		int_prim_fs_rate_val = 0x03;
		break;
	case 48000:
		int_prim_fs_rate_val = 0x04;
		break;
	case 96000:
		int_prim_fs_rate_val = 0x05;
		break;
	case 192000:
		int_prim_fs_rate_val = 0x06;
		break;
	case 384000:
		int_prim_fs_rate_val = 0x07;
		break;
	case 44100:
		int_prim_fs_rate_val = 0x08;
		break;
	};

	if (int_prim_fs_rate_val < 0 ||
	    int_prim_fs_rate_val > 0x08) {
		pr_err("%s: Codec does not support sample rate: %u\n",
			__func__, params_rate(params));
		return -EINVAL;
	}

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		ret = tasha_set_interpolator_rate(dai,
				(u8) int_prim_fs_rate_val, params_rate(params));
		if (ret) {
			pr_err("%s: cannot set sample rate: %u\n",
				__func__, params_rate(params));
			return ret;
		}
		if ((params_format(params)) == SNDRV_PCM_FORMAT_S16_LE)
			tasha->dai[dai->id].bit_width = 16;
		else if ((params_format(params)) == SNDRV_PCM_FORMAT_S24_LE)
			tasha->dai[dai->id].bit_width = 24;
		tasha->dai[dai->id].rate = params_rate(params);
		break;
	default:
		pr_err("%s: Invalid stream type %d\n", __func__,
			substream->stream);
		return -EINVAL;
	};

	return 0;
}

static int tasha_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int tasha_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static struct snd_soc_dai_ops tasha_dai_ops = {
	.startup = tasha_startup,
	.shutdown = tasha_shutdown,
	.hw_params = tasha_hw_params,
	.set_sysclk = tasha_set_dai_sysclk,
	.set_fmt = tasha_set_dai_fmt,
	.set_channel_map = tasha_set_channel_map,
	.get_channel_map = tasha_get_channel_map,
};

static struct snd_soc_dai_driver tasha_dai[] = {
	{
		.name = "tasha_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9335_RATES,
			.formats = TASHA_FORMATS_S16_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tasha_dai_ops,
	},
	{
		.name = "tasha_tx1",
		.id = AIF1_CAP,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = WCD9335_RATES,
			.formats = TASHA_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &tasha_dai_ops,
	},
	{
		.name = "tasha_rx2",
		.id = AIF2_PB,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = WCD9335_RATES,
			.formats = TASHA_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tasha_dai_ops,
	},
	{
		.name = "tasha_tx2",
		.id = AIF2_CAP,
		.capture = {
			.stream_name = "AIF2 Capture",
			.rates = WCD9335_RATES,
			.formats = TASHA_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 8,
		},
		.ops = &tasha_dai_ops,
	},
	{
		.name = "tasha_rx3",
		.id = AIF3_PB,
		.playback = {
			.stream_name = "AIF3 Playback",
			.rates = WCD9335_RATES,
			.formats = TASHA_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tasha_dai_ops,
	},
	{
		.name = "tasha_tx3",
		.id = AIF3_CAP,
		.capture = {
			.stream_name = "AIF3 Capture",
			.rates = WCD9335_RATES,
			.formats = TASHA_FORMATS,
			.rate_max = 48000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tasha_dai_ops,
	},
};

static const struct tasha_reg_mask_val tasha_codec_reg_init_val[] = {
	{WCD9335_CODEC_RPM_CLK_GATE, 0x03, 0x00},
};

static void tasha_codec_init_reg(struct snd_soc_codec *codec)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(tasha_codec_reg_init_val); i++)
		snd_soc_update_bits(codec, tasha_codec_reg_init_val[i].reg,
				tasha_codec_reg_init_val[i].mask,
				tasha_codec_reg_init_val[i].val);
}

static void tasha_update_reg_defaults(struct snd_soc_codec *codec)
{
	return;
}

static void tasha_slim_interface_init_reg(struct snd_soc_codec *codec)
{
	int i;
	struct tasha_priv *priv = snd_soc_codec_get_drvdata(codec);

	for (i = 0; i < WCD9XXX_SLIM_NUM_PORT_REG; i++)
		wcd9xxx_interface_reg_write(priv->wcd9xxx,
					    TASHA_SLIM_PGD_PORT_INT_EN0 + i,
					    0xFF);
}

static irqreturn_t tasha_slimbus_irq(int irq, void *data)
{
	struct tasha_priv *priv = data;
	unsigned long status = 0;
	int i, j, port_id, k;
	u32 bit;
	u8 val, int_val = 0;
	bool tx, cleared;
	unsigned short reg = 0;

	for (i = TASHA_SLIM_PGD_PORT_INT_STATUS_RX_0, j = 0;
	     i <= TASHA_SLIM_PGD_PORT_INT_STATUS_TX_1; i++, j++) {
		val = wcd9xxx_interface_reg_read(priv->wcd9xxx, i);
		status |= ((u32)val << (8 * j));
	}

	for_each_set_bit(j, &status, 32) {
		tx = (j >= 16 ? true : false);
		port_id = (tx ? j - 16 : j);
		val = wcd9xxx_interface_reg_read(priv->wcd9xxx,
				TASHA_SLIM_PGD_PORT_INT_RX_SOURCE0 + j);
		if (val) {
			if (!tx)
				reg = TASHA_SLIM_PGD_PORT_INT_EN0 +
					(port_id / 8);
			else
				reg = TASHA_SLIM_PGD_PORT_INT_TX_EN0 +
					(port_id / 8);
			int_val = wcd9xxx_interface_reg_read(
				priv->wcd9xxx, reg);
			/*
			 * Ignore interrupts for ports for which the
			 * interrupts are not specifically enabled.
			 */
			if (!(int_val & (1 << (port_id % 8))))
				continue;
		}
		if (val & TASHA_SLIM_IRQ_OVERFLOW)
			pr_err_ratelimited(
			   "%s: overflow error on %s port %d, value %x\n",
			   __func__, (tx ? "TX" : "RX"), port_id, val);
		if (val & TASHA_SLIM_IRQ_UNDERFLOW)
			pr_err_ratelimited(
			   "%s: underflow error on %s port %d, value %x\n",
			   __func__, (tx ? "TX" : "RX"), port_id, val);
		if ((val & TASHA_SLIM_IRQ_OVERFLOW) ||
			(val & TASHA_SLIM_IRQ_UNDERFLOW)) {
			if (!tx)
				reg = TASHA_SLIM_PGD_PORT_INT_EN0 +
					(port_id / 8);
			else
				reg = TASHA_SLIM_PGD_PORT_INT_TX_EN0 +
					(port_id / 8);
			int_val = wcd9xxx_interface_reg_read(
				priv->wcd9xxx, reg);
			if (int_val & (1 << (port_id % 8))) {
				int_val = int_val ^ (1 << (port_id % 8));
				wcd9xxx_interface_reg_write(priv->wcd9xxx,
					reg, int_val);
			}
		}
		if (val & TASHA_SLIM_IRQ_PORT_CLOSED) {
			/*
			 * INT SOURCE register starts from RX to TX
			 * but port number in the ch_mask is in opposite way
			 */
			bit = (tx ? j - 16 : j + 16);
			pr_debug("%s: %s port %d closed value %x, bit %u\n",
				 __func__, (tx ? "TX" : "RX"), port_id, val,
				 bit);
			for (k = 0, cleared = false; k < NUM_CODEC_DAIS; k++) {
				pr_debug("%s: priv->dai[%d].ch_mask = 0x%lx\n",
					 __func__, k, priv->dai[k].ch_mask);
				if (test_and_clear_bit(bit,
						       &priv->dai[k].ch_mask)) {
					cleared = true;
					if (!priv->dai[k].ch_mask)
						wake_up(&priv->dai[k].dai_wait);
					/*
					 * There are cases when multiple DAIs
					 * might be using the same slimbus
					 * channel. Hence don't break here.
					 */
				}
			}
			WARN(!cleared,
			     "Couldn't find slimbus %s port %d for closing\n",
			     (tx ? "TX" : "RX"), port_id);
		}
		wcd9xxx_interface_reg_write(priv->wcd9xxx,
					    TASHA_SLIM_PGD_PORT_INT_CLR_RX_0 +
					    (j / 8),
					    1 << (j % 8));
	}

	return IRQ_HANDLED;
}

static int tasha_setup_irqs(struct tasha_priv *tasha)
{
	int ret = 0;
	struct snd_soc_codec *codec = tasha->codec;
	struct wcd9xxx *wcd9xxx = tasha->wcd9xxx;
	struct wcd9xxx_core_resource *core_res =
				&wcd9xxx->core_res;

	ret = wcd9xxx_request_irq(core_res, WCD9XXX_IRQ_SLIMBUS,
				  tasha_slimbus_irq, "SLIMBUS Slave", tasha);
	if (ret)
		pr_err("%s: Failed to request irq %d\n", __func__,
		       WCD9XXX_IRQ_SLIMBUS);
	else
		tasha_slim_interface_init_reg(codec);

	return ret;
}

static void tasha_cleanup_irqs(struct tasha_priv *tasha)
{
	struct wcd9xxx *wcd9xxx = tasha->wcd9xxx;
	struct wcd9xxx_core_resource *core_res =
				&wcd9xxx->core_res;

	wcd9xxx_free_irq(core_res, WCD9XXX_IRQ_SLIMBUS, tasha);
}

static int tasha_handle_pdata(struct tasha_priv *tasha)
{
	return 0;
}

static int tasha_codec_probe(struct snd_soc_codec *codec)
{
	struct wcd9xxx *control;
	struct tasha_priv *tasha;
	int addr_bits = 16, data_bits = 8;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int i, ret;
	void *ptr = NULL;

	control = dev_get_drvdata(codec->dev->parent);
	codec->control_data = control->regmap;

	dev_info(codec->dev, "%s()\n", __func__);
	tasha = snd_soc_codec_get_drvdata(codec);
	snd_soc_codec_set_cache_io(codec, addr_bits, data_bits,
					     SND_SOC_REGMAP);
	/* Resource Manager Init */
	/* Class-H Init*/
	/* MBHC Init */

	tasha->codec = codec;
	for (i = 0; i < COMPANDER_MAX; i++)
		tasha->comp_enabled[i] = 0;

	tasha->intf_type = wcd9xxx_get_intf_type();
	/* Update codec register default values */
	tasha_update_reg_defaults(codec);
	pr_debug("%s: MCLK Rate = %x\n", __func__, control->mclk_rate);
	if (control->mclk_rate == TASHA_MCLK_CLK_12P288MHZ)
		snd_soc_update_bits(codec, WCD9335_CODEC_RPM_CLK_MCLK_CFG,
				    0x03, 0x00);
	else if (control->mclk_rate == TASHA_MCLK_CLK_9P6MHZ)
		snd_soc_update_bits(codec, WCD9335_CODEC_RPM_CLK_MCLK_CFG,
				    0x03, 0x01);
	tasha_codec_init_reg(codec);

	ret = tasha_handle_pdata(tasha);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: bad pdata\n", __func__);
		goto err;
	}

	ptr = kmalloc((sizeof(tasha_rx_chs) +
		       sizeof(tasha_tx_chs)), GFP_KERNEL);
	if (!ptr) {
		pr_err("%s: no mem for slim chan ctl data\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	if (tasha->intf_type == WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
		for (i = 0; i < NUM_CODEC_DAIS; i++) {
			INIT_LIST_HEAD(&tasha->dai[i].wcd9xxx_ch_list);
			init_waitqueue_head(&tasha->dai[i].dai_wait);
		}
	}

	snd_soc_add_codec_controls(codec,
			tasha_analog_gain_controls,
			ARRAY_SIZE(tasha_analog_gain_controls));
	control->num_rx_port = TASHA_RX_MAX;
	control->rx_chs = ptr;
	memcpy(control->rx_chs, tasha_rx_chs, sizeof(tasha_rx_chs));
	control->num_tx_port = TASHA_TX_MAX;
	control->tx_chs = ptr + sizeof(tasha_rx_chs);
	memcpy(control->tx_chs, tasha_tx_chs, sizeof(tasha_tx_chs));

	snd_soc_dapm_sync(dapm);

	ret = tasha_setup_irqs(tasha);
	if (ret) {
		pr_err("%s: tasha irq setup failed %d\n", __func__, ret);
		goto err_pdata;
	}
	return ret;

err_pdata:
	kfree(ptr);
err:
	return ret;
}

static int tasha_codec_remove(struct snd_soc_codec *codec)
{
	struct tasha_priv *tasha = snd_soc_codec_get_drvdata(codec);

	tasha_cleanup_irqs(tasha);
	/* Cleanup MBHC */
	/* Cleanup resmgr */

	devm_kfree(codec->dev, tasha);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_tasha = {
	.probe	= tasha_codec_probe,
	.remove	= tasha_codec_remove,
	.controls = tasha_snd_controls,
	.num_controls = ARRAY_SIZE(tasha_snd_controls),
	.dapm_widgets = tasha_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tasha_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

#ifdef CONFIG_PM
static int tasha_suspend(struct device *dev)
{
	dev_dbg(dev, "%s: system suspend\n", __func__);
	return 0;
}

static int tasha_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tasha_priv *tasha = platform_get_drvdata(pdev);

	if (!tasha) {
		dev_err(dev, "%s: tasha private data is NULL\n", __func__);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: system resume\n", __func__);
	return 0;
}

static const struct dev_pm_ops tasha_pm_ops = {
	.suspend = tasha_suspend,
	.resume = tasha_resume,
};
#endif

static int tasha_swrm_read(void *handle, int reg)
{
	struct tasha_priv *tasha;
	struct wcd9xxx *wcd9xxx;
	unsigned short swr_rd_addr_base;
	unsigned short swr_rd_data_base;
	int val, ret;

	if (!handle) {
		pr_err("%s: NULL handle\n", __func__);
		return -EINVAL;
	}
	tasha = (struct tasha_priv *)handle;
	wcd9xxx = tasha->wcd9xxx;

	dev_dbg(tasha->dev, "%s: Reading soundwire register, 0x%x\n",
		__func__, reg);
	swr_rd_addr_base = WCD9335_SWR_AHB_BRIDGE_RD_ADDR_0;
	swr_rd_data_base = WCD9335_SWR_AHB_BRIDGE_RD_DATA_0;
	/* read_lock */
	mutex_lock(&tasha->swr_read_lock);
	ret = wcd9xxx_bulk_write(&wcd9xxx->core_res, swr_rd_addr_base, 4,
				 (u8 *)&reg);
	if (ret < 0) {
		pr_err("%s: RD Addr Failure\n", __func__);
		goto err;
	}
	/* Check for RD status */
	ret = wcd9xxx_bulk_read(&wcd9xxx->core_res, swr_rd_data_base, 4,
				(u8 *)&val);
	if (ret < 0) {
		pr_err("%s: RD Data Failure\n", __func__);
		goto err;
	}
	ret = val;
err:
	/* read_unlock */
	mutex_unlock(&tasha->swr_read_lock);
	return ret;
}

static int tasha_swrm_write(void *handle, int reg, int val)
{
	struct tasha_priv *tasha;
	struct wcd9xxx *wcd9xxx;
	unsigned short swr_wr_addr_base;
	unsigned short swr_wr_data_base;
	int ret;

	if (!handle) {
		pr_err("%s: NULL handle\n", __func__);
		return -EINVAL;
	}
	tasha = (struct tasha_priv *)handle;
	wcd9xxx = tasha->wcd9xxx;

	dev_dbg(tasha->dev, "%s: writing soundwire register, reg: 0x%x, val: 0x%x\n",
		__func__, reg, val);
	swr_wr_addr_base = WCD9335_SWR_AHB_BRIDGE_WR_ADDR_0;
	swr_wr_data_base = WCD9335_SWR_AHB_BRIDGE_WR_DATA_0;

	mutex_lock(&tasha->swr_write_lock);
	/* First Write the Data to registe */
	ret = wcd9xxx_bulk_write(&wcd9xxx->core_res, swr_wr_data_base, 4,
				 (u8 *)&val);
	if (ret < 0) {
		pr_err("%s: WR Data Failure\n", __func__);
		goto err;
	}
	/* Next Write Address */
	ret = wcd9xxx_bulk_write(&wcd9xxx->core_res, swr_wr_addr_base, 4,
				 (u8 *)&reg);
	if (ret < 0) {
		pr_err("%s: WR Addr Failure\n", __func__);
		goto err;
	}
	/* Wait for WR status */

err:
	mutex_unlock(&tasha->swr_write_lock);
	return ret;
}

static int tasha_swrm_clock(void *handle, bool enable)
{
	struct tasha_priv *tasha = (struct tasha_priv *) handle;

	if (!tasha->wcd_ext_clk) {
		dev_err(tasha->dev, "%s: codec ext clk is not init'ed\n",
			__func__);
		return -EINVAL;
	}
	return 0;
}

static int tasha_swrm_handle_irq(void *handle,
				   irqreturn_t (*swrm_irq_handler)(int irq,
								   void *data),
				    void *swrm_handle,
				    int action)
{
	struct tasha_priv *tasha;
	int ret = 0;
	struct wcd9xxx *wcd9xxx;

	if (!handle) {
		pr_err("%s: null handle received\n", __func__);
		return -EINVAL;
	}
	tasha = (struct tasha_priv *) handle;
	wcd9xxx = tasha->wcd9xxx;

	if (action) {
		ret = wcd9xxx_request_irq(&wcd9xxx->core_res,
					  WCD9335_IRQ_SOUNDWIRE,
					  swrm_irq_handler,
					  "Tasha SWR Master", swrm_handle);
		if (ret)
			dev_err(tasha->dev, "%s: Failed to request irq %d\n",
				__func__, WCD9335_IRQ_SOUNDWIRE);
	} else
		wcd9xxx_free_irq(&wcd9xxx->core_res, WCD9335_IRQ_SOUNDWIRE,
				 swrm_handle);

	return ret;
}

static void wcd_swr_ctrl_add_devices(struct work_struct *work)
{
	struct tasha_priv *tasha;
	struct platform_device *pdev;
	struct device_node *node;
	struct wcd9xxx *wcd9xxx;
	struct tasha_swr_ctrl_data *swr_ctrl_data = NULL, *temp;
	int ret, ctrl_num = 0;
	struct wcd_swr_ctrl_platform_data *platdata;

	tasha = container_of(work, struct tasha_priv,
			     swr_add_devices_work);
	if (!tasha) {
		pr_err("%s: Memory for WCD9335 does not exist\n",
			__func__);
		return;
	}
	wcd9xxx = tasha->wcd9xxx;
	if (!wcd9xxx) {
		pr_err("%s: Memory for WCD9XXX does not exist\n",
			__func__);
		return;
	}
	if (!wcd9xxx->dev->of_node) {
		pr_err("%s: DT node for wcd9xxx does not exist\n",
			__func__);
		return;
	}

	platdata = &tasha->swr_plat_data;

	for_each_child_of_node(wcd9xxx->dev->of_node, node) {
		temp = krealloc(swr_ctrl_data,
			(ctrl_num + 1) * sizeof(struct tasha_swr_ctrl_data),
			GFP_KERNEL);
		if (!temp) {
			dev_err(wcd9xxx->dev, "out of memory\n");
			ret = -ENOMEM;
			goto err;
		}
		swr_ctrl_data = temp;
		swr_ctrl_data[ctrl_num].swr_pdev = NULL;
		pdev = platform_device_alloc("tasha_swr_ctrl", -1);
		if (!pdev) {
			dev_err(wcd9xxx->dev, "%s: pdev memory alloc failed\n",
				__func__);
			ret = -ENOMEM;
			goto err;
		}
		pdev->dev.parent = tasha->dev;
		pdev->dev.of_node = node;

		ret = platform_device_add_data(pdev, platdata,
					       sizeof(*platdata));
		if (ret) {
			dev_err(&pdev->dev, "%s: cannot add plat data for ctrl:%d\n",
				__func__, ctrl_num);
			goto fail_pdev_add;
		}

		ret = platform_device_add(pdev);
		if (ret) {
			dev_err(&pdev->dev, "%s: Cannot add swr platform device\n",
				__func__);
			goto fail_pdev_add;
		}

		swr_ctrl_data[ctrl_num].swr_pdev = pdev;
		ctrl_num++;
		dev_dbg(&pdev->dev, "%s: Added soundwire ctrl device(s)\n",
			__func__);
	}
	tasha->nr = ctrl_num;
	tasha->swr_ctrl_data = swr_ctrl_data;

	return;
fail_pdev_add:
	platform_device_put(pdev);
err:
	return;
}

static int tasha_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct tasha_priv *tasha;

	tasha = devm_kzalloc(&pdev->dev, sizeof(struct tasha_priv),
			    GFP_KERNEL);
	if (!tasha) {
		dev_err(&pdev->dev, "%s: cannot create memory for wcd9335\n",
			__func__);
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, tasha);

	tasha->wcd9xxx = dev_get_drvdata(pdev->dev.parent);
	tasha->dev = &pdev->dev;
	INIT_WORK(&tasha->swr_add_devices_work, wcd_swr_ctrl_add_devices);
	mutex_init(&tasha->swr_read_lock);
	mutex_init(&tasha->swr_write_lock);

	if (wcd9xxx_get_intf_type() == WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_tasha,
					     tasha_dai, ARRAY_SIZE(tasha_dai));
		if (ret) {
			dev_err(&pdev->dev, "%s: Codec registration failed\n",
				__func__);
			goto cdc_reg_fail;
		}
	}

	tasha->swr_plat_data.handle = (void *) tasha;
	tasha->swr_plat_data.read = tasha_swrm_read;
	tasha->swr_plat_data.write = tasha_swrm_write;
	tasha->swr_plat_data.clk = tasha_swrm_clock;
	tasha->swr_plat_data.handle_irq = tasha_swrm_handle_irq;

	schedule_work(&tasha->swr_add_devices_work);
	return ret;

cdc_reg_fail:
	devm_kfree(&pdev->dev, tasha);
	return ret;
}

static int tasha_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver tasha_codec_driver = {
	.probe = tasha_probe,
	.remove = tasha_remove,
	.driver = {
		.name = "tasha_codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &tasha_pm_ops,
#endif
	},
};

module_platform_driver(tasha_codec_driver);

MODULE_DESCRIPTION("Tasha Codec driver");
MODULE_LICENSE("GPL v2");
