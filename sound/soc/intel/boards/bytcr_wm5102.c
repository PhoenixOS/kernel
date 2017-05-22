/*
 *  byt_cr_dpcm_wm5102.c - ASoc Machine driver for Intel Byt CR platform
 *
 *  Based on bytcr_rt5640 driver
 *  Copyright (C) 2014 Intel Corp
 *  Author: Subhransu S. Prusty <subhransu.s.prusty@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <asm/cpu_device_id.h>
#include <asm/platform_sst_audio.h>
#include <linux/clk.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/wm5102.h"
#include "../atom/sst-atom-controls.h"
#include "../common/sst-acpi.h"
#include "../common/sst-dsp.h"

enum {
	BYT_WM5102_DMIC1_MAP,
	BYT_WM5102_DMIC2_MAP,
	BYT_WM5102_IN1_MAP,
	BYT_WM5102_IN3_MAP,
};

#define BYT_WM5102_MAP(quirk)	((quirk) & 0xff)
#define BYT_WM5102_DMIC_EN	BIT(16)
#define BYT_WM5102_MONO_SPEAKER BIT(17)
#define BYT_WM5102_DIFF_MIC     BIT(18) /* defaut is single-ended */
#define BYT_WM5102_SSP2_AIF2     BIT(19) /* default is using AIF1  */
#define BYT_WM5102_SSP0_AIF1     BIT(20)
#define BYT_WM5102_SSP0_AIF2     BIT(21)
#define BYT_WM5102_MCLK_EN	BIT(22)
#define BYT_WM5102_MCLK_25MHZ	BIT(23)

#define WM5102_MAX_SYSCLK_1 49152000 /*max sysclk for 4K family*/
#define WM5102_MAX_SYSCLK_2 45158400 /*max sysclk for 11.025K family*/

struct byt_wm5102_private {
	struct clk *mclk;
};

static unsigned long byt_wm5102_quirk = BYT_WM5102_MCLK_EN;

static void log_quirks(struct device *dev)
{
	if (BYT_WM5102_MAP(byt_wm5102_quirk) == BYT_WM5102_DMIC1_MAP)
		dev_info(dev, "quirk DMIC1_MAP enabled");
	if (BYT_WM5102_MAP(byt_wm5102_quirk) == BYT_WM5102_DMIC2_MAP)
		dev_info(dev, "quirk DMIC2_MAP enabled");
	if (BYT_WM5102_MAP(byt_wm5102_quirk) == BYT_WM5102_IN1_MAP)
		dev_info(dev, "quirk IN1_MAP enabled");
	if (BYT_WM5102_MAP(byt_wm5102_quirk) == BYT_WM5102_IN3_MAP)
		dev_info(dev, "quirk IN3_MAP enabled");
	if (byt_wm5102_quirk & BYT_WM5102_DMIC_EN)
		dev_info(dev, "quirk DMIC enabled");
	if (byt_wm5102_quirk & BYT_WM5102_MONO_SPEAKER)
		dev_info(dev, "quirk MONO_SPEAKER enabled");
	if (byt_wm5102_quirk & BYT_WM5102_DIFF_MIC)
		dev_info(dev, "quirk DIFF_MIC enabled");
	if (byt_wm5102_quirk & BYT_WM5102_SSP2_AIF2)
		dev_info(dev, "quirk SSP2_AIF2 enabled");
	if (byt_wm5102_quirk & BYT_WM5102_SSP0_AIF1)
		dev_info(dev, "quirk SSP0_AIF1 enabled");
	if (byt_wm5102_quirk & BYT_WM5102_SSP0_AIF2)
		dev_info(dev, "quirk SSP0_AIF2 enabled");
	if (byt_wm5102_quirk & BYT_WM5102_MCLK_EN)
		dev_info(dev, "quirk MCLK_EN enabled");
	if (byt_wm5102_quirk & BYT_WM5102_MCLK_25MHZ)
		dev_info(dev, "quirk MCLK_25MHZ enabled");
}


#define BYT_CODEC_DAI1	"wm5102-aif1"
#define BYT_CODEC_DAI2	"wm5102-aif2"

static inline struct snd_soc_dai *byt_get_codec_dai(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;

	list_for_each_entry(rtd, &card->rtd_list, list) {
		if (!strncmp(rtd->codec_dai->name, BYT_CODEC_DAI1,
			     strlen(BYT_CODEC_DAI1)))
			return rtd->codec_dai;
		if (!strncmp(rtd->codec_dai->name, BYT_CODEC_DAI2,
				strlen(BYT_CODEC_DAI2)))
			return rtd->codec_dai;

	}
	return NULL;
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dai *codec_dai;
	struct byt_wm5102_private *priv = snd_soc_card_get_drvdata(card);
	int ret;

	codec_dai = byt_get_codec_dai(card);
	if (!codec_dai) {
		dev_err(card->dev,
			"Codec dai not found; Unable to set platform clock\n");
		return -EIO;
	}

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if ((byt_wm5102_quirk & BYT_WM5102_MCLK_EN) && priv->mclk) {
			ret = clk_prepare_enable(priv->mclk);
			if (ret < 0) {
				dev_err(card->dev,
					"could not configure MCLK state");
				return ret;
			}
		}
		ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_SYSCLK,
					     48000 * 512,
					     SND_SOC_CLOCK_IN);
	} else {
		/*
		 * Set codec clock source to internal clock before
		 * turning off the platform clock. Codec needs clock
		 * for Jack detection and button press
		 */
		ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_SYSCLK,
					     48000 * 512,
					     SND_SOC_CLOCK_IN);
		if (!ret) {
			if ((byt_wm5102_quirk & BYT_WM5102_MCLK_EN) && priv->mclk)
				clk_disable_unprepare(priv->mclk);
		}
	}

	if (ret < 0) {
		dev_err(card->dev, "can't set codec sysclk: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct snd_soc_dapm_widget byt_wm5102_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			    platform_clock_control, SND_SOC_DAPM_PRE_PMU |
			    SND_SOC_DAPM_POST_PMD),

};

static const struct snd_soc_dapm_route byt_wm5102_audio_map[] = {
	{"Headphone", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Internal Mic", NULL, "Platform Clock"},
	{"Speaker", NULL, "Platform Clock"},

	{"Headset Mic", NULL, "MICBIAS1"},
	{"IN1L", NULL, "Headset Mic"},
	{"Headphone", NULL, "HPOUT1L"},
	{"Headphone", NULL, "HPOUT1R"},
};

static const struct snd_soc_dapm_route byt_wm5102_intmic_dmic1_map[] = {
	{"DMIC1", NULL, "Internal Mic"},
};

static const struct snd_soc_dapm_route byt_wm5102_intmic_dmic2_map[] = {
	{"DMIC2", NULL, "Internal Mic"},
};

static const struct snd_soc_dapm_route byt_wm5102_intmic_in1_map[] = {
	{"Internal Mic", NULL, "MICBIAS1"},
	{"IN1P", NULL, "Internal Mic"},
};

static const struct snd_soc_dapm_route byt_wm5102_intmic_in3_map[] = {
	{"Internal Mic", NULL, "MICBIAS1"},
	{"IN3P", NULL, "Internal Mic"},
};

static const struct snd_soc_dapm_route byt_wm5102_ssp2_aif1_map[] = {
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
	{"codec_in0", NULL, "ssp2 Rx"},
	{"codec_in1", NULL, "ssp2 Rx"},

	{"AIF1 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Rx", NULL, "AIF1 Capture"},
};

static const struct snd_soc_dapm_route byt_wm5102_ssp2_aif2_map[] = {
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
	{"codec_in0", NULL, "ssp2 Rx"},
	{"codec_in1", NULL, "ssp2 Rx"},

	{"AIF2 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Rx", NULL, "AIF2 Capture"},
};

static const struct snd_soc_dapm_route byt_wm5102_ssp0_aif1_map[] = {
	{"ssp0 Tx", NULL, "modem_out"},
	{"modem_in", NULL, "ssp0 Rx"},

	{"AIF1 Playback", NULL, "ssp0 Tx"},
	{"ssp0 Rx", NULL, "AIF1 Capture"},
};

static const struct snd_soc_dapm_route byt_wm5102_ssp0_aif2_map[] = {
	{"ssp0 Tx", NULL, "modem_out"},
	{"modem_in", NULL, "ssp0 Rx"},

	{"AIF2 Playback", NULL, "ssp0 Tx"},
	{"ssp0 Rx", NULL, "AIF2 Capture"},
};

static const struct snd_soc_dapm_route byt_wm5102_stereo_spk_map[] = {
	{"Speaker", NULL, "SPKOUTLP"},
	{"Speaker", NULL, "SPKOUTLN"},
	{"Speaker", NULL, "SPKOUTRP"},
	{"Speaker", NULL, "SPKOUTRN"},
};

static const struct snd_soc_dapm_route byt_wm5102_mono_spk_map[] = {
	{"Speaker", NULL, "SPKOUTLP"},
	{"Speaker", NULL, "SPKOUTLN"},
};

static const struct snd_kcontrol_new byt_wm5102_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Internal Mic"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static int byt_wm5102_aif1_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *wm5102_codec = codec_dai->codec;
	int ret;

	int sr = params_rate(params);
	int sr_mult = (params_rate(params) % 4000 == 0) ? (WM5102_MAX_SYSCLK_1/params_rate(params)) : (WM5102_MAX_SYSCLK_2/params_rate(params));


	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_SYSCLK,
				     params_rate(params) * 512,
				     SND_SOC_CLOCK_IN);

	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec clock %d\n", ret);
		return ret;
	}

	/*reset FLL1*/
	snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1_REFCLK,
				ARIZONA_FLL_SRC_NONE,
				0,
				0);

	snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1,
				ARIZONA_FLL_SRC_NONE,
				0,
				0);

	ret = snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1,
				ARIZONA_CLK_SRC_MCLK1,
				25000000,
				sr * sr_mult);

	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec pll: %d\n", ret);
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
			ARIZONA_CLK_SYSCLK,
			ARIZONA_CLK_SRC_FLL1,
			sr * sr_mult,
			SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to set AYNCCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
					ARIZONA_CLK_SYSCLK, 0,
					sr * sr_mult,
					SND_SOC_CLOCK_OUT);
        if (ret < 0) {
                dev_err(rtd->dev, "can't set OPCLK %d\n", ret);
        }

	return 0;
}

static int byt_wm5102_quirk_cb(const struct dmi_system_id *id)
{
	byt_wm5102_quirk = (unsigned long)id->driver_data;
	return 1;
}

static const struct dmi_system_id byt_wm5102_quirk_table[] = {
	{
		.callback = byt_wm5102_quirk_cb,
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "LENOVO"),
			DMI_MATCH(DMI_CHASSIS_VERSION, "1051F"),
		},
		.driver_data = (unsigned long *)(BYT_WM5102_MCLK_25MHZ |
						 BYT_WM5102_MCLK_EN |
						 BYT_WM5102_SSP0_AIF1),

	},
	{}
};

static int byt_wm5102_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_card *card = runtime->card;
	const struct snd_soc_dapm_route *custom_map;
	struct byt_wm5102_private *priv = snd_soc_card_get_drvdata(card);
	int num_routes;

	card->dapm.idle_bias_off = true;

	ret = snd_soc_add_card_controls(card, byt_wm5102_controls,
					ARRAY_SIZE(byt_wm5102_controls));
	if (ret) {
		dev_err(card->dev, "unable to add card controls\n");
		return ret;
	}

	if (byt_wm5102_quirk & BYT_WM5102_SSP2_AIF2) {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					byt_wm5102_ssp2_aif2_map,
					ARRAY_SIZE(byt_wm5102_ssp2_aif2_map));
	} else if (byt_wm5102_quirk & BYT_WM5102_SSP0_AIF1) {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					byt_wm5102_ssp0_aif1_map,
					ARRAY_SIZE(byt_wm5102_ssp0_aif1_map));
	} else if (byt_wm5102_quirk & BYT_WM5102_SSP0_AIF2) {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					byt_wm5102_ssp0_aif2_map,
					ARRAY_SIZE(byt_wm5102_ssp0_aif2_map));
	} else {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					byt_wm5102_ssp2_aif1_map,
					ARRAY_SIZE(byt_wm5102_ssp2_aif1_map));
	}
	if (ret)
		return ret;

	if (byt_wm5102_quirk & BYT_WM5102_MONO_SPEAKER) {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					byt_wm5102_mono_spk_map,
					ARRAY_SIZE(byt_wm5102_mono_spk_map));
	} else {
		ret = snd_soc_dapm_add_routes(&card->dapm,
					byt_wm5102_stereo_spk_map,
					ARRAY_SIZE(byt_wm5102_stereo_spk_map));
	}
	if (ret)
		return ret;

	snd_soc_dapm_ignore_suspend(&card->dapm, "Headphone");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Speaker");

	if ((byt_wm5102_quirk & BYT_WM5102_MCLK_EN) && priv->mclk) {
		/*
		 * The firmware might enable the clock at
		 * boot (this information may or may not
		 * be reflected in the enable clock register).
		 * To change the rate we must disable the clock
		 * first to cover these cases. Due to common
		 * clock framework restrictions that do not allow
		 * to disable a clock that has not been enabled,
		 * we need to enable the clock first.
		 */
		ret = clk_prepare_enable(priv->mclk);
		if (!ret)
			clk_disable_unprepare(priv->mclk);

		if (byt_wm5102_quirk & BYT_WM5102_MCLK_25MHZ)
			ret = clk_set_rate(priv->mclk, 25000000);
		else
			ret = clk_set_rate(priv->mclk, 19200000);

		if (ret)
			dev_err(card->dev, "unable to set MCLK rate\n");
	}

	return ret;
}

static const struct snd_soc_pcm_stream byt_wm5102_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static int byt_wm5102_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	int ret;

	/* The DSP will covert the FE rate to 48k, stereo */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	if ((byt_wm5102_quirk & BYT_WM5102_SSP0_AIF1) ||
		(byt_wm5102_quirk & BYT_WM5102_SSP0_AIF2)) {

		/* set SSP0 to 16-bit */
		params_set_format(params, SNDRV_PCM_FORMAT_S16_LE);

		/*
		 * Default mode for SSP configuration is TDM 4 slot, override config
		 * with explicit setting to I2S 2ch 16-bit. The word length is set with
		 * dai_set_tdm_slot() since there is no other API exposed
		 */
		ret = snd_soc_dai_set_fmt(rtd->cpu_dai,
					SND_SOC_DAIFMT_I2S     |
					SND_SOC_DAIFMT_NB_NF   |
					SND_SOC_DAIFMT_CBS_CFS
			);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set format to I2S, err %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 16);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set I2S config, err %d\n", ret);
			return ret;
		}

	} else {

		/* set SSP2 to 24-bit */
		params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);

		/*
		 * Default mode for SSP configuration is TDM 4 slot, override config
		 * with explicit setting to I2S 2ch 24-bit. The word length is set with
		 * dai_set_tdm_slot() since there is no other API exposed
		 */
		ret = snd_soc_dai_set_fmt(rtd->cpu_dai,
					SND_SOC_DAIFMT_I2S     |
					SND_SOC_DAIFMT_NB_NF   |
					SND_SOC_DAIFMT_CBS_CFS
			);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set format to I2S, err %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 24);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set I2S config, err %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int byt_wm5102_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
			SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static const struct snd_soc_ops byt_wm5102_aif1_ops = {
	.startup = byt_wm5102_aif1_startup,
};

static const struct snd_soc_ops byt_wm5102_be_ssp2_ops = {
	.hw_params = byt_wm5102_aif1_hw_params,
};

static struct snd_soc_dai_link byt_wm5102_dais[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Baytrail Audio Port",
		.stream_name = "Baytrail Audio",
		.cpu_dai_name = "media-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &byt_wm5102_aif1_ops,
	},
	[MERR_DPCM_DEEP_BUFFER] = {
		.name = "Deep-Buffer Audio Port",
		.stream_name = "Deep-Buffer Audio",
		.cpu_dai_name = "deepbuffer-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.ops = &byt_wm5102_aif1_ops,
	},
	[MERR_DPCM_COMPR] = {
		.name = "Baytrail Compressed Port",
		.stream_name = "Baytrail Compress",
		.cpu_dai_name = "compress-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
	},
		/* back ends */
	{
		.name = "SSP2-Codec",
		.id = 1,
		.cpu_dai_name = "ssp2-port", /* overwritten for ssp0 routing */
		.platform_name = "sst-mfld-platform",
		.no_pcm = 1,
		.codec_dai_name = "wm5102-aif1", /* changed w/ quirk */
		.codec_name = "wm5102-codec", /* overwritten with HID */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
						| SND_SOC_DAIFMT_CBS_CFS,
		.be_hw_params_fixup = byt_wm5102_codec_fixup,
		.ignore_suspend = 1,
		.nonatomic = true,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.init = byt_wm5102_init,
		.ops = &byt_wm5102_be_ssp2_ops,
	},
};

/* SoC card */
static struct snd_soc_card byt_wm5102_card = {
	.name = "bytcr-wm5102",
	.owner = THIS_MODULE,
	.dai_link = byt_wm5102_dais,
	.num_links = ARRAY_SIZE(byt_wm5102_dais),
	.dapm_widgets = byt_wm5102_widgets,
	.num_dapm_widgets = ARRAY_SIZE(byt_wm5102_widgets),
	.dapm_routes = byt_wm5102_audio_map,
	.num_dapm_routes = ARRAY_SIZE(byt_wm5102_audio_map),
	.fully_routed = true,
};

static char byt_wm5102_codec_name[13]; /* wm5102-codec */
static char byt_wm5102_codec_aif_name[12]; /*  = "wm5102-aif[1|2]" */
static char byt_wm5102_cpu_dai_name[10]; /*  = "ssp[0|2]-port" */

static bool is_valleyview(void)
{
	static const struct x86_cpu_id cpu_ids[] = {
		{ X86_VENDOR_INTEL, 6, 55 }, /* Valleyview, Bay Trail */
		{}
	};

	if (!x86_match_cpu(cpu_ids))
		return false;
	return true;
}

struct acpi_chan_package {   /* ACPICA seems to require 64 bit integers */
	u64 aif_value;       /* 1: AIF1, 2: AIF2 */
	u64 mclock_value;    /* usually 25MHz (0x17d7940), ignored */
};

static int snd_byt_wm5102_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct sst_acpi_mach *mach;
	int i;
	int dai_index;
	struct byt_wm5102_private *priv;
	bool is_bytcr = false;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_ATOMIC);
	if (!priv)
		return -ENOMEM;

	/* register the soc card */
	byt_wm5102_card.dev = &pdev->dev;
	mach = byt_wm5102_card.dev->platform_data;
	snd_soc_card_set_drvdata(&byt_wm5102_card, priv);

	/* fix index of codec dai */
	dai_index = MERR_DPCM_COMPR + 1;
	for (i = 0; i < ARRAY_SIZE(byt_wm5102_dais); i++) {
		if (!strcmp(byt_wm5102_dais[i].codec_name, "wm5102-codec")) {
			dai_index = i;
			break;
		}
	}

	/*
	 * swap SSP0 if bytcr is detected
	 * (will be overridden if DMI quirk is detected)
	 */
	if (is_valleyview()) {
		struct sst_platform_info *p_info = mach->pdata;
		const struct sst_res_info *res_info = p_info->res_info;

		if (res_info->acpi_ipc_irq_index == 0) {
			is_bytcr = true;
		}
	}

	if (is_bytcr) {
		/*
		 * Baytrail CR platforms may have CHAN package in BIOS, try
		 * to find relevant routing quirk based as done on Windows
		 * platforms. We have to read the information directly from the
		 * BIOS, at this stage the card is not created and the links
		 * with the codec driver/pdata are non-existent
		 */

		struct acpi_chan_package chan_package;

		/* format specified: 2 64-bit integers */
		struct acpi_buffer format = {sizeof("NN"), "NN"};
		struct acpi_buffer state = {0, NULL};
		struct sst_acpi_package_context pkg_ctx;
		bool pkg_found = false;

		state.length = sizeof(chan_package);
		state.pointer = &chan_package;

		pkg_ctx.name = "CHAN";
		pkg_ctx.length = 2;
		pkg_ctx.format = &format;
		pkg_ctx.state = &state;
		pkg_ctx.data_valid = false;

		pkg_found = sst_acpi_find_package_from_hid(mach->id, &pkg_ctx);
		if (pkg_found) {
			if (chan_package.aif_value == 1) {
				dev_info(&pdev->dev, "BIOS Routing: AIF1 connected\n");
				byt_wm5102_quirk |= BYT_WM5102_SSP0_AIF1;
			} else  if (chan_package.aif_value == 2) {
				dev_info(&pdev->dev, "BIOS Routing: AIF2 connected\n");
				byt_wm5102_quirk |= BYT_WM5102_SSP0_AIF2;
			} else {
				dev_info(&pdev->dev, "BIOS Routing isn't valid, ignored\n");
				pkg_found = false;
			}
		}

		if (!pkg_found) {
			/* no BIOS indications, assume SSP0-AIF1 connection */
			byt_wm5102_quirk |= BYT_WM5102_SSP0_AIF1;
		}

		/* change defaults for Baytrail-CR capture */
		byt_wm5102_quirk |= BYT_WM5102_IN1_MAP;
		byt_wm5102_quirk |= BYT_WM5102_DIFF_MIC;
	} else {
		byt_wm5102_quirk |= (BYT_WM5102_DMIC1_MAP |
				BYT_WM5102_DMIC_EN);
	}

	/* check quirks before creating card */
	dmi_check_system(byt_wm5102_quirk_table);
	log_quirks(&pdev->dev);

	if ((byt_wm5102_quirk & BYT_WM5102_SSP2_AIF2) ||
	    (byt_wm5102_quirk & BYT_WM5102_SSP0_AIF2)) {

		/* fixup codec aif name */
		snprintf(byt_wm5102_codec_aif_name,
			sizeof(byt_wm5102_codec_aif_name),
			"%s", "wm5102-aif2");

		byt_wm5102_dais[dai_index].codec_dai_name =
			byt_wm5102_codec_aif_name;
	}

	if ((byt_wm5102_quirk & BYT_WM5102_SSP0_AIF1) ||
	    (byt_wm5102_quirk & BYT_WM5102_SSP0_AIF2)) {

		/* fixup cpu dai name name */
		snprintf(byt_wm5102_cpu_dai_name,
			sizeof(byt_wm5102_cpu_dai_name),
			"%s", "ssp0-port");

		byt_wm5102_dais[dai_index].cpu_dai_name =
			byt_wm5102_cpu_dai_name;
	}

	if ((byt_wm5102_quirk & BYT_WM5102_MCLK_EN) && (is_valleyview())) {
		priv->mclk = devm_clk_get(&pdev->dev, "pmc_plt_clk_3");
		if (IS_ERR(priv->mclk)) {
			ret_val = PTR_ERR(priv->mclk);

			dev_err(&pdev->dev,
				"Failed to get MCLK from pmc_plt_clk_3: %d\n",
				ret_val);

			/*
			 * Fall back to bit clock usage for -ENOENT (clock not
			 * available likely due to missing dependencies), bail
			 * for all other errors, including -EPROBE_DEFER
			 */
			if (ret_val != -ENOENT)
				return ret_val;
			byt_wm5102_quirk &= ~BYT_WM5102_MCLK_EN;
		}
	}

	ret_val = devm_snd_soc_register_card(&pdev->dev, &byt_wm5102_card);

	if (ret_val) {
		dev_err(&pdev->dev, "devm_snd_soc_register_card failed %d\n",
			ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &byt_wm5102_card);
	return ret_val;
}

static struct platform_driver snd_byt_wm5102_mc_driver = {
	.driver = {
		.name = "bytcr_wm5102",
	},
	.probe = snd_byt_wm5102_mc_probe,
};

module_platform_driver(snd_byt_wm5102_mc_driver);

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail CR Machine driver");
MODULE_AUTHOR("Subhransu S. Prusty <subhransu.s.prusty@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bytcr_wm5102");
