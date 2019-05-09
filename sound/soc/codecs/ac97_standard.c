/*
* ac97std.c  --  ALSA SoC ac97std AC'97 codec support
*
* Copyright 2010-2015 Seco s.r.l.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/ac97_codec.h>

#include "ac97_standard.h"
#define DRV_NAME "ac97-standard-codec"


struct ac97std_priv {
	struct snd_ac97 *ac97;
	struct regmap   *regmap;
};

unsigned int ac97_read (struct snd_soc_codec *codec, unsigned int reg);
int ac97_write (struct snd_soc_codec *codec, unsigned int reg, unsigned int val);



static const u16 ac97std_reg[] = {
	[0x00] = 0x0140 ,  /* Reset */
	[0x02] = 0x8000 ,  /* Stereo Output Volume */
	[0x04] = 0x8000 ,  /* HP Stereo Output Volume */
	[0x0A] = 0x0000 ,  /* PC Beep Volume */
	[0x0C] = 0x8008 ,  /* Phone Volume */
	[0x0E] = 0x8008 ,  /* MIC Volume */
	[0x10] = 0x8808 ,  /* Line In Volume */
	[0x12] = 0x8808 ,  /* CD Volume */
	[0x16] = 0x8808 ,  /* AUX Volume */
	[0x18] = 0x8808 ,  /* PCM out Volume */
	[0x1A] = 0x0000 ,  /* Record Select */
	[0x1C] = 0x8000 ,  /* Record Gain */
	[0x20] = 0x0000 ,  /* General Purpose */
	[0x24] = 0x0001 ,  /* Audio Interrupt & Paging (AC'97 2.3) */
	[0x26] = 0x0000 ,  /* Powerdown control / status */
	[0x28] = 0x097C ,  /* Extended Audio ID */
	[0x2A] = 0x3830 ,  /* Extended Audio Status and Control */
	[0x2C] = 0xBB80 ,  /* PCM Front DAC Rate */
	[0x32] = 0xBB80 ,  /* PCM LR ADC Rate */
	[0x3A] = 0x2000 ,  /* S/PDIF Control */
	[0x5A] = 0x0000 ,  /* Vendor Reserved Register */
	[0x5C] = 0x00A9 ,  /* Vendor Reserved Register */
	[0x62] = 0xFFFF ,  /*  PCI SVID  Page ID = 01h */
	[0x64] = 0xFFFF ,  /*  PCI SID   Page ID = 01h */
	[0x66] = 0x0000 ,  /* S/PDIF RX Status    Page ID = 00h */
	[0x68] = 0x0000 ,  /* S/PDIF RX Status    Page ID = 00h */
	[0x6A] = 0x0000 ,  /* S/PDIF RX Control   Page ID = 00h */
	[0x6C] = 0x376A ,  /* DAC Slot Mapping  Page ID = 01h */
	[0x6E] = 0x0000 ,  /* ADC Slot Mapping  Page ID = 01h */
	[0x70] = 0x0000 ,  /* ADC / SPDIF RX Left Peak */
	[0x74] = 0x0000 ,  /* PLL Setting /Debugging */
	[0x76] = 0x1182 ,  /* Miscellaneous */
	[0x78] = 0x0070 ,  /* GPIO Control */
	[0x7A] = 0x0070 ,  /* GPIO Status */
	[0x7C] = 0x5649 ,  /* Vendor ID1 */
	[0x7E] = 0x4120 ,  /* Vendor ID2 */
};



static bool ac97std_readable_reg (unsigned int reg) {
	switch (reg) {
		case AC97_RESET ... AC97_HEADPHONE:
		case AC97_PC_BEEP ... AC97_CD:
		case AC97_AUX ... AC97_GENERAL_PURPOSE:
		case AC97_INT_PAGING ... AC97_PCM_LR_ADC_RATE:
		case AC97_SPDIF:
		case AC97_AD_TEST:
		case AC97_AC97STD_STEREO_MIC:
		case AC97_PCI_SVID ... AC97_SENSE_INFO:
		case AC97_AC97STD_DAC_SLOT_MAP:
		case AC97_AC97STD_ADC_SLOT_MAP:
		case AC97_AD_CODEC_CFG:
		case AC97_AD_SERIAL_CFG:
		case AC97_AD_MISC:
		case AC97_AC97STD_GPIO_CTRL:
		case AC97_AC97STD_GPIO_STATUS:
		case AC97_VENDOR_ID1:
		case AC97_VENDOR_ID2:
			return true;
		default:
			return false;
	}
}


static bool ac97std_writeable_reg (unsigned int reg) {
	switch (reg) {
		case AC97_RESET:
		case AC97_EXTENDED_ID:
		case AC97_PCM_SURR_DAC_RATE:
		case AC97_PCM_LFE_DAC_RATE:
		case AC97_FUNC_SELECT:
		case AC97_FUNC_INFO:
		case AC97_AC97STD_ADC_SLOT_MAP:
		case AC97_VENDOR_ID1:
		case AC97_VENDOR_ID2:
			return false;
		default:
			return ac97std_readable_reg(reg);
	}
}


static const char *ac97std_record_mux[] = {"Mic", "CD", "--", "AUX",
	"Line", "Stereo Mix", "Mono Mix", "Phone"};
static SOC_ENUM_DOUBLE_DECL(ac97std_record_enum,
		AC97_REC_SEL, 8, 0, ac97std_record_mux);

static const char *ac97std_mic_mux[] = {"Mic1", "Mic2"};
static SOC_ENUM_SINGLE_DECL(ac97std_mic_enum,
		AC97_GENERAL_PURPOSE, 8, ac97std_mic_mux);

static const char *ac97std_boost[] = {"0dB", "20dB"};
static SOC_ENUM_SINGLE_DECL(ac97std_boost_enum,
		AC97_MIC, 6, ac97std_boost);

static const char *ac97std_mic_sel[] = {"MonoMic", "StereoMic"};
static SOC_ENUM_SINGLE_DECL(ac97std_mic_sel_enum,
		AC97_AC97STD_STEREO_MIC, 2, ac97std_mic_sel);

static const DECLARE_TLV_DB_LINEAR(master_tlv, -4650, 0);
static const DECLARE_TLV_DB_LINEAR(record_tlv, 0, 2250);
static const DECLARE_TLV_DB_LINEAR(beep_tlv, -4500, 0);
static const DECLARE_TLV_DB_LINEAR(mix_tlv, -3450, 1200);

static const struct snd_kcontrol_new ac97std_snd_ac97_controls[] = {
	SOC_DOUBLE_TLV("Speaker Playback Volume", AC97_MASTER, 8, 0, 31, 1, master_tlv),
	SOC_SINGLE("Speaker Playback Switch", AC97_MASTER, 15, 1, 1),

	SOC_DOUBLE_TLV("Headphone Playback Volume", AC97_HEADPHONE, 8, 0, 31, 1, master_tlv),
	SOC_SINGLE("Headphone Playback Switch", AC97_HEADPHONE, 15, 1, 1),

	SOC_DOUBLE_TLV("PCM Playback Volume", AC97_PCM, 8, 0, 31, 1, mix_tlv),
	SOC_SINGLE("PCM Playback Switch", AC97_PCM, 15, 1, 1),

	SOC_DOUBLE_TLV("Record Capture Volume", AC97_REC_GAIN, 8, 0, 15, 0, record_tlv),
	SOC_SINGLE("Record Capture Switch", AC97_REC_GAIN, 15, 1, 1),

	SOC_SINGLE_TLV("Beep Volume", AC97_PC_BEEP, 1, 15, 1, beep_tlv),
	SOC_SINGLE("Beep Switch", AC97_PC_BEEP, 15, 1, 1),
	SOC_SINGLE_TLV("Phone Volume", AC97_PHONE, 0, 31, 0, mix_tlv),
	SOC_SINGLE("Phone Switch", AC97_PHONE, 15, 1, 1),

	/* Mono Mic and Stereo Mic's right channel controls */
	SOC_SINGLE_TLV("Mic/StereoMic_R Volume", AC97_MIC, 0, 31, 0, mix_tlv),
	SOC_SINGLE("Mic/StereoMic_R Switch", AC97_MIC, 15, 1, 1),

	/* Stereo Mic's left channel controls */
	SOC_SINGLE("StereoMic_L Switch", AC97_MIC, 7, 1, 1),
	SOC_SINGLE_TLV("StereoMic_L Volume", AC97_MIC, 8, 31, 0, mix_tlv),

	SOC_DOUBLE_TLV("Line Volume", AC97_LINE, 8, 0, 31, 0, mix_tlv),
	SOC_SINGLE("Line Switch", AC97_LINE, 15, 1, 1),
	SOC_DOUBLE_TLV("CD Volume", AC97_CD, 8, 0, 31, 0, mix_tlv),
	SOC_SINGLE("CD Switch", AC97_CD, 15, 1, 1),
	SOC_DOUBLE_TLV("AUX Volume", AC97_AUX, 8, 0, 31, 0, mix_tlv),
	SOC_SINGLE("AUX Switch", AC97_AUX, 15, 1, 1),

	SOC_SINGLE("Analog Loopback", AC97_GENERAL_PURPOSE, 7, 1, 0),

	SOC_ENUM("Mic Boost", ac97std_boost_enum),
	SOC_ENUM("Mic1/2 Mux", ac97std_mic_enum),
	SOC_ENUM("Mic Select", ac97std_mic_sel_enum),
	SOC_ENUM("Record Mux", ac97std_record_enum),
};


static const unsigned int ac97std_rates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};


static const struct snd_pcm_hw_constraint_list ac97std_rate_constraints = {
	.count	= ARRAY_SIZE(ac97std_rates),
	.list	= ac97std_rates,
};



static unsigned int ac97std_read (struct snd_soc_codec *codec, unsigned int reg) {
	struct ac97std_priv *ac97std = snd_soc_codec_get_drvdata(codec);
	u16 *cache = codec->reg_cache;

	if ( ac97std_readable_reg (reg) == true )
  		return soc_ac97_ops->read (ac97std->ac97, reg);
	else {
		reg = reg >> 1;

		if (reg >= (ARRAY_SIZE(ac97std_reg)))
			return -EIO;

		return cache[reg];
	}
}


static int ac97std_write (struct snd_soc_codec *codec, unsigned int reg, unsigned int val) {
	struct ac97std_priv *ac97std = snd_soc_codec_get_drvdata(codec);

	u16 *cache = codec->reg_cache;
	if ( ac97std_writeable_reg (reg) == true ) {
		soc_ac97_ops->write(ac97std->ac97, reg, val);
		//reg = reg >> 1;
		if (reg < (ARRAY_SIZE(ac97std_reg)))
			cache[reg] = val;
	}
	return 0;
}


static int ac97std_startup (struct snd_pcm_substream *substream, struct snd_soc_dai *dai) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
			snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE, &ac97std_rate_constraints);
		} else {
			snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE, &ac97std_rate_constraints);
	}

	return 0;
}


static int ac97std_analog_prepare (struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ac97std_priv *ac97std = snd_soc_codec_get_drvdata(codec);
	unsigned short reg;

	/* enable variable rate audio (VRA) and disable S/PDIF output */
	soc_ac97_ops->write(ac97std->ac97, AC97_EXTENDED_STATUS,
			(soc_ac97_ops->read(ac97std->ac97, AC97_EXTENDED_STATUS) | 0x1) & ~0x4);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		reg = AC97_PCM_FRONT_DAC_RATE;
	} else {
		reg = AC97_PCM_LR_ADC_RATE;
	}

	soc_ac97_ops->write(ac97std->ac97, reg, runtime->rate);
	return 0;
}


static int ac97std_digital_prepare (struct snd_pcm_substream *substream, struct snd_soc_dai *dai) {
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;

	ac97std_write (codec, AC97_SPDIF, 0x2002);

	/* enable VRA and S/PDIF output */
	ac97std_write (codec,  AC97_EXTENDED_STATUS, ac97std_read (codec, AC97_EXTENDED_STATUS) | 0x5);

	return ac97std_write (codec,  AC97_PCM_FRONT_DAC_RATE, runtime->rate);
}


static int ac97std_set_bias_level (struct snd_soc_codec *codec,	enum snd_soc_bias_level level) {
	switch (level) {
		case SND_SOC_BIAS_ON: /* full On */
		case SND_SOC_BIAS_PREPARE: /* partial On */
		case SND_SOC_BIAS_STANDBY: /* Off, with power */
			ac97std_write (codec, AC97_POWERDOWN, 0x0000);
			break;
		case SND_SOC_BIAS_OFF: /* Off, without power */
			/* disable everything including AC link */
			ac97std_write (codec, AC97_POWERDOWN, 0xffff);
			break;
	}
	return 0;
}


static int ac97std_reset (struct snd_soc_codec *codec, int try_warm) {
	struct ac97std_priv *ac97std = snd_soc_codec_get_drvdata(codec);

	if (try_warm && soc_ac97_ops->warm_reset) {
		soc_ac97_ops->warm_reset(ac97std->ac97);
		if ( (ac97std_read (codec, AC97_RESET) & 0xF000) == 0x0 )
			return 1;
	}
	soc_ac97_ops->reset(ac97std->ac97);

	if (soc_ac97_ops->warm_reset)
		soc_ac97_ops->warm_reset(ac97std->ac97);

	if ( (ac97std_read (codec, AC97_RESET) & 0xF000) == 0x0 )
		return 0;

	return -EIO;
}


static struct snd_soc_dai_ops ac97std_dai_ops_analog = {
	.startup = ac97std_startup,
	.prepare = ac97std_analog_prepare,
};


static struct snd_soc_dai_ops ac97std_dai_ops_digital = {
	.prepare = ac97std_digital_prepare,
};


struct snd_soc_dai_driver ac97std_dai[] = {
	{
		.name = "ac97std-hifi-analog",
		//.ac97_control = 1,

		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100, //SNDRV_PCM_RATE_KNOT,
			.formats = SND_SOC_STD_AC97_FMTS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = SND_SOC_STD_AC97_FMTS,
		},

		.ops = &ac97std_dai_ops_analog,
	},
	{
		.name = "ac97std-hifi-IEC958",
		///.ac97_control = 1,

		.playback = {
			.stream_name = "ac97std IEC958",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FORMAT_IEC958_SUBFRAME_BE,
		},

		.ops = &ac97std_dai_ops_digital,
	}
};

static int ac97std_codec_suspend(struct snd_soc_codec *codec)
{
	ac97std_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ac97std_codec_resume(struct snd_soc_codec *codec)
{
	u16 id, reset;

	struct ac97std_priv *ac97std = snd_soc_codec_get_drvdata(codec);

	reset = 0;
	/* give the codec an AC97 warm reset to start the link */
reset:
	if (reset > 5) {
		printk(KERN_ERR "ac97std failed to resume");
		return -EIO;
	}
	ac97std->ac97->bus->ops->warm_reset(ac97std->ac97);
	id = ac97std_read (codec, AC97_VENDOR_ID2);

	if (id != 0x4123) {
		ac97std_reset(codec, 0);
		reset++;
		goto reset;
	}
	ac97std_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int ac97std_codec_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	u16 vendor_id;

	struct ac97std_priv *ac97std = snd_soc_codec_get_drvdata(codec);

	ac97std->ac97 = snd_soc_alloc_ac97_codec (codec);
	if ( IS_ERR(ac97std->ac97) )
		return PTR_ERR(ac97std->ac97);

	if ( !soc_ac97_ops ){
		dev_err(codec->dev, "Failed to register AC97 codec: %d\n", ret);
		return -ENOMEM;
	}

	snd_soc_codec_set_drvdata (codec, ac97std->ac97);

	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	ac97std_reset (codec, 0);
	ret = ac97std_reset (codec, 1);
	if ( ret < 0 ) {
		printk(KERN_ERR "Failed to reset ac97std: AC97 link error\n");
		goto err_put_device;
	}

	ret = device_add(&ac97std->ac97->dev);
	if (ret)
		goto err_put_device;

	vendor_id = ac97std_read (codec, AC97_VENDOR_ID1);

	/* Read out vendor IDs */
	printk (KERN_INFO "ac97std SoC Audio Codec [ID = %04x - %04x]\n",
			vendor_id,
			ac97std_read (codec, AC97_VENDOR_ID2));

		if (vendor_id == 0x414c)
				ac97std_dai->playback.rates = SNDRV_PCM_RATE_48000;
		else if (vendor_id == 0x5649)
				ac97std_dai->playback.rates = SNDRV_PCM_RATE_44100;
		else
			printk(KERN_ERR "ac97_standard: Unrecognized vendor ID, bitrate could be wrong");

	/*  Set initial state of the codec  */
	ac97std_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* unmute captures and playbacks volume */
	soc_ac97_ops->write(ac97std->ac97, AC97_MASTER, 0x0000);
	soc_ac97_ops->write(ac97std->ac97, AC97_PCM, 0x0000);
	soc_ac97_ops->write(ac97std->ac97, AC97_REC_GAIN, 0x0000);

	/* At 3.3V analog supply, for the bits 3:2 should be set 10b for the lowest power instead of default 00b */
	soc_ac97_ops->write(ac97std->ac97, AC97_AD_TEST, soc_ac97_ops->read(ac97std->ac97, AC97_AD_TEST) | 0x0008);

	/* To maximize recording quality by removing white noise */
	soc_ac97_ops->write(ac97std->ac97, AC97_AD_TEST, soc_ac97_ops->read(ac97std->ac97, AC97_AD_TEST) | 0x0400);

	return 0;

err_put_device:
	put_device(&ac97std->ac97->dev);
	return ret;
}


static int ac97std_codec_remove (struct snd_soc_codec *codec) {
	struct ac97std_priv *ac97std = snd_soc_codec_get_drvdata (codec);
	snd_soc_free_ac97_codec (ac97std->ac97);
	return 0;
}


struct snd_soc_codec_driver ac97std_codec = {
	.probe             = ac97std_codec_probe,
	.remove            = ac97std_codec_remove,
	.suspend           = ac97std_codec_suspend,
	.resume            = ac97std_codec_resume,
	.set_bias_level    = ac97std_set_bias_level,
	.read              = ac97std_read,
	.write             = ac97std_write,

	.reg_cache_size    = ARRAY_SIZE(ac97std_reg),
	.reg_word_size     = sizeof(u16),
	.reg_cache_step    = 2,
	.reg_cache_default = ac97std_reg,

	.component_driver = {
		.controls          = ac97std_snd_ac97_controls,
		.num_controls      = ARRAY_SIZE(ac97std_snd_ac97_controls),
	},
};


static int ac97std_probe (struct platform_device *pdev) {
	struct ac97std_priv *ac97std;
	ac97std = devm_kzalloc (&pdev->dev, sizeof(*ac97std), GFP_KERNEL);
	if (ac97std == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, ac97std);

	return snd_soc_register_codec (&pdev->dev,
			&ac97std_codec, ac97std_dai, ARRAY_SIZE(ac97std_dai));
}


static int ac97std_remove (struct platform_device *pdev) {
	snd_soc_unregister_codec (&pdev->dev);
	return 0;
}


static const struct of_device_id ac97_standard_of_match[] = {
	{ .compatible = "seco,ac97_standard", },
	{ }
};
MODULE_DEVICE_TABLE(of, ac97_standard_of_match);


static struct platform_driver ac97std_codec_driver = {
	.driver = {
		.name           = DRV_NAME,
		.owner          = THIS_MODULE,
		.of_match_table = ac97_standard_of_match,
	},
	.probe  = ac97std_probe,
	.remove = ac97std_remove,
};

module_platform_driver(ac97std_codec_driver);


MODULE_DESCRIPTION("ASoC AC'97 standard codec driver");
MODULE_AUTHOR("Seco s.r.l.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
