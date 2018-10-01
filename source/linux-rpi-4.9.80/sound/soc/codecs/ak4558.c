/*
 * ak4558.c  --  AK4558 ALSA Soc Audio driver
 *
 * Copyright 2017
 *
 * Author: H.J.Arlt <hj.arlt@online.de>
 *
 * Based on ak4558.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include "ak4558.h"

//#define DEBUG
//#undef dev_dbg
//#define dev_dbg dev_err

#define I2C_DEVICE

/* codec private data */
struct ak4558_priv {
        struct i2c_client *i2c;
        struct regmap *regmap;
        unsigned int sysclk;
        int mute_pin;
//      struct mutex lock;
};

#ifdef I2C_DEVICE
/*
 * Bick = 1,41 MHz = 64fs
 * FS   =   48,1 KHz
 * Sample   16 Bit, 32fs
 * CS3:0 = 0,0,0,0
 * I2S Format Mode 7
 */
#define MAX_REGS  10
static struct reg_default ak4558_reg_defaults[MAX_REGS] =
{
    /*                       D7 D6 D5 D4    D3    D2    D1    D0
        00H Power Management 0  0  0  PMADR PMADL PMDAR PMDAL RSTN */
    {0, 0x1F},  // PMADR/L=11, PMDAR/L=11, NRST=1,   L/R, pwr
    /*                    D7 D6 D5 D4  D3   D2   D1   D0
        01H PLL Control   0  0  0 PLL3 PLL2 PLL1 PLL0 PMPLL */
    {1, 0x07},  // PLL BICK 32fs=3, 256fs=0, 1=PMPLL on
    /*                    D7 D6 D5 D4  D3   D2   D1   D0
        02H DAC TDM       0  0  0  0   0    0    SDS1 SDS0 */
    {2, 0x00},  // SDS=0 R1,L2, 2 chn stereo
    /*                    D7   D6   D5   D4   D3   D2   D1   D0
        03H Control 1     TDM1 TDM0 DIF2 DIF1 DIF0 ATS1 ATS0 SMUTE */
    {3, 0x18},  // TDM=0(Stereo), DIF=3(16bit I2S), ATS=0 ATT speed, MUTE=0(off)
    /*                    D7   D6   D5   D4    D3    D2   D1   D0
        04H Control 2     0    0    0    MCKS1 MCKS0 DFS1 DFS0 ACKS */
    {4, 0x01},  // MCKS=0(master only), DFS=0(8..54kHz normal speed mode), AKS=1(man=0, auto=1 speed)
    /*                     D7  D6   D5   D4    D3    D2    D1    D0
        05H Filter setting 0   FS3  FS2  FS1   FS0   BCKO1 BCKO0 LOPS */
    {5, 0x20},  // FS3:0=4(24-54khz), BCKO=0(master only), LOPS=0(pwron)
    /*                    D7     D6     D5     D4   D3   D2    D1   D0
        06H Mode Control  FIRDA2 FIRDA1 FIRDA0 SLDA SDDA SSLOW DEM1 DEM0 */
    {6, 0x29},  // Filter DAC: FIRDA=1, SLDA=0, SDDA=1, SSLOW=0, DEM=1,  default
    /*                    D7     D6     D5     D4   D3   D2   D1    D0
        07H HPFE Enable   0      0      0      0    SLAD SDAD HPFER HPFEL */
    {7, 0x07},  // Filter ADC: SLAD=0, SDAD=1, HPFER=1, HPFEL=1,         default

    {8, 0xEF},  // LOUT 0xFF=0dB, 0x01=-127dB, -0.5dB/bit
    {9, 0xEF},  // ROUT -16dB
};

static int ak4558check(struct i2c_client *i2c)
{
    struct ak4558_priv *ak4558 = i2c_get_clientdata(i2c);
    int ret, val;
    ret = regmap_read(ak4558->regmap, REG_DAC, &val);
    if ((ret < 0) || (val & 0xFC)) {
        dev_err(&i2c->dev, "ak4558 i2c failed!\n");
            return ret;
    }
    return 0;
}

static int ak4558Init(struct i2c_client *i2c)
{
    struct ak4558_priv *ak4558 = i2c_get_clientdata(i2c);
	int val, i;

	dev_dbg(&i2c->dev, "power up\n");

	val = regmap_write(ak4558->regmap, REG_PWR, AK4558_RESET);
	if (val < 0)
			return val;
	msleep(1);

	for (i=0; i<MAX_REGS; i++) {
	    regmap_write(ak4558->regmap, ak4558_reg_defaults[i].reg , ak4558_reg_defaults[i].def);
	}
#ifdef DEBUG
	for (i=0; i<MAX_REGS; i++) {
			regmap_read(ak4558->regmap, i, &val);
			dev_dbg(&i2c->dev, "reg %d = %02X", i, val);
	}
#endif
	return 0;
}

static void ak4558regsWrite(struct snd_soc_codec *codec)
{
    struct ak4558_priv *ak4558 = dev_get_drvdata(codec->dev);
    int i;
	for (i=0; i<MAX_REGS; i++) {
	    regmap_write(ak4558->regmap, ak4558_reg_defaults[i].reg , ak4558_reg_defaults[i].def);
	}
}

static void ak4558regsRead(struct snd_soc_codec *codec)
{
#ifdef DEBUG
	struct ak4558_priv *ak4558 = dev_get_drvdata(codec->dev);
    int val, i;
    for (i=0; i<MAX_REGS; i++) {
            regmap_read(ak4558->regmap, i, &val);
            dev_dbg(codec->dev, "reg %d = %02X", i, val);
    }
#endif
}

/*
 * Playback Volume
 *
 * max : 0x00 : 0 dB
 *       ( 0.5 dB step )
 * min : 0xFE : -127.0 dB
 * mute: 0xFF
 */

static const char *ak4558_deemp[]  = {"44.1kHz", "Off", "48kHz", "32kHz"};

static const struct soc_enum ak4558_enum[] = {
    /*              xreg, xshift, xitems, xtexts */
    SOC_ENUM_SINGLE(REG_MODCTL, 0, 4, ak4558_deemp),
};

static const struct snd_kcontrol_new ak4558_snd_controls[] = {
        SOC_ENUM("Playback Deemphasis", ak4558_enum[0]),
        /*                           xname, reg_left, reg_right, xshift, xmin, xmax, xinvert */
        SOC_DOUBLE_R_RANGE("Playback Volume", REG_OUTL, REG_OUTR, 0,     0,    0xFE,    0),
};
#endif

/* Externally visible pins */
static const struct snd_soc_dapm_widget ak4558_dapm_widgets[] = {
        SND_SOC_DAPM_INPUT("AINL"),
        SND_SOC_DAPM_INPUT("AINR"),

        SND_SOC_DAPM_OUTPUT("AOUTL"),
        SND_SOC_DAPM_OUTPUT("AOUTR"),
};

/* Target, Path, Source */
static const struct snd_soc_dapm_route ak4558_dapm_routes[] = {
        { "Capture", NULL, "AINL" },
        { "Capture", NULL, "AINR" },

        { "AOUTL", NULL, "Playback" },
        { "AOUTR", NULL, "Playback" },
};

static int ak4558_mute(struct snd_soc_dai *dai, int mute)
{
        struct snd_soc_codec *codec = dai->codec;
        struct ak4558_priv *ak4558 = dev_get_drvdata(codec->dev);
        unsigned int mute_reg;

        dev_dbg(codec->dev, "mute %s..\n", mute?"on":"off");

#ifdef I2C_DEVICE
        if (gpio_is_valid(ak4558->mute_pin))
            gpio_set_value(ak4558->mute_pin, mute ? 0 : 1);

        /* set unmute */
        mute_reg = ak4558_reg_defaults[REG_CTRL1].def & ~AK4558_MUTE;
        if (mute)
        	mute_reg |= AK4558_MUTE;

        ak4558regsWrite(codec);
#endif
        return 0;
}

static int ak4558_dai_set_fmt(struct snd_soc_dai *dai, unsigned int format)
{
        struct snd_soc_codec *codec = dai->codec;
        unsigned int fmt = format & SND_SOC_DAIFMT_FORMAT_MASK;

        dev_dbg(codec->dev, "ak4558 dai_set_fmt %s..\n"
             , (fmt==SND_SOC_DAIFMT_I2S)?"I2S":(fmt==SND_SOC_DAIFMT_LEFT_J)?"MSB_16":"LSB_16");

        if (fmt != SND_SOC_DAIFMT_I2S)
                return -EINVAL;

        return 0;
}

static int ak4558_dai_hw_params(struct snd_pcm_substream *substream,
                struct snd_pcm_hw_params *params,
                struct snd_soc_dai *dai)
{
        struct snd_soc_codec *codec = dai->codec;
        struct ak4558_priv *ak4558 = dev_get_drvdata(codec->dev);
        unsigned int width = params_width(params);
        unsigned int rate = params_rate(params);
        unsigned int fs, pll;

        ak4558->sysclk = snd_soc_params_to_bclk(params);
        pll = ak4558->sysclk / rate;  // 1411200 / 44100 = 32

        /*dev_dbg(dai->dev,*/printk( "ak4558 dai_hw_params sysclk %d fs %d rate %d, width %d\n"
                          , ak4558->sysclk, pll, rate, width);

#ifdef I2C_DEVICE

        /* PLL */
        //fs = snd_soc_read(codec, REG_PLL) & ~PLL_MASK;
        fs = ak4558_reg_defaults[REG_PLL].def & ~PLL_MASK;
        switch (pll) {
        case 128:
                fs |= PLL_128FS;
                break;
        case 64:
                fs |= PLL_64FS;
                break;
        case 32:
                fs |= PLL_32FS;
                break;
        default:
                fs |= PLL_256FS;
                break;
        }
        fs |= PLL_PMPLL;
        //snd_soc_write(codec, REG_PLL, fs);
        ak4558_reg_defaults[REG_PLL].def = fs;

        /* samples */
        //fs = snd_soc_read(codec, REG_FILTER_DAC) & ~AK4558_FS;
        fs = ak4558_reg_defaults[REG_FILTER_DAC].def & ~AK4558_FS;
        switch (rate) {
        case  8000:
        case 11025:
                fs |= FS_8_13K;
                break;
        case 16000:
        case 22050:
                fs |= FS_12_27K;
                break;
        case 32000:
        case 44100:
        case 48000:
                fs |= FS_24_54K;
                break;
        case 64000:
        case 96000:
                fs |= FS_48_108K;
                break;
        case 192000:
                fs |= FS_96_216K;
                break;
        default:
                return -EINVAL;
        }
        //snd_soc_write(codec, REG_FILTER_DAC, fs);
        ak4558_reg_defaults[REG_FILTER_DAC].def = fs;

        /* De-emphasis */
        //fs = snd_soc_read(codec, REG_MODCTL) & ~DEEM_MASK;
        fs = ak4558_reg_defaults[REG_MODCTL].def & ~DEEM_MASK;
        switch (rate) {
        case  8000:
        case 11025:
        case 16000:
        case 22050:
                fs |= DEEM_OFF;
                break;
        case 32000:
                fs |= DEEM_32000;
                break;
        case 44100:
                fs |= DEEM_44100;
                break;
        case 48000:
                fs |= DEEM_48000;
                break;
        case 64000:
        case 96000:
        case 192000:
                fs |= DEEM_OFF;
                break;
        default:
                return -EINVAL;
        }
        //snd_soc_write(codec, REG_MODCTL, fs);
        ak4558_reg_defaults[REG_MODCTL].def = fs;

        ak4558regsWrite(codec);

        ak4558regsRead(codec);
#endif
        return 0;
}

static int ak4558_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
        unsigned int freq, int dir)
{
        struct snd_soc_codec *codec = dai->codec;
        struct ak4558_priv *ak4558 = snd_soc_codec_get_drvdata(codec);

        dev_dbg(dai->dev, "ak4558 set_dai_sysclk freq %d..\n", freq);
        ak4558->sysclk = freq;

        return 0;
}

static const struct snd_soc_dai_ops ak4558_dai_ops = {
	/* Called by soc_card drivers */
    .set_fmt      = ak4558_dai_set_fmt,
    .set_sysclk   = ak4558_set_dai_sysclk,
    /* Called by soc-core */
    .digital_mute = ak4558_mute,
    /* ALSA control */
    .hw_params    = ak4558_dai_hw_params,
};

#define AK4558_RATES ( SNDRV_PCM_RATE_8000   \
                     | SNDRV_PCM_RATE_11025  \
                     | SNDRV_PCM_RATE_16000  \
                     | SNDRV_PCM_RATE_22050  \
                     | SNDRV_PCM_RATE_32000  \
                     | SNDRV_PCM_RATE_44100  \
                     | SNDRV_PCM_RATE_48000  \
                     | SNDRV_PCM_RATE_64000  \
                     | SNDRV_PCM_RATE_96000  \
                     | SNDRV_PCM_RATE_192000 )

static struct snd_soc_dai_driver ak4558_dai = {
	.name = "ak4558-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = AK4558_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = AK4558_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &ak4558_dai_ops,
	.symmetric_rates = 1,
};

static int ak4558_snd_probe(struct snd_soc_codec *codec)
{
//        struct ak4558_priv *ak4558 = dev_get_drvdata(codec->dev);
        dev_dbg(codec->dev, "ak4558 snd_probe..\n");
        return 0;
}

static int ak4558_snd_remove(struct snd_soc_codec *codec)
{
//        struct ak4558_priv *ak4558 = dev_get_drvdata(codec->dev);
        dev_dbg(codec->dev, "ak4558 snd_remove..\n");
        return 0;
}

#ifdef CONFIG_PM
static int ak4558_snd_suspend(struct snd_soc_codec *codec)
{
//        struct ak4558_priv *ak4558 = dev_get_drvdata(codec->dev);
        dev_dbg(codec->dev, "ak4558 snd_suspend..\n");
        return 0;
}

static int ak4558_snd_resume(struct snd_soc_codec *codec)
{
        dev_dbg(codec->dev, "ak4558 snd_resume..\n");
        snd_soc_cache_sync(codec);
        return 0;
}
#else
#define ak4558_snd_suspend NULL
#define ak4558_snd_resume NULL
#endif

static int ak4558_set_bias_level(struct snd_soc_codec *codec,
                                 enum snd_soc_bias_level level)
{
        struct ak4558_priv *ak4558 = snd_soc_codec_get_drvdata(codec);
        int ret;

        dev_dbg(codec->dev, "ak4558 snd_bias %d..\n", level);

        switch (level) {
        case SND_SOC_BIAS_ON:
#ifdef I2C_DEVICE
				ret = ak4558Init(ak4558->i2c);
				if (ret < 0) {
						dev_err(codec->dev, "Failed to init i2c: %d\n", ret);
						return ret;
				}
				if (gpio_is_valid(ak4558->mute_pin)) {
					gpio_direction_output(ak4558->mute_pin, GPIOF_OUT_INIT_HIGH); // on
				}
#endif
                break;
        case SND_SOC_BIAS_PREPARE:
        case SND_SOC_BIAS_STANDBY:
                break;
        case SND_SOC_BIAS_OFF:
                break;
        }
        return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_ak4558 = {
        .probe = ak4558_snd_probe,
        .remove = ak4558_snd_remove,
        .suspend = ak4558_snd_suspend,
        .resume = ak4558_snd_resume,
        .set_bias_level = ak4558_set_bias_level,
//      .suspend_bias_off = true,

        .component_driver = {
#ifdef I2C_DEVICE
                .controls               = ak4558_snd_controls,
                .num_controls           = ARRAY_SIZE(ak4558_snd_controls),
#endif
                .dapm_widgets           = ak4558_dapm_widgets,
                .num_dapm_widgets       = ARRAY_SIZE(ak4558_dapm_widgets),
                .dapm_routes            = ak4558_dapm_routes,
                .num_dapm_routes        = ARRAY_SIZE(ak4558_dapm_routes),
        },
};

/*
 * ******************************************************
 */
#ifdef I2C_DEVICE
static const struct regmap_config ak4558_regmap = {
    .reg_bits           = 8,
    .val_bits           = 8,
    .max_register       = 10,
    .reg_defaults       = ak4558_reg_defaults,
    .num_reg_defaults   = ARRAY_SIZE(ak4558_reg_defaults),
    .cache_type         = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(ak4558_regmap);

static int ak4558_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
        struct device_node *np = i2c->dev.of_node;
        struct ak4558_priv *ak4558;
        int ret;

        dev_dbg(&i2c->dev, "ak4558 i2c_probe..\n");

        ak4558 = devm_kzalloc(&i2c->dev, sizeof(struct ak4558_priv),
                                GFP_KERNEL);
        if (ak4558 == NULL)
            return -ENOMEM;

        //mutex_init(&ak4558->lock);

        dev_set_drvdata(&i2c->dev, ak4558);

        ak4558->regmap = devm_regmap_init_i2c(i2c, &ak4558_regmap);
        if (IS_ERR(ak4558->regmap)) {
                ret = PTR_ERR(ak4558->regmap);
                dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
                return ret;
        }
        ak4558->i2c = i2c;
        i2c_set_clientdata(i2c, ak4558);

        ret = ak4558check(i2c);
        if (ret < 0) {
                dev_err(&i2c->dev, "Failed to init i2c: %d\n", ret);
                return ret;
        }

        /* GPIO mute on amplifyer L activ */
        if (np) {
            ak4558->mute_pin = of_get_named_gpio(np, "mute-gpios", 0);

            printk("ak4558 probe amp pin %d..\n", ak4558->mute_pin);

            if (gpio_is_valid(ak4558->mute_pin)) {
                if (devm_gpio_request(&i2c->dev, ak4558->mute_pin, "amp-mute")) {
                    dev_err(&i2c->dev, "Failed requesting gpio_mute %d\n",
                                       ak4558->mute_pin);
                } else {
                    gpio_direction_output(ak4558->mute_pin, GPIOF_OUT_INIT_LOW); // mute
                }
            }
        }

        ret = snd_soc_register_codec(&i2c->dev,
                        &soc_codec_dev_ak4558, &ak4558_dai, 1);

        ret = ak4558Init(i2c);
        if (ret < 0) {
                dev_err(&i2c->dev, "Failed to init i2c: %d\n", ret);
                return ret;
        }

        dev_dbg(&i2c->dev, "ak4558 i2c_probe exit %d\n", ret);

        if (ret != 0)
                goto err_gpio;

        return 0;

err_gpio:
        dev_err(&i2c->dev, "i2c_probe failed %d!\n", ret);

        if (gpio_is_valid(ak4558->mute_pin))
                gpio_free(ak4558->mute_pin);

        return ret;
}
EXPORT_SYMBOL_GPL(ak4558_i2c_probe);

static int ak4558_i2c_remove(struct i2c_client *client)
{
        struct ak4558_priv *ak4558 = i2c_get_clientdata(client);

        if (gpio_is_valid(ak4558->mute_pin))
                gpio_free(ak4558->mute_pin);

        snd_soc_unregister_codec(&client->dev);
        return 0;
}
EXPORT_SYMBOL_GPL(ak4558_i2c_remove);

static const struct i2c_device_id ak4558_i2c_id[] = {
    { "ak4558", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ak4558_i2c_id);

static const struct of_device_id ak4558_of_match[] = {
    { .compatible = "asahi-kasei,ak4558", },
    { }
};
MODULE_DEVICE_TABLE(of, ak4558_of_match);

static struct i2c_driver ak4558_i2c_driver = {
    .probe      = ak4558_i2c_probe,
    .remove     = ak4558_i2c_remove,
    .driver = {
        .name = "ak4558",
        .of_match_table = ak4558_of_match,
    },
    .id_table   = ak4558_i2c_id,
};
module_i2c_driver(ak4558_i2c_driver);

#else

static int ak4558_soc_probe(struct platform_device *pdev)
{
    struct ak4558_priv *ak4558;
    struct device_node *np = pdev->dev.of_node;

    ak4558 = devm_kzalloc(&pdev->dev, sizeof(struct ak4558_priv),
                  GFP_KERNEL);
    if (ak4558 == NULL)
        return -ENOMEM;

    /* GPIO mute on amplifyer L activ */
    if (np) {
        ak4558->mute_pin = of_get_named_gpio(np, "mute-gpios", 0);

        printk("ak4558 probe amp pin %d..\n", ak4558->mute_pin);

        if (gpio_is_valid(ak4558->mute_pin)) {
            if (devm_gpio_request(&i2c->dev, ak4558->mute_pin, "amp-mute")) {
                dev_err(&i2c->dev, "Failed requesting gpio_mute %d\n",
                                   ak4558->mute_pin);
            } else {
                gpio_direction_output(ak4558->mute_pin, GPIOF_OUT_INIT_LOW); // mute
            }
        }
    }

	return snd_soc_register_codec(&pdev->dev,
				      &soc_codec_dev_ak4558,
				      &ak4558_dai, 1);
}

static int ak4558_soc_remove(struct platform_device *pdev)
{
    snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id ak4558_of_match[] = {
	{ .compatible = "asahi-kasei,ak4558" },
	{},
};
MODULE_DEVICE_TABLE(of, ak4558_of_match);

static struct platform_driver ak4558_driver = {
    .probe  = ak4558_soc_probe,
    .remove = ak4558_soc_remove,
	.driver = {
		.name = "ak4558-codec",
        .owner  = THIS_MODULE,
		.of_match_table = ak4558_of_match,
	},
};
module_platform_driver(ak4558_driver);

#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SoC ak4558 driver");
MODULE_AUTHOR("Hans-Juergen Arlt <hj.arlt@online.de>");
