/*
 * ASoC Driver for RPi-AK4558.
 *
 * Author:	Hans-Jürgen Arlt hj.arlt@online.de
 *		Copyright 2017
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>

#define I2C_SUPPORT

static int snd_rpi_ak4558_init(struct snd_soc_pcm_runtime *rtd)
{
        /*dev_dbg(rtd->dev,*/printk( "rpi_ak4558 init..\n");
        return 0;
}

static int snd_rpi_ak4558_hw_params(struct snd_pcm_substream *substream,
                struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
        unsigned int sample_bits = snd_pcm_format_physical_width(
                        params_format(params));

        dev_dbg(rtd->dev, "rpi_ak4558 hw_param: sample bits %d..\n", sample_bits);

        return snd_soc_dai_set_bclk_ratio(cpu_dai, sample_bits * 2);
}

/* machine stream operations */
static struct snd_soc_ops snd_rpi_ak4558_ops =
{ .hw_params = snd_rpi_ak4558_hw_params, };

static struct snd_soc_dai_link snd_rpi_ak4558_dai[] = {
{       .name = "rpi-ak4558",
        .stream_name = "rpi-ak4558 HiFi",
        .cpu_dai_name = "bcm2708-i2s.0",
        .codec_dai_name = "ak4558-hifi",
        .platform_name = "bcm2708-i2s.0",
#ifdef I2C_SUPPORT
        .codec_name = "ak4558.1-0011",
#else
        .codec_name = "ak4558-codec",
#endif
        .dai_fmt = SND_SOC_DAIFMT_I2S
                 | SND_SOC_DAIFMT_NB_NF
                 | SND_SOC_DAIFMT_CBS_CFS   // codec is slave
                 | SND_SOC_DAIFMT_CONT,     // clk cont.
        .ops = &snd_rpi_ak4558_ops,
        .init = snd_rpi_ak4558_init,
}, };

/* audio machine driver */
static struct snd_soc_card snd_rpi_ak4558 =
{       .name = "snd_rpi_ak4558",
        .owner = THIS_MODULE,
        .dai_link = snd_rpi_ak4558_dai,
        .num_links = ARRAY_SIZE(snd_rpi_ak4558_dai),
};

static int snd_rpi_ak4558_probe(struct platform_device *pdev)
{
        int ret = 0;

        snd_rpi_ak4558.dev = &pdev->dev;

        if (pdev->dev.of_node)
        {
                struct device_node *i2s_node;
                struct snd_soc_dai_link *dai = &snd_rpi_ak4558_dai[0];
                i2s_node = of_parse_phandle(pdev->dev.of_node,
                                            "i2s-controller", 0);
                if (i2s_node)
                {
                        dai->cpu_dai_name = NULL;
                        dai->cpu_of_node = i2s_node;
                        dai->platform_name = NULL;
                        dai->platform_of_node = i2s_node;
                }
        }

        ret = snd_soc_register_card(&snd_rpi_ak4558);
        if (ret)
                dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
                                ret);

        return ret;
}

static int snd_rpi_ak4558_remove(struct platform_device *pdev)
{
        return snd_soc_unregister_card(&snd_rpi_ak4558);
}

/* snd-soc-rpi-ak4558.mod.c */
static const struct of_device_id snd_rpi_ak4558_of_match[] = {
        { .compatible = "rpi,rpi-ak4558", },
        { },
};
MODULE_DEVICE_TABLE( of, snd_rpi_ak4558_of_match);

static struct platform_driver snd_rpi_ak4558_driver =
{ .driver = {
        .name = "snd-rpi-ak4558",
        .owner = THIS_MODULE,
        .of_match_table = snd_rpi_ak4558_of_match, },
  .probe = snd_rpi_ak4558_probe,
  .remove = snd_rpi_ak4558_remove,
};

module_platform_driver( snd_rpi_ak4558_driver);

MODULE_AUTHOR("Hans-Jürgen Arlt");
MODULE_DESCRIPTION("ASoC Driver for RPi-AK4558");
MODULE_LICENSE("GPL v2");
