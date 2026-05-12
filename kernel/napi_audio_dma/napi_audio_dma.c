// SPDX-License-Identifier: GPL-2.0
//
// napi_audio_dma:
//   Kernel-space audio bridge for MCP3201 (ADC) and MCP4822 (DAC) over SPI.
//   Exposes one misc char device: /dev/napi_audio_dma0
//
//   - read()  -> ADC mode:  returns S16LE mono frames
//   - write() -> DAC mode:  accepts S16LE mono frames
//
// Intended usage:
//   1) ioctl(CONFIG)
//   2) ioctl(START)
//   3) read()/write() blocks of audio
//   4) ioctl(STOP)
//
// Notes:
//   - SPI bus mode 0, 8 bits per word.
//   - MCP3201 and MCP4822 need a CS frame per 16-bit sample. A single long
//     SPI transfer keeps CS asserted and produces invalid audio.
//   - The driver therefore submits one SPI message containing many 2-byte
//     transfers with CS toggled between samples.

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/device/bus.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

#include "include/napi_audio_dma_uapi.h"

#define NAPI_DMA_NAME "napi_audio_dma0"
#define NAPI_DMA_DEFAULT_FS 8000u
#define NAPI_DMA_DEFAULT_SPI_HZ (NAPI_DMA_DEFAULT_FS * 16u)
#define NAPI_DMA_MAX_XFER_BYTES 4096u
#define NAPI_DMA_CS_HIGH_US 1u

static int bus_num = 2;
module_param(bus_num, int, 0644);
MODULE_PARM_DESC(bus_num, "SPI bus number (default 2)");

static int chip_select = 0;
module_param(chip_select, int, 0644);
MODULE_PARM_DESC(chip_select, "SPI chip select (default 0)");

static int spi_mode = 0;
module_param(spi_mode, int, 0644);
MODULE_PARM_DESC(spi_mode, "SPI mode 0..3 (default 0)");

struct napi_dma_dev {
	struct miscdevice misc;
	struct mutex lock;

	struct spi_device *spi;

	struct napi_audio_dma_config cfg;
	bool started;

	u8 *adc_tx;
	u8 *adc_rx;
	s16 *adc_pcm;
	struct spi_transfer *adc_trs;

	s16 *dac_pcm;
	u8 *dac_tx;
	struct spi_transfer *dac_trs;
};

static struct napi_dma_dev g_dev;

static int napi_dma_alloc_buffers(struct napi_dma_dev *d)
{
	const size_t max_bytes = NAPI_DMA_MAX_XFER_BYTES;
	const size_t max_frames = max_bytes / 2;

	d->adc_tx = kzalloc(max_bytes, GFP_KERNEL);
	d->adc_rx = kzalloc(max_bytes, GFP_KERNEL);
	d->adc_pcm = kmalloc(max_bytes, GFP_KERNEL);
	d->adc_trs = kcalloc(max_frames, sizeof(*d->adc_trs), GFP_KERNEL);

	d->dac_pcm = kmalloc(max_bytes, GFP_KERNEL);
	d->dac_tx = kmalloc(max_bytes, GFP_KERNEL);
	d->dac_trs = kcalloc(max_frames, sizeof(*d->dac_trs), GFP_KERNEL);

	if (!d->adc_tx || !d->adc_rx || !d->adc_pcm || !d->adc_trs ||
	    !d->dac_pcm || !d->dac_tx || !d->dac_trs)
		return -ENOMEM;

	return 0;
}

static void napi_dma_free_buffers(struct napi_dma_dev *d)
{
	kfree(d->dac_trs);
	kfree(d->dac_tx);
	kfree(d->dac_pcm);
	kfree(d->adc_trs);
	kfree(d->adc_pcm);
	kfree(d->adc_rx);
	kfree(d->adc_tx);

	d->adc_tx = NULL;
	d->adc_rx = NULL;
	d->adc_pcm = NULL;
	d->adc_trs = NULL;
	d->dac_pcm = NULL;
	d->dac_tx = NULL;
	d->dac_trs = NULL;
}

static inline u16 mcp3201_parse_u12(const u8 *rx2)
{
	return (u16)(((rx2[0] & 0x1F) << 7) | ((rx2[1] >> 1) & 0x7F));
}

static inline u16 mcp4822_word(bool ch_b, u16 u12)
{
	/* [15]=A/B, [14]=BUF(0), [13]=GA(1=1x), [12]=SHDN(1=active) */
	return (u16)(((ch_b ? 1 : 0) << 15) | (1 << 13) | (1 << 12) | (u12 & 0x0FFF));
}

static inline u16 s16_to_u12(s16 s)
{
	int v = (int)s + 32768; /* [0..65535] */
	v = (v * 4095 + 32767) / 65535;
	if (v < 0)
		v = 0;
	if (v > 4095)
		v = 4095;
	return (u16)v;
}

static u16 napi_dma_adc_sample_delay_us(const struct napi_dma_dev *d)
{
	u32 fs = d->cfg.sample_rate_hz ? d->cfg.sample_rate_hz : NAPI_DMA_DEFAULT_FS;
	u32 hz = d->spi->max_speed_hz ? d->spi->max_speed_hz : NAPI_DMA_DEFAULT_SPI_HZ;
	u64 sample_ns;
	u64 frame_ns;
	u64 delay_ns;
	u64 delay_us;

	if (!fs || !hz)
		return 0;

	/*
	 * ADC side only: keep SPI bit clock high for signal integrity, but place
	 * 16-bit conversion frames at requested sample_rate_hz.
	 */
	sample_ns = div_u64(1000000000ull, fs);
	frame_ns = div_u64(16ull * 1000000000ull + (u64)hz - 1ull, hz);
	if (sample_ns <= frame_ns)
		return 0;

	delay_ns = sample_ns - frame_ns;
	delay_us = div_u64(delay_ns + 999ull, 1000ull);
	if (delay_us > 65535ull)
		delay_us = 65535ull;
	return (u16)delay_us;
}

static int napi_dma_apply_spi_cfg_locked(struct napi_dma_dev *d)
{
	u32 hz = d->cfg.spi_speed_hz;

	if (!d->spi)
		return -ENODEV;

	if (hz == 0) {
		u32 fs = d->cfg.sample_rate_hz ? d->cfg.sample_rate_hz : NAPI_DMA_DEFAULT_FS;
		hz = fs * 16u;
	}

	d->spi->mode = (u16)(spi_mode & 0x3);
	d->spi->bits_per_word = 8;
	d->spi->max_speed_hz = hz;

	return spi_setup(d->spi);
}

static ssize_t napi_dma_read_adc_locked(struct napi_dma_dev *d, char __user *buf, size_t count)
{
	size_t n;
	u8 *tx;
	u8 *rx;
	s16 *pcm;
	struct spi_transfer *trs;
	int ret;
	size_t i;
	size_t frames;
	u16 sample_delay_us;

	if (!d->started)
		return -EPIPE;
	if (d->cfg.mode != NAPI_AUDIO_DMA_MODE_ADC)
		return -EINVAL;
	if (count < 2)
		return 0;

	n = count & ~((size_t)1); /* even bytes only */
	if (n > NAPI_DMA_MAX_XFER_BYTES)
		n = NAPI_DMA_MAX_XFER_BYTES;

	tx = d->adc_tx;
	rx = d->adc_rx;
	pcm = d->adc_pcm;
	trs = d->adc_trs;
	if (!tx || !rx || !pcm || !trs)
		return -ENOMEM;

	frames = n / 2;
	memset(tx, 0, n);

	sample_delay_us = napi_dma_adc_sample_delay_us(d);

	for (i = 0; i < frames; ++i) {
		trs[i].tx_buf = &tx[i * 2];
		trs[i].rx_buf = &rx[i * 2];
		trs[i].len = 2;
		trs[i].speed_hz = d->spi->max_speed_hz;
		trs[i].bits_per_word = 8;
		trs[i].cs_change = (i + 1 < frames) ? 1 : 0;
		if (sample_delay_us && i + 1 < frames) {
			trs[i].cs_change_delay.value = sample_delay_us;
			trs[i].cs_change_delay.unit = SPI_DELAY_UNIT_USECS;
		}
	}

	ret = spi_sync_transfer(d->spi, trs, frames);
	if (ret < 0)
		return ret;

	for (i = 0; i < frames; ++i) {
		u16 u12 = mcp3201_parse_u12(&rx[i * 2]);
		pcm[i] = (s16)(((int)u12 - 2048) << 4);
	}

	if (copy_to_user(buf, pcm, n)) {
		ret = -EFAULT;
		return ret;
	}

	ret = (int)n;
	return ret;
}

static ssize_t napi_dma_write_dac_locked(struct napi_dma_dev *d, const char __user *buf, size_t count)
{
	size_t n;
	s16 *pcm;
	u8 *tx;
	struct spi_transfer *trs;
	bool ch_b;
	int ret;
	size_t i;
	size_t frames;

	if (!d->started)
		return -EPIPE;
	if (d->cfg.mode != NAPI_AUDIO_DMA_MODE_DAC)
		return -EINVAL;
	if (count < 2)
		return 0;

	n = count & ~((size_t)1);
	if (n > NAPI_DMA_MAX_XFER_BYTES)
		n = NAPI_DMA_MAX_XFER_BYTES;

	pcm = d->dac_pcm;
	tx = d->dac_tx;
	trs = d->dac_trs;
	if (!pcm || !tx || !trs)
		return -ENOMEM;

	frames = n / 2;

	if (copy_from_user(pcm, buf, n)) {
		ret = -EFAULT;
		return ret;
	}

	ch_b = (d->cfg.flags & NAPI_AUDIO_DMA_F_DAC_CH_B) != 0;
	for (i = 0; i < frames; ++i) {
		u16 u12 = s16_to_u12(pcm[i]);
		u16 w = mcp4822_word(ch_b, u12);
		tx[i * 2 + 0] = (u8)(w >> 8);
		tx[i * 2 + 1] = (u8)(w & 0xFF);
	}

	for (i = 0; i < frames; ++i) {
		trs[i].tx_buf = &tx[i * 2];
		trs[i].len = 2;
		trs[i].speed_hz = d->spi->max_speed_hz;
		trs[i].bits_per_word = 8;
		trs[i].cs_change = (i + 1 < frames) ? 1 : 0;
		if (i + 1 < frames) {
			trs[i].cs_change_delay.value = NAPI_DMA_CS_HIGH_US;
			trs[i].cs_change_delay.unit = SPI_DELAY_UNIT_USECS;
		}
	}

	ret = spi_sync_transfer(d->spi, trs, frames);
	if (ret < 0)
		return ret;

	ret = (int)n;
	return ret;
}

static int napi_dma_open(struct inode *inode, struct file *file)
{
	file->private_data = &g_dev;
	return 0;
}

static long napi_dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct napi_dma_dev *d = file->private_data;
	long ret = 0;

	if (!d)
		return -ENODEV;

	mutex_lock(&d->lock);

	switch (cmd) {
	case NAPI_AUDIO_DMA_IOC_CONFIG: {
		struct napi_audio_dma_config cfg;

		if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg))) {
			ret = -EFAULT;
			break;
		}
		if (cfg.api_version != NAPI_AUDIO_DMA_API_VERSION) {
			ret = -EINVAL;
			break;
		}
		if (cfg.mode != NAPI_AUDIO_DMA_MODE_ADC &&
		    cfg.mode != NAPI_AUDIO_DMA_MODE_DAC) {
			ret = -EINVAL;
			break;
		}
		if (cfg.sample_rate_hz == 0)
			cfg.sample_rate_hz = NAPI_DMA_DEFAULT_FS;

		d->cfg = cfg;
		ret = napi_dma_apply_spi_cfg_locked(d);
		break;
	}

	case NAPI_AUDIO_DMA_IOC_START:
		if (d->cfg.api_version != NAPI_AUDIO_DMA_API_VERSION) {
			ret = -EINVAL;
			break;
		}
		ret = napi_dma_apply_spi_cfg_locked(d);
		if (ret == 0)
			d->started = true;
		break;

	case NAPI_AUDIO_DMA_IOC_STOP:
		d->started = false;
		ret = 0;
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&d->lock);
	return ret;
}

static ssize_t napi_dma_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct napi_dma_dev *d = file->private_data;
	ssize_t ret;

	if (!d)
		return -ENODEV;

	mutex_lock(&d->lock);
	ret = napi_dma_read_adc_locked(d, buf, count);
	mutex_unlock(&d->lock);
	return ret;
}

static ssize_t napi_dma_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct napi_dma_dev *d = file->private_data;
	ssize_t ret;

	if (!d)
		return -ENODEV;

	mutex_lock(&d->lock);
	ret = napi_dma_write_dac_locked(d, buf, count);
	mutex_unlock(&d->lock);
	return ret;
}

static const struct file_operations napi_dma_fops = {
	.owner          = THIS_MODULE,
	.open           = napi_dma_open,
	.read           = napi_dma_read,
	.write          = napi_dma_write,
	.unlocked_ioctl = napi_dma_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = napi_dma_ioctl,
#endif
};

static int napi_dma_bind_spi(struct napi_dma_dev *d)
{
	char spi_name[32];
	struct device *dev;
	struct spi_device *spi;

	snprintf(spi_name, sizeof(spi_name), "spi%d.%d", bus_num, chip_select);
	dev = bus_find_device_by_name(&spi_bus_type, NULL, spi_name);
	if (!dev)
		return -ENODEV;

	spi = to_spi_device(dev);
	d->spi = spi_dev_get(spi);
	put_device(dev);
	if (!d->spi)
		return -ENODEV;

	d->spi->mode = (u16)(spi_mode & 0x3);
	d->spi->bits_per_word = 8;
	d->spi->max_speed_hz = NAPI_DMA_DEFAULT_SPI_HZ;
	return spi_setup(d->spi);
}

static void napi_dma_unbind_spi(struct napi_dma_dev *d)
{
	if (d->spi) {
		spi_dev_put(d->spi);
		d->spi = NULL;
	}
}

static int __init napi_dma_init(void)
{
	int ret;

	memset(&g_dev, 0, sizeof(g_dev));
	mutex_init(&g_dev.lock);

	g_dev.cfg.api_version = NAPI_AUDIO_DMA_API_VERSION;
	g_dev.cfg.mode = NAPI_AUDIO_DMA_MODE_ADC;
	g_dev.cfg.sample_rate_hz = NAPI_DMA_DEFAULT_FS;
	g_dev.cfg.spi_speed_hz = NAPI_DMA_DEFAULT_SPI_HZ;
	g_dev.cfg.flags = NAPI_AUDIO_DMA_F_ADC_PARSE_MCP3201;

	ret = napi_dma_bind_spi(&g_dev);
	if (ret) {
		pr_err("napi_audio_dma: failed to bind SPI bus=%d cs=%d: %d\n",
		       bus_num, chip_select, ret);
		return ret;
	}

	ret = napi_dma_alloc_buffers(&g_dev);
	if (ret) {
		pr_err("napi_audio_dma: buffer allocation failed: %d\n", ret);
		napi_dma_unbind_spi(&g_dev);
		return ret;
	}

	g_dev.misc.minor = MISC_DYNAMIC_MINOR;
	g_dev.misc.name = NAPI_DMA_NAME;
	g_dev.misc.fops = &napi_dma_fops;
	g_dev.misc.mode = 0660;

	ret = misc_register(&g_dev.misc);
	if (ret) {
		pr_err("napi_audio_dma: misc_register failed: %d\n", ret);
		napi_dma_free_buffers(&g_dev);
		napi_dma_unbind_spi(&g_dev);
		return ret;
	}

	pr_info("napi_audio_dma: loaded (/dev/%s), spi=%d.%d mode=%d\n",
		NAPI_DMA_NAME, bus_num, chip_select, spi_mode & 0x3);
	return 0;
}

static void __exit napi_dma_exit(void)
{
	mutex_lock(&g_dev.lock);
	g_dev.started = false;
	mutex_unlock(&g_dev.lock);

	misc_deregister(&g_dev.misc);
	napi_dma_free_buffers(&g_dev);
	napi_dma_unbind_spi(&g_dev);
	pr_info("napi_audio_dma: unloaded\n");
}

module_init(napi_dma_init);
module_exit(napi_dma_exit);

MODULE_AUTHOR("Mobile-radiostation");
MODULE_DESCRIPTION("Kernel-space SPI audio bridge with DMA-eligible transfers");
MODULE_LICENSE("GPL");
