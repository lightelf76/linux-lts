// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021-2023 Samuel Holland
 *
 * Partly based on drivers/leds/leds-turris-omnia.c, which is:
 * Copyright (c) 2020 by Marek Behún
 */

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>          /* usleep_range() */
#include <linux/dma-mapping.h>    /* dmam_alloc_attrs(), DMA_ATTR_WRITE_COMBINE */
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/interrupt.h>      /* irqreturn_t, IRQ_HANDLED, devm_request_irq() */
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define LEDC_CTRL_REG			0x0000
#define LEDC_CTRL_REG_DATA_LENGTH	GENMASK(28, 16)
#define LEDC_CTRL_REG_RGB_MODE		GENMASK(8, 6)
#define LEDC_CTRL_REG_LEDC_EN		BIT(0)

#define LEDC_T01_TIMING_CTRL_REG	0x0004
#define LEDC_T01_TIMING_CTRL_REG_T1H	GENMASK(26, 21)
#define LEDC_T01_TIMING_CTRL_REG_T1L	GENMASK(20, 16)
#define LEDC_T01_TIMING_CTRL_REG_T0H	GENMASK(10, 6)
#define LEDC_T01_TIMING_CTRL_REG_T0L	GENMASK(5, 0)

#define LEDC_RESET_TIMING_CTRL_REG	0x000c
#define LEDC_RESET_TIMING_CTRL_REG_TR		GENMASK(28, 16)
#define LEDC_RESET_TIMING_CTRL_REG_LED_NUM	GENMASK(9, 0)

#define LEDC_DATA_REG			0x0014

#define LEDC_DMA_CTRL_REG		0x0018
#define LEDC_DMA_CTRL_REG_DMA_EN	BIT(5)
#define LEDC_DMA_CTRL_REG_FIFO_TRIG_LEVEL	GENMASK(4, 0)

#define LEDC_INT_CTRL_REG		0x001c
#define LEDC_INT_CTRL_REG_GLOBAL_INT_EN		BIT(5)
#define LEDC_INT_CTRL_REG_FIFO_CPUREQ_INT_EN	BIT(1)
#define LEDC_INT_CTRL_REG_TRANS_FINISH_INT_EN	BIT(0)

#define LEDC_INT_STS_REG		0x0020
#define LEDC_INT_STS_REG_FIFO_WLW		GENMASK(15, 10)
#define LEDC_INT_STS_REG_FIFO_CPUREQ_INT	BIT(1)
#define LEDC_INT_STS_REG_TRANS_FINISH_INT	BIT(0)

#define LEDC_FIFO_DEPTH			32U
#define LEDC_MAX_LEDS			1024
#define LEDC_CHANNELS_PER_LED		3 /* RGB */

/* Каждый светодиод кодируется одним 32-битным словом (младшие 24 бита) */
#define LEDS_TO_BYTES(n)		((n) * sizeof(u32))

struct sun50i_a100_ledc_timing {
	u32 t0h_ns;
	u32 t0l_ns;
	u32 t1h_ns;
	u32 t1l_ns;
	u32 treset_ns;
};

struct sun50i_a100_ledc {
	struct device *dev;
	void __iomem *base;
	struct clk *bus_clk;
	struct clk *mod_clk;
	struct reset_control *reset;
	u32 *buffer;
	struct dma_chan *dma_chan;
	dma_addr_t dma_handle;
	unsigned int pio_length;
	unsigned int pio_offset;
	spinlock_t irq_lock;
	unsigned int next_length;
	bool xfer_active;
	u32 format;
	struct sun50i_a100_ledc_timing timing;
	u32 max_addr;		/* последнее использованное количество светодиодов - 1 */
	u32 num_leds;		/* последнее использованное количество светодиодов */

	/* символьное устройство */
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct device *dev_node;
        struct mutex write_lock;   
};

static int sun50i_a100_ledc_dma_xfer(struct sun50i_a100_ledc *priv, unsigned int length)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;

	desc = dmaengine_prep_slave_single(priv->dma_chan, priv->dma_handle,
					   LEDS_TO_BYTES(length), DMA_MEM_TO_DEV, 0);
	if (!desc)
		return -ENOMEM;

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie))
		return -EIO;

	dma_async_issue_pending(priv->dma_chan);
	return 0;
}

static void sun50i_a100_ledc_pio_xfer(struct sun50i_a100_ledc *priv, unsigned int fifo_used)
{
	unsigned int burst, length, offset;
	u32 control;

	length = priv->pio_length;
	offset = priv->pio_offset;
	burst = min(length, LEDC_FIFO_DEPTH - fifo_used);
	iowrite32_rep(priv->base + LEDC_DATA_REG, priv->buffer + offset, burst);

	if (burst < length) {
		priv->pio_length = length - burst;
		priv->pio_offset = offset + burst;
		if (!offset) {
			control = readl(priv->base + LEDC_INT_CTRL_REG);
			control |= LEDC_INT_CTRL_REG_FIFO_CPUREQ_INT_EN;
			writel(control, priv->base + LEDC_INT_CTRL_REG);
		}
	} else {
		/* Disable the request IRQ once all data is written. */
		control = readl(priv->base + LEDC_INT_CTRL_REG);
		control &= ~LEDC_INT_CTRL_REG_FIFO_CPUREQ_INT_EN;
		writel(control, priv->base + LEDC_INT_CTRL_REG);
	}
}

static void sun50i_a100_ledc_start_xfer(struct sun50i_a100_ledc *priv, unsigned int length)
{
	bool use_dma = false;
	u32 control;

	if (priv->dma_chan && length > LEDC_FIFO_DEPTH) {
		int ret;

		ret = sun50i_a100_ledc_dma_xfer(priv, length);
		if (ret)
			dev_warn(priv->dev, "Failed to set up DMA (%d), using PIO\n", ret);
		else
			use_dma = true;
	}

	/* The DMA trigger level must be at least the burst length. */
	control = FIELD_PREP(LEDC_DMA_CTRL_REG_DMA_EN, use_dma) |
		  FIELD_PREP_CONST(LEDC_DMA_CTRL_REG_FIFO_TRIG_LEVEL, LEDC_FIFO_DEPTH / 2);
	writel(control, priv->base + LEDC_DMA_CTRL_REG);

	control = readl(priv->base + LEDC_CTRL_REG);
	control &= ~LEDC_CTRL_REG_DATA_LENGTH;
	control |= FIELD_PREP(LEDC_CTRL_REG_DATA_LENGTH, length) | LEDC_CTRL_REG_LEDC_EN;
	writel(control, priv->base + LEDC_CTRL_REG);

	if (!use_dma) {
		/* The FIFO is empty when starting a new transfer. */
		unsigned int fifo_used = 0;

		priv->pio_length = length;
		priv->pio_offset = 0;
		sun50i_a100_ledc_pio_xfer(priv, fifo_used);
	}
}

static irqreturn_t sun50i_a100_ledc_irq(int irq, void *data)
{
	struct sun50i_a100_ledc *priv = data;
	u32 status;

	status = readl(priv->base + LEDC_INT_STS_REG);

	if (status & LEDC_INT_STS_REG_TRANS_FINISH_INT) {
		unsigned int next_length;

		spin_lock(&priv->irq_lock);
		/* If another transfer is queued, dequeue and start it. */
		next_length = priv->next_length;
		if (next_length)
			priv->next_length = 0;
		else
			priv->xfer_active = false;
		spin_unlock(&priv->irq_lock);

		if (next_length)
			sun50i_a100_ledc_start_xfer(priv, next_length);
	} else if (status & LEDC_INT_STS_REG_FIFO_CPUREQ_INT) {
		/* Continue the current transfer. */
		sun50i_a100_ledc_pio_xfer(priv, FIELD_GET(LEDC_INT_STS_REG_FIFO_WLW, status));
	}

	/* Clear the W1C status bits. */
	writel(status, priv->base + LEDC_INT_STS_REG);

	return IRQ_HANDLED;
}

static const char *const sun50i_a100_ledc_formats[] = {
	"rgb", "rbg", "grb", "gbr", "brg", "bgr",
};

static int sun50i_a100_ledc_parse_format(struct device *dev,
					 struct sun50i_a100_ledc *priv)
{
	const char *format = "grb";
	int i;

	device_property_read_string(dev, "allwinner,pixel-format", &format);
	i = match_string(sun50i_a100_ledc_formats, ARRAY_SIZE(sun50i_a100_ledc_formats),
			 format);
	if (i < 0)
		return dev_err_probe(dev, i, "Bad pixel format '%s'\n", format);

	priv->format = i;
	return 0;
}

static void sun50i_a100_ledc_set_format(struct sun50i_a100_ledc *priv)
{
	u32 control;

	control = readl(priv->base + LEDC_CTRL_REG);
	control &= ~LEDC_CTRL_REG_RGB_MODE;
	control |= FIELD_PREP(LEDC_CTRL_REG_RGB_MODE, priv->format);
	writel(control, priv->base + LEDC_CTRL_REG);
}

static const struct sun50i_a100_ledc_timing sun50i_a100_ledc_default_timing = {
	.t0h_ns = 336,
	.t0l_ns = 840,
	.t1h_ns = 882,
	.t1l_ns = 294,
	.treset_ns = 300000,
};

static int sun50i_a100_ledc_parse_timing(struct device *dev,
					 struct sun50i_a100_ledc *priv)
{
	struct sun50i_a100_ledc_timing *timing = &priv->timing;

	*timing = sun50i_a100_ledc_default_timing;
	device_property_read_u32(dev, "allwinner,t0h-ns", &timing->t0h_ns);
	device_property_read_u32(dev, "allwinner,t0l-ns", &timing->t0l_ns);
	device_property_read_u32(dev, "allwinner,t1h-ns", &timing->t1h_ns);
	device_property_read_u32(dev, "allwinner,t1l-ns", &timing->t1l_ns);
	device_property_read_u32(dev, "allwinner,treset-ns", &timing->treset_ns);

	return 0;
}

static void sun50i_a100_ledc_set_timing(struct sun50i_a100_ledc *priv)
{
	const struct sun50i_a100_ledc_timing *timing = &priv->timing;
	unsigned long mod_freq = clk_get_rate(priv->mod_clk);
	u32 cycle_ns;
	u32 control;

	if (!mod_freq)
		return;

	cycle_ns = NSEC_PER_SEC / mod_freq;

	control = FIELD_PREP(LEDC_T01_TIMING_CTRL_REG_T1H, timing->t1h_ns / cycle_ns) |
		  FIELD_PREP(LEDC_T01_TIMING_CTRL_REG_T1L, timing->t1l_ns / cycle_ns) |
		  FIELD_PREP(LEDC_T01_TIMING_CTRL_REG_T0H, timing->t0h_ns / cycle_ns) |
		  FIELD_PREP(LEDC_T01_TIMING_CTRL_REG_T0L, timing->t0l_ns / cycle_ns);
	writel(control, priv->base + LEDC_T01_TIMING_CTRL_REG);

	/*
	 * LED_NUM теперь устанавливается динамически в ledc_write().
	 * Здесь настраиваем только время сброса (TR).
	 */
	control = FIELD_PREP(LEDC_RESET_TIMING_CTRL_REG_TR, timing->treset_ns / cycle_ns);
	writel(control, priv->base + LEDC_RESET_TIMING_CTRL_REG);
}

static int sun50i_a100_ledc_resume(struct device *dev)
{
	struct sun50i_a100_ledc *priv = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(priv->reset);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->bus_clk);
	if (ret)
		goto err_assert_reset;

	ret = clk_prepare_enable(priv->mod_clk);
	if (ret)
		goto err_disable_bus_clk;

	sun50i_a100_ledc_set_format(priv);
	sun50i_a100_ledc_set_timing(priv);

	/* Восстанавливаем количество светодиодов, если оно уже было установлено */
	if (priv->num_leds) {
		u32 val = readl(priv->base + LEDC_RESET_TIMING_CTRL_REG);
		val &= ~LEDC_RESET_TIMING_CTRL_REG_LED_NUM;
		val |= FIELD_PREP(LEDC_RESET_TIMING_CTRL_REG_LED_NUM, priv->max_addr);
		writel(val, priv->base + LEDC_RESET_TIMING_CTRL_REG);
	}

	writel(LEDC_INT_CTRL_REG_GLOBAL_INT_EN | LEDC_INT_CTRL_REG_TRANS_FINISH_INT_EN,
	       priv->base + LEDC_INT_CTRL_REG);

	return 0;

err_disable_bus_clk:
	clk_disable_unprepare(priv->bus_clk);
err_assert_reset:
	reset_control_assert(priv->reset);
	return ret;
}

static int sun50i_a100_ledc_suspend(struct device *dev)
{
	struct sun50i_a100_ledc *priv = dev_get_drvdata(dev);

	/* Wait for all transfers to complete. */
	for (;;) {
		unsigned long flags;
		bool xfer_active;

		spin_lock_irqsave(&priv->irq_lock, flags);
		xfer_active = priv->xfer_active;
		spin_unlock_irqrestore(&priv->irq_lock, flags);

		if (!xfer_active)
			break;

		usleep_range(1000, 1100);
	}

	clk_disable_unprepare(priv->mod_clk);
	clk_disable_unprepare(priv->bus_clk);
	reset_control_assert(priv->reset);

	return 0;
}

static void sun50i_a100_ledc_dma_cleanup(void *data)
{
	struct sun50i_a100_ledc *priv = data;

	dma_release_channel(priv->dma_chan);
}

/* ---------- Символьное устройство /dev/ledc ---------- */

static int ledc_open(struct inode *inode, struct file *file)
{
	struct sun50i_a100_ledc *priv = container_of(inode->i_cdev,
						     struct sun50i_a100_ledc, cdev);
	file->private_data = priv;
	return 0;
}

static int ledc_release(struct inode *inode, struct file *file)
{
	return 0;
}
/*
static ssize_t ledc_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct sun50i_a100_ledc *priv = file->private_data;
	size_t bytes_per_led = sizeof(u32); 
	size_t max_bytes = LEDC_MAX_LEDS * bytes_per_led;
	unsigned int num_leds;
	unsigned long flags;
	unsigned int next_length;
	bool xfer_active;

	if (count == 0 || count % bytes_per_led != 0)
		return -EINVAL;

	if (count > max_bytes)
		return -EFBIG;

	num_leds = count / bytes_per_led;

	if (copy_from_user(priv->buffer, buf, count))
		return -EFAULT;

	priv->num_leds = num_leds;
	priv->max_addr = num_leds - 1;

	{
		u32 val = readl(priv->base + LEDC_RESET_TIMING_CTRL_REG);
		val &= ~LEDC_RESET_TIMING_CTRL_REG_LED_NUM;
		val |= FIELD_PREP(LEDC_RESET_TIMING_CTRL_REG_LED_NUM, priv->max_addr);
		writel(val, priv->base + LEDC_RESET_TIMING_CTRL_REG);
	}

	spin_lock_irqsave(&priv->irq_lock, flags);

	next_length = max(priv->next_length, num_leds);
	xfer_active = priv->xfer_active;

	if (xfer_active)
		priv->next_length = next_length;
	else
		priv->xfer_active = true;

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	if (!xfer_active)
		sun50i_a100_ledc_start_xfer(priv, next_length);

	return count;
}
*/

static ssize_t ledc_write(struct file *file, const char __user *buf,
                          size_t count, loff_t *ppos)
{
    struct sun50i_a100_ledc *priv = file->private_data;
    size_t bytes_per_led = sizeof(u32);
    size_t max_bytes = LEDC_MAX_LEDS * bytes_per_led;
    unsigned int num_leds, val;

    if (count == 0 || count % bytes_per_led != 0)
        return -EINVAL;
    if (count > max_bytes)
        return -EFBIG;

    num_leds = count / bytes_per_led;

    /* Сериализуем записи и ждём завершения предыдущей передачи */
    if (mutex_lock_interruptible(&priv->write_lock))
        return -ERESTARTSYS;

    /* Ждём, пока предыдущая передача завершится (или таймаут) */
    {
        unsigned long timeout = jiffies + msecs_to_jiffies(1000);
        while (priv->xfer_active) {
            if (time_after(jiffies, timeout)) {
                mutex_unlock(&priv->write_lock);
                return -ETIMEDOUT;
            }
            usleep_range(1000, 2000);
        }
    }

    if (copy_from_user(priv->buffer, buf, count)) {
        mutex_unlock(&priv->write_lock);
        return -EFAULT;
    }

    /* Синхронизация DMA перед запуском */
    if (priv->dma_chan)
        dma_sync_single_for_device(dmaengine_get_dma_device(priv->dma_chan),
                                   priv->dma_handle, count, DMA_TO_DEVICE);

    priv->num_leds = num_leds;
    priv->max_addr = num_leds - 1;

    val = readl(priv->base + LEDC_RESET_TIMING_CTRL_REG);
    val &= ~LEDC_RESET_TIMING_CTRL_REG_LED_NUM;
    val |= FIELD_PREP(LEDC_RESET_TIMING_CTRL_REG_LED_NUM, priv->max_addr);
    writel(val, priv->base + LEDC_RESET_TIMING_CTRL_REG);

    priv->xfer_active = true;
    sun50i_a100_ledc_start_xfer(priv, num_leds);

    mutex_unlock(&priv->write_lock);
    return count;
}
static const struct file_operations ledc_fops = {
	.owner   = THIS_MODULE,
	.open    = ledc_open,
	.write   = ledc_write,
	.release = ledc_release,
};

/* ---------- Probe / Remove ---------- */

static int sun50i_a100_ledc_probe(struct platform_device *pdev)
{
	struct dma_slave_config dma_cfg = {};
	struct device *dev = &pdev->dev;
	struct sun50i_a100_ledc *priv;
	struct resource *mem;
	int irq, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	spin_lock_init(&priv->irq_lock);
	mutex_init(&priv->write_lock);
	dev_set_drvdata(dev, priv);

	ret = sun50i_a100_ledc_parse_format(dev, priv);
	if (ret)
		return ret;

	ret = sun50i_a100_ledc_parse_timing(dev, priv);
	if (ret)
		return ret;

	priv->base = devm_platform_get_and_ioremap_resource(pdev, 0, &mem);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->bus_clk = devm_clk_get(dev, "bus");
	if (IS_ERR(priv->bus_clk))
		return PTR_ERR(priv->bus_clk);

	priv->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(priv->mod_clk))
		return PTR_ERR(priv->mod_clk);

	priv->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->reset))
		return PTR_ERR(priv->reset);

	priv->dma_chan = dma_request_chan(dev, "tx");
	if (IS_ERR(priv->dma_chan)) {
		if (PTR_ERR(priv->dma_chan) != -ENODEV)
			return PTR_ERR(priv->dma_chan);
		priv->dma_chan = NULL;
		priv->buffer = devm_kzalloc(dev, LEDS_TO_BYTES(LEDC_MAX_LEDS),
					    GFP_KERNEL);
		if (!priv->buffer)
			return -ENOMEM;
	} else {
		ret = devm_add_action_or_reset(dev, sun50i_a100_ledc_dma_cleanup, priv);
		if (ret)
			return ret;

		dma_cfg.dst_addr = mem->start + LEDC_DATA_REG;
		dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_cfg.dst_maxburst = LEDC_FIFO_DEPTH / 2;

		ret = dmaengine_slave_config(priv->dma_chan, &dma_cfg);
		if (ret)
			return ret;

		priv->buffer = dmam_alloc_attrs(dmaengine_get_dma_device(priv->dma_chan),
						LEDS_TO_BYTES(LEDC_MAX_LEDS),
						&priv->dma_handle, GFP_KERNEL,
						DMA_ATTR_WRITE_COMBINE);
		if (!priv->buffer)
			return -ENOMEM;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, sun50i_a100_ledc_irq, 0, dev_name(dev), priv);
	if (ret)
		return ret;

	ret = sun50i_a100_ledc_resume(dev);
	if (ret)
		return ret;

	/* Регистрация символьного устройства */
	ret = alloc_chrdev_region(&priv->devt, 0, 1, "ledc");
	if (ret)
		return ret;

	cdev_init(&priv->cdev, &ledc_fops);
	priv->cdev.owner = THIS_MODULE;

	ret = cdev_add(&priv->cdev, priv->devt, 1);
	if (ret)
		goto err_unregister_chrdev;

	priv->class = class_create("ledc");
	if (IS_ERR(priv->class)) {
		ret = PTR_ERR(priv->class);
		goto err_cdev_del;
	}

	priv->dev_node = device_create(priv->class, NULL, priv->devt, NULL, "ledc");
	if (IS_ERR(priv->dev_node)) {
		ret = PTR_ERR(priv->dev_node);
		goto err_class_destroy;
	}

	dev_info(dev, "LEDC ready, /dev/ledc (max %u LEDs)\n", LEDC_MAX_LEDS);
	return 0;

err_class_destroy:
	class_destroy(priv->class);
err_cdev_del:
	cdev_del(&priv->cdev);
err_unregister_chrdev:
	unregister_chrdev_region(priv->devt, 1);
	return ret;
}

static void sun50i_a100_ledc_remove(struct platform_device *pdev)
{
	struct sun50i_a100_ledc *priv = platform_get_drvdata(pdev);

	device_destroy(priv->class, priv->devt);
	class_destroy(priv->class);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->devt, 1);

	sun50i_a100_ledc_suspend(&pdev->dev);
}

static const struct of_device_id sun50i_a100_ledc_of_match[] = {
	{ .compatible = "allwinner,sun50i-a100-ledc" },
	{}
};
MODULE_DEVICE_TABLE(of, sun50i_a100_ledc_of_match);

static DEFINE_SIMPLE_DEV_PM_OPS(sun50i_a100_ledc_pm,
				sun50i_a100_ledc_suspend,
				sun50i_a100_ledc_resume);

static struct platform_driver sun50i_a100_ledc_driver = {
	.probe = sun50i_a100_ledc_probe,
	.remove = sun50i_a100_ledc_remove,
	.shutdown = sun50i_a100_ledc_remove,
	.driver = {
		.name = "sun50i-a100-ledc",
		.of_match_table = sun50i_a100_ledc_of_match,
		.pm = pm_ptr(&sun50i_a100_ledc_pm),
	},
};
module_platform_driver(sun50i_a100_ledc_driver);

MODULE_ALIAS("platform:sun50i-a100-ledc");
MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("Allwinner A100 LED controller driver");
MODULE_LICENSE("GPL");
