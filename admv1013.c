// SPDX-License-Identifier: GPL-2.0+
/*
 * ADMV1013 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include <linux/iio/sysfs.h>

/* ADMV1013 Register Map */
#define ADMV1013_REG_SPI_CONTROL		0x00
#define ADMV1013_REG_ALARM 			0x01
#define ADMV1013_REG_ALARM_MASKS		0x02
#define ADMV1013_REG_ENABLE			0x03
#define ADMV1013_REG_LO_AMP_I			0x05
#define ADMV1013_REG_LO_AMP_Q			0x06
#define ADMV1013_REG_OFFSET_ADJUST_I 		0x07
#define ADMV1013_REG_OFFSET_ADJUST_Q		0x08
#define ADMV1013_REG_QUAD			0x09
#define ADMV1013_REG_VVA_TEMP_COMP		0x0A

/* ADMV1013_REG_SPI_CONTROL Map */
#define ADMV1013_PARITY_EN_MSK       		BIT(15)
#define ADMV1013_PARITY_EN(x)         		FIELD_PREP(ADMV1013_PARITY_EN_MSK, x)
#define ADMV1013_SPI_SOFT_RESET_MSK		BIT(14)
#define ADMV1013_SPI_SOFT_RESET(x)         	FIELD_PREP(ADMV1013_SPI_SOFT_RESET_MSK, x)
#define ADMV1013_CHIP_ID_MSK			GENMASK(11, 4)
#define ADMV1013_CHIP_ID             		0xA
#define ADMV1013_REVISION_ID_MSK		GENMASK(3, 0)
#define ADMV1013_REVISION_ID(x)        		FIELD_PREP(ADMV1013_REVISION_ID_MSK, x)

/* ADMV1013_REG_ALARM Map */
#define ADMV1013_PARITY_ERROR_MSK       	BIT(15)
#define ADMV1013_PARITY_ERROR(x)         	FIELD_PREP(ADMV1013_PARITY_ERROR_MSK, x)
#define ADMV1013_TOO_FEW_ERRORS_MSK		BIT(14)
#define ADMV1013_TOO_FEW_ERRORS(x)         	FIELD_PREP(ADMV1013_TOO_FEW_ERRORS_MSK, x)
#define ADMV1013_TOO_MANY_ERRORS_MSK		BIT(13)
#define ADMV1013_TOO_MANY_ERRORS(x)         	FIELD_PREP(ADMV1013_TOO_MANY_ERRORS_MSK, x)
#define ADMV1013_ADDRESS_RANGE_ERROR_MSK	BIT(12)
#define ADMV1013_ADDRESS_RANGE_ERROR(x)         FIELD_PREP(ADMV1013_ADDRESS_RANGE_ERROR_MSK, x)

/* ADMV1013_REG_ENABLE Map */
#define ADMV1013_VGA_PD_MSK       		BIT(15)
#define ADMV1013_VGA_PD(x)         		FIELD_PREP(ADMV1013_VGA_PD_MSK, x)
#define ADMV1013_MIXER_PD_MSK       		BIT(14)
#define ADMV1013_MIXER_PD(x)         		FIELD_PREP(ADMV1013_MIXER_PD_MSK, x)
#define ADMV1013_QUAD_PD_MSK			GENMASK(13, 11)
#define ADMV1013_QUAD_PD(x)         		FIELD_PREP(ADMV1013_QUAD_PD_MSK, x)
#define ADMV1013_BG_PD_MSK			BIT(10)
#define ADMV1013_BG_PD(x)         		FIELD_PREP(ADMV1013_BG_PD_MSK, x)
#define ADMV1013_MIXER_IF_EN_MSK		BIT(7)
#define ADMV1013_MIXER_IF_EN(x)         	FIELD_PREP(ADMV1013_MIXER_IF_EN_MSK, x)
#define ADMV1013_DET_EN_MSK			BIT(5)
#define ADMV1013_DET_EN(x)         		FIELD_PREP(ADMV1013_DET_EN_MSK, x)

/* ADMV1013_REG_LO_AMP_I Map */
#define ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK	GENMASK(13, 7)
#define ADMV1013_LOAMP_PH_ADJ_I_FINE(x)        	FIELD_PREP(ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK, x)
#define ADMV1013_MIXER_VGATE_MSK		GENMASK(6, 0)
#define ADMV1013_MIXER_VGATE(x)         	FIELD_PREP(ADMV1013_MIXER_VGATE_MSK, x)

/* ADMV1013_REG_LO_AMP_Q Map */
#define ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK	GENMASK(13, 7)
#define ADMV1013_LOAMP_PH_ADJ_Q_FINE(x)        	FIELD_PREP(ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK, x)

/* ADMV1013_REG_OFFSET_ADJUST_I Map */
#define ADMV1013_MIXER_OFF_ADJ_I_P_MSK		GENMASK(15, 9)
#define ADMV1013_MIXER_OFF_ADJ_I_P(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_I_P_MSK, x)
#define ADMV1013_MIXER_OFF_ADJ_I_N_MSK		GENMASK(8, 2)
#define ADMV1013_MIXER_OFF_ADJ_I_N(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_I_N_MSK, x)

/* ADMV1013_REG_OFFSET_ADJUST_Q Map */
#define ADMV1013_MIXER_OFF_ADJ_Q_P_MSK		GENMASK(15, 9)
#define ADMV1013_MIXER_OFF_ADJ_Q_P(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_Q_P_MSK, x)
#define ADMV1013_MIXER_OFF_ADJ_Q_N_MSK		GENMASK(8, 2)
#define ADMV1013_MIXER_OFF_ADJ_Q_N(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_Q_N_MSK, x)

/* ADMV1013_REG_QUAD Map */
#define ADMV1013_QUAD_SE_MODE_MSK              GENMASK(9, 6)
#define ADMV1013_QUAD_SE_MODE(x)               FIELD_PREP(ADMV1013_QUAD_SE_MODE_MSK, x)
#define ADMV1013_QUAD_FILTERS_MSK              GENMASK(3, 0)
#define ADMV1013_QUAD_FILTERS(x)               FIELD_PREP(ADMV1013_QUAD_FILTERS_MSK, x)

/* ADMV1013_REG_VVA_TEMP_COMP Map */
#define ADMV1013_VVA_TEMP_COMP_MSK		GENMASK(15, 0)
#define ADMV1013_VVA_TEMP_COMP(x)  		FIELD_PREP(ADMV1013_VVA_TEMP_COMP_MSK, x)

#define ADMV1013_MAX_SPI_READ 3
#define ADMV1013_SPI_READ_BUFFER_SIZE (ADMV1013_MAX_SPI_READ + 1)

enum supported_parts {
	ADMV1013,
};

struct admv1013_dev {
	struct spi_device 	*spi;
	struct regmap		*regmap;
	struct clk 		*clkin;
	struct clock_scale	*clkscale;
	struct notifier_block	nb;
	u8			quad_se_mode;
	u64			clkin_freq;
	bool			parity_en;
};

static void check_parity(u32 input, u32 *count)
{
	u32 i = 0;
	while(input) {
		i += input & 1;
		input >>= 1;
	}

	*count = i;
}

static int admv1013_regmap_spi_read(void *context,
				  const void *reg, size_t reg_size,
				  void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	u8 result[ADMV1013_SPI_READ_BUFFER_SIZE];

	if (val_size > ADMV1013_MAX_SPI_READ)
		return -EINVAL;

	return spi_write_then_read(spi, reg, 1, result, val_size + 1);
	// TODO: Bit shifting and parity check
}

static int admv1013_regmap_spi_write(void *context, const void *data,
				   size_t count)
{
	struct device *device = context;
	struct spi_device *spi = to_spi_device(device);
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct admv1013_dev *dev = iio_priv(indio_dev);
	u32 cnt, *buf;

	buf = data;
	*buf <<= 1;

	if (dev->parity_en)
	{
		check_parity(*buf, &cnt);

		if (cnt % 2 == 0)
			*buf |= 0x1;
	}

	return spi_write(spi, buf, count);
}

static int admv1013_regmap_spi_update_bits(void *context, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	struct device *device = context;
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct admv1013_dev *dev = iio_priv(indio_dev);
	u32 data, temp;
	int status;

	status = regmap_read(dev->regmap, reg, &data);
	if (status < 0)
		return status;

	temp = data & ~mask;
	temp |= val & mask;

	return regmap_write(dev->regmap, reg, temp);
}

static const struct regmap_config admv1013_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.read_flag_mask = BIT(7),
	.max_register = 0x0B,
};

static struct regmap_bus admv1013_regmap_bus = {
	.read = admv1013_regmap_spi_read,
	.write = admv1013_regmap_spi_write,
	.reg_update_bits = admv1013_regmap_spi_update_bits,
	.read_flag_mask = BIT(7),
	.max_raw_read = ADMV1013_MAX_SPI_READ,
};

enum admv1013_iio_dev_attr {
	IF_AMP_COARSE_GAIN_I,
	IF_AMP_COARSE_GAIN_Q,
	IF_AMP_FINE_GAIN_I,
	IF_AMP_FINE_GAIN_Q,
	LOAMP_PH_ADJ_I_FINE,
	LOAMP_PH_ADJ_Q_FINE,
};

static ssize_t admv1013_store(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv1013_dev *dev = iio_priv(indio_dev);
	u16 mask = 0, val = 0;
	u8 reg = 0;
	int ret = 0;

	ret = kstrtou16(buf, 10, &val);
	if (ret)
		return ret;

	switch ((u32)this_attr->address) {
	case MIXER_OFF_ADJ_I_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_P_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_I_P(val);
		break;
	case MIXER_OFF_ADJ_I_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_N_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_I_N(val);
		break;
	case MIXER_OFF_ADJ_Q_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_P_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_Q_P(val);
		break;
	case MIXER_OFF_ADJ_Q_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_N_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_Q_N(val);
		break;
	case LOAMP_PH_ADJ_I_FINE:
		reg = ADMV1013_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK;
		val = ADMV1013_LOAMP_PH_ADJ_I_FINE(val);
		break;
	case LOAMP_PH_ADJ_Q_FINE:
		reg = ADMV1013_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK;
		val = ADMV1013_LOAMP_PH_ADJ_Q_FINE(val);
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(dev->regmap, reg, mask, val);

	return ret ? ret : len;
}

static ssize_t admv1013_show(struct device *device,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv1013_dev *dev = iio_priv(indio_dev);
	int ret = 0;
	u16 mask = 0, data_shift = 0;
	u32 val = 0;
	u8 reg = 0;

	switch ((u32)this_attr->address) {
	case MIXER_OFF_ADJ_I_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_P_MSK;
		data_shift = 9;
		break;
	case MIXER_OFF_ADJ_I_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_N_MSK;
		data_shift = 2;
		break;
	case MIXER_OFF_ADJ_Q_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_P_MSK;
		data_shift = 9;
		break;
	case MIXER_OFF_ADJ_Q_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_N_MSK;
		data_shift = 2;
		break;
	case LOAMP_PH_ADJ_I_FINE:
		reg = ADMV1013_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK;
		data_shift = 7;
		break;
	case LOAMP_PH_ADJ_Q_FINE:
		reg = ADMV1013_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK;
		data_shift = 7;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(dev->regmap, reg, &val);
	if (ret < 0)
		return ret;

	val = (val & mask) >> data_shift;

	return sprintf(buf, "%d\n", val);
}

static IIO_DEVICE_ATTR(mixer_off_adj_i_p, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_I_P);

static IIO_DEVICE_ATTR(mixer_off_adj_i_n, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_I_N);

static IIO_DEVICE_ATTR(mixer_off_adj_q_p, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_Q_P);

static IIO_DEVICE_ATTR(mixer_off_adj_q_n, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_Q_N);

static IIO_DEVICE_ATTR(loamp_ph_adj_i_fine, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       LOAMP_PH_ADJ_I_FINE);

static IIO_DEVICE_ATTR(loamp_ph_adj_q_fine, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       LOAMP_PH_ADJ_Q_FINE);

static struct attribute *admv1013_attributes[] = {
	&iio_dev_attr_mixer_off_adj_i_p.dev_attr.attr,
	&iio_dev_attr_mixer_off_adj_i_n.dev_attr.attr,
	&iio_dev_attr_mixer_off_adj_q_p.dev_attr.attr,
	&iio_dev_attr_mixer_off_adj_q_n.dev_attr.attr,
	&iio_dev_attr_loamp_ph_adj_i_fine.dev_attr.attr,
	&iio_dev_attr_loamp_ph_adj_q_fine.dev_attr.attr,
	NULL
};

static const struct attribute_group admv1013_attribute_group = {
	.attrs = admv1013_attributes,
};

static int admv1013_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct admv1013_dev *dev = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(dev->regmap, reg, read_val);
	else
		return regmap_write(dev->regmap, reg, write_val);
}

static const struct iio_info admv1013_info = {
	.debugfs_reg_access = &admv1013_reg_access,
	.attrs = &admv1013_attribute_group,
};

static int admv1013_freq_change(struct notifier_block *nb, unsigned long flags, void *data)
{
	struct admv1013_dev *dev = container_of(nb, struct admv1013_dev, nb);
	struct clk_notifier_data *cnd = data;

	/* cache the new rate */
	dev->clkin_freq = clk_get_rate_scaled(cnd->clk, dev->clkscale);

	return NOTIFY_OK;
}

static void admv1013_clk_notifier_unreg(void *data)
{
	struct admv1013_dev *dev = data;

	clk_notifier_unregister(dev->clkin, &dev->nb);
}

static int admv1013_init(struct admv1013_dev *dev)
{
	int ret;
	u32 chip_id;

	/* Perform a software reset */
	ret = regmap_update_bits(dev->regmap, ADMV1013_REG_SPI_CONTROL,
				 ADMV1013_SPI_SOFT_RESET_MSK,
				 ADMV1013_SPI_SOFT_RESET(1));
	if (ret < 0)
		return ret;

	ret = regmap_write(dev->regmap, ADMV1013_REG_VVA_TEMP_COMP, 0xE700);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(dev->regmap, ADMV1013_REG_ENABLE,
				 ADMV1013_P1DB_COMPENSATION_MSK,
				 ADMV1013_P1DB_COMPENSATION(3));
	if (ret < 0)
		return ret;

	ret = regmap_read(dev->regmap, ADMV1013_REG_SPI_CONTROL, &chip_id);
	if (ret < 0)
		return ret;

	chip_id = (chip_id & ADMV1013_CHIP_ID_MSK) >> 4;
	if (chip_id != ADMV1013_CHIP_ID)
		return -EINVAL;

	return regmap_update_bits(dev->regmap, ADMV1013_REG_QUAD,
				 ADMV1013_QUAD_SE_MODE_MSK,
				 ADMV1013_QUAD_SE_MODE(dev->quad_se_mode));

}

static void admv1013_clk_disable(void *data)
{
	struct admv1013_dev *dev = data;

	clk_disable_unprepare(dev->clkin);
}

static int admv1013_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct admv1013_dev *dev;
	struct clock_scale dev_clkscale;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init(&spi->dev, &admv1013_regmap_bus,
				  &spi->dev, &admv1013_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "invalid regmap");
		return PTR_ERR(regmap);
	}

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;
	dev->spi = spi;

	ret = of_property_read_u8(spi->dev.of_node, "adi,quad-se-mode", &dev->quad_se_mode);
	if (ret < 0) {
		dev_err(&spi->dev, "adi,quad-se-mode property not defined!");
		return -EINVAL;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &admv1013_info;
	indio_dev->name = "admv1013";

	dev->clkin = devm_clk_get(&spi->dev, "lo_in");
	if (IS_ERR(dev->clkin)) {
		return PTR_ERR(dev->clkin);
	}

	ret = clk_prepare_enable(dev->clkin);
	if (ret < 0) {
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, admv1013_clk_disable, dev);
	if (ret < 0) {
		return ret;
	}

	of_clk_get_scale(spi->dev.of_node, "lo_in", &dev_clkscale);

	dev->clkscale = &dev_clkscale;

	dev->clkin_freq = clk_get_rate_scaled(dev->clkin, dev->clkscale);
	dev->nb.notifier_call = admv1013_freq_change;
	ret = clk_notifier_register(dev->clkin, &dev->nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1013_clk_notifier_unreg, dev);
	if (ret < 0) {
		return ret;
	}

	// ret = admv1013_init(dev);
	// if (ret < 0) {
	// 	dev_err(&spi->dev, "admv1013 init failed\n");
	// 	return ret;
	// }

	dev_info(&spi->dev, "ADMV1013 PROBED");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id admv1013_id[] = {
	{ "admv1013", ADMV1013 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv1013_id);

static const struct of_device_id admv1013_of_match[] = {
	{ .compatible = "adi,admv1013" },
	{},
};
MODULE_DEVICE_TABLE(of, admv1013_of_match);

static struct spi_driver admv1013_driver = {
	.driver = {
			.name = "admv1013",
			.of_match_table = admv1013_of_match,
		},
	.probe = admv1013_probe,
	.id_table = admv1013_id,
};
module_spi_driver(admv1013_driver);


MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADMV1013");
MODULE_LICENSE("GPL v2");