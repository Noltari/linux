// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Microchip EMC2101 fan controller.
 *
 * Copyright 2025 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/util_macros.h>

#define REG_TEMP_INT			0x00
#define REG_TEMP_EXT_HI			0x01
#define REG_STATUS			0x02
#define  ADC_BUSY			BIT(7)
#define  TEMP_INT_HIGH			BIT(6)
#define  EEPROM_ERROR			BIT(5)
#define  TEMP_EXT_HIGH			BIT(4)
#define  TEMP_EXT_LOW			BIT(3)
#define  TEMP_EXT_FAULT			BIT(2)
#define  TEMP_EXT_CRIT			BIT(1)
#define  TACH_LOW			BIT(0)
#define REG_CONFIG			0x03
#define  ALERT_IRQ_ACK			BIT(7)
#define  FAN_STANDBY_ENABLE		BIT(6)
#define  FAN_STANDBY_MODE		BIT(5)
#define  FAN_MODE_SHIFT			4
#define  FAN_MODE_DAC			(1 << FAN_MODE_SHIFT)
#define  FAN_MODE_PWM			(0 << FAN_MODE_SHIFT)
#define  FAN_MODE_MASK			(1 << FAN_MODE_SHIFT)
#define  SMBUS_TOUT_DISABLE		BIT(3)
#define  PIN_FUNC_SHIFT			2
#define  PIN_FUNC_TACH			(1 << PIN_FUNC_SHIFT)
#define  PIN_FUNC_IRQ			(0 << PIN_FUNC_SHIFT)
#define  PIN_FUNC_MASK			(1 << PIN_FUNC_SHIFT)
#define  TEMP_EXT_CRIT_UNLOCK		BIT(1)
#define  PIN_ASSERT_SHIFT		0
#define  PIN_ASSERT_3_EXC		(1 << PIN_ASSERT_SHIFT)
#define  PIN_ASSERT_1_EXC		(0 << PIN_ASSERT_SHIFT)
#define  PIN_ASSERT_MASK		(1 << PIN_ASSERT_SHIFT)
#define REG_CONV_RATE			0x04
#define  CONV_RATE_SHIFT		0
#define  CONV_RATE_MASK			(0xf << CONV_RATE_SHIFT)
#define REG_TEMP_INT_MAX		0x05
#define REG_TEMP_EXT_MAX_HI		0x07
#define REG_TEMP_EXT_MIN_HI		0x08
#define REG_TEMP_EXT_FORCE		0x0c
#define REG_ONE_SHOT			0x0f
#define REG_TEMP_EXT_LO			0x10
#define REG_SCRATCHPAD_1		0x11
#define REG_SCRATCHPAD_2		0x12
#define REG_TEMP_EXT_MAX_LO		0x13
#define REG_TEMP_EXT_MIN_LO		0x14
#define REG_ALERT_MASK			0x16
#define  IRQ_TEMP_INT_MAX_DISABLE	BIT(6)
#define  IRQ_TEMP_EXT_MAX_DISABLE	BIT(4)
#define  IRQ_TEMP_EXT_MIN_DISABLE	BIT(3)
#define  IRQ_TEMP_EXT_CRIT_DISABLE	BIT(1)
#define  IRQ_TACH_MIN_DISABLE		BIT(0)
#define REG_EXT_IDEALITY		0x17
#define  EXT_IDEALITY_SHIFT		0
#define  EXT_IDEALITY_MIN		0x08 /* 0.9949 */
#define  EXT_IDEALITY_DEF		0x12 /* 1.0080 */
#define  EXT_IDEALITY_MAX		0x37 /* 1.0566 */
#define  EXT_IDEALITY_START		9845
#define  EXT_IDEALITY_STEP		13
#define  EXT_IDEALITY_VAL(x)		(EXT_IDEALITY_START + \
					 (x * EXT_IDEALITY_STEP))
#define  EXT_IDEALITY_MASK		(0x3f << EXT_IDEALITY_SHIFT)
#define REG_BETA_COMP			0x18
#define  BETA_COMP_AUTO			BIT(3)
#define  BETA_COMP_SHIFT		0
#define  BETA_COMP_DISABLE		(7 << BETA_COMP_SHIFT)
#define  BETA_COMP_2_33			(6 << BETA_COMP_SHIFT)
#define  BETA_COMP_1_00			(5 << BETA_COMP_SHIFT)
#define  BETA_COMP_0_43			(4 << BETA_COMP_SHIFT)
#define  BETA_COMP_0_33			(3 << BETA_COMP_SHIFT)
#define  BETA_COMP_0_25			(2 << BETA_COMP_SHIFT)
#define  BETA_COMP_0_18			(1 << BETA_COMP_SHIFT)
#define  BETA_COMP_0_11			(0 << BETA_COMP_SHIFT)
#define  BETA_COMP_MASK			(0x7 << BETA_COMP_SHIFT)
#define REG_TEMP_EXT_CRIT		0x19
#define REG_TEMP_EXT_CRIT_HYST		0x21
#define REG_TACH_LO			0x46
#define REG_TACH_HI			0x47
#define REG_TACH_MIN_LO			0x48
#define REG_TACH_MIN_HI			0x49
#define REG_FAN_CONFIG			0x4a
#define  FAN_EXT_TEMP_FORCE		BIT(6)
#define  FAN_LUT_DISABLE		BIT(5)
#define  FAN_POL_INV			BIT(4)
#define  FAN_CLK_SEL			BIT(3)
#define  FAN_CLK_OVR			BIT(2)
#define  TACH_MODE_SHIFT		0
#define  TACH_MIN_FALSE_READ		(0 << TACH_MODE_SHIFT)
#define  TACH_MIN_FFFF_READ		(1 << TACH_MODE_SHIFT)
#define  TACH_MODE_MASK			(0x3 << TACH_MODE_SHIFT)
#define REG_FAN_SPIN			0x4b
#define  FAN_SPIN_TACH_ABORT		BIT(5)
#define  FAN_SPIN_DRIVE_SHIFT		3
#define  FAN_SPIN_DRIVE_100		(3 << FAN_SPIN_DRIVE_SHIFT)
#define  FAN_SPIN_DRIVE_75		(2 << FAN_SPIN_DRIVE_SHIFT)
#define  FAN_SPIN_DRIVE_50		(1 << FAN_SPIN_DRIVE_SHIFT)
#define  FAN_SPIN_DRIVE_0		(0 << FAN_SPIN_DRIVE_SHIFT)
#define  FAN_SPIN_DRIVE_MASK		(0x3 << FAN_SPIN_DRIVE_SHIFT)
#define  FAN_SPIN_TIME_SHIFT		0
#define  FAN_SPIN_TIME_3200		(7 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_1600		(6 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_800		(5 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_400		(4 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_200		(3 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_100		(2 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_50		(1 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_NONE		(0 << FAN_SPIN_TIME_SHIFT)
#define  FAN_SPIN_TIME_MASK		(0x7 << FAN_SPIN_TIME_SHIFT)
#define REG_FAN_SET			0x4c
#define  FAN_SET_SHIFT			0
#define  FAN_SET_MASK			(0x3f << FAN_SET_SHIFT)
#define REG_PWM_FREQ			0x4d
#define  PWM_FREQ_SHIFT			0
#define  PWM_FREQ_MASK			(0x1f << PWM_FREQ_SHIFT)
#define REG_PWM_FREQ_DIV		0x4e
#define REG_FAN_LUT_HYST		0x4f
#define  FAN_LUT_HYST_SHIFT		0
#define  FAN_LUT_HYST_MASK		(0x1f << FAN_LUT_HYST_SHIFT)
#define REG_FAN_LUT_TEMP(x)		(0x50 + (0x2 * (x)))
/* RW only with FAN_LUT_DISABLE */
#define  FAN_LUT_TEMP_SHIFT		0
#define  FAN_LUT_TEMP_MASK		(0x7f << FAN_LUT_TEMP_SHIFT)
#define REG_FAN_LUT_SPEED(x)		(0x51 + (0x2 * (x)))
/* RW only with FAN_LUT_DISABLE */
#define  FAN_LUT_SPEED_SHIFT		0
#define  FAN_LUT_SPEED_MASK		(0x3f << FAN_LUT_SPEED_SHIFT)
#define REG_AVG_FILTER			0xbf
#define  FILTER_SHIFT			1
#define  FILTER_L2			(3 << FILTER_SHIFT)
#define  FILTER_L1			(1 << FILTER_SHIFT)
#define  FILTER_NONE			(0 << FILTER_SHIFT)
#define  FILTER_MASK			(0x3 << FILTER_SHIFT)
#define  ALERT_PIN_SHIFT		0
#define  ALERT_PIN_TEMP_COMP		(1 << ALERT_PIN_SHIFT)
#define  ALERT_PIN_INTERRUPT		(0 << ALERT_PIN_SHIFT)
#define  ALERT_PIN_MASK			(1 << ALERT_PIN_SHIFT)
#define REG_PRODUCT_ID			0xfd
#define REG_MANUFACTURER_ID		0xfe
#define REG_REVISION			0xff

#define CLK_FREQ_ALT			1400
#define CLK_FREQ_BASE			360000

#define FAN_LUT_COUNT			8
#define FAN_RPM_FACTOR			5400000

#define MANUFACTURER_ID			0x5d

#define TEMP_EXT_HI_FAULT		0x7f
#define TEMP_EXT_LO_FAULT_OPEN		0x00
#define TEMP_EXT_LO_FAULT_SHORT		0xe0

#define TEMP_LO_FRAC			125
#define TEMP_LO_SHIFT			5
#define TEMP_LO_MASK			(0x3 << TEMP_LO_SHIFT)

#define TEMP_MIN			-64
#define TEMP_MAX			127
#define TEMP_MAX_FRAC			750

enum emc2101_temp_channels {
	EMC2101_TC_INT = 0,
	EMC2101_TC_EXT,
	EMC2101_TC_FORCE,
	EMC2101_TC_NUM
};

enum emc2101_temp_diode {
	EMC2101_TD_CPU = 1,
	EMC2101_TD_2N3904 = 2,
};

enum ecm2101_product_id {
	EMC2101 = 0x16,
	EMC2101_R = 0x28,
};

enum emc2101_pwm_mode {
	EMC2101_PM_MANUAL = 1,
	EMC2101_PM_LUT = 2,
	EMC2101_PM_LUT_INV = 3,
};

struct emc2101_data {
	struct i2c_client *client;
	struct mutex mutex;
};

static const u16 emc2101_conv_time[] = {
	62, 125, 250, 500, 1000, 2000, 4000, 8000, 16000, 32000
};

static inline int emc2101_read_u8(struct i2c_client *client, u8 reg, u8 *val)
{
	struct device *dev = &client->dev;
	s32 data = i2c_smbus_read_byte_data(client, reg);
	int ret;

	if (data < 0) {
		ret = data;
		dev_err(dev, "error reading reg=%x err=%d", reg, data);
	} else {
		ret = 0;
		*val = data;
	}

	return ret;
}

static inline int emc2101_read_u16(struct i2c_client *client, u8 reg_hi,
				   u8 reg_lo, u16 *val)
{
	u8 val_hi, val_lo;
	int ret;

	ret = emc2101_read_u8(client, reg_lo, &val_lo);
	if (ret)
		return ret;

	ret = emc2101_read_u8(client, reg_hi, &val_hi);
	if (ret)
		return ret;

	*val = (val_hi) << 8;
	*val |= val_lo;

	return ret;
}

static inline int emc2101_write_u8(struct i2c_client *client, u8 reg, u8 val)
{
	struct device *dev = &client->dev;
	s32 data = i2c_smbus_write_byte_data(client, reg, val);

	if (data < 0)
		dev_err(dev, "error writing reg=%x err=%d", reg, data);

	return data;
}

static inline int emc2101_write_u16(struct i2c_client *client, u8 reg_hi,
				    u8 reg_lo, u16 val)
{
	const u8 val_hi = (val >> 8) & 0xff;
	const u8 val_lo = val & 0xff;
	int ret;

	ret = emc2101_write_u8(client, reg_lo, val_lo);
	if (ret)
		return ret;

	return emc2101_write_u8(client, reg_hi, val_hi);
}

static inline int emc2101_rmw_u8(struct i2c_client *client, u8 reg, u8 clr,
				 u8 set)
{
	int ret;
	u8 val;

	ret = emc2101_read_u8(client, reg, &val);
	if (ret)
		return ret;

	val &= ~clr;
	val |= set;

	return emc2101_write_u8(client, reg, val);
}

static inline int emc2101_check_u8(struct i2c_client *client, u8 reg, u8 mask,
				   bool *val)
{
	u8 reg_val;
	int ret;

	ret = emc2101_read_u8(client, reg, &reg_val);
	if (!ret)
		*val = (reg_val & mask) == mask;

	return ret;
}

static int emc2101_temp_read(struct emc2101_data *data, u8 reg, long *temp)
{
	struct i2c_client *client = data->client;
	int ret;
	u8 val;

	ret = emc2101_read_u8(client, reg, &val);
	if (!ret)
		*temp = val * 1000;

	return ret;
}

static int emc2101_temp_write(struct emc2101_data *data, u8 reg, long temp)
{
	struct i2c_client *client = data->client;
	const u8 val = (u8) clamp_val(temp / 1000, TEMP_MIN, TEMP_MAX);

	return emc2101_write_u8(client, reg, val);
}

static int emc2101_temp_frac_read(struct emc2101_data *data, u8 reg_hi,
				  u8 reg_lo, long *temp)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	u8 val_hi, val_lo;
	int ret;

	ret = emc2101_read_u8(client, reg_hi, &val_hi);
	if (ret)
		return ret;

	ret = emc2101_read_u8(client, reg_lo, &val_lo);
	if (!ret && val_hi == TEMP_EXT_HI_FAULT) {
		switch (val_lo & TEMP_LO_MASK) {
		case TEMP_EXT_LO_FAULT_OPEN:
			ret = -ENODATA;
			dev_warn(dev, "[%02x, %02x]: diode fault (open)",
				 reg_hi, reg_lo);
			break;
		case TEMP_EXT_LO_FAULT_SHORT:
			ret = -ENODATA;
			dev_warn(dev, "[%02x, %02x]: diode fault (short)",
				 reg_hi, reg_lo);
			break;
		default:
			break;
		}
	}

	if (!ret) {
		*temp = (val_lo & TEMP_LO_MASK) >> TEMP_LO_SHIFT;
		*temp *= TEMP_LO_FRAC;
		*temp += val_hi * 1000;
	}

	return ret;
}

static int emc2101_temp_frac_write(struct emc2101_data *data, u8 reg_hi,
				   u8 reg_lo, long temp)
{
	struct i2c_client *client = data->client;
	const u8 val_hi = (u8) clamp_val(temp / 1000, TEMP_MIN, TEMP_MAX);
	long fraction = temp - (val_hi * 1000);
	u8 val_lo;
	int ret;

	if (val_hi == TEMP_MAX && fraction > TEMP_MAX_FRAC)
		fraction = clamp_val(fraction, 0, TEMP_MAX_FRAC);
	fraction /= TEMP_LO_FRAC;
	val_lo = (fraction << TEMP_LO_SHIFT) & TEMP_LO_MASK;

	ret = emc2101_write_u8(client, reg_hi, val_hi);
	if (ret)
		return ret;

	return emc2101_write_u8(client, reg_lo, val_lo);
}

static int emc2101_pwm_read(struct emc2101_data *data, u8 reg, long *pwm)
{
	struct i2c_client *client = data->client;
	int ret;
	u8 val;

	ret = emc2101_read_u8(client, reg, &val);
	if (!ret)
		*pwm = val;

	return ret;
}

static int emc2101_pwm_write(struct emc2101_data *data, u8 reg, long pwm)
{
	struct i2c_client *client = data->client;
	const u8 val = (u8) clamp_val(pwm, 1, PWM_FREQ_MASK);

	return emc2101_write_u8(client, reg, val);
}

/* *** */

static ssize_t __pwm_auto_point_pwm_show(struct device *dev,
					 struct device_attribute *devattr,
					 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct emc2101_data *data = dev_get_drvdata(dev);
	const u8 reg = REG_FAN_LUT_SPEED(attr->index);
	long lut_pwm;
	int ret;

	ret = emc2101_pwm_read(data, reg, &lut_pwm);
	if (ret)
		return ret;

	return sprintf(buf, "%lu\n", lut_pwm);
}

static ssize_t pwm_auto_point_pwm_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = __pwm_auto_point_pwm_show(dev, devattr, buf);
	mutex_unlock(&data->mutex);

	return ret;
}

static ssize_t __pwm_auto_point_pwm_store(struct device *dev,
					  struct device_attribute *devattr,
					  const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct emc2101_data *data = dev_get_drvdata(dev);
	const u8 reg = REG_FAN_LUT_SPEED(attr->index);
	long lut_pwm;
	int ret;

	ret = kstrtol(buf, 10, &lut_pwm);
	if (ret)
		return ret;

	ret = emc2101_pwm_write(data, reg, lut_pwm);

	return !ret ? count : ret;
}

static ssize_t pwm_auto_point_pwm_store(struct device *dev,
					struct device_attribute *devattr,
					const char *buf, size_t count)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = __pwm_auto_point_pwm_store(dev, devattr, buf, count);
	mutex_unlock(&data->mutex);

	return ret;
}

static ssize_t __pwm_auto_point_temp_show(struct device *dev,
					  struct device_attribute *devattr,
					  char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct emc2101_data *data = dev_get_drvdata(dev);
	const u8 reg = REG_FAN_LUT_TEMP(attr->index);
	long lut_temp;
	int ret;

	ret = emc2101_temp_read(data, reg, &lut_temp);
	if (ret)
		return ret;

	return sprintf(buf, "%lu\n", lut_temp);
}

static ssize_t pwm_auto_point_temp_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = __pwm_auto_point_temp_show(dev, devattr, buf);
	mutex_unlock(&data->mutex);

	return ret;
}

static ssize_t __pwm_auto_point_temp_store(struct device *dev,
					   struct device_attribute *devattr,
					   const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	long cur_temp, lut_temp;
	unsigned int i;
	int ret;

	ret = kstrtol(buf, 10, &lut_temp);
	if (ret)
		return ret;

	ret = emc2101_rmw_u8(client, REG_FAN_CONFIG, 0, FAN_LUT_DISABLE);
	if (ret)
		return ret;

	for (i = 0; i < FAN_LUT_COUNT; i++)
	{
		const u8 reg = REG_FAN_LUT_TEMP(attr->index);

		ret = emc2101_temp_read(data, reg, &cur_temp);
		if (ret)
			return ret;

		if (i < attr->index) {
			if (cur_temp > lut_temp)
				ret = emc2101_temp_write(data, reg, lut_temp);
		} else if (i > attr->index) {
			if (cur_temp < lut_temp)
				ret = emc2101_temp_write(data, reg, lut_temp);
		} else {
			ret = emc2101_temp_write(data, reg, lut_temp);
		}

		if (ret)
			return ret;
	}

	ret = emc2101_rmw_u8(client, REG_FAN_CONFIG, FAN_LUT_DISABLE, 0);
	if (ret)
		return ret;

	return !ret ? count : ret;
}

static ssize_t pwm_auto_point_temp_store(struct device *dev,
					 struct device_attribute *devattr,
					 const char *buf, size_t count)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = __pwm_auto_point_temp_store(dev, devattr, buf, count);
	mutex_unlock(&data->mutex);

	return ret;
}

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point1_pwm, pwm_auto_point_pwm, 0);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point1_temp, pwm_auto_point_temp, 0);

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point2_pwm, pwm_auto_point_pwm, 1);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point2_temp, pwm_auto_point_temp, 1);

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point3_pwm, pwm_auto_point_pwm, 2);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point3_temp, pwm_auto_point_temp, 2);

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point4_pwm, pwm_auto_point_pwm, 3);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point4_temp, pwm_auto_point_temp, 3);

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point5_pwm, pwm_auto_point_pwm, 4);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point5_temp, pwm_auto_point_temp, 4);

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point6_pwm, pwm_auto_point_pwm, 5);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point6_temp, pwm_auto_point_temp, 5);

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point7_pwm, pwm_auto_point_pwm, 6);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point7_temp, pwm_auto_point_temp, 6);

static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point8_pwm, pwm_auto_point_pwm, 7);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point8_temp, pwm_auto_point_temp, 7);

static struct attribute *emc2101_hwmon_attributes[] = {
	&sensor_dev_attr_pwm1_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point2_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point3_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point3_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point4_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point4_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point5_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point5_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point6_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point6_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point7_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point7_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point8_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point8_temp.dev_attr.attr,
	NULL
};

static const struct attribute_group emc2101_hwmon_group = {
	.attrs = emc2101_hwmon_attributes,
};
__ATTRIBUTE_GROUPS(emc2101_hwmon);

static int emc2101_temp_ext_crit_alarm_read(struct emc2101_data *data,
					    long *val)
{
	struct i2c_client *client = data->client;
	bool temp_crit;
	int ret;

	ret = emc2101_check_u8(client, REG_STATUS, TEMP_EXT_CRIT,
			       &temp_crit);
	if (!ret)
		*val = temp_crit;

	return ret;
}

static int emc2101_temp_ext_crit_hyst_read(struct emc2101_data *data, long *val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_read(data, REG_TEMP_EXT_CRIT_HYST, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_crit_hyst_write(struct emc2101_data *data, long val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_write(data, REG_TEMP_EXT_CRIT_HYST, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_crit_read(struct emc2101_data *data, long *val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_read(data, REG_TEMP_EXT_CRIT, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int __emc2101_temp_ext_crit_write(struct emc2101_data *data, long val)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	bool temp_ext_crit_unlock;
	int ret;

	ret = emc2101_check_u8(client, REG_CONFIG, TEMP_EXT_CRIT_UNLOCK,
			       &temp_ext_crit_unlock);
	if (ret)
		return ret;

	if (temp_ext_crit_unlock) {
		dev_err(dev, "critical temperature can only be updated once");
		return -EIO;
	}

	ret = emc2101_rmw_u8(client, REG_CONFIG, 0, TEMP_EXT_CRIT_UNLOCK);
	if (ret)
		return ret;

	return emc2101_temp_write(data, REG_TEMP_EXT_CRIT, val);
}

static int emc2101_temp_ext_crit_write(struct emc2101_data *data, long val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = __emc2101_temp_ext_crit_write(data, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_fault_read(struct emc2101_data *data, long *val)
{
	struct i2c_client *client = data->client;
	bool temp_fault;
	int ret;

	ret = emc2101_check_u8(client, REG_STATUS, TEMP_EXT_FAULT,
			       &temp_fault);
	if (!ret)
		*val = temp_fault;

	return ret;
}

static int emc2101_temp_ext_max_alarm_read(struct emc2101_data *data,
					   long *val)
{
	struct i2c_client *client = data->client;
	bool temp_high;
	int ret;

	ret = emc2101_check_u8(client, REG_STATUS, TEMP_EXT_HIGH, &temp_high);
	if (!ret)
		*val = temp_high;

	return ret;
}

static int emc2101_temp_ext_max_read(struct emc2101_data *data, long *val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_frac_read(data, REG_TEMP_EXT_MAX_HI,
				     REG_TEMP_EXT_MAX_LO, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_max_write(struct emc2101_data *data, long val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_frac_write(data, REG_TEMP_EXT_MAX_HI,
				      REG_TEMP_EXT_MAX_LO, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_min_alarm_read(struct emc2101_data *data,
					   long *val)
{
	struct i2c_client *client = data->client;
	bool temp_low;
	int ret;

	ret = emc2101_check_u8(client, REG_STATUS, TEMP_EXT_LOW, &temp_low);
	if (!ret)
		*val = temp_low;

	return ret;
}

static int emc2101_temp_ext_min_read(struct emc2101_data *data, long *val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_frac_read(data, REG_TEMP_EXT_MIN_HI,
				     REG_TEMP_EXT_MIN_LO, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_min_write(struct emc2101_data *data, long val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_frac_write(data, REG_TEMP_EXT_MIN_HI,
				      REG_TEMP_EXT_MIN_LO, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_type_read(struct emc2101_data *data, long *val)
{
	struct i2c_client *client = data->client;
	bool beta_comp_disable;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_check_u8(client, REG_BETA_COMP, BETA_COMP_DISABLE,
			       &beta_comp_disable);
	mutex_unlock(&data->mutex);

	if (ret)
		return ret;

	if (beta_comp_disable)
		*val = EMC2101_TD_2N3904;
	else
		*val = EMC2101_TD_CPU;

	return ret;
}

static int emc2101_temp_ext_type_write(struct emc2101_data *data, long val)
{
	struct i2c_client *client = data->client;
	u8 set, clr;
	int ret;

	switch (val) {
	case EMC2101_TD_CPU:
		clr = BETA_COMP_MASK;
		set = BETA_COMP_AUTO;
		break;
	case EMC2101_TD_2N3904:
		clr = BETA_COMP_AUTO | BETA_COMP_MASK;
		set = BETA_COMP_DISABLE;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&data->mutex);
	ret = emc2101_rmw_u8(client, REG_BETA_COMP, clr, set);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_ext_input_read(struct emc2101_data *data, long *val)
{
	return emc2101_temp_frac_read(data, REG_TEMP_EXT_HI, REG_TEMP_EXT_LO,
				      val);
}

static int emc2101_temp_force_input_read(struct emc2101_data *data, long *val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_read(data, REG_TEMP_EXT_FORCE, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_force_input_write(struct emc2101_data *data, long val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_write(data, REG_TEMP_EXT_FORCE, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_int_max_alarm_read(struct emc2101_data *data,
					   long *val)
{
	struct i2c_client *client = data->client;
	bool temp_high;
	int ret;

	ret = emc2101_check_u8(client, REG_STATUS, TEMP_INT_HIGH,
			       &temp_high);
	if (!ret)
		*val = temp_high;

	return ret;
}

static int emc2101_temp_int_max_read(struct emc2101_data *data, long *val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_read(data, REG_TEMP_INT_MAX, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_int_max_write(struct emc2101_data *data, long val)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_temp_write(data, REG_TEMP_INT_MAX, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_int_input_read(struct emc2101_data *data, long *val)
{
	return emc2101_temp_read(data, REG_TEMP_INT, val);
}

static int emc2101_fan_div_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 pwm_freq_div;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_read_u8(client, REG_PWM_FREQ_DIV, &pwm_freq_div);
	mutex_unlock(&data->mutex);

	if (!ret)
		*val = pwm_freq_div;

	return ret;
}

static int emc2101_fan_div_write(struct device *dev, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 pwm_freq_div = val;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_write_u8(client, REG_PWM_FREQ_DIV, pwm_freq_div);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_fan_input_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u16 tach_count;
	int ret;

	ret = emc2101_read_u16(client, REG_TACH_HI, REG_TACH_LO, &tach_count);
	if (ret)
		return ret;

	*val = FAN_RPM_FACTOR / tach_count;

	return ret;
}

static int emc2101_fan_min_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u16 tach_count;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_read_u16(client, REG_TACH_MIN_HI, REG_TACH_MIN_LO,
			       &tach_count);
	mutex_unlock(&data->mutex);

	if (ret)
		return ret;

	*val = FAN_RPM_FACTOR / tach_count;

	return ret;
}

static int emc2101_fan_min_write(struct device *dev, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const u16 tach_count = FAN_RPM_FACTOR / val;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_write_u16(client, REG_TACH_MIN_HI, REG_TACH_MIN_LO,
				tach_count);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_fan_min_alarm_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	bool tach_low;
	int ret;

	ret = emc2101_check_u8(client, REG_STATUS, TACH_LOW, &tach_low);
	if (!ret)
		*val = tach_low;

	return ret;
}

static int emc2101_pwm_auto_channels_temp_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 fan_config;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_read_u8(client, REG_FAN_CONFIG, &fan_config);
	mutex_unlock(&data->mutex);

	if (ret)
		return ret;

	if (fan_config && FAN_EXT_TEMP_FORCE)
		*val = 3;
	else
		*val = 2;

	return ret;
}

static int emc2101_pwm_auto_channels_temp_write(struct device *dev, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 clr, set;
	int ret;

	switch (val) {
	case 2:
		clr = FAN_EXT_TEMP_FORCE;
		set = 0;
		break;
	case 3:
		clr = 0;
		set = FAN_EXT_TEMP_FORCE;
		break;
	default:
		return -EOPNOTSUPP;
	}

	mutex_lock(&data->mutex);
	ret = emc2101_rmw_u8(client, REG_FAN_CONFIG, clr, set);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_pwm_enable_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 fan_config;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_read_u8(client, REG_FAN_CONFIG, &fan_config);
	mutex_unlock(&data->mutex);

	if (ret)
		return ret;

	if (fan_config && FAN_LUT_DISABLE)
		*val = EMC2101_PM_MANUAL;
	else if (fan_config && FAN_POL_INV)
		*val = EMC2101_PM_LUT_INV;
	else
		*val = EMC2101_PM_LUT;

	return ret;
}

static int emc2101_pwm_enable_write(struct device *dev, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 clr, set;
	int ret;

	switch (val) {
	case EMC2101_PM_MANUAL:
		clr = 0;
		set = FAN_LUT_DISABLE;
		break;
	case EMC2101_PM_LUT:
		clr = FAN_LUT_DISABLE | FAN_POL_INV;
		set = 0;
		break;
	case EMC2101_PM_LUT_INV:
		clr = FAN_LUT_DISABLE;
		set = FAN_POL_INV;
		break;
	}

	mutex_lock(&data->mutex);
	ret = emc2101_rmw_u8(client, REG_FAN_CONFIG, clr, set);
	mutex_unlock(&data->mutex);

	return ret;
}

static int __emc2101_pwm_freq_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 fan_config, pwm_freq, pwm_freq_div;
	unsigned int base_clk, div;
	int ret;

	ret = emc2101_read_u8(client, REG_FAN_CONFIG, &fan_config);
	if (ret)
		return ret;

	ret = emc2101_read_u8(client, REG_PWM_FREQ, &pwm_freq);
	if (ret)
		return ret;

	if (fan_config & FAN_CLK_OVR) {
		ret = emc2101_read_u8(client, REG_PWM_FREQ_DIV, &pwm_freq_div);
		if (ret)
			return ret;
	} else {
		pwm_freq_div = 1;
	}

	if (fan_config & FAN_CLK_SEL)
		base_clk = CLK_FREQ_ALT;
	else
		base_clk = CLK_FREQ_BASE;

	div = 2 * pwm_freq * pwm_freq_div;
	if (div)
		*val = base_clk / div;
	else
		*val = 0;

	return ret;
}

static int emc2101_pwm_freq_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = __emc2101_pwm_freq_read(dev, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int __emc2101_pwm_freq_write(struct device *dev, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 fan_config, pwm_freq, pwm_freq_div;
	unsigned int base_clk;
	int ret;

	ret = emc2101_read_u8(client, REG_FAN_CONFIG, &fan_config);
	if (ret)
		return ret;

	if (fan_config & FAN_CLK_OVR) {
		ret = emc2101_read_u8(client, REG_PWM_FREQ_DIV, &pwm_freq_div);
		if (ret)
			return ret;
	} else {
		pwm_freq_div = 1;
	}

	if (fan_config & FAN_CLK_SEL)
		base_clk = CLK_FREQ_ALT;
	else
		base_clk = CLK_FREQ_BASE;

	pwm_freq = base_clk / (2 * pwm_freq_div * val);

	return emc2101_pwm_write(data, REG_PWM_FREQ, pwm_freq);
}

static int emc2101_pwm_freq_write(struct device *dev, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = __emc2101_pwm_freq_write(dev, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_pwm_input_read(struct device *dev, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 fan_set;
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_read_u8(client, REG_FAN_SET, &fan_set);
	mutex_unlock(&data->mutex);

	if (!ret)
		*val = fan_set;

	return ret;
}

static int emc2101_pwm_input_write(struct device *dev, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 fan_set = clamp_val(val, 0, FAN_SET_MASK);
	int ret;

	mutex_lock(&data->mutex);
	ret = emc2101_write_u8(client, REG_FAN_SET, fan_set);
	mutex_unlock(&data->mutex);

	return ret;
}

static int emc2101_temp_crit_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_crit_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_crit_write(struct device *dev, int ch, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_crit_write(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_crit_hyst_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_crit_hyst_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_crit_hyst_write(struct device *dev, int ch, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_crit_hyst_write(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_crit_alarm_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_crit_alarm_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_fault_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_fault_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_input_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_input_read(data, val);
		break;
	case EMC2101_TC_FORCE:
		ret = emc2101_temp_force_input_read(data, val);
		break;
	case EMC2101_TC_INT:
		ret = emc2101_temp_int_input_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_input_write(struct device *dev, int ch, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_FORCE:
		ret = emc2101_temp_force_input_write(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_label(struct device *dev, int ch, const char **str)
{
	int ret = 0;

	switch (ch) {
	case EMC2101_TC_EXT:
		*str = "external";
		break;
	case EMC2101_TC_FORCE:
		*str = "force";
		break;
	case EMC2101_TC_INT:
		*str = "internal";
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_max_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_max_read(data, val);
		break;
	case EMC2101_TC_INT:
		ret = emc2101_temp_int_max_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_max_write(struct device *dev, int ch, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_max_write(data, val);
		break;
	case EMC2101_TC_INT:
		ret = emc2101_temp_int_max_write(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_max_alarm_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_max_alarm_read(data, val);
		break;
	case EMC2101_TC_INT:
		ret = emc2101_temp_int_max_alarm_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_min_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_min_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_min_write(struct device *dev, int ch, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_min_write(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_min_alarm_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_min_alarm_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_type_read(struct device *dev, int ch, long *val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_type_read(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_temp_type_write(struct device *dev, int ch, long val)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	int ret;

	switch (ch) {
	case EMC2101_TC_EXT:
		ret = emc2101_temp_ext_type_write(data, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int emc2101_update_interval_read(struct device *dev,
					long *update_interval)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;
	u8 val;

	mutex_lock(&data->mutex);
	ret = emc2101_read_u8(client, REG_CONV_RATE, &val);
	mutex_unlock(&data->mutex);

	if (!ret) {
		val &= CONV_RATE_MASK;
		if (val < ARRAY_SIZE(emc2101_conv_time))
			*update_interval = emc2101_conv_time[val];
		else
			*update_interval = 32000;
	}

	return ret;
}

static int emc2101_update_interval_write(struct device *dev,
					 long update_interval)
{
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;
	u8 val;

	val = find_closest(update_interval, emc2101_conv_time,
			   ARRAY_SIZE(emc2101_conv_time));

	mutex_lock(&data->mutex);
	ret = emc2101_write_u8(client, REG_CONV_RATE, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static inline int emc2101_max_channels(enum hwmon_sensor_types type)
{
	if (type == hwmon_temp)
		return EMC2101_TC_NUM;
	else
		return 1;
}

static inline umode_t emc2101_temp_input_is_visible(int channel)
{
	if (channel == EMC2101_TC_FORCE)
		return 0644;
	else
		return 0444;
}

static umode_t emc2101_is_visible(const void *data,
				  enum hwmon_sensor_types type, u32 attr,
				  int channel)
{
	if (channel >= emc2101_max_channels(type))
		return 0;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_min_alarm:
			return 0444;
		case hwmon_fan_div:
		case hwmon_fan_min:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_auto_channels_temp:
		case hwmon_pwm_enable:
		case hwmon_pwm_freq:
		case hwmon_pwm_input:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_crit_alarm:
		case hwmon_temp_fault:
		case hwmon_temp_label:
		case hwmon_temp_max_alarm:
		case hwmon_temp_min_alarm:
			return 0444;
		case hwmon_temp_crit:
		case hwmon_temp_crit_hyst:
		case hwmon_temp_max:
		case hwmon_temp_min:
		case hwmon_temp_type:
			return 0644;
		case hwmon_temp_input:
			return emc2101_temp_input_is_visible(channel);
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
};

static int emc2101_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return emc2101_update_interval_read(dev, val);
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_div:
			return emc2101_fan_div_read(dev, val);
		case hwmon_fan_input:
			return emc2101_fan_input_read(dev, val);
		case hwmon_fan_min:
			return emc2101_fan_min_read(dev, val);
		case hwmon_fan_min_alarm:
			return emc2101_fan_min_alarm_read(dev, val);
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_auto_channels_temp:
			return emc2101_pwm_auto_channels_temp_read(dev, val);
		case hwmon_pwm_enable:
			return emc2101_pwm_enable_read(dev, val);
		case hwmon_pwm_freq:
			return emc2101_pwm_freq_read(dev, val);
		case hwmon_pwm_input:
			return emc2101_pwm_input_read(dev, val);
		default:
			break;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_crit:
			return emc2101_temp_crit_read(dev, channel, val);
		case hwmon_temp_crit_alarm:
			return emc2101_temp_crit_alarm_read(dev, channel, val);
		case hwmon_temp_crit_hyst:
			return emc2101_temp_crit_hyst_read(dev, channel, val);
		case hwmon_temp_fault:
			return emc2101_temp_fault_read(dev, channel, val);
		case hwmon_temp_input:
			return emc2101_temp_input_read(dev, channel, val);
		case hwmon_temp_max:
			return emc2101_temp_max_read(dev, channel, val);
		case hwmon_temp_max_alarm:
			return emc2101_temp_max_alarm_read(dev, channel, val);
		case hwmon_temp_min:
			return emc2101_temp_min_read(dev, channel, val);
		case hwmon_temp_min_alarm:
			return emc2101_temp_min_alarm_read(dev, channel, val);
		case hwmon_temp_type:
			return emc2101_temp_type_read(dev, channel, val);
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static int emc2101_read_string(struct device *dev,
			       enum hwmon_sensor_types type, u32 attr,
			       int channel, const char **str)
{
	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_label:
			return emc2101_temp_label(dev, channel, str);
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static int emc2101_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return emc2101_update_interval_write(dev, val);
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_div:
			return emc2101_fan_div_write(dev, val);
		case hwmon_fan_min:
			return emc2101_fan_min_write(dev, val);
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_auto_channels_temp:
			return emc2101_pwm_auto_channels_temp_write(dev, val);
		case hwmon_pwm_enable:
			return emc2101_pwm_enable_write(dev, val);
		case hwmon_pwm_freq:
			return emc2101_pwm_freq_write(dev, val);
		case hwmon_pwm_input:
			return emc2101_pwm_input_write(dev, val);
		default:
			break;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_crit:
			return emc2101_temp_crit_write(dev, channel, val);
		case hwmon_temp_crit_hyst:
			return emc2101_temp_crit_hyst_write(dev, channel, val);
		case hwmon_temp_input:
			return emc2101_temp_input_write(dev, channel, val);
		case hwmon_temp_max:
			return emc2101_temp_max_write(dev, channel, val);
		case hwmon_temp_min:
			return emc2101_temp_min_write(dev, channel, val);
		case hwmon_temp_type:
			return emc2101_temp_type_write(dev, channel, val);
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

#define EMC2101_CHIP_CFG	HWMON_C_UPDATE_INTERVAL
#define EMC2101_FAN_CFG		(HWMON_F_DIV |\
				 HWMON_F_INPUT |\
				 HWMON_F_MIN |\
				 HWMON_F_MIN_ALARM |\
				 HWMON_F_TARGET)
#define EMC2101_PWM_CFG		(HWMON_PWM_AUTO_CHANNELS_TEMP |\
				 HWMON_PWM_ENABLE |\
				 HWMON_PWM_FREQ |\
				 HWMON_PWM_INPUT)
#define EMC2101_TEMP_INT_CFG	(HWMON_T_INPUT |\
				 HWMON_T_LABEL |\
				 HWMON_T_MAX |\
				 HWMON_T_MAX_ALARM)
#define EMC2101_TEMP_EXT_CFG	(HWMON_T_CRIT |\
				 HWMON_T_CRIT_ALARM |\
				 HWMON_T_CRIT_HYST |\
				 HWMON_T_FAULT |\
				 HWMON_T_INPUT |\
				 HWMON_T_LABEL |\
				 HWMON_T_MAX |\
				 HWMON_T_MAX_ALARM |\
				 HWMON_T_MIN |\
				 HWMON_T_MIN_ALARM |\
				 HWMON_T_TYPE)
#define EMC2101_TEMP_FORCE_CFG	(HWMON_T_INPUT |\
				 HWMON_T_LABEL)

static const struct hwmon_channel_info * const emc2101_info[] = {
	HWMON_CHANNEL_INFO(chip, EMC2101_CHIP_CFG),
	HWMON_CHANNEL_INFO(fan, EMC2101_FAN_CFG),
	HWMON_CHANNEL_INFO(pwm, EMC2101_PWM_CFG),
	HWMON_CHANNEL_INFO(temp, EMC2101_TEMP_INT_CFG,
			   EMC2101_TEMP_EXT_CFG,
			   EMC2101_TEMP_FORCE_CFG),
	NULL
};

static const struct hwmon_ops emc2101_ops = {
	.is_visible = emc2101_is_visible,
	.read = emc2101_read,
	.read_string = emc2101_read_string,
	.write = emc2101_write,
};

static const struct hwmon_chip_info emc2101_chip_info = {
	.info = emc2101_info,
	.ops = &emc2101_ops,
};

static int emc2101_init(struct device *dev)
{
	static const u8 lut_t[FAN_LUT_COUNT] = {  30,   35,   40,   45,
						  50,   55,   60,   65};
	static const u8 lut_s[FAN_LUT_COUNT] = {0x12, 0x19, 0x1f, 0x25,
						0x2c, 0x32, 0x38, 0x3f};
	struct emc2101_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned int i;
	int ret;

	ret = emc2101_rmw_u8(client, REG_ALERT_MASK, 0,
			     (IRQ_TACH_MIN_DISABLE |
			      IRQ_TEMP_EXT_CRIT_DISABLE |
			      IRQ_TEMP_EXT_MIN_DISABLE |
			      IRQ_TEMP_EXT_MAX_DISABLE |
			      IRQ_TEMP_INT_MAX_DISABLE));
	if (ret)
		return ret;

	ret = emc2101_write_u8(client, REG_FAN_CONFIG,
			       (FAN_CLK_OVR |
				FAN_LUT_DISABLE |
				FAN_POL_INV |
				TACH_MIN_FALSE_READ));
	if (ret)
		return ret;
	for (i = 0; i < FAN_LUT_COUNT; i++) {
		ret = emc2101_write_u8(client, REG_FAN_LUT_TEMP(i), lut_t[i]);
		if (ret)
			return ret;
		ret = emc2101_write_u8(client, REG_FAN_LUT_SPEED(i), lut_s[i]);
		if (ret)
			return ret;
	}
	ret = emc2101_rmw_u8(client, REG_FAN_CONFIG, FAN_LUT_DISABLE, 0);
	if (ret)
		return ret;

	ret = emc2101_write_u8(client, REG_FAN_SPIN,
			       (FAN_SPIN_DRIVE_100 |
				FAN_SPIN_TACH_ABORT |
				FAN_SPIN_TIME_3200));
	if (ret)
		return ret;

	ret = emc2101_write_u8(client, REG_PWM_FREQ, PWM_FREQ_MASK);
	if (ret)
		return ret;

	return emc2101_write_u8(client, REG_PWM_FREQ_DIV, 1);
}

static int emc2101_probe(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	struct device *dev = &client->dev;
	struct emc2101_data *data;
	struct device *hwmon_dev;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	data = devm_kzalloc(&client->dev, sizeof(struct emc2101_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->mutex);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 data,
							 &emc2101_chip_info,
							 emc2101_hwmon_groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(&client->dev, "%s: sensor '%s'\n",
		 dev_name(hwmon_dev), client->name);

	return emc2101_init(dev);
}

static int emc2101_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	s32 manufacturer, product, revision;
	struct device *dev = &adapter->dev;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	manufacturer = i2c_smbus_read_byte_data(client, REG_MANUFACTURER_ID);
	if (manufacturer != MANUFACTURER_ID)
		return -ENODEV;

	product = i2c_smbus_read_byte_data(client, REG_PRODUCT_ID);
	switch (product) {
	case EMC2101:
		strscpy(info->type, "emc2101", I2C_NAME_SIZE);
		break;
	case EMC2101_R:
		strscpy(info->type, "emc2101-r", I2C_NAME_SIZE);
		break;
	default:
		return -ENODEV;
	}

	revision = i2c_smbus_read_byte_data(client, REG_REVISION);

	dev_info(dev, "Found %s at 0x%02x (rev 0x%02x).\n",
		 info->type, client->addr, revision);

	return 0;
}

static const struct i2c_device_id emc2101_ids[] = {
	{ "emc2101" },
	{ "emc2101-r" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, emc2101_ids);

static const struct of_device_id emc2101_of_match_table[] = {
	{ .compatible = "microchip,emc2101", },
	{ .compatible = "microchip,emc2101-r", },
	{ },
};
MODULE_DEVICE_TABLE(of, emc2101_of_match_table);

static const unsigned short emc2101_address_list[] = {
	0x4c, I2C_CLIENT_END
};

static struct i2c_driver emc2101_driver = {
	.address_list = emc2101_address_list,
	.detect = emc2101_detect,
	.driver = {
		.name = "emc2101",
		.of_match_table = emc2101_of_match_table,
	},
	.id_table = emc2101_ids,
	.probe = emc2101_probe,
};
module_i2c_driver(emc2101_driver);

MODULE_AUTHOR("Álvaro Fernández Rojas <noltari@gmail.com>");
MODULE_DESCRIPTION("Microchip EMC2101 hwmon driver");
MODULE_LICENSE("GPL");
