// SPDX-License-Identifier: GPL-2.0-only
/*
 * Logitech Squeezebox Radio MSP430 companion controller
 *
 * The MSP430 owns the rotary encoders, IR receiver, RTC, and battery/charger
 * telemetry. RTC and system power-off use their native kernel frameworks.
 *
 * Input events, telemetry attributes, the driver name and the
 * /sys/devices/platform/i2c-adapter:i2c-N path reproduce the SqueezeOS 7.x
 * (2.6.26) ABI consumed by SqueezePlay and monitor_msp430. Charger controls
 * and MSP430 firmware programming are deliberately excluded.
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/processor.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>

#define MSP430_ID_REG			1
#define MSP430_ID_VALUE		0xcc
#define MSP430_VERSION_REG		2
#define MSP430_FLAGS_REG		10
#define MSP430_NAV_ENCODER_REG		20
#define MSP430_VOLUME_ENCODER_REG	21
#define MSP430_IR_REG			30
#define MSP430_AMBIENT_REG		40
#define MSP430_BATTERY_CHARGE_REG	50
#define MSP430_BATTERY_CAPACITY_REG	52
#define MSP430_BATTERY_VOLTAGE_REG	55
#define MSP430_SYSTEM_VOLTAGE_REG	57
#define MSP430_BATTERY_VMON1_REG	59
#define MSP430_BATTERY_VMON2_REG	61
#define MSP430_BATTERY_TEMPERATURE_REG	62
#define MSP430_WALL_VOLTAGE_REG	63
#define MSP430_CHARGER_EVENT_REG	64
#define MSP430_CHARGER_STATE_REG	65
#define MSP430_POWER_MODE_REG		66
#define MSP430_TIME_REG		70
#define MSP430_ALARM_REG		71
#define MSP430_KILL_REG		99

#define MSP430_FLAG_NAV	BIT(0)
#define MSP430_FLAG_VOLUME	BIT(1)
#define MSP430_FLAG_IR		BIT(2)
#define MSP430_FLAG_BATTERY	BIT(4)

/* SqueezeOS ABI: EV_SW code 1 toggles on every battery/charger event. */
#define MSP430_SW_BATTERY	1

struct squeezebox_msp430 {
	struct i2c_client *client;
	struct input_dev *input;
	struct mutex lock;
	bool battery_switch;
	struct rtc_device *rtc;
};

struct squeezebox_msp430_attribute {
	struct device_attribute dev_attr;
	u8 reg;
	u8 len;
};

static int squeezebox_msp430_read(struct squeezebox_msp430 *msp, u8 reg,
				  void *buf, size_t len)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(msp->client, reg, len, buf);
	if (ret < 0)
		return ret;
	if (ret != len)
		return -EIO;

	return 0;
}

static int squeezebox_msp430_write(struct squeezebox_msp430 *msp, u8 reg,
				   const void *buf, size_t len)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(msp->client, reg, len, buf);
	if (ret < 0)
		return ret;

	return 0;
}

static void squeezebox_msp430_report_input(struct squeezebox_msp430 *msp,
					   unsigned int flags)
{
	u8 value[4];
	bool sync = false;

	if (flags & MSP430_FLAG_BATTERY) {
		/*
		 * SqueezePlay re-reads charger_state on any value change, so
		 * toggle the switch instead of reporting a level.
		 */
		msp->battery_switch = !msp->battery_switch;
		input_report_switch(msp->input, MSP430_SW_BATTERY,
				    msp->battery_switch);
		sync = true;
	}

	if (flags & (MSP430_FLAG_NAV | MSP430_FLAG_VOLUME)) {
		if (!squeezebox_msp430_read(msp, MSP430_NAV_ENCODER_REG, value, 1))
			input_report_rel(msp->input, REL_WHEEL, (s8)value[0]);
		if (!squeezebox_msp430_read(msp, MSP430_VOLUME_ENCODER_REG, value, 1))
			input_report_rel(msp->input, REL_MISC, (s8)value[0]);
		sync = true;
	}

	if (flags & MSP430_FLAG_IR) {
		if (!squeezebox_msp430_read(msp, MSP430_IR_REG, value, sizeof(value)))
			input_event(msp->input, EV_MSC, MSC_RAW,
				    get_unaligned_le32(value));
		sync = true;
	}

	if (sync)
		input_sync(msp->input);
}

static irqreturn_t squeezebox_msp430_irq_thread(int irq, void *data)
{
	struct squeezebox_msp430 *msp = data;
	int flags;

	mutex_lock(&msp->lock);
	flags = i2c_smbus_read_byte_data(msp->client, MSP430_FLAGS_REG);
	if (flags < 0)
		dev_err_ratelimited(&msp->client->dev,
				    "failed to read interrupt flags: %d\n", flags);
	else if (flags)
		squeezebox_msp430_report_input(msp, flags);
	mutex_unlock(&msp->lock);

	return IRQ_HANDLED;
}

static int squeezebox_msp430_rtc_read_time(struct device *dev,
					   struct rtc_time *tm)
{
	struct squeezebox_msp430 *msp = dev_get_drvdata(dev);
	u8 value[4];
	int ret;

	mutex_lock(&msp->lock);
	ret = squeezebox_msp430_read(msp, MSP430_TIME_REG, value, sizeof(value));
	mutex_unlock(&msp->lock);
	if (ret)
		return ret;

	rtc_time64_to_tm(get_unaligned_le32(value), tm);
	return 0;
}

static int squeezebox_msp430_rtc_set_time(struct device *dev,
					  struct rtc_time *tm)
{
	struct squeezebox_msp430 *msp = dev_get_drvdata(dev);
	time64_t seconds;
	u8 value[4];
	int ret;

	seconds = rtc_tm_to_time64(tm);
	if (seconds < 0 || seconds > U32_MAX)
		return -ERANGE;

	put_unaligned_le32(seconds, value);

	mutex_lock(&msp->lock);
	ret = squeezebox_msp430_write(msp, MSP430_TIME_REG, value, sizeof(value));
	mutex_unlock(&msp->lock);

	return ret;
}


static int squeezebox_msp430_rtc_read_alarm(struct device *dev,
					    struct rtc_wkalrm *alarm)
{
	struct squeezebox_msp430 *msp = dev_get_drvdata(dev);
	u8 value[4];
	int ret;

	mutex_lock(&msp->lock);
	ret = squeezebox_msp430_read(msp, MSP430_ALARM_REG, value, sizeof(value));
	mutex_unlock(&msp->lock);
	if (ret)
		return ret;

	rtc_time64_to_tm(get_unaligned_le32(value), &alarm->time);
	/*
	 * The legacy protocol has no alarm-enable register. Writing an alarm
	 * timestamp arms it, so an existing timestamp is reported as enabled.
	 */
	alarm->enabled = 1;
	alarm->pending = 0;

	return 0;
}

static int squeezebox_msp430_rtc_set_alarm(struct device *dev,
					   struct rtc_wkalrm *alarm)
{
	struct squeezebox_msp430 *msp = dev_get_drvdata(dev);
	time64_t seconds;
	u8 value[4];
	int ret;

	if (!alarm->enabled)
		return -EOPNOTSUPP;

	seconds = rtc_tm_to_time64(&alarm->time);
	if (seconds < 0 || seconds > U32_MAX)
		return -ERANGE;

	put_unaligned_le32(seconds, value);

	mutex_lock(&msp->lock);
	ret = squeezebox_msp430_write(msp, MSP430_ALARM_REG, value, sizeof(value));
	mutex_unlock(&msp->lock);

	return ret;
}
static const struct rtc_class_ops squeezebox_msp430_rtc_ops = {
	.read_time = squeezebox_msp430_rtc_read_time,
	.set_time = squeezebox_msp430_rtc_set_time,
	.read_alarm = squeezebox_msp430_rtc_read_alarm,
	.set_alarm = squeezebox_msp430_rtc_set_alarm,
};

/* Legacy format: the little-endian register value as an unsigned decimal. */
static ssize_t squeezebox_msp430_show_register(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct squeezebox_msp430_attribute *msp_attr =
		container_of(attr, struct squeezebox_msp430_attribute, dev_attr);
	struct squeezebox_msp430 *msp = dev_get_drvdata(dev);
	u8 value[4];
	u32 number = 0;
	int i, ret;

	mutex_lock(&msp->lock);
	ret = squeezebox_msp430_read(msp, msp_attr->reg, value, msp_attr->len);
	mutex_unlock(&msp->lock);
	if (ret)
		return ret;

	for (i = msp_attr->len - 1; i >= 0; i--)
		number = number << 8 | value[i];

	return sysfs_emit(buf, "%u\n", number);
}

/*
 * SqueezePlay arms the wake alarm with an epoch time and clears it with
 * 4294967295. The RTC-class alarm writes the same register.
 */
static ssize_t squeezebox_msp430_store_alarm_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct squeezebox_msp430 *msp = dev_get_drvdata(dev);
	u8 value[4];
	u32 seconds;
	int ret;

	ret = kstrtou32(buf, 10, &seconds);
	if (ret)
		return ret;

	put_unaligned_le32(seconds, value);

	mutex_lock(&msp->lock);
	ret = squeezebox_msp430_write(msp, MSP430_ALARM_REG, value, sizeof(value));
	mutex_unlock(&msp->lock);

	return ret ? ret : count;
}

#define MSP430_ATTR(_name, _reg, _len) \
	static struct squeezebox_msp430_attribute msp430_attr_##_name = { \
		.dev_attr = __ATTR(_name, 0444, squeezebox_msp430_show_register, NULL), \
		.reg = _reg, \
		.len = _len, \
	}

MSP430_ATTR(fw, MSP430_VERSION_REG, 2);
MSP430_ATTR(ambient, MSP430_AMBIENT_REG, 4);
MSP430_ATTR(battery_charge, MSP430_BATTERY_CHARGE_REG, 2);
MSP430_ATTR(battery_capacity, MSP430_BATTERY_CAPACITY_REG, 2);
MSP430_ATTR(battery_voltage, MSP430_BATTERY_VOLTAGE_REG, 2);
MSP430_ATTR(sys_voltage, MSP430_SYSTEM_VOLTAGE_REG, 2);
MSP430_ATTR(battery_vmon1_voltage, MSP430_BATTERY_VMON1_REG, 2);
MSP430_ATTR(battery_vmon2_voltage, MSP430_BATTERY_VMON2_REG, 2);
MSP430_ATTR(battery_temperature, MSP430_BATTERY_TEMPERATURE_REG, 2);
MSP430_ATTR(wall_voltage, MSP430_WALL_VOLTAGE_REG, 2);
MSP430_ATTR(charger_event, MSP430_CHARGER_EVENT_REG, 2);
MSP430_ATTR(charger_state, MSP430_CHARGER_STATE_REG, 2);
MSP430_ATTR(power_mode, MSP430_POWER_MODE_REG, 2);

static struct squeezebox_msp430_attribute msp430_attr_alarm_time = {
	.dev_attr = __ATTR(alarm_time, 0644, squeezebox_msp430_show_register,
			   squeezebox_msp430_store_alarm_time),
	.reg = MSP430_ALARM_REG,
	.len = 4,
};

static struct attribute *squeezebox_msp430_attrs[] = {
	&msp430_attr_fw.dev_attr.attr,
	&msp430_attr_ambient.dev_attr.attr,
	&msp430_attr_battery_charge.dev_attr.attr,
	&msp430_attr_battery_capacity.dev_attr.attr,
	&msp430_attr_battery_voltage.dev_attr.attr,
	&msp430_attr_sys_voltage.dev_attr.attr,
	&msp430_attr_battery_vmon1_voltage.dev_attr.attr,
	&msp430_attr_battery_vmon2_voltage.dev_attr.attr,
	&msp430_attr_battery_temperature.dev_attr.attr,
	&msp430_attr_wall_voltage.dev_attr.attr,
	&msp430_attr_charger_event.dev_attr.attr,
	&msp430_attr_charger_state.dev_attr.attr,
	&msp430_attr_power_mode.dev_attr.attr,
	&msp430_attr_alarm_time.dev_attr.attr,
	NULL,
};

static const struct attribute_group squeezebox_msp430_attr_group = {
	.attrs = squeezebox_msp430_attrs,
};

static void squeezebox_msp430_put_kobject(void *kobj)
{
	kobject_put(kobj);
}

/*
 * SqueezePlay opens part of the telemetry through the 2.6.26 path
 * /sys/devices/platform/i2c-adapter:i2c-1/1-0010/. Recreate that directory
 * with a link to the client device. Board-specific glue, not for upstream.
 */
static int squeezebox_msp430_add_legacy_path(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kobject *adapter;
	char name[32];
	int ret;

	snprintf(name, sizeof(name), "i2c-adapter:i2c-%d", client->adapter->nr);
	adapter = kobject_create_and_add(name, &platform_bus.kobj);
	if (!adapter)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, squeezebox_msp430_put_kobject,
				       adapter);
	if (ret)
		return ret;

	return sysfs_create_link(adapter, &dev->kobj, dev_name(dev));
}

static int squeezebox_msp430_power_off(struct sys_off_data *data)
{
	struct squeezebox_msp430 *msp = data->cb_data;
	u8 command = 0xde;
	int ret;

	mutex_lock(&msp->lock);
	ret = squeezebox_msp430_write(msp, MSP430_KILL_REG, &command,
				      sizeof(command));
	mutex_unlock(&msp->lock);
	if (ret) {
		dev_err(&msp->client->dev, "failed to request power-off: %d\n",
			ret);
		return NOTIFY_STOP;
	}

	/*
	 * The MSP430 cuts system power asynchronously after accepting this
	 * command. Returning lets the generic shutdown path continue and
	 * produces a spurious warning before the board loses power. The legacy
	 * driver likewise did not return after this write.
	 */
	for (;;)
		cpu_relax();
}


static int squeezebox_msp430_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct squeezebox_msp430 *msp;
	int ret;

	ret = i2c_smbus_read_byte_data(client, MSP430_ID_REG);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read identity\n");
	if (ret != MSP430_ID_VALUE)
		return dev_err_probe(dev, -ENODEV, "unexpected identity %#x\n", ret);

	msp = devm_kzalloc(dev, sizeof(*msp), GFP_KERNEL);
	if (!msp)
		return -ENOMEM;

	msp->client = client;
	mutex_init(&msp->lock);
	i2c_set_clientdata(client, msp);

	msp->input = devm_input_allocate_device(dev);
	if (!msp->input)
		return -ENOMEM;

	/*
	 * SqueezePlay's baby_bsp opens the evdev whose name contains "msp430".
	 * Without an EV_KEY capability mousedev does not bind, so the wheel is
	 * not delivered twice (through baby_bsp and SDL's mouse), as on 2.6.26.
	 */
	msp->input->name = "msp430";
	msp->input->phys = "squeezebox-msp430/input0";
	msp->input->id.bustype = BUS_I2C;
	input_set_capability(msp->input, EV_REL, REL_WHEEL);
	input_set_capability(msp->input, EV_REL, REL_MISC);
	input_set_capability(msp->input, EV_MSC, MSC_RAW);
	input_set_capability(msp->input, EV_SW, MSP430_SW_BATTERY);

	ret = input_register_device(msp->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	msp->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(msp->rtc))
		return dev_err_probe(dev, PTR_ERR(msp->rtc),
				     "failed to allocate RTC\n");
	msp->rtc->ops = &squeezebox_msp430_rtc_ops;
	msp->rtc->range_max = U32_MAX;

	ret = devm_rtc_register_device(msp->rtc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register RTC\n");

	ret = devm_device_add_group(dev, &squeezebox_msp430_attr_group);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create telemetry attributes\n");

	ret = squeezebox_msp430_add_legacy_path(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create legacy sysfs path\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
			squeezebox_msp430_irq_thread,
			IRQF_ONESHOT | IRQF_TRIGGER_LOW, dev_name(dev), msp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	if (of_device_is_system_power_controller(dev->of_node)) {
		ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
						    SYS_OFF_PRIO_DEFAULT,
						    squeezebox_msp430_power_off,
						    msp);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to register power-off handler\n");
	}

	dev_info(dev, "MSP430 companion controller detected\n");
	return 0;
}

static const struct of_device_id squeezebox_msp430_of_match[] = {
	{ .compatible = "logitech,squeezebox-radio-msp430" },
	{ }
};
MODULE_DEVICE_TABLE(of, squeezebox_msp430_of_match);

static const struct i2c_device_id squeezebox_msp430_id[] = {
	{ "squeezebox-msp430" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, squeezebox_msp430_id);

static struct i2c_driver squeezebox_msp430_driver = {
	.driver = {
		/* monitor_msp430 reads /sys/bus/i2c/drivers/msp430/1-0010/fw. */
		.name = "msp430",
		.of_match_table = squeezebox_msp430_of_match,
	},
	.probe = squeezebox_msp430_probe,
	.id_table = squeezebox_msp430_id,
};
module_i2c_driver(squeezebox_msp430_driver);

MODULE_AUTHOR("Logitech");
MODULE_DESCRIPTION("Logitech Squeezebox Radio MSP430 companion controller");
MODULE_LICENSE("GPL");
