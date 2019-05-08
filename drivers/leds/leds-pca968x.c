/*
 * Copyright 2016 SECO srl
 *
 * Author: Davide Cardillo <davide.cardillo@gmail.com>
 *
 * Based on leds-pca963x.c
 *
 * LED driver for the PCA968x I2C LED driver
 *
 * This driver uses the sled class.
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/sled.h>
#include "leds-pca968x.h"



#define DRV_VERSION                         "1.0"

#define PCA968x_INFO(fmt, arg...)           printk(KERN_INFO "NXP,PCA968x: " fmt "\n" , ## arg)
#define PCA968x_ERR(fmt, arg...)            printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define PCA968x_DBG(fmt, arg...)            pr_debug("%s: " fmt "\n" , __func__ , ## arg)



/* __________________________________________________________________________
* |_____________________________ CHIP DEFINITION ____________________________|
*/
enum pca968x_type {
	pca9685,
};


struct pca968x_chipdef {
	int               led_num;
	int               reg_num;
	unsigned long     osc_internal;
	int               res_steps;
	uint8_t           *reg_list;
};


/* __________________________________________________________________________
* |___________________________ PCA968X DEFINITION ___________________________|
*/
static uint8_t pca9685_reg_list[] = {
	PCA9685_REG_MODE1,  PCA9685_REG_MODE2,  PCA9685_REG_SUBADR1,
	PCA9685_REG_SUBADR2,  PCA9685_REG_SUBADR3,  PCA9685_REG_ALLCALLADR,
  	PCA9685_REG_LED0_ON_L,   PCA9685_REG_LDE0_ON_H,   PCA9685_REG_LED0_OFF_L,   PCA9685_REG_LDE0_OFF_H,
  	PCA9685_REG_LED1_ON_L,   PCA9685_REG_LDE1_ON_H,   PCA9685_REG_LED1_OFF_L,   PCA9685_REG_LDE1_OFF_H,
  	PCA9685_REG_LED2_ON_L,   PCA9685_REG_LDE2_ON_H,   PCA9685_REG_LED2_OFF_L,   PCA9685_REG_LDE2_OFF_H,
  	PCA9685_REG_LED3_ON_L,   PCA9685_REG_LDE3_ON_H,   PCA9685_REG_LED3_OFF_L,   PCA9685_REG_LDE3_OFF_H,
  	PCA9685_REG_LED4_ON_L,   PCA9685_REG_LDE4_ON_H,   PCA9685_REG_LED4_OFF_L,   PCA9685_REG_LDE4_OFF_H,
  	PCA9685_REG_LED5_ON_L,   PCA9685_REG_LDE5_ON_H,   PCA9685_REG_LED5_OFF_L,   PCA9685_REG_LDE5_OFF_H,
  	PCA9685_REG_LED6_ON_L,   PCA9685_REG_LDE6_ON_H,   PCA9685_REG_LED6_OFF_L,   PCA9685_REG_LDE6_OFF_H,
  	PCA9685_REG_LED7_ON_L,   PCA9685_REG_LDE7_ON_H,   PCA9685_REG_LED7_OFF_L,   PCA9685_REG_LDE7_OFF_H,
  	PCA9685_REG_LED8_ON_L,   PCA9685_REG_LDE8_ON_H,   PCA9685_REG_LED8_OFF_L,   PCA9685_REG_LDE8_OFF_H,
  	PCA9685_REG_LED9_ON_L,   PCA9685_REG_LDE9_ON_H,   PCA9685_REG_LED9_OFF_L,   PCA9685_REG_LDE9_OFF_H,
  	PCA9685_REG_LED10_ON_L,  PCA9685_REG_LDE10_ON_H,  PCA9685_REG_LED10_OFF_L,  PCA9685_REG_LDE10_OFF_H,
  	PCA9685_REG_LED11_ON_L,  PCA9685_REG_LDE11_ON_H,  PCA9685_REG_LED11_OFF_L,  PCA9685_REG_LDE11_OFF_H,
  	PCA9685_REG_LED12_ON_L,  PCA9685_REG_LDE12_ON_H,  PCA9685_REG_LED12_OFF_L,  PCA9685_REG_LDE12_OFF_H,
  	PCA9685_REG_LED13_ON_L,  PCA9685_REG_LDE13_ON_H,  PCA9685_REG_LED13_OFF_L,  PCA9685_REG_LDE13_OFF_H,
  	PCA9685_REG_LED14_ON_L,  PCA9685_REG_LDE14_ON_H,  PCA9685_REG_LED14_OFF_L,  PCA9685_REG_LDE14_OFF_H,
  	PCA9685_REG_LED15_ON_L,  PCA9685_REG_LDE15_ON_H,  PCA9685_REG_LED15_OFF_L,  PCA9685_REG_LDE15_OFF_H,
	PCA9685_REG_ALL_LED_ON_L, PCA9685_REG_ALL_LED_ON_H, PCA9685_REG_ALL_LED_OFF_L,  PCA9685_REG_ALL_LED_OFF_H,
	PCA9685_REG_PRE_SCALE, 	PCA9685_REG_TEST_MODE
};




/* __________________________________________________________________________
* |________________________________ CHIP LIST _______________________________|
*/
static struct pca968x_chipdef pca968x_chipdefs[] = {
	[pca9685] = {
		.led_num       = 16,
		.reg_num       = PCA9685_REG_NUMBER,
		.osc_internal  = PCA9685_OSC_INTERNAL,
		.res_steps     = 4096,
		.reg_list      = pca9685_reg_list,
	},
};


#define PCA968X_OFFSET_LED_ON_L             0
#define PCA968X_OFFSET_LED_ON_H             1
#define PCA968X_OFFSET_LED_OFF_L            2
#define PCA968X_OFFSET_LED_OFF_H            3

#define PCA968X_FULL_SET                    (1 << 4)


#define LED_NAME_LEN                        64

#define LED_BASE_ADDR(x)                    PCA9685_REG_LED0_ON_L + ((x) * 4)

struct pca968x_led {
	int       led_id;
	char      name[LED_NAME_LEN];
	uint8_t   reg_base;
	bool      enable;
	int       duty;
	int       delay;
	int       d_on, d_off;
};


struct pca968x {
	struct i2c_client   *client;
	/*   chip attributes  */
	uint8_t             *reg_list;
	int                 reg_num;
	int                 led_num;
	unsigned long       osc_internal;
	int                 res_steps;

	struct pca968x_led  *leds;
	int                 used_leds;

	int                 gpio_enable;
	unsigned long       osc;
	/*  global PWM info  */
	int                 pwm_freq;
	int                 pwm_max_freq;
	int                 pwm_min_freq;
	int                 pwm_min_duty;
	int                 pwm_max_duty;
	int                 pwm_min_delay;
	int                 pwm_max_delay;
	/*  global status  */
	bool                enable;
	bool                sw_reset;
	bool                sleep_mode;
};


/*   __________________________________________________________________________
 *  |                                                                          |
 *  |                                PWM FUNCTIONS                             |
 *  |__________________________________________________________________________|
*/

                       /*   ---------- GLOBAL ----------  */

static void set_sleep_mode (struct pca968x *chip, int en) {

	uint8_t val = i2c_smbus_read_byte_data (chip->client, PCA9685_REG_MODE1);
	val = en ? val | (1 << 4) : val & ~(1 << 4);
	i2c_smbus_write_byte_data (chip->client, PCA9685_REG_MODE1, val);
	udelay (500);
}


static void set_sw_reset (struct pca968x *chip, int en) {
	uint8_t val = i2c_smbus_read_byte_data (chip->client, PCA9685_REG_MODE1);
	val = en ? val | (1 << 8) : val & ~(1 << 8);
	i2c_smbus_write_byte_data (chip->client, PCA9685_REG_MODE1, val);
	udelay (500);
}


static int get_global_enable (struct pca968x *chip) {
	return !(gpio_get_value_cansleep (chip->gpio_enable));
}


static void set_global_enable (struct pca968x *chip, int en) {
	chip->enable = !!en ? true : false;
	gpio_set_value_cansleep (chip->gpio_enable, !en);
}


static void set_all_led_off (struct pca968x *chip) {
	i2c_smbus_write_byte_data (chip->client, PCA9685_REG_ALL_LED_OFF_H, 0x10);
}

#define TIME_NS      1000000000
static unsigned long rate2period (unsigned long rate) {
	return TIME_NS / rate;
}


static unsigned long prescale2rate (struct pca968x *chip, int prescale) {
	return (unsigned long)(chip->osc / (chip->res_steps * (prescale)));
}


static unsigned int rate2prescale (struct pca968x *chip, unsigned long rate) {
	return (unsigned int)((chip->osc / (chip->res_steps * rate)) - 1);
}


static int set_pwm_frequency (struct pca968x *chip, unsigned long rate) {
	unsigned int prescale;

	if ( rate > chip->pwm_max_freq || rate < chip->pwm_min_freq )
		return -EINVAL;

	prescale = rate2prescale (chip, rate);

	set_sleep_mode (chip, 1);
	i2c_smbus_write_byte_data (chip->client, PCA9685_REG_PRE_SCALE, (uint8_t)prescale);
	set_sleep_mode (chip, 0);

	return 0;
}


                       /*   ---------- SIGNLE LED ----------  */

static int get_led_enable (struct pca968x *chip, int led_id) {
	int     reg = chip->leds[led_id].reg_base + PCA968X_OFFSET_LED_OFF_H;
	uint8_t val = i2c_smbus_read_byte_data (chip->client, reg);

	return (val & PCA968X_FULL_SET) ? 0 : 1;
}


static void set_led_enable (struct pca968x *chip, int led_id, int en) {
	int     reg = chip->leds[led_id].reg_base + PCA968X_OFFSET_LED_OFF_H;
	uint8_t val;

	if ( chip->leds[led_id].enable != !!en ) {
		val = i2c_smbus_read_byte_data (chip->client, reg);
		val = en ? val & ~PCA968X_FULL_SET : val | PCA968X_FULL_SET;
		i2c_smbus_write_byte_data (chip->client, reg, val);

		chip->leds[led_id].enable = !!en;
	}
}


static void set_led_timing (struct pca968x *chip, int led_id) {
	int                d_on, d_off;
	uint8_t            val_l, val_h;
	struct pca968x_led *led  = &chip->leds[led_id];
	int                delay = led->delay;
	int                duty  = led->duty;

	d_on = delay;

	if ( (d_on + duty) < chip->res_steps ) {
		d_off = d_on + duty;
	} else {
		d_off = duty - (chip->res_steps - d_on);
	}

	if ( d_on != led->d_on ) {
		led->d_on = d_on;

		val_l = (uint8_t)(d_on & 0xFF);
		val_h = ((uint8_t)(d_on >> 8) & 0x0F);

		i2c_smbus_write_byte_data (chip->client,
				led->reg_base + PCA968X_OFFSET_LED_ON_L, val_l);

		i2c_smbus_write_byte_data (chip->client,
				led->reg_base + PCA968X_OFFSET_LED_ON_H, val_h);
	}

	if ( d_off != led->d_off ) {
		led->d_off = d_off;

		val_l = (uint8_t)(d_off & 0xFF);
		val_h = (uint8_t)((d_off >> 8) & 0x0F);

		i2c_smbus_write_byte_data (chip->client,
				led->reg_base + PCA968X_OFFSET_LED_OFF_L, val_l);

		i2c_smbus_write_byte_data (chip->client,
				led->reg_base + PCA968X_OFFSET_LED_OFF_H, val_h);
	}
}


static void set_led_duty (struct pca968x *chip, int led_id, int duty) {
	chip->leds[led_id].duty = duty;
	set_led_timing (chip, led_id);
}


static void set_led_delay (struct pca968x *chip, int led_id, int delay) {
	chip->leds[led_id].delay = delay;
	set_led_timing (chip, led_id);
}



static void set_pca968x (struct pca968x *chip) {
	set_global_enable (chip, chip->enable);
	set_pwm_frequency (chip, chip->pwm_freq);
	set_sleep_mode    (chip, chip->sleep_mode);
	set_sw_reset      (chip, chip->sw_reset);
}


static void set_led (struct pca968x *chip, int led_id) {
	set_led_enable (chip, led_id, chip->leds[led_id].enable);
	set_led_delay  (chip, led_id, chip->leds[led_id].delay);
	set_led_duty   (chip, led_id, chip->leds[led_id].duty);
}

/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/*   __________________________________________________________________________
 *  |                                                                          |
 *  |                               SLED INTERFACE                             |
 *  |__________________________________________________________________________|
*/
int led_pwm_sled_get_enable (struct sled_device *sdev, int *enable) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);
	int                 id       = sdev->led_pwm->id_internal;

	pca_chip->leds[id].enable = get_led_enable (pca_chip, id);
	sdev->led_pwm->enable = pca_chip->leds[id].enable;
	*enable = sdev->led_pwm->enable;
	return 0;
}


int led_pwm_sled_set_enable (struct sled_device *sdev, int enable) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);
	int                 id       = sdev->led_pwm->id_internal;

	if ( enable == 0 || enable == 1 ) {
		sdev->led_pwm->enable = enable;
		set_led_enable (pca_chip, id, enable);
		return 0;
	} else
		return -EINVAL;
}


int led_pwm_sled_get_max_period (struct sled_device *sdev, unsigned long *max) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	*max = rate2period ((unsigned long)pca_chip->pwm_min_freq);;
	return 0;
}


int led_pwm_sled_get_mim_period (struct sled_device *sdev, unsigned long *min) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	*min = rate2period ((unsigned long)pca_chip->pwm_max_freq);;
	return 0;
}


int led_pwm_sled_get_max_duty (struct sled_device *sdev, unsigned long *max) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	*max = (unsigned long)pca_chip->pwm_max_duty;
	return 0;
}


int led_pwm_sled_get_duty (struct sled_device *sdev, unsigned long *duty) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);
	int                 id       = sdev->led_pwm->id_internal;

	*duty = (unsigned long)pca_chip->leds[id].duty;
	return 0;
}


int led_pwm_sled_set_duty (struct sled_device *sdev, unsigned long duty) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);
	int                 id       = sdev->led_pwm->id_internal;

	if ( duty > pca_chip->pwm_max_duty )
		return -EINVAL;
	else {
		set_led_duty (pca_chip, id, duty);
	}
	return 0;
}


int led_pwm_sled_get_max_delay (struct sled_device *sdev, unsigned long *max) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	*max = (unsigned long)pca_chip->pwm_max_delay;
	return 0;
}


int led_pwm_sled_get_delay (struct sled_device *sdev, unsigned long *delay) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);
	int                 id       = sdev->led_pwm->id_internal;

	*delay = (unsigned long)pca_chip->leds[id].delay;
	return 0;
}


int led_pwm_sled_set_delay (struct sled_device *sdev, unsigned long delay) {
	struct device      *dev      = sled_to_device_driver(sdev);
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);
	int                 id       = sdev->led_pwm->id_internal;

	if ( delay > pca_chip->pwm_max_delay )
		return -EINVAL;
	else {
		set_led_delay (pca_chip, id, delay);
	}
	return 0;
}


static const struct sled_class_ops led_pwm_sled_ops = {
	.set_enable     = led_pwm_sled_set_enable,
	.get_enable     = led_pwm_sled_get_enable,
	.set_stby_state = NULL,
	.get_stby_state = NULL,
	.set_duty       = led_pwm_sled_set_duty,
	.get_duty       = led_pwm_sled_get_duty,
	.get_delay      = led_pwm_sled_get_delay,
	.set_delay      = led_pwm_sled_set_delay,
	.set_stby_duty  = NULL,
	.get_stby_duty  = NULL,
	.get_max_duty   = led_pwm_sled_get_max_duty,
	.get_max_delay  = led_pwm_sled_get_max_delay,
	.set_period     = NULL,
	.get_period     = NULL,
	.get_max_period = led_pwm_sled_get_max_period,
	.get_min_period = led_pwm_sled_get_mim_period,
};



/*   __________________________________________________________________________
 *  |                                                                          |
 *  |                               SYSFS INTERFACE                            |
 *  |__________________________________________________________________________|
 */
static ssize_t pca968x_dump_reg (struct device *dev, struct device_attribute *attr,	char *buf) {
	ssize_t            retval  = (ssize_t)0;
	uint8_t            reg_val = 0;
	int                i       = 0;
	unsigned char      msg[900];
	unsigned char      tmp[50];

	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	if ( !pca_chip )
		retval = -EINVAL;
	else {
		sprintf (msg, "\n#\tvalue");
		for ( i = 0; i < pca_chip->reg_num ; i++ ) {
			reg_val = i2c_smbus_read_byte_data (client, pca_chip->reg_list[i]);
			sprintf (tmp, "\n0x%02X\t0x%02X", pca_chip->reg_list[i], reg_val);
			strcat (msg, tmp);
		}
	}

	retval = sprintf (buf, "%s\n", msg);
	return retval;
}


static ssize_t pca968x_write_reg (struct device *dev, struct device_attribute *attr,
									const char *buf, size_t n)
{
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	unsigned long      reg       = 0;
	unsigned long      value     = 0;
	char               *start    = (char *)buf;

	while ( *start == ' ' )
		start++;

	reg = simple_strtoul (start, &start, 16);

	while ( *start == ' ' )
		start++;

	value = simple_strtoul (start, NULL, 16);

	i2c_smbus_write_byte_data (client, pca_chip->reg_list[reg], (uint8_t)value);

	return n;
}


static ssize_t pca968x_enable_show (struct device *dev, struct device_attribute *attr, char *buf) {
	ssize_t            retval    = (ssize_t)-1;
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	if ( pca_chip )
		retval = sprintf (buf, "%s\n", get_global_enable (pca_chip) ? "enabled" : "disabled");

	return retval;

}


static ssize_t pca968x_enable_store (struct device *dev, struct device_attribute *attr,
									const char *buf, size_t n)
{
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	char               *start    = (char *)buf;
	unsigned long      value;

	while ( *start == ' ' )
		start++;

	value = simple_strtoul (start, NULL, 10);

	set_global_enable (pca_chip, (int)value);

	return n;
}


static ssize_t pca968x_frequency_show (struct device *dev, struct device_attribute *attr, char *buf) {
	ssize_t            retval    = (ssize_t)-1;
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);
	int                prescale  = 0;
	unsigned long      rate      = 0;

	if ( pca_chip ) {
		prescale = i2c_smbus_read_byte_data (client, PCA9685_REG_PRE_SCALE);
		rate = prescale2rate (pca_chip, prescale);

		retval = sprintf (buf, "%luHz\n", rate);
	}

	return retval;
}


static ssize_t pca968x_frequency_store (struct device *dev, struct device_attribute *attr,
									const char *buf, size_t n)
{
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	char               *start    = (char *)buf;
	unsigned long      value;

	while ( *start == ' ' )
		start++;

	value = simple_strtoul (start, NULL, 10);

	set_pwm_frequency (pca_chip, value);

	return n;
}


static ssize_t pca968x_max_frequency_show (struct device *dev, struct device_attribute *attr, char *buf) {
	ssize_t            retval    = (ssize_t)-1;
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	if ( pca_chip ) {
		retval = sprintf (buf, "%luHz\n", (unsigned long)pca_chip->pwm_max_freq);
	}

	return retval;
}


static ssize_t pca968x_min_frequency_show (struct device *dev, struct device_attribute *attr, char *buf) {
	ssize_t            retval    = (ssize_t)-1;
	struct i2c_client  *client   = to_i2c_client(dev);
	struct pca968x     *pca_chip = (struct pca968x *)i2c_get_clientdata (client);

	if ( pca_chip ) {
		retval = sprintf (buf, "%luHz\n", (unsigned long)pca_chip->pwm_min_freq);
	}

	return retval;
}


static DEVICE_ATTR(regs,          0664, pca968x_dump_reg, pca968x_write_reg);
static DEVICE_ATTR(enable,        0664, pca968x_enable_show, pca968x_enable_store);
static DEVICE_ATTR(frequency,     0664, pca968x_frequency_show, pca968x_frequency_store);
static DEVICE_ATTR(max_frequency, 0444, pca968x_max_frequency_show, NULL);
static DEVICE_ATTR(min_frequency, 0444, pca968x_min_frequency_show, NULL);


static struct attribute *pca968x_attrs[] = {
	&dev_attr_regs.attr,
	&dev_attr_enable.attr,
	&dev_attr_frequency.attr,
	&dev_attr_max_frequency.attr,
	&dev_attr_min_frequency.attr,
	NULL,
};


static struct attribute_group pca968x_attr_group = {
	.attrs = pca968x_attrs,
};
/* __________________________________________________________________________
* |__________________________________________________________________________|
*/


/*   __________________________________________________________________________
 *  |                                                                          |
 *  |                                INIT / PROBE                              |
 *  |__________________________________________________________________________|
 */
static void pca968x_poweroff_all (struct i2c_client *client, struct pca968x *pca_chip) {
	int led;
	pca_chip->enable = false;
	set_pca968x (pca_chip);
	for ( led = 0 ; led < pca_chip->led_num ; led++ ) {
		if ( pca_chip->leds[led].led_id != -1 ) {
			pca_chip->leds[led].enable = false;
			set_led (pca_chip, led);
		}
	}
}


static int pca968x_create_sled (struct i2c_client *client, struct pca968x *pca_chip) {
	int led, sled;

	sled = 0;
	for ( led = 0 ; led < pca_chip->led_num ; led++ ) {
		struct led_pwm_data *pd;

		if ( pca_chip->leds[led].led_id == -1 )
			continue;

		pd = devm_kzalloc (&client->dev, sizeof (pd), GFP_KERNEL);
		if (!pd) {
			// err
		}

		pd->enable       = pca_chip->leds[led].enable;
		pd->duty         = pca_chip->leds[led].duty;
		pd->delay        = pca_chip->leds[led].delay;
		pd->period       = rate2period ((unsigned long)pca_chip->pwm_freq);
		pd->max_duty     = pca_chip->pwm_max_duty;
		pd->max_delay    = pca_chip->pwm_max_delay;
		pd->lth_duty     = pca_chip->pwm_min_duty;
		pd->stby_en      = 0;	// not supported
		pd->period_max   = rate2period ((unsigned long)pca_chip->pwm_min_freq);
		pd->period_min   = rate2period ((unsigned long)pca_chip->pwm_max_freq);
		pd->id_internal  = pca_chip->leds[led].led_id;

		sled_device_register (pca_chip->leds[led].name,
								&client->dev, &led_pwm_sled_ops,
								LED_TYPE_PWM, pd, THIS_MODULE);

		sled++;
	}

	return pca_chip->used_leds - sled;
}


static int pca968x_chip_init (struct i2c_client *client, struct pca968x *pca_chip,
								struct pca968x_chipdef *chip) {
	int i;

	pca_chip->reg_list     = chip->reg_list;
	pca_chip->reg_num      = chip->reg_num;
	pca_chip->led_num      = chip->led_num;
	pca_chip->osc_internal = chip->osc_internal;
	pca_chip->osc          = chip->osc_internal;
	pca_chip->res_steps    = chip->res_steps;

	pca_chip->used_leds    = 0;
	pca_chip->leds = devm_kzalloc (&client->dev,
						sizeof (struct pca968x_led) * pca_chip->led_num, GFP_KERNEL);
	if ( !pca_chip->leds ) {
		return -ENOMEM;
	}

	for ( i = 0 ; i < pca_chip->led_num ; i++ ) {
		pca_chip->leds[i].led_id   = -1;
		pca_chip->leds[i].reg_base = LED_BASE_ADDR(i);
	}

	return 0;
}


static int pca968x_dt_init (struct i2c_client *client, struct pca968x *pca_chip) {

	struct device_node   *np = client->dev.of_node, *child;
	int                  count, gpio_en, res, err;
	u32                  reg, val;
	const char           *label;

	count = of_get_child_count (np);
	if ( !count || count == 0) {
		PCA968x_ERR ("wrong number of leds!!!");
		err = -EINVAL;
		goto err_data;
	}

	gpio_en = of_get_named_gpio (np, "gpio-enable", 0);
	if ( !gpio_is_valid (gpio_en) ) {
		err = -EINVAL;
		PCA968x_ERR ("missing gpio-enable!!!");
		goto err_data;
	}
	pca_chip->gpio_enable = gpio_en;

	res = of_property_read_u32 (np, "pwm-max-freq", &val);
	if ( res ) {
		err = -EINVAL;
		PCA968x_ERR ("missing max frequency parameter!!!");
		goto err_data;
	}
	pca_chip->pwm_max_freq = (int)val;

	res = of_property_read_u32 (np, "pwm-min-freq", &val);
	if ( res ) {
		err = -EINVAL;
		PCA968x_ERR ("missing min frequency parameter!!!");
		goto err_data;

	}
	pca_chip->pwm_min_freq = (int)val;

	res = of_property_read_u32 (np, "pwm-frequency", &val);
	if ( res ) {
		PCA968x_ERR ("missing default PWM frequency... set to middle!!!");
		pca_chip->pwm_freq = (pca_chip->pwm_max_freq + pca_chip->pwm_min_freq)/2;
	} else
		pca_chip->pwm_freq = (int)val;

	res = of_property_read_u32 (np, "pwm-max-duty", &val);
	if ( res ) {
		err = -EINVAL;
		PCA968x_ERR ("missing max duty parameter!!!");
		goto err_data;
	}
	pca_chip->pwm_max_duty = (int)val;

	res = of_property_read_u32 (np, "pwm-min-duty", &val);
	if ( res ) {
		err = -EINVAL;
		PCA968x_ERR ("missing min duty parameter!!!");
		goto err_data;
	}
	pca_chip->pwm_min_duty = (int)val;

	res = of_property_read_u32 (np, "pwm-max-delay", &val);
	if ( res ) {
		err = -EINVAL;
		PCA968x_ERR ("missing max delay parameter!!!");
		goto err_data;
	}
	pca_chip->pwm_max_delay = (int)val;

	res = of_property_read_u32 (np, "pwm-min-delay", &val);
	if ( res ) {
		err = -EINVAL;
		PCA968x_ERR ("missing min delay parameter!!!");
		goto err_data;
	}
	pca_chip->pwm_min_delay = (int)val;

	for_each_child_of_node (np, child) {
		label = (char *)kzalloc (sizeof (char) * LED_NAME_LEN, GFP_KERNEL);

		res = of_property_read_u32 (child, "reg", &reg);
		if ( (res != 0) || (reg >= pca_chip->led_num) )
			continue;
		pca_chip->leds[reg].led_id = reg;
		pca_chip->used_leds++;

		label = of_get_property (child, "label", NULL);
		if ( label ) {
			strcpy (pca_chip->leds[reg].name, label);
		} else
			strcpy (pca_chip->leds[reg].name, "pca9685x_led");

		res = of_property_read_u32 (child, "led-duty", &val);
		if ( res ) {
			PCA968x_ERR ("missing default PWM duty for led%d... set to middle!!!", reg);
			pca_chip->leds[reg].duty = (pca_chip->pwm_min_duty + pca_chip->pwm_max_duty) / 2;
		} else
			pca_chip->leds[reg].duty = (int)val;

		res = of_property_read_u32 (child, "led-delay", &val);
		if ( res ) {
			PCA968x_ERR ("missing default PWM delay for led%d... set to middle!!!", reg);
			pca_chip->leds[reg].delay = (pca_chip->pwm_min_delay + pca_chip->pwm_max_delay) / 2;
		} else
			pca_chip->leds[reg].delay = (int)val;

		/*  establish initial state  */
		pca_chip->leds[reg].enable = of_property_read_bool (child, "led,startup_enable");

	}

	/*  establish initial state  */
	pca_chip->enable     = of_property_read_bool (np, "startup_enable");
	pca_chip->sw_reset   = of_property_read_bool (np, "soft_reset");
	pca_chip->sleep_mode = of_property_read_bool (np, "sleep_mode");

	return 0;
err_data:
	return err;
}


static int pca968x_probe (struct i2c_client *client,
							const struct i2c_device_id *id)
{
	struct pca968x_chipdef  *chip;
	struct pca968x          *pca968x_chip;
	int                     ret, err, led;

	PCA968x_INFO("Starting probe...");

	chip = &pca968x_chipdefs[id->driver_data];
	pca968x_chip = devm_kzalloc (&client->dev, sizeof(*pca968x_chip), GFP_KERNEL);
	if ( !pca968x_chip ) {
		err = -ENOMEM;
		goto err_no_chip;
	}

	ret = pca968x_chip_init (client, pca968x_chip, chip);
	if ( ret ) {
		err = ret;
		goto err_chip_init;
	}

	ret = pca968x_dt_init (client, pca968x_chip);
	if ( ret ) {
		err = ret;
		goto err_dt_init;
	}

	/*  Require gpio to manage the OE# signal  */
	ret = devm_gpio_request_one (&client->dev, pca968x_chip->gpio_enable,
					   	GPIOF_OUT_INIT_HIGH, "PCA-OE#");
	if ( ret ) {
		err = ret;
		PCA968x_ERR ("impossible to require gpio-enable!!!");
		goto err_gpio;
	}

	i2c_set_clientdata (client, pca968x_chip);
	pca968x_chip->client = client;

    ret = sysfs_create_group (&client->dev.kobj, &pca968x_attr_group);
	if ( ret ) {
		err = ret;
		PCA968x_ERR ("impossible to create sys interface: %d", err);
		goto err_sysfs;
	}

	ret = pca968x_create_sled (client, pca968x_chip);
	if ( ret ) {
		err = ret;
		PCA968x_ERR ("impossible to create sled device: %d", err);
		goto err_sled_dev;
	}

	set_pca968x (pca968x_chip);
	for ( led = 0 ; led < pca968x_chip->led_num ; led++ ) {
		if ( pca968x_chip->leds[led].led_id != -1 ) {
			set_led (pca968x_chip, led);
		}
	}

	return 0;

err_sled_dev:
	sysfs_remove_group (&client->dev.kobj, &pca968x_attr_group);
err_sysfs:
	devm_gpio_free (&client->dev, pca968x_chip->gpio_enable);
err_gpio:
err_dt_init:
	kfree (pca968x_chip->leds);
err_chip_init:
	kfree (pca968x_chip);
err_no_chip:
	return err;
}


static int pca968x_remove (struct i2c_client *client) {
	struct pca968x *pca968x_chip = (struct pca968x *)i2c_get_clientdata (client);
	pca968x_poweroff_all (client, pca968x_chip);
	set_global_enable (pca968x_chip, 0);
	sysfs_remove_group (&client->dev.kobj, &pca968x_attr_group);
	devm_gpio_free (&client->dev, pca968x_chip->gpio_enable);
	kfree (pca968x_chip->leds);
	kfree (pca968x_chip);
	return 0;
}


static void pca968x_shutdown (struct i2c_client *client) {
	struct pca968x *pca968x_chip = (struct pca968x *)i2c_get_clientdata (client);
	set_all_led_off (pca968x_chip);
}


static const struct i2c_device_id pca968x_id[] = {
	{ "pca9685", pca9685 },
	{ /*  SENTINEL  */ },
};
MODULE_DEVICE_TABLE(i2c, pca963x_id);


static const struct of_device_id of_pca968x_match[] = {
	{ .compatible = "nxp,pca9685", },
	{  /*  SENTINEL  */ },
};


static struct i2c_driver pca968x_driver = {
	.driver   = {
		.name	        = "leds-pca968x",
		.owner	        = THIS_MODULE,
		.of_match_table = of_match_ptr (of_pca968x_match),
	},
	.probe	  = pca968x_probe,
	.remove   = pca968x_remove,
	.shutdown = pca968x_shutdown,
	.id_table = pca968x_id,
};

module_i2c_driver(pca968x_driver);

MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("PCA968X PWM LED driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL v2");
