/*
 * An SPI driver for the Philips PCF2123 RTC
 * Copyright 2009 Cyber Switching, Inc.
 *
 * Author: Chris Verges <chrisv@cyberswitching.com>
 * Maintainers: http://www.cyberswitching.com
 *
 * based on the RS5C348 driver in this same directory.
 *
 * Thanks to Christian Pellegrin <chripell@fsfe.org> for
 * the sysfs contributions to this driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Please note that the CS is active high, so platform data
 * should look something like:
 *
 * static struct spi_board_info ek_spi_devices[] = {
 *	...
 *	{
 *		.modalias		= "rtc-pcf2123",
 *		.chip_select		= 1,
 *		.controller_data	= (void *)AT91_PIN_PA10,
 *		.max_speed_hz		= 1000 * 1000,
 *		.mode			= SPI_CS_HIGH,
 *		.bus_num		= 0,
 *	},
 *	...
 *};
 *
 */

#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include <linux/pcf2123_io.h>

/* REGISTERS */
#define PCF2123_REG_CTRL1	(0x00)	/* Control Register 1 */
#define PCF2123_REG_CTRL2	(0x01)	/* Control Register 2 */
/* -------- TIME -------- */
#define PCF2123_REG_SC		(0x02)	/* datetime */
#define PCF2123_REG_MN		(0x03)
#define PCF2123_REG_HR		(0x04)
#define PCF2123_REG_DM		(0x05)
#define PCF2123_REG_DW		(0x06)
#define PCF2123_REG_MO		(0x07)
#define PCF2123_REG_YR		(0x08)
/* -------- ALARM -------- */
#define PCF2123_REG_ALRM_MN	(0x09)	/* Alarm Registers */
#define PCF2123_REG_ALRM_HR	(0x0a)
#define PCF2123_REG_ALRM_DM	(0x0b)
#define PCF2123_REG_ALRM_DW	(0x0c)
#define PCF2123_REG_OFFSET	(0x0d)	/* Clock Rate Offset Register */
#define PCF2123_REG_TMR_CLKOUT	(0x0e)	/* Timer Registers */
#define PCF2123_REG_CTDWN_TMR	(0x0f)

/* PCF2123_REG_CTRL1 BITS */
#define CTRL1_CLEAR		(0)	/* Clear */
#define CTRL1_CORR_INT		BIT(1)	/* Correction irq enable */
#define CTRL1_12_HOUR		BIT(2)	/* 12 hour time */
#define CTRL1_SW_RESET	(BIT(3) | BIT(4) | BIT(6))	/* Software reset */
#define CTRL1_STOP		BIT(5)	/* Stop the clock */
#define CTRL1_EXT_TEST		BIT(7)	/* External clock test mode */

/* PCF2123_REG_CTRL2 BITS */
#define CTRL2_TIE		BIT(0)	/* Countdown timer irq enable */
#define CTRL2_AIE		BIT(1)	/* Alarm irq enable */
#define CTRL2_TF		BIT(2)	/* Countdown timer flag */
#define CTRL2_AF		BIT(3)	/* Alarm flag */
#define CTRL2_TI_TP		BIT(4)	/* Irq pin generates pulse */
#define CTRL2_MSF		BIT(5)	/* Minute or second irq flag */
#define CTRL2_SI		BIT(6)	/* Second irq enable */
#define CTRL2_MI		BIT(7)	/* Minute irq enable */

/* PCF2123_REG_SC BITS */
#define OSC_HAS_STOPPED		BIT(7)	/* Clock has been stopped */

/* PCF2123_REG_ALRM_XX BITS */
#define ALRM_ENABLE		BIT(7)	/* MN, HR, DM, or DW alarm enable */

/* PCF2123_REG_TMR_CLKOUT BITS */
#define CD_TMR_4096KHZ		(0)	/* 4096 KHz countdown timer */
#define CD_TMR_64HZ		(1)	/* 64 Hz countdown timer */
#define CD_TMR_1HZ		(2)	/* 1 Hz countdown timer */
#define CD_TMR_60th_HZ		(3)	/* 60th Hz countdown timer */
#define CD_TMR_TE		BIT(3)	/* Countdown timer enable */

/* PCF2123_REG_OFFSET BITS */
#define OFFSET_SIGN_BIT		6	/* 2's complement sign bit */
#define OFFSET_COARSE		BIT(7)	/* Coarse mode offset */
#define OFFSET_STEP		(2170)	/* Offset step in parts per billion */

#define PCF2123_MASK_ENABLE     (1 << 7)
#define PCF2123_MASK_ALM_EN_MN  (1 << 0)
#define PCF2123_MASK_ALM_EN_HR  (1 << 1)
#define PCF2123_MASK_ALM_EN_DM  (1 << 2)
#define PCF2123_MASK_ALM_EN_DW  (1 << 3)

/* READ/WRITE ADDRESS BITS */
#define PCF2123_WRITE		BIT(4)
#define PCF2123_READ		(BIT(4) | BIT(7))

#define	SECO_OFFSET			 1
#define PCF2123_SECO_OFFSET      (0x0B) /*  Seco RTC Compensation Offset -2.057 s/day */


static struct spi_driver pcf2123_driver;

struct pcf2123_sysfs_reg {
	struct device_attribute attr;
	char name[2];
};

struct pcf2123_plat_data {
	struct rtc_device *rtc;
	struct pcf2123_sysfs_reg regs[16];
};

/* --------------------------------------------------------------------------
                                 LOGGING FUNCTIONS
   -------------------------------------------------------------------------- */

#define PCF_LOGGING 0

#if PCF_LOGGING

#define LOG_OP_WRITE   'w'
#define LOG_OP_READ    'r'

struct logger_entry {
	unsigned int      time;
	char              op;
	struct rtc_time   tm;
	struct list_head  list;
};

static struct mutex         logger_lock;
static struct logger_entry  *logger_list;

#define log_list_add(item, head, uop, tm_date)       item = kzalloc (sizeof (struct logger_entry), GFP_KERNEL); \
							item->tm.tm_sec  = tm_date->tm_sec; \
							item->tm.tm_min  = tm_date->tm_min; \
							item->tm.tm_hour = tm_date->tm_hour; \
							item->tm.tm_mday = tm_date->tm_mday; \
							item->tm.tm_wday = tm_date->tm_wday; \
							item->tm.tm_mon  = tm_date->tm_mon; \
							item->tm.tm_year = tm_date->tm_year; \
							item->op = uop; \
							item->time = jiffies_to_msecs (jiffies); \
							list_add_tail (&item->list, &head->list)


static inline void logger_insert (char op, struct rtc_time *tm) {
	struct logger_entry *tmp;
	if ( !logger_list )
		return;
	mutex_lock (&logger_lock);
	log_list_add (tmp, logger_list, op, tm);
	mutex_unlock (&logger_lock);
}


static inline void logger_del_all (void) {
	struct list_head *pos, *q;
	struct logger_entry *tmp;

	if ( !logger_list )
		return;

	mutex_lock (&logger_lock);
	list_for_each_safe (pos, q, &logger_list->list) {
		tmp = list_entry (pos, struct logger_entry, list);
		list_del (pos);
		kfree (tmp);
	}
	mutex_unlock (&logger_lock);
}

#endif /* PCF_LOGGING */


/*
 * Causes a 30 nanosecond delay to ensure that the PCF2123 chip select
 * is released properly after an SPI write.  This function should be
 * called after EVERY read/write call over SPI.
 */
static inline void pcf2123_delay_trec(void)
{
	ndelay(30);
}

static int pcf2123_read(struct device *dev, u8 reg, u8 *rxbuf, size_t size)
{
	struct spi_device *spi = to_spi_device(dev);
	int ret;

	reg |= PCF2123_READ;
	ret = spi_write_then_read(spi, &reg, 1, rxbuf, size);
	pcf2123_delay_trec();

	return ret;
}

static int pcf2123_write(struct device *dev, u8 *txbuf, size_t size)
{
	struct spi_device *spi = to_spi_device(dev);
	int ret;

	txbuf[0] |= PCF2123_WRITE;
	ret = spi_write(spi, txbuf, size);
	pcf2123_delay_trec();

	return ret;
}

static int pcf2123_write_reg(struct device *dev, u8 reg, u8 val)
{
	u8 txbuf[2];

	txbuf[0] = reg;
	txbuf[1] = val;
	return pcf2123_write(dev, txbuf, sizeof(txbuf));
}

static ssize_t pcf2123_show(struct device *dev, struct device_attribute *attr,
			    char *buffer)
{
	struct pcf2123_sysfs_reg *r;
	u8 rxbuf[1];
	unsigned long reg;
	int ret;

	r = container_of(attr, struct pcf2123_sysfs_reg, attr);

	ret = kstrtoul(r->name, 16, &reg);
	if (ret)
		return ret;

	ret = pcf2123_read(dev, reg, rxbuf, 1);
	if (ret < 0)
		return -EIO;

	return sprintf(buffer, "0x%x\n", rxbuf[0]);
}

static ssize_t pcf2123_store(struct device *dev, struct device_attribute *attr,
			     const char *buffer, size_t count)
{
	struct pcf2123_sysfs_reg *r;
	unsigned long reg;
	unsigned long val;

	int ret;

	r = container_of(attr, struct pcf2123_sysfs_reg, attr);

	ret = kstrtoul(r->name, 16, &reg);
	if (ret)
		return ret;

	ret = kstrtoul(buffer, 10, &val);
	if (ret)
		return ret;

	ret = pcf2123_write_reg(dev, reg, val);
	if (ret < 0)
		return -EIO;
	return count;
}

static int pcf2123_read_offset(struct device *dev, long *offset)
{
	int ret;
	s8 reg;

	ret = pcf2123_read(dev, PCF2123_REG_OFFSET, &reg, 1);
	if (ret < 0)
		return ret;

	if (reg & OFFSET_COARSE)
		reg <<= 1; /* multiply by 2 and sign extend */
	else
		reg = sign_extend32(reg, OFFSET_SIGN_BIT);

	*offset = ((long)reg) * OFFSET_STEP;

	return 0;
}



/*
 * The offset register is a 7 bit signed value with a coarse bit in bit 7.
 * The main difference between the two is normal offset adjusts the first
 * second of n minutes every other hour, with 61, 62 and 63 being shoved
 * into the 60th minute.
 * The coarse adjustment does the same, but every hour.
 * the two overlap, with every even normal offset value corresponding
 * to a coarse offset. Based on this algorithm, it seems that despite the
 * name, coarse offset is a better fit for overlapping values.
 */
static int pcf2123_set_offset(struct device *dev, long offset)
{
	s8 reg;

	if (offset > OFFSET_STEP * 127)
		reg = 127;
	else if (offset < OFFSET_STEP * -128)
		reg = -128;
	else
		reg = (s8)((offset + (OFFSET_STEP >> 1)) / OFFSET_STEP);

	/* choose fine offset only for odd values in the normal range */
	if (reg & 1 && reg <= 63 && reg >= -64) {
		/* Normal offset. Clear the coarse bit */
		reg &= ~OFFSET_COARSE;
	} else {
		/* Coarse offset. Divide by 2 and set the coarse bit */
		reg >>= 1;
		reg |= OFFSET_COARSE;
	}

	return pcf2123_write_reg(dev, PCF2123_REG_OFFSET, reg);
}

static int pcf2123_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u8 rxbuf[7];
	int ret;

	ret = pcf2123_read(dev, PCF2123_REG_SC, rxbuf, sizeof(rxbuf));
	if (ret < 0)
		return ret;

	if (rxbuf[0] & OSC_HAS_STOPPED) {
		dev_info(dev, "clock was stopped. Time is not valid\n");
		return -EINVAL;
	}

	tm->tm_sec = bcd2bin(rxbuf[0] & 0x7F);
	tm->tm_min = bcd2bin(rxbuf[1] & 0x7F);
	tm->tm_hour = bcd2bin(rxbuf[2] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(rxbuf[3] & 0x3F);
	tm->tm_wday = rxbuf[4] & 0x07;
	tm->tm_mon = bcd2bin(rxbuf[5] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(rxbuf[6]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */

	dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
			"mday=%d, mon=%d, year=%d, wday=%d\n",
			__func__,
			tm->tm_sec, tm->tm_min, tm->tm_hour,
			tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	return rtc_valid_tm(tm);
}

static int pcf2123_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	u8 txbuf[8];
	int ret;

	dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
			"mday=%d, mon=%d, year=%d, wday=%d\n",
			__func__,
			tm->tm_sec, tm->tm_min, tm->tm_hour,
			tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* Stop the counter first */
	ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL1, CTRL1_STOP);
	if (ret < 0)
		return ret;

	/* Set the new time */
	txbuf[0] = PCF2123_REG_SC;
	txbuf[1] = bin2bcd(tm->tm_sec & 0x7F);
	txbuf[2] = bin2bcd(tm->tm_min & 0x7F);
	txbuf[3] = bin2bcd(tm->tm_hour & 0x3F);
	txbuf[4] = bin2bcd(tm->tm_mday & 0x3F);
	txbuf[5] = tm->tm_wday & 0x07;
	txbuf[6] = bin2bcd((tm->tm_mon + 1) & 0x1F); /* rtc mn 1-12 */
	txbuf[7] = bin2bcd(tm->tm_year < 100 ? tm->tm_year : tm->tm_year - 100);

	ret = pcf2123_write(dev, txbuf, sizeof(txbuf));
	if (ret < 0)
		return ret;

	/* Start the counter */
	ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL1, CTRL1_CLEAR);
	if (ret < 0)
		return ret;

	return 0;
}


#define GET_ALM_EN(reg)                   (!!((reg) & PCF2123_MASK_ENABLE))
#define SET_ALM_EN(reg, en)               ((reg) = (en) ? (reg) | PCF2123_MASK_ENABLE : \
		(reg) & ~PCF2123_MASK_ENABLE) 

#define GET_REG_ALM_EN(reg, mask)        (!!((reg) & (mask)))
#define SET_REG_ALM_EN(reg, mask, en)    ((reg) = (en) ? (reg) | (mask) : \
		(reg) & ~(mask))

int pcf2123_rtc_read_reg_alrm (struct device *dev) {
	int ret;
	s8 rxbuf[4];

	ret = pcf2123_read(dev, PCF2123_REG_ALRM_MN, &rxbuf[0], sizeof(rxbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	printk (KERN_ERR "dump alarm reg:\n %02x  %02x  %02x  %02x\n",
			rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);

	ret = pcf2123_read(dev, PCF2123_REG_CTRL2, &rxbuf[0], 1);
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	printk (KERN_ERR "dump ctrl2 reg:\n %02x\n",
			rxbuf[0]);

	return 0;
}

int pcf2123_rtc_clear_int_alrm (struct device *dev) {
	int ret;
	s8 rxbuf_en[1];

	ret = pcf2123_read(dev, PCF2123_REG_CTRL2, &rxbuf_en[0], 1);
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL2, rxbuf_en[0] & ~((u8)1 << 3));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	//pcf2123_rtc_read_reg_alrm (dev);
	return 0;
}


int pcf2123_rtc_read_alrm (struct device *dev, struct rtc_wkalrm *alrm) {
	s8 rxbuf[4];
	int ret;

	ret = pcf2123_read(dev, PCF2123_REG_ALRM_MN, &rxbuf[0], sizeof(rxbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	ret = pcf2123_rtc_read_time (dev, &alrm->time);

	alrm->time.tm_min  = bcd2bin(rxbuf[0] & 0x7F);
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_MN, GET_ALM_EN(rxbuf[0]));
	alrm->time.tm_hour = bcd2bin(rxbuf[1] & 0x7F);
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_HR, GET_ALM_EN(rxbuf[1]));
	alrm->time.tm_mday = bcd2bin(rxbuf[2] & 0x3F);
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_DM, GET_ALM_EN(rxbuf[2]));
	alrm->time.tm_wday = rxbuf[3] & 0x07;
	SET_REG_ALM_EN (alrm->enabled, PCF2123_MASK_ALM_EN_DW, GET_ALM_EN(rxbuf[3]));

	dev_dbg(dev, "%s: alrm is mins=%d, hours=%d, mday=%d, wday=%d\n",
			__func__,
			alrm->time.tm_min, alrm->time.tm_hour,
			alrm->time.tm_mday, alrm->time.tm_wday);

	if (rtc_valid_alrm (alrm) < 0)
		dev_err(dev, "retrieved alrm is not valid.\n");

	//	pcf2123_rtc_read_reg_alrm (dev);

	return 0;
}


int pcf2123_rtc_set_alrm (struct device *dev, struct rtc_wkalrm *alrm) {
	struct spi_device *spi = to_spi_device(dev);
	s8 txbuf[5];
	int ret, en_int;
	s8 rxbuf_en[1];

	/* Unset the AIE flag, temporarily */

	ret = pcf2123_read(dev, PCF2123_REG_CTRL2, &rxbuf_en[0], 1);
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	
	ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL2, rxbuf_en[0] & ~((u8)1 << 1));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();

	dev_dbg(dev, "%s: alrm is mins=%d, hours=%d, mday=%d, wday=%d\n",
			__func__,
			alrm->time.tm_min, alrm->time.tm_hour,
			alrm->time.tm_mday, alrm->time.tm_wday);

	/* Set the alrm */
	txbuf[0] = PCF2123_WRITE | PCF2123_REG_ALRM_MN;
	txbuf[1] = bin2bcd(alrm->time.tm_min & 0x7F);
	SET_ALM_EN (txbuf[1], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_MN));
	txbuf[2] = bin2bcd(alrm->time.tm_hour & 0x3F);
	SET_ALM_EN (txbuf[2], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_HR));
	txbuf[3] = bin2bcd(alrm->time.tm_mday & 0x3F);
	SET_ALM_EN (txbuf[3], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_DM));
	txbuf[4] = bin2bcd(alrm->time.tm_wday & 0x07);
	SET_ALM_EN (txbuf[4], GET_REG_ALM_EN(alrm->enabled, PCF2123_MASK_ALM_EN_DW));

	ret = spi_write(spi, txbuf, sizeof(txbuf));
	if (ret < 0)
		return ret;
	pcf2123_delay_trec();


	/* Set the AIE flag, if needed (enable == 1) */

	en_int = (u8)alrm->enabled == 0x0F? 0 : 1;

	if (en_int) {
		ret = pcf2123_read(dev, PCF2123_REG_CTRL2, &rxbuf_en[0], 1);
		if (ret < 0)
			return ret;
		pcf2123_delay_trec();

		ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL2, rxbuf_en[0] & ~((u8)1 << 1));
		if (ret < 0)
			return ret;
		pcf2123_delay_trec();
	}


	//	pcf2123_rtc_read_reg_alrm (dev);
	return 0;
}





static int pcf2123_reset(struct device *dev)
{
	int ret;
	u8  rxbuf[2];

	ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL1, CTRL1_SW_RESET);
	if (ret < 0)
		return ret;

	/* Stop the counter */
	dev_dbg(dev, "stopping RTC\n");
	ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL1, CTRL1_STOP);
	if (ret < 0)
		return ret;

	/* See if the counter was actually stopped */
	dev_dbg(dev, "checking for presence of RTC\n");
	ret = pcf2123_read(dev, PCF2123_REG_CTRL1, rxbuf, sizeof(rxbuf));
	if (ret < 0)
		return ret;

	dev_dbg(dev, "received data from RTC (0x%02X 0x%02X)\n",
		rxbuf[0], rxbuf[1]);
	if (!(rxbuf[0] & CTRL1_STOP))
		return -ENODEV;

	/* Start the counter */
	ret = pcf2123_write_reg(dev, PCF2123_REG_CTRL1, CTRL1_CLEAR);
	if (ret < 0)
		return ret;

	return 0;
}


/*  ___________________________________________________________________________
 * |                                                                           |
 * |                                SYSFS INTERFACE                            |
 * |___________________________________________________________________________|
 */

#if PCF_LOGGING
static ssize_t pcf2123_rtc_sysfs_log_show (struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct logger_entry *tmp;
	struct list_head *pos, *q;
	int tot = 0;
	unsigned char str[200];
	unsigned char *msg;
	ssize_t retval;
	struct rtc_time tm;


	mutex_lock (&logger_lock);
	list_for_each_safe (pos, q, &logger_list->list) {
		tot++;
	}

	msg = kzalloc (sizeof (str) * (tot+1), GFP_KERNEL);
	if ( !msg ) {
		mutex_unlock (&logger_lock);
		return -ENOMEM;
	}

	list_for_each_safe (pos, q, &logger_list->list) {
		tmp = list_entry (pos, struct logger_entry, list);
		tm.tm_sec = tmp->tm.tm_sec;
		tm.tm_min = tmp->tm.tm_min;
		tm.tm_hour = tmp->tm.tm_hour;
		tm.tm_mday = tmp->tm.tm_mday;
		tm.tm_wday = tmp->tm.tm_wday;
		tm.tm_mon = tmp->tm.tm_mon;
		tm.tm_year = tmp->tm.tm_year;
		sprintf (str, "\n[%10u] op: %c,  tm is secs=%d, mins=%d, hours=%d, "
			"mday=%d, mon=%d, year=%d, wday=%d\n",
			tmp->time, tmp->op,	tm.tm_sec, tm.tm_min, tm.tm_hour,
			tm.tm_mday, tm.tm_mon, tm.tm_year, tm.tm_wday);

		strcat (msg, str);
	}
	mutex_unlock (&logger_lock);

	sprintf (str, "\n total operation: %d", tot);
	strcat (msg, str);

	retval = sprintf (buf, "%s\n", msg);
	return retval;
}

static ssize_t pcf2123_rtc_sysfs_log_set (struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	long operation;
	int  err_conv;

	err_conv = kstrtoul(buf, 10, &operation);

	if (err_conv == 0) {
		if ((u16)operation < 0)
			return -EINVAL;

		switch (operation) {
			case 0: // clear list
				logger_del_all ();
				break;
			default:
				break;
		}
	} else
		return err_conv;

	return count;
}

static DEVICE_ATTR(logging, S_IRUGO | S_IWUSR,
		pcf2123_rtc_sysfs_log_show, pcf2123_rtc_sysfs_log_set);

#endif /* PCF_LOGGING */


static ssize_t pcf2123_rtc_sysfs_show_alarm(struct device *dev, struct device_attribute *attr,
		char *buf) {

	ssize_t retval;
	struct rtc_wkalrm alm;

	retval = pcf2123_rtc_read_alrm (dev, &alm);
	retval = sprintf (buf, "%02d:%02d:%02d:%01d\n",
			alm.time.tm_min, alm.time.tm_hour, alm.time.tm_mday, alm.time.tm_wday);

	return retval;
}



static ssize_t pcf2123_rtc_sysfs_set_alarm(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t retval;
	struct rtc_wkalrm alm;
	char *buf_ptr;
	int idx;
	char *elm;
	long min = 0;
	long hr = 0;
	long dw = 0;
    long dm = 0;

	/*  mm:hh:DD:d
	 * mm = minutes
	 * hh = hours
	 * DD = day of month
	 * d  = day of week
	 */

	buf_ptr = (char *)buf;

	/*  check if the string has the correct form */
#define STRING_LEN 11
	if (strlen (buf_ptr) != STRING_LEN)
		return -EINVAL;

	idx = 0;
	elm = kzalloc (sizeof (char) * 2, GFP_KERNEL);
	while (idx != STRING_LEN) {
		switch (idx) {
			case 0:
				elm[0] = *buf_ptr;
				elm[1] = *(++buf_ptr);
				min = simple_strtol (elm, NULL, 10);
				if ((min < 0) || (min > 59))
					return -EINVAL;
				buf_ptr++;
				break;
			case 3:
				elm[0] = *buf_ptr;
				elm[1] = *(++buf_ptr);
				hr = simple_strtol (elm, NULL, 10);
				if ((hr < 0) || (hr > 23))
					return -EINVAL;
				buf_ptr++;
				break;
			case 6:
				elm[0] = *buf_ptr;
				elm[1] = *(++buf_ptr);
				dm = simple_strtol (elm, NULL, 10);
				if ((dm < 0) || (dm > 31))
					return -EINVAL;
				buf_ptr++;
				break;
			case 9:
				elm[0] = '0';
				elm[1] = *buf_ptr;
				dw = simple_strtol (elm, NULL, 10);
				if ((dw < 0) || (dw > 6))
					return -EINVAL;
				break;
			case 2:
			case 5:
			case 8:
				if (*buf_ptr != ':')
					return -EINVAL;
				buf_ptr++;
				break;
			default:
				break;
		}
		idx++;
	}

	retval = pcf2123_rtc_read_alrm (dev, &alm);
	if (retval)
		return -EIO;

	alm.time.tm_min  = (int)min;
	alm.time.tm_hour = (int)hr;
	alm.time.tm_mday = (int)dm;
	alm.time.tm_wday = (int)dw;

	retval = pcf2123_rtc_set_alrm (dev, &alm);
	return count;
}


static ssize_t pcf2123_rtc_sysfs_show_alarm_en (struct device *dev, struct device_attribute *attr, 
		char *buf)
{
	ssize_t retval;
	struct rtc_wkalrm alm;

	retval = pcf2123_rtc_read_alrm (dev, &alm);
	retval = sprintf (buf, "min:hr:DD:d\n-----------\n  %d: %d: %d:%d\n", 
			(~alm.enabled & 0x1) >> 0,
			(~alm.enabled & 0x2) >> 1,
			(~alm.enabled & 0x4) >> 2,
			(~alm.enabled & 0x8) >> 3);

	return retval;

}


static ssize_t pcf2123_rtc_sysfs_set_alarm_en (struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t retval;
	struct rtc_wkalrm alm;
	char *buf_ptr;
	int idx;
	char *elm;
	long flag;
	u8 enable = 0x00;

	/* -------------------------
	 * |mm|hh|DD| d| 0| 0| 0| 0|
	 * -------------------------
	 * mm = minutes
	 * hh = hours
	 * DD = day of month
	 * d  = day of week
	 *
	 * each flag can assume only 0/1 value.
	 */

	buf_ptr = (char *)buf;

	/* check if the string has the correct form */
#define STRING_LEN_EN 8
#define POS_EN_MIN    0
#define POS_EN_HR     2
#define POS_EN_DM     4
#define POS_EN_DW     6
	if (strlen (buf_ptr) != STRING_LEN_EN)
		return -EINVAL;

	idx = 0;
	elm = kzalloc (sizeof (char) * 2, GFP_KERNEL);
	elm[0] = '0';
	while (idx != STRING_LEN_EN) {
		switch (idx) {
			case POS_EN_MIN:
			case POS_EN_HR:
			case POS_EN_DM:
			case POS_EN_DW:
				elm[1] = *buf_ptr;
				flag = simple_strtol (elm, NULL, 10);
				if (flag != 0 && flag != 1)
					return -EINVAL;
				enable = !flag ? enable | (1 << (idx >> 1)) : enable & ~(1 << (idx >> 1));
				buf_ptr++;
				break;
			case 1:
			case 3:
			case 5:
				if (*buf_ptr != ':')
					return -EINVAL;
				buf_ptr++;
				break;

			default:
				break;
		}
		idx++;
	}

	retval = pcf2123_rtc_read_alrm (dev, &alm);
	if (retval)
		return -EIO;

	alm.enabled  = (int)enable;

	retval = pcf2123_rtc_set_alrm (dev, &alm);
	return count;
}


static DEVICE_ATTR(alarm, S_IRUGO | S_IWUSR,
		pcf2123_rtc_sysfs_show_alarm, pcf2123_rtc_sysfs_set_alarm);
static DEVICE_ATTR(enable_alarm, S_IRUGO | S_IWUSR,
		pcf2123_rtc_sysfs_show_alarm_en, pcf2123_rtc_sysfs_set_alarm_en);


/*  ___________________________________________________________________________
 * |___________________________________________________________________________|
 */



/*  ___________________________________________________________________________
 * |                                                                           |
 * |                                    IOCTL                                  |
 * |___________________________________________________________________________|
 */

static int pcf2123_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg) {
	int err = 0;
	int retval = 0;
	struct alrm_pcf alarm;
	struct rtc_wkalrm alrm;

	switch (cmd) {

		case PCF2123_RTC_IOCTL_ALM_READ:
			if (copy_from_user (&alarm, (const void __user *)arg, sizeof (alarm))) {
				return -EFAULT;
			}
			err = pcf2123_rtc_read_alrm (dev, &alrm);
			alarm.min    = alrm.time.tm_min;
			alarm.hr     = alrm.time.tm_hour;
			alarm.mday   = alrm.time.tm_mday;
			alarm.wday   = alrm.time.tm_wday;
			alarm.enable = ~alrm.enabled;

			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &alarm, sizeof (alarm))) {
				retval = -EFAULT;
			}
			break;

		case PCF2123_RTC_IOCTL_ALM_WRITE:
			if (copy_from_user (&alarm, (const void __user *)arg, sizeof (alarm))) {
				return -EFAULT;
			}
			alrm.time.tm_min    = alarm.min;
			alrm.time.tm_hour   = alarm.hr;
			alrm.time.tm_mday   = alarm.mday;
			alrm.time.tm_wday   = alarm.wday;
//			alrm.enabled        = 0x00;
			alrm.enabled        = (u8)~alarm.enable;
//			for (i = 0 ; i < 4 ; i++)
//				alrm.enabled |= ~((u8)alarm.enable & ~(1 << i)) & (1 << i);

			err = pcf2123_rtc_set_alrm (dev, &alrm);
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &alarm, sizeof (alarm))) {
				retval = -EFAULT;
			}

			break;

		default:
			break;

	}
	return retval;
}
/*  ___________________________________________________________________________
 * |___________________________________________________________________________|
 */


static const struct rtc_class_ops pcf2123_rtc_ops = {
	.read_time	= pcf2123_rtc_read_time,
	.set_time	= pcf2123_rtc_set_time,
	.read_offset	= pcf2123_read_offset,
	.set_offset	= pcf2123_set_offset,
	.ioctl          = pcf2123_rtc_ioctl,
};

static int pcf2123_probe(struct spi_device *spi)
{
	struct rtc_device *rtc;
	struct rtc_time tm;
	struct pcf2123_plat_data *pdata;
	int ret, i;

	pdata = devm_kzalloc(&spi->dev, sizeof(struct pcf2123_plat_data),
				GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	spi->dev.platform_data = pdata;

#if PCF_LOGGING
	/*  LOGGER INIT */
	logger_list = kzalloc (sizeof (struct logger_entry), GFP_KERNEL);
	if ( logger_list != NULL ) {
		mutex_init (&logger_lock);
		INIT_LIST_HEAD (&logger_list->list);
	}
	/*  -----  */
#endif /* PCF_LOGGING */

	ret = pcf2123_rtc_read_time(&spi->dev, &tm);
	if (ret < 0) {
		ret = pcf2123_reset(&spi->dev);
		if (ret < 0) {
			dev_err(&spi->dev, "chip not found\n");
			goto kfree_exit;
		}
	}

	/* Set Seco RTC compensation */
#if SECO_OFFSET
	pcf2123_set_offset(&spi->dev, (long)PCF2123_SECO_OFFSET);
	pcf2123_delay_trec();
#endif

	dev_info(&spi->dev, "spiclk %u KHz.\n",
			(spi->max_speed_hz + 500) / 1000);

	/* Finalize the initialization */
	rtc = devm_rtc_device_register(&spi->dev, pcf2123_driver.driver.name,
			&pcf2123_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&spi->dev, "failed to register.\n");
		ret = PTR_ERR(rtc);
		goto kfree_exit;
	}

	pdata->rtc = rtc;

	pcf2123_rtc_clear_int_alrm (&spi->dev);

	for (i = 0; i < 16; i++) {
		sysfs_attr_init(&pdata->regs[i].attr.attr);
		sprintf(pdata->regs[i].name, "%1x", i);
		pdata->regs[i].attr.attr.mode = S_IRUGO | S_IWUSR;
		pdata->regs[i].attr.attr.name = pdata->regs[i].name;
		pdata->regs[i].attr.show = pcf2123_show;
		pdata->regs[i].attr.store = pcf2123_store;
		ret = device_create_file(&spi->dev, &pdata->regs[i].attr);
		if (ret) {
			dev_err(&spi->dev, "Unable to create sysfs %s\n",
				pdata->regs[i].name);
			goto sysfs_exit;
		}
	}

	ret = device_create_file (&spi->dev, &dev_attr_alarm);
	if (ret)
		dev_err(&spi->dev, "failed to create alarm attribute, %d\n", ret);
	ret = device_create_file (&spi->dev, &dev_attr_enable_alarm);
	if (ret)
		dev_err(&spi->dev, "failed to create alarm attribute, %d\n", ret);


#if PCF_LOGGING
ret = device_create_file (&spi->dev, &dev_attr_logging);
	if (ret)
		dev_err(&spi->dev, "failed to create logging attribute, %d\n", ret);
#endif  /*  PCF_LOGGING  */

	return 0;

sysfs_exit:
	for (i--; i >= 0; i--)
		device_remove_file(&spi->dev, &pdata->regs[i].attr);

kfree_exit:
	spi->dev.platform_data = NULL;
	return ret;
}

static int pcf2123_remove(struct spi_device *spi)
{
	struct pcf2123_plat_data *pdata = dev_get_platdata(&spi->dev);
	int i;

	if (pdata) {
		for (i = 0; i < 16; i++)
			if (pdata->regs[i].name[0])
				device_remove_file(&spi->dev,
						   &pdata->regs[i].attr);
	}

	return 0;
}

static void pcf2123_shutdown(struct spi_device *spi) {
	s8 rxbuf[1];
	int ret;

	ret = pcf2123_read(&spi->dev, PCF2123_READ, &rxbuf[0], 1);
	if ( ret < 0 )
		return;
		
	ret = pcf2123_write_reg(&spi->dev, PCF2123_REG_CTRL2, rxbuf[0] & 0xF7);
}

#ifdef CONFIG_OF
static const struct of_device_id pcf2123_dt_ids[] = {
	{ .compatible = "nxp,rtc-pcf2123", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pcf2123_dt_ids);
#endif

static struct spi_driver pcf2123_driver = {
	.driver	= {
			.name	= "rtc-pcf2123",
			.of_match_table = of_match_ptr(pcf2123_dt_ids),
	},
	.probe	= pcf2123_probe,
	.remove	= pcf2123_remove,
	.shutdown = pcf2123_shutdown,
};

module_spi_driver(pcf2123_driver);

MODULE_AUTHOR("Chris Verges <chrisv@cyberswitching.com>");
MODULE_DESCRIPTION("NXP PCF2123 RTC driver");
MODULE_LICENSE("GPL");
