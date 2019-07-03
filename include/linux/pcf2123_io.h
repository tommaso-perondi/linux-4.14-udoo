
#ifndef __PCF2123_IO_RTC_H
#define __PCF2123_IO_RTC_H


#include <linux/ioctl.h>

struct alrm_pcf {
	int min;
	int hr;
	int wday;
	int mday;
	int enable;
};

#define PCF2123_RTC_IOCTL_ALM_READ	_IOR('o', 1, struct alrm_pcf)
#define PCF2123_RTC_IOCTL_ALM_WRITE	_IOWR('o', 2, struct alrm_pcf)


#endif    /*  __PCF2123_IO_RTC_H  */

