#ifndef _IPA_TEST_MODULE_H_
#define _IPA_TEST_MODULE_H_

#include "linux/msm_ipa.h"
#include <linux/ioctl.h>

#define IPA_TEST_IOC_MAGIC 0xA5
#define IPA_TEST_IOCTL_GET_HW_TYPE 1
#define IPA_TEST_IOC_GET_HW_TYPE _IO(IPA_TEST_IOC_MAGIC, \
		IPA_TEST_IOCTL_GET_HW_TYPE)

#endif /* _IPA_TEST_MODULE_H_ */
