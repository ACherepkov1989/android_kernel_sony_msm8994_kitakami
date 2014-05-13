#ifndef __MSM_IOMMU_TEST__
#define __MSM_IOMMU_TEST__

#include <linux/ioctl.h>

#define MSM_IOMMU_MAGIC     'I'

#define NAME_LEN 30

#define TEST_FLAG_BASIC 1

struct get_next_cb {
	unsigned int iommu_no;
	unsigned int cb_no;
	char iommu_name[NAME_LEN];
	char cb_name[NAME_LEN];
	unsigned int iommu_secure;
	unsigned int cb_secure;
	unsigned int valid_iommu;
	unsigned int valid_cb;
	unsigned int lpae_enabled;
};

#define MAX_BFB_REGS 30

struct test_iommu {
	unsigned int iommu_no;
	unsigned int cb_no;
	unsigned int bfb_regs[MAX_BFB_REGS];
	unsigned int bfb_data[MAX_BFB_REGS];
	unsigned int bfb_size;
	int ret_code;
	unsigned int flags;
	int cats_tbu_id;
	unsigned int is_mm_tbu;
};

struct target_struct {
	char name[NAME_LEN];
};

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/*
 * strncpy is considered "unsafe" and strlcpy doesn't exist on all
 * systems (notably glibc-based ones). Here's a strncpy that
 * guarantees null termination.
 */
#define STRNCPY_SAFE(dst, src, n) do {				\
		char *p;					\
		int l = MIN((int) (n-1), (int) strlen(src));	\
		p = (char *) memcpy(dst, src, l);		\
		*(p + l) = '\0';				\
	} while (0)

#define IOC_IOMMU_GET_NXT_IOMMU_CB _IOWR(MSM_IOMMU_MAGIC, 0, struct get_next_cb)
#define IOC_IOMMU_GET_TARGET _IOR(MSM_IOMMU_MAGIC, 1, struct target_struct)
#define IOC_IOMMU_TEST_IOMMU_VA2PA _IOWR(MSM_IOMMU_MAGIC, 2, \
						struct test_iommu)
#define IOC_IOMMU_TEST_IOMMU_INT _IOWR(MSM_IOMMU_MAGIC, 3, \
						struct test_iommu)
#define IOC_IOMMU_TEST_IOMMU_BFB _IOWR(MSM_IOMMU_MAGIC, 4, struct test_iommu)
#define IOC_IOMMU_TEST_CATS _IOWR(MSM_IOMMU_MAGIC, 5, struct test_iommu)

#endif
