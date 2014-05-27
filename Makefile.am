
ACLOCAL_AMFLAGS = -I m4

topdir           = $(prefix)

BUILD_TEST=

if KERNELMODULES
if USEION
BUILD_TEST += ion
endif

if USEIPC_LOGGING
BUILD_TEST += ipc_logging
endif

if USEMEMORY_PROF
BUILD_TEST += memory_prof
endif
if USEMSM_BUS
BUILD_TEST += msm_bus
endif
if USEWATCHDOG
BUILD_TEST += watchdog
endif
if USEOCMEM
BUILD_TEST += ocmem
endif
if USEIOMMU
BUILD_TEST += msm_iommu
endif
if USEBAM_DMUX_LOOPBACK
BUILD_TEST += bam_dmux_loopback
endif
endif
if USESPS
BUILD_TEST += sps
endif
if USEV4L2APPS
BUILD_TEST += v4l2apps
endif
if USEIP_ACCELERATOR
BUILD_TEST += ip_accelerator
endif
if USEHRTIMER
BUILD_TEST += hrtimer
endif

SUBDIRS = $(BUILD_TEST)

