
ACLOCAL_AMFLAGS = -I m4

topdir           = $(prefix)

BUILD_TEST=

if KERNELMODULES
if USEMSM_BUS
BUILD_TEST += msm_bus
endif
if USEWATCHDOG
BUILD_TEST += watchdog
endif
endif

if USESPS
BUILD_TEST += sps
endif

SUBDIRS = $(BUILD_TEST)

