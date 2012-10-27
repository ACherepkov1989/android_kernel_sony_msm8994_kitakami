
ACLOCAL_AMFLAGS = -I m4

topdir           = $(prefix)

BUILD_TEST=

if KERNELMODULES
if USEION
BUILD_TEST += ion
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
endif

if USESPS
BUILD_TEST += sps
endif

SUBDIRS = $(BUILD_TEST)

