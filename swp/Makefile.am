swpdir = $(prefix)/swp

CFLAGS = -Werror -g -O2 -marm -march=armv5
LINK = $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) --static -o $@

KERNEL_FLAGS ?= ARCH=arm
swp_PROGRAMS = swp_test
swp_test_SOURCES = swp_test.c
dist_swp_SCRIPTS = swp_test.sh run.sh
dist_swp_DATA = README
