swpdir = $(prefix)/swp

KERNEL_FLAGS ?= ARCH=arm
swp_PROGRAMS = swp_test
swp_test_SOURCES = swp_test.c
swp_test_LDFLAGS = --static
swp_test_CFLAGS = -Werror -marm -march=armv5
dist_swp_SCRIPTS = swp_test.sh run.sh
dist_swp_DATA = README
