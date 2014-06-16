bam_dmux_loopbackdir = $(prefix)/bam_dmux_loopback
module = bam_dmux_loopback.ko
dist_bam_dmux_loopback_SCRIPTS  = bam_dmux_loopback_ktest.sh run.sh
dist_bam_dmux_loopback_DATA = $(module) README

KERNEL_FLAGS ?= ARCH=arm

kmake  = $(MAKE) $(KERNEL_FLAGS) -C $(KERNEL_DIR) M=$(CURDIR)

$(module): loopback.c loopback_sps.c loopback_smsm.c loopback_a2.c
	$(kmake) modules

all-local: $(module)

install-exec-local: $(module)
	$(kmake) INSTALL_MOD_PATH=$(DESTDIR)$(prefix)/modules modules_install

# "make distclean" will always run clean-local in this directory,
# # regardless of the KERNELMODULES conditional. Therefore, ensure
# # KERNEL_DIR exists before running clean. Further, don't fail even
# # if there is a problem.
clean-local:
	-test ! -d "$(KERNEL_DIR)" || $(kmake) clean
