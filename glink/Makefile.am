glinkdir = $(prefix)/glink
dist_glink_SCRIPTS  = glink_ktest.sh run.sh
dist_glink_DATA = $(module) README

KERNEL_FLAGS ?= ARCH=arm
CC_FLAGS = -Idrivers/soc/qcom

module = glink_test.ko
kmake  = $(MAKE) $(KERNEL_FLAGS) $(CC_FLAGS) -C $(KERNEL_DIR) M=$(CURDIR)

$(module): glink_mock_xprt.c glink_unit_test.c glink_loopback_xprt.c glink_loopback_client.c
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
