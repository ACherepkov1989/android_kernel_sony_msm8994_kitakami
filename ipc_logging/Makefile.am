KERNEL_FLAGS ?= ARCH=arm

module = ipc_logging_test.ko
kmake  = $(MAKE) $(KERNEL_FLAGS) -C $(KERNEL_DIR) M=$(CURDIR)

$(module):
	$(kmake) modules

all-local: $(module)

install-exec-local: $(module)
	$(kmake) INSTALL_MOD_PATH=$(DESTDIR)$(prefix)/modules modules_install

# "make distclean" will always run clean-local in this directory,
# regardless of the KERNELMODULES conditional. Therefore, ensure
# KERNEL_DIR exists before running clean. Further, don't fail even
# if there is a problem.
clean-local:
	-test ! -d "$(KERNEL_DIR)" || $(kmake) clean


ipc_loggingdir           = $(prefix)/ipc_logging

