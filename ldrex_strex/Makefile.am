KERNEL_FLAGS ?= ARCH=arm
EXTRA_CFLAGS	= -DDEBUG

module = ldrex_test_module.ko
kmake  = $(MAKE) $(KERNEL_FLAGS) -C $(KERNEL_DIR) M=$(CURDIR)

$(module):
	$(kmake) modules

all-local: $(module)

install-exec-local: $(module)
	$(kmake) INSTALL_MOD_PATH=$(DESTDIR)$(prefix)/modules modules_install

dist_ldrex_DATA		= README.txt $(module)

# "make distclean" will always run clean-local in this directory,
# regardless of the KERNELMODULES conditional. Therefore, ensure
# KERNEL_DIR exists before running clean. Further, don't fail even
# if there is a problem.
clean-local:
	-test ! -d "$(KERNEL_DIR)" || $(kmake) clean

ldrexdir			= $(prefix)/ldrex
dist_ldrex_SCRIPTS	= ldrex_test.sh run.sh


