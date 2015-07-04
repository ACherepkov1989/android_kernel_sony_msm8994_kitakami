memory_profdir = $(prefix)/memory_prof
memory_prof_PROGRAMS = memory_prof memfeast
memory_prof_SOURCES	= memory_prof.c alloc_profiles.c memory_prof_util.c \
	op-alloc.c op-print.c op-simple-ops.c op-sleep.c op-alloc-pages.c \
	op-user-alloc.c op-iommu-map.c op-iommu-unmap.c \
	op-iommu-map-range.c op-iommu-unmap-range.c op-iommu-attach.c \
	op-iommu-detach.c op-ion-cache.c concurrency-latency-kpi.c
memory_prof_CFLAGS = -lm -Werror -DALLOC_PROFILES_PATH=$(memory_profdir)
memfeast_SOURCES = memfeast.c memory_prof_util.c
dist_memory_prof_SCRIPTS  = memory_prof.sh run.sh
dist_memory_prof_DATA = \
	README.txt \
	alloc_profiles/1MB-10MB.txt \
	alloc_profiles/4KB-1MB.txt \
	alloc_profiles/alloc_pages.txt \
	alloc_profiles/builtin.txt \
	alloc_profiles/cache_ops.txt \
	alloc_profiles/cache_ops1.txt \
	alloc_profiles/cache_ops2.txt \
	alloc_profiles/cp_heap.txt \
	alloc_profiles/general.txt \
	alloc_profiles/iommu_attach.txt \
	alloc_profiles/iommu_detach.txt \
	alloc_profiles/iommu_map.txt \
	alloc_profiles/one_1GB_buffer_from_system.txt \
	alloc_profiles/pil1_heap.txt \
	alloc_profiles/qseecom_heap.txt \
	alloc_profiles/simple.txt \
	alloc_profiles/system.txt \
	alloc_profiles/tons-of-clients.txt \
	alloc_profiles/user_alloc.txt

KERNEL_FLAGS ?= ARCH=arm

all_modules = the_memory_prof_module.ko

kmake  = $(MAKE) $(KERNEL_FLAGS) -C $(KERNEL_DIR) M=$(CURDIR)

the_memory_prof_module.ko: memory_prof_module.c timing_debug.c
	$(kmake) modules

all-local: $(all_modules)

install-exec-local: $(all_modules)
	$(kmake) INSTALL_MOD_PATH=$(DESTDIR)$(prefix)/modules modules_install

# "make distclean" will always run clean-local in this directory,
# regardless of the KERNELMODULES conditional. Therefore, ensure
# KERNEL_DIR exists before running clean. Further, don't fail even
# if there is a problem.
clean-local:
	-test ! -d "$(KERNEL_DIR)" || $(kmake) clean
