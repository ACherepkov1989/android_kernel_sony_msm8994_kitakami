Test: memory_prof

Usage: memory_prof [OPTIONS]...

OPTIONS:
  -h         Print this message and exit
  -a         Do the adversarial test (same as -l)
  -b         Do basic sanity tests
  -e         Do Ion heap profiling
  -k         Do kernel alloc profiling (requires kernel module)
  -l         Do leak test (leak an ion handle)
  -m         Do map extra test (requires kernel module)
  -n         Do the nominal test (same as -b)
  -o         Do OOM test (alloc from Ion Iommu heap until OOM)
  -p MS      Sleep for MS milliseconds between stuff (for debugging)
  -r         Do the repeatability test
  -s         Do the stress test (same as -e)

Description:

These tests are useful for catching performance regressions in Ion or
general memory code (using the -e and -k options). They can also catch
other Ion regressions by performing some basic sanity tests (the -b,
-m, and -l options).

Notes:

This test suite is accompanied by a kernel module that must be
inserted for certain test cases (namely -k and -m). The memory_prof.sh
script will take care of inserting the kernel module and running the
memory_prof binary for you. However, sometimes it's useful to be able
run the memory_prof binary directly without inserting the kernel
module.

Target support: 8974
