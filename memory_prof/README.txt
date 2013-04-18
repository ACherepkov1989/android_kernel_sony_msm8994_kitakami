Test: memory_prof

Usage: memory_prof [OPTIONS]...

OPTIONS:
  -h         Print this message and exit
  -a         Do the adversarial test (same as -l)
  -b         Do basic sanity tests
  -e         Do Ion heap profiling
  -k         Do kernel alloc profiling
  -l         Do leak test (leak an ion handle)
  -m         Do map extra test
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

Target support: 8974
