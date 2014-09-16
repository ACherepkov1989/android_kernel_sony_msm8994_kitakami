memory_prof is a suite of tools for profiling memory-related
performance as well as catching regressions. The suite is named after
the original memory_prof program, but now includes other programs and
scripts that can be used for memory profiling.


                                                                  __
                                                                 / _|
 _ __ ___   ___ _ __ ___   ___  _ __ _   _       _ __  _ __ ___ | |_
| '_ ` _ \ / _ \ '_ ` _ \ / _ \| '__| | | |     | '_ \| '__/ _ \|  _|
| | | | | |  __/ | | | | | (_) | |  | |_| |     | |_) | | | (_) | |
|_| |_| |_|\___|_| |_| |_|\___/|_|   \__, |     | .__/|_|  \___/|_|
                                      __/ |_____| |
                                     |___/______|_|

Usage: See memory_prof -h

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

    Information about the format of allocation profiles (specified
    with -i) can be found at the end of this document in Appendix A.

Target support: All


                           __               _
                          / _|             | |
 _ __ ___   ___ _ __ ___ | |_ ___  __ _ ___| |_
| '_ ` _ \ / _ \ '_ ` _ \|  _/ _ \/ _` / __| __|
| | | | | |  __/ | | | | | ||  __/ (_| \__ \ |_
|_| |_| |_|\___|_| |_| |_|_| \___|\__,_|___/\__|

Usage: See memfeast -h

Description:

    memfeast reliably and predictably forces the system into a
    low-memory condition.

Target support: All





-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*


Appendix A: Allocation Profiles for memory_prof

    The heap profiling test (-e) has support for custom "allocation
    profile" input files (specified with -i). The format of the
    allocation profile file is a comma-separated values file with the
    following columns:

        op
        [rest...]

    The `op' field specifies what kind of operation this line
    holds. The following operations are currently supported:

        - alloc
        - sleep
        - print
        - simple_alloc
        - simple_basic_sanity
        - simple_profile
        - simple_free
        - alloc_pages
        - create_unused_client
        - free_all_unused_clients
        - user_alloc
        - iommu_map_range

    Each operation is described in detail below.

    The remaining fields ([rest...])  are defined differently for
    different values of `op'.

    In all cases, all defined fields are *required* and cannot be left
    empty (e.g. `something,,other'). For example, if you don't have
    any flags to pass for alloc, put a 0 rather than leaving it blank.

    o `op' == alloc

      When `op' == alloc, an ION_IOC_ALLOC/ION_IOC_FREE will be
      performed and profiled. Additionally, mmap'ing and memset'ing
      the buffer will be performed and profiled if profile_mmap and
      profile_memset are set.

      The following remaining fields are defined:

          reps
          heap_id
          flags
          alloc_size
          alloc_size_label
          quiet_on_failure
          profile_mmap
          profile_memset

      - reps :: How many times to repeat this allocation

      - heap_id :: Heap to use for allocation. Should correspond to a
                   heap_id from `enum ion_heap_ids'. E.g.:
                   ION_CP_MM_HEAP_ID

      - flags :: Flags to be used for allocation. Can parse bitwise OR'd
                 ION_FLAG_* constants (e.g.:
                 ION_SECURE|ION_FLAG_CACHED). No spaces please.

      - alloc_size :: The size of the buffer to be allocated. Can be
        any valid size string (e.g. "4KB", "2MB", etc). Supported
        suffixes are "KB" "MB" and "GB" (or no suffix for bytes).

      - alloc_size_label :: A human- (and script-) readable string
           describing the allocation size

      - quiet_on_failure :: Whether we should print an error message if
           this allocation fails

      - profile_mmap :: Whether we should profile mmap

      - profile_memset :: Whether we should profile memset

      Blank lines and lines beginning with '#' are skipped.

      See alloc_profiles/general.txt for a full example.


    o `op' == sleep

      When `op' == sleep, a usleep will be inserted with the specified
      number of microseconds.

      The following remaining fields are defined:

          time_us

      - time_us :: The time (in microseconds) to sleep

    o `op' == print

      When `op' == print, the remaining text on the line is printed to
      stdout.

      The following remaining fields are defined:

          rest

      - rest :: The text to print

    o `op' == simple_alloc

      When `op' == simple_alloc, an ION_IOC_ALLOC will be performed. A
      matching ION_IOC_FREE will *not* be performed. To free a buffer
      allocated with `simple_alloc' you should use the `simple_free'
      op (defined below) with the same alloc_id field.

      The following remaining fields are defined:

          alloc_id
          heap_id
          flags
          alloc_size
          alloc_size_label

      - alloc_id :: a user-defined ID that can be used in a
        `simple_free' line (see below) to free this allocation. This
        can actually be any string.

      - heap_id, flags, alloc_size, alloc_size_label :: the same
        was as for the `alloc' op, above

      See `simple_free' for an example of how this can be used.

    o `op' == simple_basic_sanity

      When `op' == simple_basic_sanity, we iterate through all buffers
      previously allocated with `simple_alloc' until we find one with
      a matching alloc_id. We do basic sanity testing of the first one
      we find with the same algorithm as `./memory_prof -b'.

      The following remaining fields are defined:

          alloc_id

      Important: this operation will always fail on anything except a
      freshly allocated buffer. For example, the following sequence is
      BAD:

          simple_alloc,pizza,ION_SYSTEM_HEAP_ID,ION_FLAG_CACHED,0x100000,1MB
          simple_profile,pizza
          simple_basic_sanity,pizza

      But the following sequence is GOOD:

          simple_alloc,pizza,ION_SYSTEM_HEAP_ID,ION_FLAG_CACHED,0x100000,1MB
          simple_basic_sanity,pizza
          simple_profile,pizza

    o `op' == simple_profile

      When `op' == simple_profile, we iterate through all buffers
      previously allocated with `simple_alloc' until we find one with
      a matching alloc_id. We profile the first one we find with the
      same algorithm as `op' == alloc.

      The following remaining fields are defined:

          alloc_id

    o `op' == simple_free

      When `op' == simple_free, an ION_IOC_FREE will be performed on
      the buffer identified by alloc_id. See the `simple_alloc' op
      above.

      The following remaining fields are defined:

          alloc_id

      - alloc_id :: the user-defined ID that was used in an earlier
        `simple_alloc'

      Here's an example of an allocation profile using
      simple_alloc/simple_profile/simple_free:

          simple_alloc,1,ION_SYSTEM_HEAP_ID,ION_FLAG_CACHED,0x100000,1MB
          simple_alloc,pizza,ION_SYSTEM_HEAP_ID,ION_FLAG_CACHED,0x100000,1MB
          simple_profile,pizza
          # there are now two Ion buffers allocated. Now free them both:
          simple_free,1
          simple_free,pizza

    o `op' == alloc_pages

      alloc_pages is used to directly profile the kernel's buddy
      allocator.

      The following remaining fields are defined:

          order
          gfp_flags

      - order :: the order of the page to allocate with the kernel's
        `alloc_pages' routine.

      - gfp_flags :: the gfp flags to use. Note that these are not
        real gfp flags as known by the kernel. Since the kernel's
        actual gfp flags are not exported to userspace and it would be
        too much effort to try to mirror all of them, we define some
        new flags for the most commonly used gfp flags. Currently
        supported flags:

          MP_GFP_KERNEL
          MP_GFP_HIGHMEM
          MP_GFP_ZERO
          MP_GFP_HIGHUSER
          MP_GFP_NOWARN
          MP_GFP_NORETRY
          MP_GFP_NO_KSWAPD
          MP_GFP_WAIT
          MP_GFPNOT_WAIT

        You can't compose gfp flags the same way you can in the kernel
        like: GFP_KERNEL & ~GFP_WAIT since these are simple,
        independent bitfields. To accomplish not'ing something out, a
        dedicated flag must be used, like MP_GFPNOT_WAIT. Note,
        however, that these *can* be |'d together just like the flags
        in `alloc' and `simple_alloc', e.g.: MP_GFP_KERNEL|MP_GFP_ZERO.

        Here's an example allocation profile using alloc_pages:

          alloc_pages,1,MP_GFP_KERNEL
          alloc_pages,1,MP_GFP_KERNEL|MP_GFP_ZERO
          alloc_pages,9,MP_GFP_KERNEL
          alloc_pages,9,MP_GFP_KERNEL|MP_GFP_ZERO

    o `op' == create_unused_client

      When `op' == create_unused_client, we will create a new Ion
      client by opening /dev/ion. No allocations or any other work
      whatsoever is performed. No other fields are used for this
      op. This is useful for profiling the client creation and
      destruction code.

    o `op' == free_all_unused_clients

      When `op' == free_all_unused_clients, all clients previously
      created with create_unused_client are free'd by close()'ing all
      file descriptors returned by the previous calls to open().

    o `op' == user_alloc

      When `op' == user_alloc, profile the libc `malloc' or `mmap'
      functions.

      The following remaining fields are defined:

          allocator
          alloc_size
          usage_size
          usage_fn

      - allocator :: the underlying memory allocation function to
        profile. Currently supported values: "malloc", "mmap". Note
        that when mmap is used we also pass the MAP_POPULATE flag, so
        the result will also include the time taken to fault the pages
        in.

      - alloc_size :: the size of the buffer to profile. Can be any
        valid size string (e.g. "4KB", "2MB", etc). Supported suffixes
        are "KB" "MB" and "GB" (or no suffix for bytes).

      - usage_size :: how many of the allocated bytes to run through
        usage_fn. Can be any valid size string, similar to alloc_size.

      - usage_fn :: the function to use to fiddle with the allocated
        memory. Supported values:

          - nop :: don't touch the memory

          - memset-n :: memset the memory to `n'

            `n' is passed to `strtol' with a `base' of 0 so it can
            take any of the values supported there (see strtol(3)).

            e.g. memset-0xa5 would result in memset(buf, 0xa5, size)
                 memset-0 would result in memset(buf, 0, size)

    o `op' == iommu_map_range
    o `op' == iommu_unmap_range
    o `op' == iommu_map
    o `op' == iommu_unmap

      When `op' equals one of the four operations above, profile the
      corresponding api in the kernel.

      The following remaining fields are defined:

          context_name
          chunk_order
          nchunks
          iterations
          prot
          flags

      - context_name :: the Iommu context to use

      - chunk_order :: the order of the pages to be used for each
        chunk

      - nchunks :: how many chunks to allocate

      - iterations :: number of interations to run

      - prot :: protection flags to be used for the mapping. Similar
        to the gfp_flags field for the `alloc_pages' op, the real
        kernel Iommu prot flags are not exported to userspace so we
        create some of our own definitions that later get mapped to
        the kernel's Iommu prot flags

          MP_IOMMU_WRITE
          MP_IOMMU_READ
          MP_IOMMU_CACHE

        The note on the gfp_kernel field about flag composition also
        applies here.

      - flags :: Addition flags used to control the test

         MP_IOMMU_ATTACH - Attach to context bank
         MP_IOMMU_SECURE - Test secure iommu functionality

        The note on the gfp_kernel field about flag composition also
        applies here.

    o `op' == iommu_attach
    o `op' == iommu_detach

      When `op' equals one of the two operations above, profile the
      corresponding api in the kernel.

      The following remaining fields are defined:

          context_name
          iterations
          flags

      - context_name :: the Iommu context to use

      - iterations :: number of interations to run

      - flags :: Addition flags used to control the test

         MP_IOMMU_SECURE - Test secure iommu functionality

        The note on the gfp_kernel field about flag composition also
        applies here.

    o `op' == ion_cache

      When `op' == cache, an ION_IOC_CLEAN_CACHES/ION_IOC_INV_CACHES/
      ION_IOC_CLEAN_INV_CACHES will be performed and profiled on an
      allocated ion buffer. The buffer will be free'd after the
      profiling is complete.

      The following remaining fields are defined:

          reps
          heap_id
          flags
          alloc_size
          alloc_size_label
          cache_clean
          cache_invalidate

      - reps :: How many times to repeat this allocation

      - heap_id :: Heap to use for allocation. Should correspond to a
                   heap_id from `enum ion_heap_ids'. E.g.:
                   ION_CP_MM_HEAP_ID

      - flags :: Flags to be used for allocation. Can parse bitwise OR'd
                 ION_FLAG_* constants (e.g.:
                 ION_SECURE|ION_FLAG_CACHED). No spaces please.

      - alloc_size :: The size of the buffer to be allocated. Can be
        any valid size string (e.g. "4KB", "2MB", etc). Supported
        suffixes are "KB" "MB" and "GB" (or no suffix for bytes).

      - alloc_size_label :: A human- (and script-) readable string
           describing the allocation size

      - cache_clean:: Whether we want to profile cache clean operation

      - cache_invalidate :: Whether we want to profile cache invalidate
                    operation

      For profiling ION_ION_CLEAN_INV_CACHES both cache_clean and
      cache_invalidate should be true.

      Blank lines and lines beginning with '#' are skipped.

      See alloc_profiles/cache_ops.txt for a full example.
