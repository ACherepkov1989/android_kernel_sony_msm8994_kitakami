#!/usr/bin/env python

# Copyright (c) 2013, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#     * Neither the name of The Linux Foundation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Quick and very dirty parser for the output of `memory_prof.sh -e'

# This should be run on the host (not the target) with the output from
# the target piped in (e.g. copy/paste/cat) or passed as a file on the
# command line

# Dependencies: matplotlib, numpy


# NOTE: THIS HAS GOTTEN UNWIELDY. YOU HAVE BEEN WARNED.


import sys
import fileinput
import operator
from itertools import cycle
from optparse import OptionParser
import numpy as np
import matplotlib.pyplot as plt


ST_PREFIX_DATA_ROW      = "=> "
ST_PREFIX_PREALLOC_SIZE = "==> "
ST_PREFIX_NUM_REPS      = "===> "


def get_data_lines(data):
    return [line.lstrip(ST_PREFIX_DATA_ROW).rstrip('\n') for line in data if line.startswith(ST_PREFIX_DATA_ROW)]

def extract_data_for_size(data, target_sz):
    cached_timings = {
        'ION_IOC_ALLOC': [],
        'ION_IOC_FREE': [],
    }
    uncached_timings = {
        'ION_IOC_ALLOC': [],
        'ION_IOC_FREE': [],
    }
    heaps = []

    for line in get_data_lines(data):
        # for the format, see print_stats_results in memory_prof.c
        (ion_op, heap_id, caching, sz, lbl_av, average, lbl_std, std_dev) = line.split(' ')
        if sz != target_sz: continue
        av = max(float(average), 0)
        if heap_id not in heaps:
            heaps.append(heap_id)
        if caching == 'cached':
            cached_timings[ion_op].append(av)
            # we don't do measure for uncached from
            # ION_SYSTEM_HEAP_ID. Just add a 0 to keep the number of bars
            # even.
            if heap_id == "ION_SYSTEM_HEAP_ID":
                uncached_timings.append(0)
        else:
            uncached_timings[ion_op].append(av)

    return (heaps, cached_timings, uncached_timings)

def extract_all_data(data):
    timings = {
        'ION_IOC_ALLOC': {},
        'ION_IOC_FREE': {},
    }

    for line in get_data_lines(data):
        # for the format, see print_stats_results in memory_prof.c
        (ion_op, heap_id, caching, sz, lbl_av, average, lbl_std, std_dev) = line.split(' ')
        timings[ion_op].setdefault(heap_id, {})
        av = max(float(average), 0)
        timings[ion_op][heap_id].setdefault(caching, {})
        timings[ion_op][heap_id][caching][sz] = float(average)

    return timings

def compare_heaps_for_a_size(data, target_sz, num_reps, ion_op, text_only=False, target=None):
    (heaps, cached_timings, uncached_timings) = extract_data_for_size(data, target_sz)
    cached_timings = cached_timings[ion_op]
    uncached_timings = uncached_timings[ion_op]

    title = ('Ion %s times\n' % ion_op) \
            + ('Target: %s' % ("%s\n" % target) if target is not None else "") \
            + ('(%s with ION_IOC_ALLOC, average of %d reps)' % (target_sz, num_reps))

    print title
    print

    for (heap, cached, uncached) in zip(heaps, cached_timings, uncached_timings):
        print "%25s   (cached): %f" % (heap, cached)
        print "%25s (uncached): %s" % (heap, str(uncached) if uncached != 0 else "N/A")

    if text_only:
        return

    ind = np.arange(len(heaps))
    width = .35

    fig = plt.figure()
    ax = fig.add_subplot(111)
    cached_rects = ax.bar(ind, cached_timings, width, color='r')
    uncached_rects = ax.bar(ind + width, uncached_timings, width, color='y')

    ax.set_title(title)
    ax.set_ylabel('Time (ms)')
    ax.set_xticks(ind + width)
    ax.set_xticklabels(heaps)

    ax.legend( (cached_rects[0], uncached_rects[0]), ('Cached', 'Uncached') )

    plt.show()

def first_key_element(d):
    "Returns the first element found in the dict `d'."
    return d[d.keys()[0]]

def compare_times_for_heaps(data, num_reps, ion_op, text_only=False, target=None):
    timings = extract_all_data(data)[ion_op]

    # we need to sort the size strings, which are a few levels in
    # (through some keys that might or might not exist)
    first_heap_timing = first_key_element(timings)
    sizes_for_heap_timing = first_key_element(first_heap_timing)
    sorted_keys = sorted(
        sizes_for_heap_timing.keys(),
        key=lambda v: float(v.split('MB')[0])
    )

    for target_heap in timings.keys():
        heap_timings = timings[target_heap]
        print '%s times for %s\n' % (ion_op, target_heap)

        format_str = '%6s %10s %10s'
        print format_str % (
            'Size', 'Cached', 'Uncached',
        )

        for k in sorted_keys:
            print format_str % (
                k,
                ('%5.2f' % heap_timings['cached'][k]) if 'cached' in heap_timings else 'NA',
                ('%5.2f' % heap_timings['uncached'][k]) if 'uncached' in heap_timings else 'NA',
            )

        print '\n'

    if text_only:
        return

    fig = plt.figure()
    ax = fig.add_subplot(111)

    sorted_keys_numbers = [float(s.split('MB')[0]) for s in sorted_keys]

    shapes_cycler = cycle(["o", "v", "^" , "<", ">"])

    for target_heap in timings.keys():
        for caching in ('cached', 'uncached'):
            heap_timings = timings[target_heap]
            if caching in heap_timings:
                lbl = "%s (%s)" % (target_heap, caching)
                ax.plot(sorted_keys_numbers,
                        [heap_timings[caching][k] for k in sorted_keys],
                        next(shapes_cycler) + '-',
                        label=lbl)

    title = ("Ion %s times\n" % ion_op) \
            + ('%s' % ("Target: %s\n" % target) if target is not None else "") \
            + ("(average of %d reps)" % num_reps)

    ax.set_ylabel("Time (ms)")
    ax.set_xlabel("Allocation size (MB)")
    ax.set_title(title)
    ax.legend(loc="upper left")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    target_sz = None

    parser = OptionParser()
    parser.add_option("-c", "--compare-heaps", action="store_true",
                      help="Compare same-sized allocations across heaps")
    parser.add_option("-z", "--compare-alloc-sizes", action="store_true",
                      help="Compare same-heap allocations across sizes")
    parser.add_option("-s", "--size", metavar="SIZE",
                      help="Allocation size to plot (e.g. '8MB'). Only used with -c")
    # parser.add_option("-e", "--heap", metavar="HEAP",
    #                   help="Heap to plot (e.g. 'ION_CP_MM_HEAP_ID'), or 'ALL'. Only used with -z")
    parser.add_option("-t", "--text-only", action="store_true")
    parser.add_option("--target")
    parser.add_option("-o", "--ion-op",
                      default="ION_IOC_ALLOC",
                      help="Ion operation to display (currently supported: ION_IOC_ALLOC, ION_IOC_FREE)")

    (options, args) = parser.parse_args()

    if options.compare_heaps and not options.size:
        print "You must provide a size (-s) when comparing same-sized allocations across heaps (-c)"
        sys.exit(1)

    if not options.compare_heaps and not options.compare_alloc_sizes:
        print "You must specify either -c or -z"
        sys.exit(1)

    # snarf:
    data = [line for line in fileinput.input(args)]

    # get the num reps:
    repsline = [line for line in data if line.startswith(ST_PREFIX_NUM_REPS)][0]
    num_reps = int(repsline.split(' ')[-1])

    if options.compare_heaps:
        compare_heaps_for_a_size(data, options.size, num_reps,
                                 text_only=options.text_only,
                                 target=options.target)

    if options.compare_alloc_sizes:
        compare_times_for_heaps(data, num_reps,
                                text_only=options.text_only,
                                target=options.target)
