Test:       ldrex_test.sh

Usage: ldrex_test.sh [-n | --nominal] [-s | --stress] [-a | --adversarial]
			[-r | --repeatability] [-c | --case] <case_num>

Only one option each time. They are not supposed to be used in conjunction with each other.
When use -n, -s, -a, -r, all the cases will be run one by one. The specified test case will
be tested if option is -c.

       case_num: different case num with different test case

Note:

This testcase is only meant for 32-bit ARM and not for 64-bit yet. Another kernel test change
of loading and storing exclusive register for 64-bit instruction(ldxr and stxr) will be
submited later.

Description:
The LDREX and STREX instructions split the operation of atomically updating
memory into two separate steps. Together, they provide atomic updates in
conjunction with exclusive monitors that track exclusive memory accesses.

An exclusive monitor is a simple state machine, with the possible states open and
exclusive. To support synchronization between processors, a system must implement
two sets of monitors, local and global. A Load-Exclusive operation updates the
monitors to exclusive state. A Store-Exclusive operation accesses the monitor(s) to
determine whether it can complete successfully. A Store-Exclusive can succeed only if
all accessed exclusive monitors are in the exclusive state.

case_num :

0 --- Test Stop

1 --- cacheable: 4 LDREX on same memory location - same cpu

2 --- cacheable: NR_CPUS LDREX on different memory location - same cpu

3 --- Uncacheable: 4 LDREX on same memory location - same cpu

4 --- Uncacheable: 1 LDREX on different memory location - same cpu

5 --- cacheable:  4 LDREX on same memory location for all present cores

6 --- cacheable: 1 LDREX on different memory location for all present cores

7 --- Uncacheable:  2 LDREX on same memory location for all present cores

8 --- Uncacheable: 1 LDREX on different memory location for all present cores

9 --- Uncacheable: 1 LDREX on same memory location for all present cores
except waiting thread for cpu (NR_CPUS -1)

10 --- cacheable: 1 LDREX on different memory location for all present cores
except waiting thread for cpu (NR_CPUS -1)

11 --- cacheable: LDREX and STREX for all present cores with multiple threads
