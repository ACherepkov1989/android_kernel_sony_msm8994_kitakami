# Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#!/bin/sh
if [ -d /system/lib/modules/ ]; then
	modpath=/system/lib/modules
else
	modpath=/kernel-tests/modules/lib/modules/$(uname -r)/extra
fi

ldrex_test_mod=$modpath/ldrex_test_module.ko
test_debugfs_dev=/sys/kernel/debug
ldrex_dev_sys_case=${test_debugfs_dev}/ldrex_test/test_case
ldrex_module_name=ldrex_test_module

# Function ldrex_test_init
# Parameters:
# return 0 on success otherwise return 1
ldrex_test_init(){
	if [ ! -d $test_debugfs_dev ]; then
		mount -t debugfs nodev /sys/kernel/debug
		if [ $? -ne 0 ]; then
			echo "ldrex: Couldn't mount debugfs."
			return 1
		fi
	fi

	if [ ! -d $modpath ]; then
		echo "ldrex: Couldn't find a path to the kernel module."
		return 1
	fi

	if [ ! -d  /lib/modules/$(uname -r) ]; then
		mount -o remount, rw /
		mkdir -p /lib/modules/$(uname -r)
		return $?
	fi

	# remove ldrex_test_module before test
	lsmod | grep "$ldrex_module_name"
	if [ $? -eq 0 ]; then
		rmmod $ldrex_module_name
		if [ $? -ne 0 ]; then
			echo "ERROR: failed to remove module $ldrex_test_mod"
			return 1
		fi
	fi

	return 0
}

# Function ldrex_test
# Parameters:
# 1) test type
# 2) case number of the ldrex test
# return 0 on success otherwise return 1
ldrex_test(){
	# insert ldrex_test_module
	insmod $ldrex_test_mod
	if [ $? -ne 0 ] || [ ! -e $ldrex_test_dev_sys ]; then
		echo "ERROR: failed to load module $ldrex_test_mod"
		return 1
	fi

	if [ -z "$1" ]; then
		echo "ERROR: please input the test type"
		return 1
	else
		test_type=$1
	fi

	#check test type
	case $1 in
	"0")
	#test the case
	if [ -z "$2" ]; then
		if [ -f $ldrex_dev_sys_case ]; then
                       case_num=`cat ${ldrex_dev_sys_case}`
                else
                       echo "ERROR: failed to get the case number"
                       return 1
                fi
        else
                case_num=$2
                echo "ldrex_test, case_num : $case_num"
        fi
	# Start to test
        echo "=== ldrex test: test case -- $case_num  ==="
        echo $case_num > $ldrex_dev_sys_case
        if [ $? -ne 0 ]; then
                echo "ERROR: failed to pass parameter(case_num) to driver"
                return 1
        fi

	;;
	"1" | "2" | "3" | "4")
	#normal test
	i=1
	while [ $i -lt 12 ]
	do
	# Start to test
		echo "test case $i"
		echo $i > $ldrex_dev_sys_case
		if [ $? -ne 0 ]; then
			echo "ERROR: failed to pass parameter(case_num) to driver"
			return 1
		fi
		i=$(($i+1))
	done
	;;

	"2")
	esac
}
case_num=0
test_type=0
echo "$1"

if [ $# -eq 0 ];then
    echo "Usage: $0 [-n | --nominal] [-s | --stress] [-a | --adversarial] [-r | --repeatability] [-c | --case <ldrex_test_case_num>]"
    echo "ldrex_test_case_num : case num of the test case"
    exit 1
fi

while [ $# -gt 0 ]
do
    case $1 in
    -n | --nominal)
    test_type=1
    shift 1
    ;;
    -s | --stress)
    test_type=2
    shift 1
    ;;
    -a | --adversarial)
    test_type=3
    shift 1
    ;;
    -r | --repeatability)
    test_type=4
    shift 1
    ;;
    -c | --case)
    case_num=$2
    shift 2;
    ;;
    -h | --help | *)
    echo "Usage: $0 [-n | --nominal] [-s | --stress] [-a | --adversarial] [-r | --repeatability] [-c | --case <ldrex_test_case_num>]"
    echo "ldrex_test_case_num : case num of the test case"
    exit 1
    ;;
    esac
done

#init ldrex
ldrex_test_init
if [ $? -ne 0 ]; then
	exit 1
fi

result=1
#do nominal test
echo "=== Running Test ==="
ldrex_test $test_type $case_num
if [ $? -eq 0 ]; then
	echo "Test Passed"
	result=0
else
	echo "Test Failed"
	result=1
fi

# remove ldrex_test_module after test
lsmod | grep "$ldrex_module_name"
if [ $? -eq 0 ]; then
	rmmod $ldrex_module_name
	if [ $? -ne 0 ]; then
		echo "ERROR: failed to remove module $ldrex_test_mod"
		exit 1
	fi
fi

exit $result
