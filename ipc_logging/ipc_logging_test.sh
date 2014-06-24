# Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#  Unit test for the ipc_logging.
#
# Invoked by run.sh
# This will run nominal, adversarial, repeatability and stability test cases

if [ -d /system/lib/modules/ ]; then
	modpath=/system/lib/modules
else
	echo "ERROR:Modules directory not present"
	exit 1
fi

ipc_logging_test_mod=${modpath}/ipc_logging_test.ko

insmod $ipc_logging_test_mod

if [ $? -ne 0 ]; then
	echo "ERROR: Failed to load module $ipc_logging_test_mod"
	exit 1
fi

cd /d/ipc_logging_test
count=1

for i in $*
do
	case $i in
	-n | --nominal )
	while [ $count -le 1 ]
	do
	echo "$count"
	cat ut_basic
	cat ut_wrap_test
	cat ut_nd_read
	echo "Completed test $count"
	(( count++ ))
	done
	;;
	-a | --adversarial )
	echo "Not Implemented yet"
	;;
	-r | --repeatability )
	while [ $count -le 10 ]
	do
	cat ut_basic
	cat ut_wrap_test
	cat ut_nd_read
	echo "Completed test $count"
	(( count++ ))
	done
	;;
	-s | --stress)
	while [ $count -le 1000 ]
	do
	cat ut_basic
	cat ut_wrap_test
	cat ut_nd_read
	echo "Completed test $count"
	(( count++ ))
	done
	;;
	-h | --help | *)
	echo "Usage: $0 [-n | --nominal] [-a | --adversarial] [-r | --repeatability] [-s | --stress]"
	;;
	esac
done

rmmod $ipc_logging_test_mod

if [ $? -ne 0 ]; then
	echo "ERROR: Failed to unload module $ipc_logging_test_mod"
fi
