#! /bin/sh

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

echo "Greetings IP accelerator user!"

ipa_test_module_path=\
"
	/system/lib/modules \
	/kernel-tests/modules/lib/modules/`uname -r`/extra/ \
	/usr/kernel-tests/modules/lib/modules/`uname -r`/extra/ \
"

for p in ${ipa_test_module_path}; do
	if [ -d $p ]; then
		modpath=$p
		break
	fi
done

if [ ! -f /dev/ipa ]; then
	ipa_major=`cat /proc/devices | grep ipa | cut -d" " -f1`
	ipa_minor=0
	mknod   /dev/ipa   c   $ipa_major   $ipa_minor
	echo -e "IPA character file was created successfully\n"
fi

udevd -d
echo "UDEV started"
insmod ${modpath}/ipa_test_module.ko
echo "ITAKEM was loaded successfully\n"
