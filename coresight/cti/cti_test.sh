#! /bin/sh --

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

# Script to test the QDSS CTI driver.
echo "-----Coresight cti Test Starting-----"
echo "--------------------------------------------"
source "$(dirname $0)/../cs_common.sh"

CTI_MAX_TRIGGERS=8
CTI_MAX_CHANNELS=4
if [ !-d /sys/bus/coresight/devices ]; then
    echo "/sys/bus/coresight/ don't exist"
    return 1
fi
CTI_MAX_NUMBERS=`ls /sys/bus/coresight/devices | grep 'coresight-cti[0-8]'| wc -l`

if [ $CTI_MAX_NUMBERS -lt 0 ]; then
    echo "cti number is lower than zero"
    return 1
fi

let cti_number=$CTI_MAX_NUMBERS-1
trig_rang=$( seq 1 $CTI_MAX_TRIGGERS)
channel_rang=$( seq 1 $CTI_MAX_CHANNELS)
f=0
t=0
if [ $t -lt $cti_number ]; then
    for trig in $trig_rang
    do
        for channel in $channel_rang
        do
            # start do map trigin to channel test
            echo $trig $channel  >  $ctipath$t/map_trigin
            trigin=`cat $ctipath$t/show_trigin | cut -b 4`
            channelin=`cat $ctipath$t/show_trigin | cut -b 8`
            if [ "$trig" == "$trigin" ] && [ "$channel" == "$channelin" ]; then
                echo "succeed map trigin $trig to coresight-cti$t channel $channel"
                #unmap trigin
                echo $trig $channel > $ctipath$t/unmap_trigin
                trigin=`cat $ctipath$t/show_trigin | cut -b 4`
                channelin=`cat $ctipath$t/show_trigin | cut -b 8`
                if [ "$trigin" = "" ] && [ "$channelin" = "" ]; then
                    echo "succeed to unmap trigin $trig to coresight-cti$t channel $channel"
                else
                    echo "failed to unmap trigin $trig to coresight-cti$t channel $channel"
                    let f=f+1
                    echo 1 > $ctipath$t/reset
                fi
            else
                echo "Failed map trigin $trig to coresight-cti$t channel $channel"
                let f=f+1
                echo 1 > $ctipath$t/reset
            fi
            #start do map trigout to channel test
            echo $trig $channel > $ctipath$t/map_trigout
            trigout=`cat $ctipath$t/show_trigout | cut -b 4`
            channelout=`cat $ctipath$t/show_trigout | cut -b 8`
            if [ "$trig" == "$trigout" ] && [ "$channel" == "$channelout" ]; then
                echo $trig $channel  >  $ctipath$t/unmap_trigout
                trigout=`cat $ctipath$t/show_trigout | cut -b 4`
                channelout=`cat $ctipath$t/show_trigout | cut -b 8`
                if [ "$trigout" = "" ] && [ "$channelout" = "" ]; then
                    echo "succeed to unmap trigout $trig to coresight-cti$t channel $channel"
                else
                    echo "failed to unmap trigout $trig to coresight-cti$t channel $channel"
                    let f=f+1
                    echo 1 > $ctipath$t/reset
                fi
            else
                echo "Fail map trigout $trig to coresight-cti$t channel $channel"
                let f=f+1
                echo 1 > $ctipath/reset
            fi
        done
        let let t=t+1
    done
fi
# $f count the map and unmap failed times
if [ "$f" == "0" ]; then
    echo "CTI map/unmap Test PASS "
fi
