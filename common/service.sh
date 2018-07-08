#!/system/bin/sh
# Please don't hardcode /magisk/modname/... ; instead, please use $MODDIR/...
# This will make your scripts compatible even if Magisk change its mount point in the future
MODDIR=${0%/*}

# This script will be executed in late_start service mode
# More info in the main Magisk thread

sleep 15
echo "735 745 765 795 825 845 865 945 975 995 1005 1015 745 755 755 825 845 865 895 905 915 925 940 955 965 975 995" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table

sleep 75
echo "725 745 765 795 825 845 865 945 965 985 995 1005 735 735 735 805 835 865 895 905 915 925 940 955 965 975 995" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table

sleep 325
if service call power 12 | grep "00000000 00000000"
   then
     input keyevent 26
   fi
chown 0.0 /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
sleep 1
echo "4" > /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
echo "725 745 765 795 825 845 865 945 965 985 995 1005 735 735 735 805 835 865 895 905 915 925 940 955 965 975 995" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table
sleep 1
echo "3" > /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
echo "725 745 765 795 825 845 865 945 965 985 995 1005 735 735 735 805 835 865 895 905 915 925 940 955 965 975 995" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table
sleep 1
echo "2" > /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
echo "725 745 765 795 825 845 865 945 965 985 995 1005 735 735 735 805 835 865 895 905 915 925 940 955 965 975 995" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table
sleep 1
echo "1" > /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
echo "725 745 765 795 825 845 865 945 965 985 995 1005 735 735 735 805 835 865 895 905 915 925 940 955 965 975 995" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table
sleep 1
chown 1000.1000 /sys/devices/system/cpu/cpu4/core_ctl/min_cpus
chown 1000.1000 /sys/devices/system/cpu/cpu4/core_ctl/max_cpus
