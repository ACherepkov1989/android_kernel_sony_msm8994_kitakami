#!/system/bin/sh
# Please don't hardcode /magisk/modname/... ; instead, please use $MODDIR/...
# This will make your scripts compatible even if Magisk change its mount point in the future
MODDIR=${0%/*}

# This script will be executed in late_start service mode
# More info in the main Magisk thread

sleep 5
echo "735 745 765 795 825 845 865 945 975 995 1005 1015 745 755 755 825 845 865 895 905 915 925 940 955 965 975 985" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table

sleep 75
echo "725 745 765 795 825 845 865 945 965 985 995 1005 735 735 735 805 835 865 895 905 915 925 940 955 965 975 985" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table

sleep 360
echo "725 745 765 795 825 845 865 945 965 985 995 1005 735 735 735 805 835 865 895 905 915 925 940 955 965 975 985" > /sys/devices/system/cpu/cpu0/cpufreq/UV_mV_table
