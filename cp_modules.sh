#!/bin/bash

curr_folder=$(pwd)
curr_device=$1

showHeader() {
    echo " "
    echo "cp_modules kernel helper which copies modules to its proper locations"
    echo " "
    echo "Written by Stane1983"
    echo " "
}

showError() {
    echo "Error: $1"
    echo " "
}

showMsg() {
    echo "$1"
}

checkModulesBuilt() {
    if [ -f "$curr_folder/drivers/amlogic/mali/mali.ko" ] ; then
        return 0
    else
        return 1
    fi
}

checkDevice() {
    if [ ! "$curr_device" = "" ]; then
        if [ -d "$curr_folder/../device/amlogic/$curr_device" ] ; then
            return 0
        else
            return 1
        fi
    else
        return 1
    fi
}

checkAndroidBuilt() {
    if [ -f "$curr_folder/../out/target/product/$curr_device/root/boot/mali.ko" ] ; then
        return 0
    else
        return 1
    fi
}

copyDeviceFolder() {
    cp $curr_folder/drivers/amlogic/mali/mali.ko $curr_folder/../device/amlogic/$curr_device/mali.ko
    cp $curr_folder/drivers/amlogic/ump/ump.ko $curr_folder/../device/amlogic/$curr_device/ump.ko
    cp $curr_folder/drivers/amlogic/wifi/rtl8xxx_CU/8192cu.ko $curr_folder/../device/amlogic/$curr_device/8192cu.ko
    cp $curr_folder/net/wireless/cfg80211.ko $curr_folder/../device/amlogic/$curr_device/cfg80211.ko
}

copyOutFolder() {
    cp $curr_folder/drivers/amlogic/mali/mali.ko ../out/target/product/$curr_device/root/boot/mali.ko
    cp $curr_folder/drivers/amlogic/ump/ump.ko ../out/target/product/$curr_device/root/boot/ump.ko
    cp $curr_folder/drivers/amlogic/wifi/rtl8xxx_CU/8192cu.ko ../out/target/product/$curr_device/system/lib/8192cu.ko
    cp $curr_folder/net/wireless/cfg80211.ko ../out/target/product/$curr_device/system/lib/cfg80211.ko
}

showHeader

copyFiles() {
    showMsg "Copying modules to device folder..."
    copyDeviceFolder
    if checkAndroidBuilt ; then
        showMsg "Copying modules to output folder..."
        copyOutFolder
    fi
}

if checkDevice ; then
    if checkModulesBuilt ; then
        copyFiles
    else
        showError "Modules are not built! Use make modules to create them."
    fi
else
    showError "Device $curr_device does not exist!"
fi
