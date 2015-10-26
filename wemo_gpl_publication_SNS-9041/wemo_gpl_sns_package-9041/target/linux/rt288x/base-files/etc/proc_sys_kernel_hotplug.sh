#!/bin/sh

#WNC_Roger 20140514 to fix bug of mdev not remove device node after unplug usb storage
#/etc/inittab
# ::sysinit:echo "/etc/proc_sys_kernel_hotplug.sh" > /proc/sys/kernel/hotplug
#/etc/usb_mount_umount.sh # busybox-mdev script for /etc/mdev.conf

#echo "ACTION=$action" > /dev/ttyS1
#echo "DEVPATH=${DEVPATH}" > /dev/ttyS1
#echo "FIRMWARE=${FIRMWARE}" > /dev/ttyS1
#echo "SUBSYSTEM=${SUBSYSTEM}" > /dev/ttyS1
#echo "MAJOR=${MAJOR}" > /dev/ttyS1
#echo "MINOR=${MINOR}" > /dev/ttyS1

if [ "${SUBSYSTEM}" = "block" ] ; then
    type="b"
    name="`basename ${DEVPATH}`"
    export MDEV="${name}"
    dev_name="/dev/${name}"
    action="${ACTION}"
    if [ "${action}" = "add" ] ; then
        mknod ${dev_name} ${type} ${MAJOR} ${MINOR}
        echo added ${dev_name} ${type} ${MAJOR} ${MINOR} > /dev/ttyS1
        sleep 3
        mkdir -p /media/${name}
        echo mount ${dev_name} /media/${name} > /dev/ttyS1
        mount ${dev_name} /media/${name}
        mount | grep media > /dev/ttyS1
    else
        if [ "${action}" = "remove" ] ; then
            if [ -e ${dev_name} ] ; then
                echo found ${dev_name} > /dev/ttyS1
                umount /media/${name}
                rm -rf /media/${name}
                sleep 1
                rm -f ${dev_name}
            fi
            echo removed ${dev_name} > /dev/ttyS1
        fi
    fi
else
    exit 0
fi

