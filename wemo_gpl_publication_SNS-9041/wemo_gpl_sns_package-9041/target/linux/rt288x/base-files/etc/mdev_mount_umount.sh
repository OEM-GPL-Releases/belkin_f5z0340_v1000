#!/bin/sh

device_mount_max_retry=3
device_mount_delay_sec=1
device="/dev/$MDEV"
dir="/media/$MDEV"

case "$ACTION" in
	add|"")
		echo "Add $device from mdev" >> /dev/console
		mkdir -p $dir
		/etc/force_mount_media.sh $MDEV $device_mount_max_retry $device_mount_delay_sec
		/etc/check_mount_media.sh $MDEV
		;;
	remove)
		echo "Remove $device from mdev" >> /dev/console
		umount $dir
		sleep  $device_mount_delay_sec
		rm -rf $dir
		;;
esac

