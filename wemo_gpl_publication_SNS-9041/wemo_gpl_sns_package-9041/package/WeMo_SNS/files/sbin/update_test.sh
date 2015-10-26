#!/bin/sh

if [ $1 -eq 0 ]; then
   echo "Sleeping to test for successful startup after update"
   # give time for the update script to finish up then take over 
   # /tmp/update.log
   sleep 10
   sh -x $0 1 >> /tmp/update.log 2>&1
   exit $?
fi

restore=0
# Give time for the components to start running and erase their
# flag files.  If any of flag files still exist then the component
# failed to startup or failed its internal health checks, roll back
# 
sleep 60
for file in /tmp/*.starting; do
   if [ -f $file ]; then
      echo "Error: $file is still present"
      restore=1
   fi
done


if [ $restore == 1 ]; then
   echo "will be changed boot partition to alternate partition by uboot"
   rm /update.errs
   rm /update.status
   reboot
else
	boot_state=`fw_printenv -n bootstate`
	if [ "${boot_state}x" == "1x" ]; then
		fw_setenv bootstate 0
	fi
	if [ "${boot_state}x" == "3x" ]; then
		fw_setenv bootstate 2
	fi
	fw_setenv check_boot 0
fi

# the update either worked or we've rolled it back, delete the
# backups
rm -rf /update_tmp
rm -f /tmp/update.log
echo "update_test.sh exiting"
