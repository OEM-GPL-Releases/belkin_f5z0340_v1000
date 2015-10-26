#! /bin/sh
#

logfile=/tmp/update.log
FIRMWARE_NAME=firmware.bin
STRIP_FIRMWARE=firmware.strip

if [ ! -f $logfile ]; then
# run with logging
   sh -x $0 $1 > $logfile 2>&1
   status=$?
   cat $logfile > /dev/console
   sleep 5
   echo "Exiting with status $status" > /dev/console
   exit $status
fi

report_err() {
   echo $1 > $errfile
   echo $2 >> $errfile
   echo $3 >> $errfile
   echo "Error: $3"
}

errfile="/update.errs"

cd /tmp
rm -rf update

echo "Processing $1"
export GNUPGHOME=/root/.gnupg

# make sure the update file exists
if [ ! -f $1 ]; then
   report_err 5 2 "The specified update file '$1' does not exist."
   return 2
fi

if [ "/tmp/firmware.img" != $1 ]; then
	# make sure the update is signed
	gpg --import --ignore-time-conflict /root/.gnupg/WeMoPubKey.asc
	gpg --ignore-time-conflict -o $FIRMWARE_NAME -d $1
	status=$?
	if [ $status -ne 0 ]; then
	   report_err 1 $status "Signature test failed."
	   return $status
	fi	

	rm $1

	# if Gemtek header is existed, strip 19 bytes Gemtek header.
	GEMTEK_HDR=gemtek.hdr
	FILE_LENGTH=`cat "$FIRMWARE_NAME" | wc -c`
	FILE_LENGTH_WO_GEMTEK=`expr "$FILE_LENGTH" - 19`
	dd if="$FIRMWARE_NAME" of="$GEMTEK_HDR" bs=1 count=19 > /dev/console
	gemtek_string="`cat $GEMTEK_HDR | cut -b 6-16`"
	if [ "$gemtek_string" == "GMTKPlugIns" ]
	then
		echo "Gemtek header is founded: strip 19 bytes header"
		dd if="$FIRMWARE_NAME" of="$STRIP_FIRMWARE" skip=19 bs=1 count="$FILE_LENGTH_WO_GEMTEK" > /dev/console
		rm $FIRMWARE_NAME
		FIRMWARE_NAME=$STRIP_FIRMWARE
		FILE_LENGTH=$FILE_LENGTH_WO_GEMTEK
	fi	

	# verify belkin header and chksum _start
	BELKIN_HDR=belkin.hdr
	IMAGE_LENTGH=`expr "$FILE_LENGTH" - 256`
	dd if="$FIRMWARE_NAME" of="$BELKIN_HDR" skip="$IMAGE_LENTGH" bs=1 count=256 > /dev/console

	magic_string="`cat $BELKIN_HDR | cut -b 1-9`"
	hdr_version="`cat $BELKIN_HDR | cut -b 10-11`"
	hdr_length="`cat $BELKIN_HDR | cut -b 12-16`"

	sku_length="`cat $BELKIN_HDR | cut -b 17`"
	sku_decimal_len=`printf "%d" 0x"$sku_length"`
	sku_end=`expr 18 + $sku_decimal_len - 2`
	sku_string="`cat $BELKIN_HDR | cut -b 18-$sku_end`"

	img_cksum="`cat $BELKIN_HDR | cut -b 33-40`"
	sign_type="`cat $BELKIN_HDR | cut -b 41`"
	signer="`cat $BELKIN_HDR | cut -b 42-48`"

	kernel_ofs="`cat $BELKIN_HDR | cut -b 50-56`"
	rfs_ofs="`cat $BELKIN_HDR | cut -b 58-64`"

	if [ "$magic_string" != ".BELKIN.." ]
	then
		status=1
		report_err 3 $status  "Fail : verify magic string"
		return $status
	fi

	crc1=`dd if="$FIRMWARE_NAME" bs="$IMAGE_LENTGH" count=1| cksum | cut -d' ' -f1`
	hex_cksum=`printf "%08X" "$crc1"`
	if [ "$img_cksum" != "$hex_cksum" ]
	then
		status=1
		report_err 3 $status  "Fail : verify image checksum"
		return $status
	fi

	PRODUCT_FILE="/etc/product.name"
	if [ -e $PRODUCT_FILE ]
	then
		sku_in_rfs="`cat $PRODUCT_FILE`"
		if [ "$sku_in_rfs" != $sku_string ]
		then
			if [ "$sku_in_rfs" == "WeMo_link" -a "$sku_string" == "LEDLight" ]
			then
				echo "Continue Update Link to LEDLight ($sku_in_rfs) to ($sku_string)"			
			else
				status=1
				report_err 3 $status  "Fail : verify product name"
				return $status
			fi
		fi
	fi
	# verify belkin header and chksum _end
else
	mv $1 $FIRMWARE_NAME
fi

state_1st_updated=1
state_2nd_updated=3	

mounted_mtd=`cat /proc/mtd | grep "rootfs_data" | cut -d':' -f1 | cut -f2 -d"d"`
case "$mounted_mtd" in
	3 | 6 )
		echo "Updating 'B' image"
		mtd_other=Firmware_2
		newstate=$state_2nd_updated
		newpart=1
		;;

	5 | 8 )
		echo "Updating 'A' image"
		mtd_other=Firmware_1
		newstate=$state_1st_updated
		newpart=0
		;;

	*)
		echo "Invalid Boot partition : Updating A' image by default"
		mtd_other=Firmware_1
		newstate=$state_1st_updated
		newpart=0
		;;
esac

mtd write $FIRMWARE_NAME $mtd_other
status=$?
if [ $status -ne 0 ]; then
   echo "mtd write failed ($status)"
   return $status
fi

fw_setenv bootstate $newstate

# bootpart is changed temporaily to get backward compatibility for WNC old bootloaders (SDK_1.0.00.002 ~ 004)
fw_setenv bootpart $newpart

check_boot=`fw_printenv -n check_boot`
if [ "${check_boot}x" != "0x" ]; then
   fw_setenv check_boot 0
fi

echo $1 > Belkin_settings/last_update

echo "Rebooting..."

#reboot


echo "firmware_update.sh exiting"
return $status


