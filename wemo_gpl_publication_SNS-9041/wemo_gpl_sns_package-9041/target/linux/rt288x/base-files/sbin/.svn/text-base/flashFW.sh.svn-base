FIRMWARE=$1
PARTITION=$2
CURR_PARTITION=`fw_printenv | grep bootstate | cut -d '=' -f 2`
if [ "$PARTITION" = "Firmware_1" ]; then
    echo "Flashing $FIRMWARE into partition Firmware_1..."
    mtd write $FIRMWARE Firmware_1
    fw_setenv bootstate 1
    fw_setenv check_boot 0
elif [ "$PARTITION" = "Firmware_2" ]; then                                                      
    echo "Flashing $FIRMWARE into partition Firmware_2..."
    mtd write $FIRMWARE Firmware_2
    fw_setenv bootstate 3
    fw_setenv check_boot 0
elif [ "$PARTITION" = "" ]; then
    if [ "$CURR_PARTITION" = "0" ]; then
        echo "Flashing $FIRMWARE into partition Firmware_2..."
        mtd write $FIRMWARE Firmware_2
        fw_setenv bootstate 3
        fw_setenv check_boot 0
    elif [ "$CURR_PARTITION" = "2" ]; then
        echo "Flashing $FIRMWARE into partition Firmware_1..."
        mtd write $FIRMWARE Firmware_1
        fw_setenv bootstate 1
        fw_setenv check_boot 0
    else
        echo ERROR! CURR_PARTITION=$CURR_PARTITION
    fi
else
    echo Usage: flashFW [firmware name] [partition name]
fi