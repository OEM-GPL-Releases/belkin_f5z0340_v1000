#!/bin/sh

mii_mgr -s -p 0 -r 0 -v 0x3900
mii_mgr -s -p 1 -r 0 -v 0x3900
mii_mgr -s -p 2 -r 0 -v 0x3900
mii_mgr -s -p 3 -r 0 -v 0x3900
mii_mgr -s -p 4 -r 0 -v 0x3900

val=`df | grep "mini_fo:/overlay" | awk '{print $1}'`
counter=0
while [ "$val" != "mini_fo:/overlay" ] && [ "$counter" -lt 30 ]
do
	sleep 1
	counter=`expr $counter + 1`
	val=`df | grep "mini_fo:/overlay" | awk '{print $1}'`
done

while true; do
    /sbin/natClient &>/dev/console &
    /sbin/wemoApp -webdir /tmp/Belkin_settings/ &>/dev/console
    killall natClient
    killall wemoApp
    if [ "$(nvram get SAVE_MULTIPLE_LOGS)" = "{ NULL String }" -o "$(nvram get SAVE_MULTIPLE_LOGS)" = "" ]; then
	    rm -f /tmp/messages-*
    fi
    cp /var/log/messages /tmp/messages-$(date +%Y-%m-%d-%H:%M)
    sleep 2
done
