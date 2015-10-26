#!/bin/sh
#
# $Id: //WIFI_SOC/release/SDK_4_1_0_0/source/user/rt2880_app/scripts/ntp.sh#1 $
#
# usage: ntp.sh
#

srv=`uci get system.@rdate[0].server`
sync=`uci get system.@rdate[0].synctime`
tz=`uci get system.@system[0].timezone`


killall -q ntpclient

if [ "$srv" = "" ]; then
	exit 0
fi


#if [ "$sync" = "" ]; then
#	sync=1
#elif [ $sync -lt 300 -o $sync -le 0 ]; then
#	sync=1
#fi

sync=`expr $sync \* 3600`

if [ "$tz" = "" ]; then
	tz="UCT_000"
fi

#debug
echo "serv=$srv"
echo "sync=$sync"
echo "tz=$tz"

echo $tz > /etc/tmpTZ
sed -e 's#.*_\(-*\)0*\(.*\)#GMT-\1\2#' /etc/tmpTZ > /etc/tmpTZ2
sed -e 's#\(.*\)--\(.*\)#\1\2#' /etc/tmpTZ2 > /etc/TZ
rm -rf /etc/tmpTZ
rm -rf /etc/tmpTZ2
ntpclient -s -c 0 -h $srv -i $sync &