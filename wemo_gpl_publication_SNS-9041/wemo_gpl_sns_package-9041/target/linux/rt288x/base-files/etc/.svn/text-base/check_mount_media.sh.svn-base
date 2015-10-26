#!/bin/sh

mounted=`mount | grep "$1 " | wc -l`
#echo "/media/$1 mounted=$mounted" >> /dev/console

if [ $mounted == "1" ]; then
     mount | grep $1 >> /dev/console
     df -h | grep $1 >> /dev/console
else
     echo $1 not mounted! >> /dev/console
     rm -rf /media/$1
fi

