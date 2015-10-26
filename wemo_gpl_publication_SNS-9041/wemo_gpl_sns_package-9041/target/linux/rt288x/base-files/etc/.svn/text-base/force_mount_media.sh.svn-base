#!/bin/sh

mounted=`mount | grep "$1 " | wc -l`
num=$2
while [ $mounted -lt 1 -a $num -gt 0 ]
do
     sleep $3
     mount "/dev/$1" "/media/$1"
     mounted=`mount | grep "$1 " | wc -l`
     num=`expr $num - 1`
done

