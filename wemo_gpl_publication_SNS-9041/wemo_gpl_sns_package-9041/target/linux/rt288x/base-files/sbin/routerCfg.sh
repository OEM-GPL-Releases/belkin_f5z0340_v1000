#!/bin/sh
uci set dhcp.lan.ignore=0
uci set dhcp.lan.start=100
uci set dhcp.lan.limit=150
uci set dhcp.lan.leasetime=12h
uci set network.lan.proto=static
uci set network.lan.ipaddr=10.22.22.1
uci set network.lan.netmask=255.255.255.0
uci set wireless.@wifi-iface[0].network=lan
uci commit
/etc/init.d/network restart
