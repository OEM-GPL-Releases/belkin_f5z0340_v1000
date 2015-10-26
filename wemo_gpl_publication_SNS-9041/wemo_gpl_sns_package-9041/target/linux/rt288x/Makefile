#
# Copyright (C) 2006-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

ARCH:=mipsel
BOARD:=rt288x
BOARDNAME:=Ralink RT288x
FEATURES:=usb
DEVICE_TYPE := other

LINUX_VERSION:=2.6.21

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += dnsmasq ntpclient kmod-rt_rdm kmod-raeth kmod-rt2860v2ap

define Target/Description
	Build firmware images for Ralink based switch and sensor devices.
endef

$(eval $(call BuildTarget))
