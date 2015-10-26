#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoNetCam
  NAME := Ralink RT288x based WemoNetcam module
  PACKAGES := WeMo_NetCam
endef

define Profile/WemoNetCam/Config
	select PACKAGE_WeMo_NetCam
endef

define Profile/WemoNetCam/Description
        Package set supporting WemoNetcam produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoNetCam))
