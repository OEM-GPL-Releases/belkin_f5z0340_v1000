#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoLight
  NAME := Ralink RT288x based Wemo Light Switch module
  PACKAGES := WeMo_Light
endef

define Profile/WemoLight/Config
	select PACKAGE_WeMo_Light
endef

define Profile/WemoLight/Description
        Package set supporting Wemo Light Switch produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoLight))
