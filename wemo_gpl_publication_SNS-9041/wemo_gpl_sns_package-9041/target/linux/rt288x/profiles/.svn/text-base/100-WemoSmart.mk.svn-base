#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoSmart
  NAME := Ralink RT288x based WemoSmart module
  PACKAGES := WeMo_Smart
endef

define Profile/WemoSmart/Config
	select PACKAGE_WeMo_Smart
	select PACKAGE_wasp
endef

define Profile/WemoSmart/Description
        Package set supporting WemoSmart produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoSmart))
