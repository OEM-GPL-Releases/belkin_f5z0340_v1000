#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoLink
  NAME := Ralink RT288x based WemoLink module
  PACKAGES := WeMo_Link
endef

define Profile/WemoLink/Config
	select PACKAGE_WeMo_Link
endef

define Profile/WemoLink/Description
        Package set supporting WemoLink produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoLink))
