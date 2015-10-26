#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoJarden
  NAME := Ralink RT288x based WemoJarden module
  PACKAGES := WeMo_Jarden
endef

define Profile/WemoJarden/Config
	select PACKAGE_WeMo_Jarden
	select PACKAGE_wasp
endef

define Profile/WemoJarden/Description
        Package set supporting WemoJarden produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoJarden))
