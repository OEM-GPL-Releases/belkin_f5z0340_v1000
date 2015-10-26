#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoSNS
  NAME := Ralink RT288x based WemoSNS module
  PACKAGES := WeMo_SNS
endef

define Profile/WemoSNS/Config
	select PACKAGE_WeMo_SNS
endef

define Profile/WemoSNS/Description
        Package set supporting WemoSNS produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoSNS))
