#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoInsight
  NAME := Ralink RT288x based WemoInsight module
  PACKAGES := WeMo_Insight
endef

define Profile/WemoInsight/Config
	select PACKAGE_WeMo_Insight
endef

define Profile/WemoInsight/Description
        Package set supporting WemoInsight produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoInsight))
