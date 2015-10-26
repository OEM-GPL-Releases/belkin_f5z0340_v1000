#
# Copyright (C) 2007-2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/WemoInsightCR
  NAME := Ralink RT288x based WemoInsight Cost Reduction module
  PACKAGES := WeMo_Insight
endef

define Profile/WemoInsightCR/Config
	select PACKAGE_WeMo_Insight
endef

define Profile/WemoInsightCR/Description
        Package set supporting WemoInsight Cost Reduction produced based on Ralink RT3883 (and RT3662)
endef
$(eval $(call Profile,WemoInsightCR))
