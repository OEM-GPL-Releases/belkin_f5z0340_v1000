#
# Copyright (C) 2006 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=belkin_tool
PKG_VERSION:=1

include $(INCLUDE_DIR)/host-build.mk

define Host/Configure
	$(call Host/Configure/Default,\
		--datarootdir=$(STAGING_DIR_HOST)/share \
	)
endef

define Host/Compile
	$(CP) Belkin_Plug-Ins_public_key.asc $(HOST_BUILD_DIR)/. 
	$(CP) fw_crc_hdr $(HOST_BUILD_DIR)/. 
	$(CP) makeImage $(HOST_BUILD_DIR)/. 
endef

define Host/Install
	$(INSTALL_DIR) $(STAGING_DIR_HOST)/bin/belkin
	$(CP) $(HOST_BUILD_DIR)/Belkin_Plug-Ins_public_key.asc $(STAGING_DIR_HOST)/bin/belkin//. 
	$(CP) $(HOST_BUILD_DIR)/fw_crc_hdr $(STAGING_DIR_HOST)/bin/belkin//. 
	$(CP) $(HOST_BUILD_DIR)/makeImage $(STAGING_DIR_HOST)/bin/belkin//. 
endef

define Host/Clean
	$(call Host/Clean/Default)
endef

$(eval $(call HostBuild))
