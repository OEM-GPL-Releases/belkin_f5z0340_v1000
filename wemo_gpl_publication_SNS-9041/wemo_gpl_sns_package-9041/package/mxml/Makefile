#
# Copyright (C) 2006-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=mxml
PKG_VERSION:=2.9
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=http://www.msweet.org/files/project3
PKG_MD5SUM:=e21cad0f7aacd18f942aa0568a8dee19

PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

CONFIGURE_PREFIX := "$(PKG_INSTALL_DIR)/usr"
#CONFIGURE_ARGS = $(TARGET_CONFIGURE_OPTS) --prefix="$(PKG_INSTALL_DIR)"

define Package/libmxml
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE:=Mini XML library
  URL:=http://www.msweet.org/index.php
endef

define Build/Compile
	+$(MAKE) $(PKG_JOBS) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		libmxml.so.1.5
endef

define Package/libmxml/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libmxml.so* $(1)/usr/lib/
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/*.h $(1)/usr/include/

	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libmxml.so* $(1)/usr/lib/
endef

$(eval $(call BuildPackage,libmxml))
