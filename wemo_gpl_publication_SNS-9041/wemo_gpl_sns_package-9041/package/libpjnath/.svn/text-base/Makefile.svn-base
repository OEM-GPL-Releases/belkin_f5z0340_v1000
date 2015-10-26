#
# Copyright (C) 2006-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=pjproject
PKG_VERSION:=1.8.10
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=http://www.pjsip.org/release/$(PKG_VERSION)/
PKG_MD5SUM:=c72db8c4794e8d236f8ec0f9b847ec19

PKG_FIXUP:=libtool
PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

#TARGET_CFLAGS :=-O2
#TARGET_LDFLAGS += -lgcc_s
TARGET_CONFIGURE_OPTS += LD=$(TARGET_CC)

define Package/libpjnath/Default
  SECTION:=libs
  CATEGORY:=Libraries
  URL:=http://www.pjsip.org/
  DEPENDS += +libopenssl
endef

define Package/libpjnath
  $(call Package/libpjnath/Default)
  TITLE:=PJNATH NAT Traversal Helper Library
endef

define Package/libpjnath/description
  Open Source NAT Traversal Helper Library
  PJNATH is a new library, for helping the applications with NAT traversal. 
  It implements the latest specification of Session Traversal Utilities for 
  NAT (STUN), Obtaining Relay Addresses from STUN (TURN), and 
  Interactive Connectivity Establishment (ICE). 
endef

define Build/Configure
	$(INSTALL_DIR) $(PKG_BUILD_DIR)
	echo "Handle Fragmentation...$(HANDLE_FRAG)";
ifeq ($(CONFIG_NETCAM_NetCam),y)
	  $(CP) files/user.mak.HDv1 $(PKG_BUILD_DIR)/user.mak
	  $(CP) files/ip_helper_generic.c $(PKG_BUILD_DIR)/pjlib/src/pj/ip_header_generic.c
	  $(CP) files/os_core_unix.c $(PKG_BUILD_DIR)/pjlib/src/pj/os_core_unix.c
endif
ifeq ($(CONFIG_NETCAM_HD_V1),y)
	  $(CP) files/user.mak.HDv1 $(PKG_BUILD_DIR)/user.mak
	  $(CP) files/ip_helper_generic.c $(PKG_BUILD_DIR)/pjlib/src/pj/ip_header_generic.c
	  $(CP) files/os_core_unix.c $(PKG_BUILD_DIR)/pjlib/src/pj/os_core_unix.c
endif
ifeq ($(CONFIG_NETCAM_HD_V2),y)
	  $(CP) files/user.mak.HDv2 $(PKG_BUILD_DIR)/user.mak
	  $(CP) files/ip_helper_generic.c $(PKG_BUILD_DIR)/pjlib/src/pj/ip_header_generic.c
	  $(CP) files/os_core_unix.c $(PKG_BUILD_DIR)/pjlib/src/pj/os_core_unix.c
endif
ifeq ($(CONFIG_NETCAM_LS),y)
	  $(CP) files/user.mak.HDv2 $(PKG_BUILD_DIR)/user.mak
	  $(CP) files/ip_helper_generic.c $(PKG_BUILD_DIR)/pjlib/src/pj/ip_header_generic.c
	  $(CP) files/os_core_unix.c $(PKG_BUILD_DIR)/pjlib/src/pj/os_core_unix.c
endif
	$(CP) files/ip_helper_generic.c $(PKG_BUILD_DIR)/pjnath/src/
	$(CP) files/turn_session.c $(PKG_BUILD_DIR)/pjnath/src/pjnath/turn_session.c; \
	$(call Build/Configure/Default,)
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/* $(1)/usr/include
	$(INSTALL_DIR) $(1)/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib $(1)/usr/lib/
endef

define Package/libpjnath/install
endef

$(eval $(call BuildPackage,libpjnath))
