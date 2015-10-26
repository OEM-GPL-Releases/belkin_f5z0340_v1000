#
# Makefile
#
# Copyright (c) 2012-2014 Belkin International, Inc. and/or its affiliates.
# All rights reserved.
#
# Permission to use, copy, modify, and/or distribute this software for any 
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
#
#
# THE SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
# INCIDENTAL, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER 
# RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
# NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH
# THE USE OR PERFORMANCE OF THIS SOFTWARE.

include $(TOPDIR)/rules.mk

PKG_NAME:=NatClient
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/belkin.mk

define NATCLIENT
$$(OUTPUT): $$(OBJS)
	$$(CC) -o $$@ $$^ $$(LDFLAGS)

clean:
	rm -f $$(OUTPUT) $$(OBJS)
endef

export NATCLIENT
export OUTPUT := natClient
export OBJS := $(patsubst %.c,%.o,$(notdir $(wildcard src/*.cpp) $(wildcard src/*.c)))
export VPATH := $(CURDIR)/src

TARGET_CFLAGS +=  -I$(STAGING_DIR)/usr/include/upnp
ifeq ($(CONFIG_PACKAGE_netcam_nvram),y)
TARGET_LDFLAGS += -lnvram -lnetcam_profile -lWeMoNetCamAPI
TARGET_LDFLAGS += -lAddlInfra -lNetworkControl -lDeviceControl -lRemoteAccess
TARGET_LDFLAGS += -lWemoDB -lUPnPHndlr -lPluginHndlr
TARGET_LDFLAGS += -lupnp -lthreadutil -lixml -lpthread -lssl -lcrypto
TARGET_LDFLAGS += -lcurl -lcares -lm
TARGET_LDFLAGS += -lmxml -lsqlite3
TARGET_LDFLAGS += -lpmortem
else
TARGET_LDFLAGS += -lbelkin_nvram
endif
TARGET_LDFLAGS += -lbelkin_api
TARGET_LDFLAGS += -lAddlInfra
TARGET_LDFLAGS += -lthreadutil -lixml -lmxml -lpthread
TARGET_LDFLAGS += -lssl -lcrypto -lcares -lcurl -lz
TARGET_LDFLAGS += -lpjnath-pc-linux-gnu -lpjlib-util-pc-linux-gnu
TARGET_LDFLAGS += -lpj-pc-linux-gnu -luuid
ifeq ($(CONFIG_TARGET_rt288x_WemoLink),y)
TARGET_LDFLAGS += -lAddlInfra -lNetworkControl -lDeviceControl -lRemoteAccess
TARGET_LDFLAGS += -lsysevent -lrt -lulog -lUPnPHndlr -lsubdevice -lupnp  -lPluginHndlr
TARGET_LDFLAGS += -lzigbee_api -lWemoDB -lsqlite3 -lpmortem
TARGET_LDFLAGS += -lcurl -lcares -lm
endif
TARGET_LDFLAGS += -lpmortem

EXTRA_CFLAGS += -D_REENTRANT -DTHREADSAFE=1 -D__RINIT_M
EXTRA_CFLAGS += -DPJ_IS_LITTLE_ENDIAN=1 -DPJ_IS_BIG_ENDIAN=0

define Package/NatClient
  CATEGORY := Belkin
  SUBMENU  := Apps
  SECTION  := utils
  TITLE    := NAT client App
ifeq ($(CONFIG_PACKAGE_netcam_nvram),y)
  DEPENDS  += WeMo_NetCam
endif
ifeq ($(CONFIG_TARGET_rt288x_WemoLink),y)
  DEPENDS  += +sysevent +UPnP +subdevice
endif
  DEPENDS  += +AddlInfra
  DEPENDS  += +libopenssl +curl +libcares
  DEPENDS  += +libpjnath +libuuid
endef

define Package/NatClient/description
	WeMo NAT client application
endef

define Build/Prepare
	$(INSTALL_DIR) $(PKG_BUILD_DIR)
	echo "$$$$NATCLIENT" > $(PKG_BUILD_DIR)/Makefile
endef

define Build/Compile
	$(MAKE_VARS) $(MAKE) -C $(PKG_BUILD_DIR)/$(MAKE_PATH) $(MAKE_FLAGS)
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) src/natClient.h $(1)/usr/include
	$(CP) src/turn_wrapper.h $(1)/usr/include
endef

define Package/NatClient/install
	$(INSTALL_DIR) $(1)/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/$(OUTPUT) $(1)/sbin/
endef

$(eval $(call BuildPackage,NatClient))
