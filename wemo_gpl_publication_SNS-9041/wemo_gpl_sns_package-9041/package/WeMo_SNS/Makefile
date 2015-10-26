include $(TOPDIR)/rules.mk

PKG_NAME:=WeMo_SNS
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/belkin.mk

define Package/WeMo_SNS
  CATEGORY := Belkin
  SUBMENU  := Apps
  SECTION  := utils
  TITLE    := WeMo SNS App
  DEPENDS  += +libcares +libuuid +libupnp +libpjnath +libmxml +sqlite +NatClient
  DEPENDS  += +gnupg
  KCONFIG  += CONFIG_WEMO_FLASH_OPTION_1=y
  KCONFIG  += CONFIG_MTD_ROOTFS_ROOT_DEV=y
  KCONFIG  += CONFIG_MTD_ROOTFS_SPLIT=y
  KCONFIG  += CONFIG_ROOTFS_IN_FLASH_NO_PADDING=y
  KCONFIG  += CONFIG_RT2880_ROOTFS_IN_FLASH=y
  KCONFIG  += CONFIG_RALINK_GPIO_SNS=y
  KCONFIG  += CONFIG_RF_OFFSET=0x00040000
  KCONFIG  += CONFIG_NVRAM_OFFSET=0x00fe0000
endef

define Package/WeMo_SNS/description
	WeMo SNS primary application
endef

define Build/Prepare
endef

define Build/Compile
endef

define Package/WeMo_SNS/install
	$(CP) ./files/* $(1)/
	rm -rf $(1)/usr/include
endef

$(eval $(call BuildPackage,WeMo_SNS))

