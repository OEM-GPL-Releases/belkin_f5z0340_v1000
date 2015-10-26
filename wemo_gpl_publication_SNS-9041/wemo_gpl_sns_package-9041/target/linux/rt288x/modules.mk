define KernelPackage/rt_rdm
  SUBMENU:=$(NETWORK_DEVICES_MENU)
  TITLE:=Ralink Reg Debug Module
  DEPENDS:=@TARGET_rt288x
  KCONFIG:=CONFIG_RALINK_RDM
  FILES:=$(LINUX_DIR)/drivers/net/rt_rdm/rt_rdm.$(LINUX_KMOD_SUFFIX)
  AUTOLOAD:=$(call AutoLoad,50,rt_rdm)
endef

define KernelPackage/rt_rdm/description
 Ralink Reg Debug Module
endef

$(eval $(call KernelPackage,rt_rdm))

define KernelPackage/raeth
  SUBMENU:=$(NETWORK_DEVICES_MENU)
  TITLE:=Ralink ethernet support
  DEPENDS:=@TARGET_rt288x
  KCONFIG:=CONFIG_RAETH
  FILES:=$(LINUX_DIR)/drivers/net/raeth/raeth.$(LINUX_KMOD_SUFFIX)
  AUTOLOAD:=$(call AutoLoad,50,raeth)
endef

define KernelPackage/raeth/description
 Kernel support for Ralink ethernet support
endef

$(eval $(call KernelPackage,raeth))

define KernelPackage/rt2860v2ap
  SUBMENU:=$(WIRELESS_MENU)
  TITLE:=Ralink RT2860 802.11n AP support
  DEPENDS:=@TARGET_rt288x
  KCONFIG:=CONFIG_RT2860V2_AP
  FILES:=$(LINUX_DIR)/drivers/net/wireless/rt2860v2_ap/rt2860v2_ap.$(LINUX_KMOD_SUFFIX)
  AUTOLOAD:=$(call AutoLoad,50,rt2860v2_ap)
endef

define KernelPackage/rt2860v2ap/description
 Kernel support for Ralink RT2860 802.11n AP support
endef

$(eval $(call KernelPackage,rt2860v2ap))

define KernelPackage/ralink_i2c
  SUBMENU:=I2C support
  TITLE:=Ralink RT2880 I2C Support
  DEPENDS:=@TARGET_rt288x
  KCONFIG:=CONFIG_RALINK_I2C
  FILES:=$(LINUX_DIR)/drivers/char/i2c_drv.$(LINUX_KMOD_SUFFIX)
  AUTOLOAD:=$(call AutoLoad,50,i2c_drv)
endef

define KernelPackage/ralink_i2c/description
 Kernel support for Ralink RT2880 I2C
endef

$(eval $(call KernelPackage,ralink_i2c))

