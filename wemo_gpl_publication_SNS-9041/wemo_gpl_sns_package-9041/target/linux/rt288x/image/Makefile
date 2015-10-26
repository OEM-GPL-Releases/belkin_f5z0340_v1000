# 
# Copyright (C) 2006 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

ifeq ($(filter y, $(CONFIG_TARGET_rt288x_WemoInsightCR) $(CONFIG_TARGET_rt288x_WemoInsight)),)
	KERNEL_PART_SIZE=1M
else
	KERNEL_PART_SIZE=1280K
endif

define Image/BuildKernel
	$(STAGING_DIR_HOST)/bin/lzma e $(KDIR)/vmlinux $(KDIR)/vmlinux.lzma
	mkimage -A mips -O linux -T kernel -a 0x80000000 -C lzma \
		-e $(shell readelf -h $(KDIR)/vmlinux.elf | grep "Entry" | awk '{print $$4}') \
		-n 'MIPS OpenWrt Linux-$(LINUX_VERSION)' \
		-d $(KDIR)/vmlinux.lzma $(KDIR)/uImage
	cp $(KDIR)/uImage $(BIN_DIR)/openwrt-$(BOARD)-uImage
endef

#  FIXME  !!
#  When I tried to use 4k for the block size mkfs.jffs2 complained:
#  "Erase size 0x1000 too small. Increasing to 8KiB minimum"
#  This should be revisited when time allows.  For now use 64k to match
#  prior usage.
#  FIXME  !!
#WNC0_tsunghao Thu 25 Jul 2013 02:06:00 PM CST, removed redundant DEAD 4 bytes
#define Image/Build/squashfs
#  ( \
#    dd if=$(KDIR)/uImage bs=$(KERNEL_PART_SIZE) conv=sync; \
#    dd if=$(KDIR)/root.$(1) bs=64k conv=sync; \
#    echo -ne '\xde\xad\xc0\xde'; \
#  ) > $(BIN_DIR)/openwrt-$(BOARD)-$(1).img
#endef
define Image/Build/squashfs
  ( \
    dd if=$(KDIR)/uImage bs=$(KERNEL_PART_SIZE) conv=sync; \
    dd if=$(KDIR)/root.$(1) bs=64k conv=sync; \
  ) > $(BIN_DIR)/openwrt-$(BOARD)-$(1).img
endef
#WNC0

define Image/Build
	$(call Image/Build/$(1),$(1))
endef

$(eval $(call BuildImage))

