# Wrapper around OpenWRT makefile to take care of building multiple
# products based on target profiles.


# The wemo_target variable is used to generate target definitions,
# theoretically reducing cut-and-paste errors.  Adding a new WeMo
# product should be as easy as adding a line to the "help" target and
# adding a $(eval $(call wemo_target,product-name)) near the bottom of
# the file.

define wemo_target

.PHONY: $(1) $(1)-config $(1)-prod $(1)-diag
$(1)-config:
	if [ -f .config ]; then rm -r .config; fi
	if [ -d tmp ]; then rm -rf tmp; fi
	if [ -d bin ]; then rm -rf bin; fi
	./scripts/kconfig.pl + configs/rt288x configs/$1 > .config

$(1): $(1)-config
	make -f Makefile defconfig
	@sed -i '/# CONFIG_PACKAGE_tcpdump is not set/c\CONFIG_PACKAGE_tcpdump=y' .config
	@sed -i '/# CONFIG_PACKAGE_libpcap is not set/c\CONFIG_PACKAGE_libpcap=y' .config
	@sed -i '/# CONFIG_PACKAGE_strace is not set/c\CONFIG_PACKAGE_strace=y' .config
	@sed -i '/# CONFIG_PACKAGE_pmortemd is not set/c\CONFIG_PACKAGE_pmortemd=y' .config
	make -f Makefile specific_clean	
	make world
	make image

$(1)-prod: $(1)-config
	@echo CONFIG_WEMO_PRODUCTION_IMAGE=y >> .config
	@echo CONFIG_BUSYBOX_CONFIG_GETTY=y >> .config
	@echo CONFIG_BUSYBOX_CONFIG_LOGIN=y >> .config
	@sed -i '/CONFIG_BUILD_SUFFIX=/s/"\([^=]\+\)"/"\1_prod"/' .config
	@sed -i '/# CONFIG_PACKAGE_pmortemd is not set/c\CONFIG_PACKAGE_pmortemd=y' .config
	make -f Makefile defconfig
	@sed -i '/dropbear.*=/s/\([^=]\+\).*/# \1 is not set/' .config
	@sed -i '/iperf.*=/s/\([^=]\+\).*/# \1 is not set/' .config
	@sed -i '/uclibcxx.*=/s/\([^=]\+\).*/# \1 is not set/' .config
	make -f Makefile specific_clean
	make world
	make image

# Make diagnostic instrumented image.  Enables building of dmalloc
# library & utility.  Also causes setting of DMALLOC compiler symbol
# so dmalloc-conditional options are enabled.
$(1)-diag: $(1)-config
	@echo CONFIG_PACKAGE_belkin_diag=y >> .config
	@echo CONFIG_PACKAGE_dmalloc-utils=y >> .config
	@echo CONFIG_PACKAGE_libdmalloc=y >> .config
	@echo CONFIG_PACKAGE_dmtest=y >> .config
	@sed -i '/CONFIG_BUILD_SUFFIX=/s/"\([^=]\+\)"/"\1_diag"/' .config
	make -f Makefile defconfig
	@echo CONFIG_WEMO_DMALLOC=y >> .config
	@echo CONFIG_WEMO_DIAG=y >> .config
	make world
	make image
endef

all: .config
	make -f Makefile world

.config:
	@echo Make what? >&2
	@make -s help
	@exit 1

help:
	@echo make targets:
	@echo - smart - Smart module
	@echo - jarden - Jarden module \(deprecated\)
	@echo - link - Link\(LEDLight\) module
	@echo - sns - SNS module
	@echo - insight - Insight module
	@echo - light - Light Switch module
	@echo - insightCR - Insight Cost Reduction 
	@echo - netcam - NetCam module
	@echo
	@echo In addition to the above targets you can append "-prod" to
	@echo the name to build a production image or "-diag" to build a
	@echo diagnotic image.  For example, '"make smart-prod"' will
	@echo make a production version of the smart module image.

.PHONY: world image
world:
	make -f Makefile world

image:
	make -f tools/belkin/image.mk TOPDIR=$(shell pwd)

clean:
	make -C scripts/config clean
	make -f Makefile clean
	rm -rf staging_dir tmp

veryclean:
	make -C scripts/config clean
	rm -rf .config build_dir staging_dir tmp bin output

$(eval $(call wemo_target,smart))
$(eval $(call wemo_target,jarden))
$(eval $(call wemo_target,link))
$(eval $(call wemo_target,sns))
$(eval $(call wemo_target,insight))
$(eval $(call wemo_target,light))
$(eval $(call wemo_target,insightCR))
$(eval $(call wemo_target,netcam))

# The default target (%:) is what makes this file a wrapper.  The
# OperWRT build environment generates hundreds of targets.  The rule
# below delegates the processing of all those targets back into the
# OpenWRT build system.

%:
	make -f Makefile $@

.PHONY: help smart light report
