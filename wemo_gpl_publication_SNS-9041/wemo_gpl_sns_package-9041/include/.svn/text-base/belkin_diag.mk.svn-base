# Things to help packages that use belkin_diag.
# Use this to simplify using the belkin_diag libs in your library or
# application.  To do this, add this to your main define like so:
# define Package/something
#   CATEGORY := who_cares
#   ....
#   TITLE    := Something
#   $(call BELKIN_DIAG_DEPENDS)
# endef
define BELKIN_DIAG_DEPENDS
  DEPENDS           += +PACKAGE_belkin_diag:belkin_diag
  PKG_BUILD_DEPENDS += +PACKAGE_belkin_diag:belkin_diag
endef

export BELKIN_DIAG_DEPENDS

define BELKIN_DIAG_PREP
	$(INSTALL_DIR) $(STAGING_DIR)/include
	$(INSTALL_DATA) ../belkin_api/src/belkin_diag.h $(STAGING_DIR)/include
endef

export BELKIN_DIAG_PREP
