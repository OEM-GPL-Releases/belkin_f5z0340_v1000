# File to hold Makefile utilities.
# Currently only holds desuffix.  Feel free to add others.

# Use sed to strip the suffix from a variable.  Variable value is
# modified.
# Argument 1 is variable to modify
# Argument 2 is one or more suffixes to remove
define desuffix
  $(foreach suffix, $(2), \
    $(eval $(1) := $(shell echo "$($(1))" | sed "s/$(suffix)$$//")) \
   )
endef
