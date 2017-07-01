# parts of build/core/tasks/kernel.mk

WL_ENABLED := $(if $(wildcard $(WL_PATH)),$(shell grep ^CONFIG_WL=[my] $(KERNEL_CONFIG_FILE)))
WL_SRC := $(WL_PATH)/hybrid-v35$(if $(filter x86,$(TARGET_KERNEL_ARCH)),,_64)-nodebug-pcoem-6_30_223_271.tar.gz
WL_LIB := $(WL_PATH)/lib$(if $(filter x86,$(TARGET_KERNEL_ARCH)),32,64)
$(WL_SRC):
	@echo Downloading $(@F)...
	$(hide) curl -k https://docs.broadcom.com/docs-and-downloads/docs/linux_sta/$(@F) > $@

$(WL_LIB): $(WL_SRC) $(wildcard $(WL_PATH)/*.patch)
	$(hide) tar zxf $< -C $(@D) --overwrite -m && \
		rm -rf $@ && mv $(@D)/lib $@ && touch $@ && \
		patch -p1 -d $(@D) -i wl.patch && \
		patch -p1 -d $(@D) -i linux-recent.patch && \
		patch -p1 -d $(@D) -i linux-48.patch

$(INSTALLED_KERNEL_TARGET): $(if $(WL_ENABLED),$(WL_LIB))
