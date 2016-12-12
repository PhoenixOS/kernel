DRIVER_NAME := hdmi_lpe_audio

ccflags-y += -Idrivers/gpu/drm/i915

$(DRIVER_NAME)-objs += \
	intel_hdmi_lpe_audio.o

obj-$(CONFIG_HDMI_LPE_AUDIO) += $(DRIVER_NAME).o
