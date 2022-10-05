#
# Makefile for the usb FUSB302 driver (if driver is inside kernel tree).
#

ccflags-$(CONFIG_USB_FUSB302) := -DFSC_PLATFORM_LINUX -DFSC_HAVE_SNK -DFSC_HAVE_SRC -DFSC_HAVE_DRP \
-DFSC_HAVE_ACCMODE -DFSC_HAVE_VDM -DFSC_HAVE_DP -DFSC_DEBUG
ccflags-$(CONFIG_USB_FUSB302) += -DFSC_INTERRUPT_TRIGGERED -DFSC_ILLEGAL_CABLE

obj-$(CONFIG_USB_FUSB302) += fusb30x_whole.o

fusb30x_whole-objs  := Platform_Linux/fusb30x_driver.o \
                    Platform_Linux/fusb30x_global.o \
                    Platform_Linux/platform.o \
			        Platform_Linux/platform_helpers.o \
			        Platform_Linux/hostcomm.o \
			        core/core.o \
			        core/fusb30X.o \
			        core/Log.o \
			        core/PDPolicy.o \
			        core/PDProtocol.o \
			        core/TypeC.o \
			        core/vdm/bitfield_translators.o \
			        core/vdm/vdm.o \
			        core/vdm/vdm_callbacks.o \
			        core/vdm/vdm_config.o \
			        core/vdm/DisplayPort/configure.o \
			        core/vdm/DisplayPort/dp.o \
			        core/vdm/DisplayPort/dp_system_stubs.o
