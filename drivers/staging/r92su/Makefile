r92su-objs	+= 						\
	michael.o aes_ccm.o eeprom.o event.o debug.o main.o	\
	tkip.o wep.o sta.o usb.o pwr.o cmd.o fw.o rx.o hw.o tx.o\


r92su-$(CONFIG_R92SU_DEBUGFS)	+= debugfs.o
r92su-$(CONFIG_R92SU_TRACING)	+= trace.o
obj-$(CONFIG_R92SU)		+= r92su.o
