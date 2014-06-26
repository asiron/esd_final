# Target specific macros
TOPPERS_KERNEL = OSEK_COM
TARGET = esd_final
TARGET_SOURCES = \
	esd_final.c
TOPPERS_OSEK_OIL_SOURCE = ./esd_final.oil

# Don't modify below part
O_PATH ?= build
include ../../ecrobot/ecrobot.mak