TOP_DIR = ..
sinclude $(TOP_DIR)/tools/w800/conf.mk

EXCLUDES = wm_slave_spi_demo.c wm_fatfs_demo.c wm_sdio_host_demo.c

ifndef PDIR
GEN_LIBS = libdemo$(LIB_EXT)
COMPONENTS_libdemo = console/libconsole$(LIB_EXT)
endif

#DEFINES +=

sinclude $(TOP_DIR)/tools/w800/rules.mk
INCLUDES := $(INCLUDES) -I $(PDIR)include
PDIR := ../$(PDIR)
sinclude $(PDIR)Makefile