TOP_DIR = ../../../..
sinclude $(TOP_DIR)/tools/w800/conf.mk

ifndef PDIR
GEN_LIBS = libmath$(LIB_EXT)
endif

#DEFINES +=

sinclude $(TOP_DIR)/tools/w800/rules.mk
INCLUDES += -I ./ -I $(PDIR)include

PDIR := ../$(PDIR)
sinclude $(PDIR)Makefile