TOP_DIR = ../../../..
sinclude $(TOP_DIR)/tools/w800/conf.mk

EXCLUDES = aesMatrix.c
CSRCS = $(filter-out $(EXCLUDES), $(wildcard *.c))

ifndef PDIR
GEN_LIBS = libsymmetric$(LIB_EXT)
endif

#DEFINES +=

sinclude $(TOP_DIR)/tools/w800/rules.mk
INCLUDES := $(INCLUDES) -I $(PDIR)include
PDIR := ../$(PDIR)
sinclude $(PDIR)Makefile