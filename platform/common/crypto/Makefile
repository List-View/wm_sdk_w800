TOP_DIR = ../../..
sinclude $(TOP_DIR)/tools/w800/conf.mk

ifndef PDIR
GEN_LIBS = libcrypto$(LIB_EXT)
COMPONENTS_libcrypto = digest/libdigest$(LIB_EXT) 	\
					   keyformat/libkeyformat$(LIB_EXT)	\
					   math/libmath$(LIB_EXT)	\
					   prng/libprng$(LIB_EXT)	\
					   pubkey/libpubkey$(LIB_EXT)	\
					   symmetric/libsymmetric$(LIB_EXT)
endif

#DEFINES +=

sinclude $(TOP_DIR)/tools/w800/rules.mk
INCLUDES := $(INCLUDES) -I ./ -I $(PDIR)include
PDIR := ../$(PDIR)
sinclude $(PDIR)Makefile