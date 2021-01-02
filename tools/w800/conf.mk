sinclude $(TOP_DIR)/tools/w800/.config
CONFIG_W800_USE_LIB ?= n
CONFIG_W800_FIRMWARE_DEBUG ?= n
CONFIG_ARCH_TYPE ?= w800

TARGET ?= $(subst ",,$(CONFIG_W800_TARGET_NAME))

ODIR := $(TOP_DIR)/bin/build
OBJODIR := $(ODIR)/$(TARGET)/obj
UP_EXTRACT_DIR := ../../lib
LIBODIR := $(ODIR)/$(TARGET)/lib
IMAGEODIR := $(ODIR)/$(TARGET)/image
BINODIR := $(ODIR)/$(TARGET)/bin
FIRMWAREDIR := $(TOP_DIR)/bin
SDK_TOOLS := $(TOP_DIR)/tools/$(CONFIG_ARCH_TYPE)
CA_PATH := $(SDK_TOOLS)/ca
VER_TOOL ?= $(SDK_TOOLS)/wm_getver
WM_TOOL ?= $(SDK_TOOLS)/wm_tool
SEC_BOOT := $(SDK_TOOLS)/w800_secboot.img

ifeq ($(CONFIG_W800_USE_LIB),y)
USE_LIB = 1
else
USE_LIB = 0
endif

DL_PORT ?= $(subst ",,$(CONFIG_W800_DOWNLOAD_PORT))
DL_BAUD ?= $(CONFIG_W800_DOWNLOAD_RATE)

IMG_TYPE := $(CONFIG_W800_IMAGE_TYPE)
IMG_HEADER := $(CONFIG_W800_IMAGE_HEADER)
RUN_ADDRESS := $(CONFIG_W800_RUN_ADDRESS)
UPD_ADDRESS := $(CONFIG_W800_UPDATE_ADDRESS)

PRIKEY_SEL := $(CONFIG_W800_PRIKEY_SEL)
SIGNATURE := $(CONFIG_W800_IMAGE_SIGNATURE)
CODE_ENCRYPT := $(CONFIG_W800_CODE_ENCRYPT)
SIGN_PUBKEY_SRC := $(CONFIG_W800_SIGN_PUBKEY_SRC)

optimization ?= -O2

ifeq ($(CONFIG_W800_FIRMWARE_DEBUG),y)
optimization += -g
endif

# YES; NO
VERBOSE ?= NO

UNAME_O:=$(shell uname -o)
UNAME_S:=$(shell uname -s)

$(shell gcc $(SDK_TOOLS)/wm_getver.c -Wall -O2 -o $(VER_TOOL))

TOOL_CHAIN_PATH = $(subst ",,$(CONFIG_W800_TOOLCHAIN_PATH))

# select which tools to use as compiler, librarian and linker
ifeq ($(VERBOSE),YES)
    AR = $(TOOL_CHAIN_PATH)csky-elfabiv2-ar
    ASM = $(TOOL_CHAIN_PATH)csky-elfabiv2-gcc
    CC = $(TOOL_CHAIN_PATH)csky-elfabiv2-gcc
    CPP = $(TOOL_CHAIN_PATH)csky-elfabiv2-g++
    LINK = $(TOOL_CHAIN_PATH)csky-elfabiv2-ld
    OBJCOPY = $(TOOL_CHAIN_PATH)csky-elfabiv2-objcopy
    OBJDUMP = $(TOOL_CHAIN_PATH)csky-elfabiv2-objdump
else
    AR = @echo "AR $<" 2>/dev/null; $(TOOL_CHAIN_PATH)csky-elfabiv2-ar
    ASM = @echo "ASM $<"; $(TOOL_CHAIN_PATH)csky-elfabiv2-gcc
    CC = @echo "CC  $<"; $(TOOL_CHAIN_PATH)csky-elfabiv2-gcc
    CPP = @echo "CPP $<"; $(TOOL_CHAIN_PATH)csky-elfabiv2-g++
    LINK = @echo "LINK $<"; $(TOOL_CHAIN_PATH)csky-elfabiv2-ld
    OBJCOPY = @echo "OBJCOPY $<"; $(TOOL_CHAIN_PATH)csky-elfabiv2-objcopy
    OBJDUMP = @echo "OBJDUMP $<"; $(TOOL_CHAIN_PATH)csky-elfabiv2-objdump
endif

LDDIR = $(TOP_DIR)/ld/$(CONFIG_ARCH_TYPE)
LD_FILE = $(LDDIR)/gcc_csky.ld

LIB_EXT = .a

CCFLAGS := -Wall \
    -DTLS_CONFIG_CPU_XT804=1 \
    -DGCC_COMPILE=1 \
    -mcpu=ck804ef \
    $(optimization) \
    -std=gnu99 \
    -c  \
    -mhard-float  \
    -Wall  \
    -fdata-sections  \
    -ffunction-sections

ASMFLAGS := -Wall \
    -DTLS_CONFIG_CPU_XT804=1 \
    -DGCC_COMPILE=1 \
    -mcpu=ck804ef \
    $(optimization) \
    -std=gnu99 \
    -c  \
    -mhard-float \
    -Wa,--gdwarf2 \
    -fdata-sections  \
    -ffunction-sections

ARFLAGS := ru

ARFLAGS_2 = xo

LINKFLAGS := -mcpu=ck804ef \
    -nostartfiles \
    -mhard-float \
    -lm \
    -Wl,-T$(LD_FILE)

MAP := -Wl,-ckmap=$(IMAGEODIR)/$(TARGET).map

ifneq ($(PRIKEY_SEL),0)
    $(IMG_TYPE) = $(IMG_TYPE) + 32 * $(PRIKEY_SEL)
endif

ifeq ($(CODE_ENCRYPT),1)
    $(IMG_TYPE) = $(IMG_TYPE) + 16
    $(PRIKEY_SEL) = $(PRIKEY_SEL) + 1
endif

ifeq ($(SIGNATURE),1)
    $(IMG_TYPE) = $(IMG_TYPE) + 256
endif

ifeq ($(SIGN_PUBKEY_SRC),1)
    $(IMG_TYPE) = $(IMG_TYPE) + 512
endif

sinclude $(TOP_DIR)/tools/w800/inc.mk