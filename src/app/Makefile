TOP_DIR = ../..
sinclude $(TOP_DIR)/tools/w800/conf.mk

ifndef PDIR
GEN_LIBS = libapp$(LIB_EXT)
COMPONENTS_libapp = cjson/libcjson$(LIB_EXT) 	\
					dhcpserver/libdhcpserver$(LIB_EXT) 	\
					dnsserver/libdnsserver$(LIB_EXT) 	\
					httpclient/libhttpclient$(LIB_EXT) 	\
					iperf/libiperf$(LIB_EXT) 	\
					libcoap/libcoap$(LIB_EXT) 	\
					libwebsockets-2.1-stable/libwebsockets$(LIB_EXT) 	\
					matrixssl/libmatrixssl$(LIB_EXT) 	\
					mdns/libmdns$(LIB_EXT) 	\
					mqtt/libmqtt$(LIB_EXT) 	\
					ntp/libntp$(LIB_EXT) 	\
					oneshotconfig/liboneshotconfig$(LIB_EXT) 	\
					ota/libota$(LIB_EXT) 	\
					ping/libping$(LIB_EXT) 	\
					polarssl/libpolarssl$(LIB_EXT) 	\
					sslserver/libsslserver$(LIB_EXT) 	\
					web/libweb$(LIB_EXT) 	\
					wm_atcmd/libwm_atcmd$(LIB_EXT) \
					mbedtls/libmbedtls$(LIB_EXT)
endif

#DEFINES +=

sinclude $(TOP_DIR)/tools/w800/rules.mk

INCLUDES := $(INCLUDES) -I $(PDIR)include

PDIR := ../$(PDIR)
sinclude $(PDIR)Makefile