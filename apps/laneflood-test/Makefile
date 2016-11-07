CONTIKI_PROJECT_CLIENT = udp-client-stream
CONTIKI_PROJECT_SERVER = udp-server-stream

CONTIKI_PROJECT = $(CONTIKI_PROJECT_CLIENT) $(CONTIKI_PROJECT_SERVER)

all: $(CONTIKI_PROJECT)

OBJDUMP = msp430-objdump

CFLAGS += -DUIP_UDP=1 -DUIP_CONF_TCP=0 -DNETSTACK_CONF_WITH_IPV6=1
#CFLAGS += -DUIP_UDP=0 -DUIP_CONF_TCP=1 -DNETSTACK_CONF_WITH_IPV6=1

bytes_client: 
	@$(OBJDUMP) -h $(CONTIKI_PROJECT_CLIENT).sky | perl -ne '$$b{$$1}=hex $$2 if /^\s*\d+\s*\.(text|data|bss)\s+(\S+)/; END { printf("%16d bytes in ROM\n%16d bytes in RAM\n",$$b{text}+$$b{data},$$b{data}+$$b{bss}); }'

bytes_client2: 
	@$(OBJDUMP) -h $(CONTIKI_PROJECT_CLIENT2).sky | perl -ne '$$b{$$1}=hex $$2 if /^\s*\d+\s*\.(text|data|bss)\s+(\S+)/; END { printf("%16d bytes in ROM\n%16d bytes in RAM\n",$$b{text}+$$b{data},$$b{data}+$$b{bss}); }'

bytes_server:
	@$(OBJDUMP) -h $(CONTIKI_PROJECT_SERVER).sky | perl -ne '$$b{$$1}=hex $$2 if /^\s*\d+\s*\.(text|data|bss)\s+(\S+)/; END { printf("%16d bytes in ROM\n%16d bytes in RAM\n",$$b{text}+$$b{data},$$b{data}+$$b{bss}); }'

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ FLOCKLAB +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
flocklab_no_debug: CFLAGS += -DINITIATOR_NODE_ID=1 \
							 -DUSE_TINYOS_NODEIDS=1 -DTINYOS_SERIAL_FRAMES=0 \
							 -DFLOCKLAB=1 -DCOOJA=0 -DINDRIYA=0 -DLOCAL_NODES=0 \
							 -DBURN_NODE_ID=0 \
							 -DGLOSSY_DEBUG=0 -DGLOSSY_DEBUG_PINS=0 -DGLOSSY_DEBUG_STATUS=0
flocklab_no_debug: $(CONTIKI_PROJECT_CLIENT) bytes_client $(CONTIKI_PROJECT_SERVER) bytes_server
	cp $(CONTIKI_PROJECT_CLIENT).sky $(CONTIKI_PROJECT_CLIENT).flocklab_no_debug.sky
	cp $(CONTIKI_PROJECT_SERVER).sky $(CONTIKI_PROJECT_SERVER).flocklab_no_debug.sky

flocklab_debug_pins: CFLAGS += -DINITIATOR_NODE_ID=1 \
				-DUSE_TINYOS_NODEIDS=1 -DTINYOS_SERIAL_FRAMES=0 \
				-DFLOCKLAB=1 -DCOOJA=0 -DLOCAL_NODES=0 -DBURN_NODE_ID=0 \
				-DGLOSSY_DEBUG=0 -DGLOSSY_DEBUG_PINS=1 -DGLOSSY_DEBUG_STATUS=0
flocklab_debug_pins: $(CONTIKI_PROJECT_CLIENT) bytes_client $(CONTIKI_PROJECT_SERVER) bytes_server
	cp $(CONTIKI_PROJECT_CLIENT).sky $(CONTIKI_PROJECT_CLIENT).flocklab_debug_pins.sky
	cp $(CONTIKI_PROJECT_SERVER).sky $(CONTIKI_PROJECT_SERVER).flocklab_debug_pins.sky
	

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ COOJA +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cooja_no_debug: CFLAGS += -DUSE_TINYOS_NODEIDS=0 -DFLOCKLAB=0 -DCOOJA=1 -DLOCAL_NODES=0 -DBURN_NODE_ID=0 -DGLOSSY_DEBUG=0 -DGLOSSY_DEBUG_PINS=0 -DGLOSSY_DEBUG_STATUS=0
cooja_no_debug: $(CONTIKI_PROJECT_CLIENT) bytes_client $(CONTIKI_PROJECT_SERVER) bytes_server
	cp $(CONTIKI_PROJECT_CLIENT).sky $(CONTIKI_PROJECT_CLIENT).cooja_no_debug.sky
	cp $(CONTIKI_PROJECT_SERVER).sky $(CONTIKI_PROJECT_SERVER).cooja_no_debug.sky

cooja_debug_all: CFLAGS += -DUSE_TINYOS_NODEIDS=0 -DFLOCKLAB=0 -DCOOJA=1 -DLOCAL_NODES=0 -DBURN_NODE_ID=0 -DGLOSSY_DEBUG=1 -DGLOSSY_DEBUG_PINS=1 -DGLOSSY_DEBUG_STATUS=1
cooja_debug_all: $(CONTIKI_PROJECT_CLIENT) bytes_client $(CONTIKI_PROJECT_SERVER) bytes_server
	cp $(CONTIKI_PROJECT_CLIENT).sky $(CONTIKI_PROJECT_CLIENT).cooja_debug_all.sky
	cp $(CONTIKI_PROJECT_SERVER).sky $(CONTIKI_PROJECT_SERVER).cooja_debug_all.sky


CONTIKI = ../..
CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
