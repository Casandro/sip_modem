# Aggregate build for the sip_modem suite.
# Each subproject has its own Makefile; this just recurses into them.

SUBDIRS = sip_interface modem_fsk modem_v22bis line_sim

.PHONY: all clean $(SUBDIRS)

all: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for d in $(SUBDIRS); do $(MAKE) -C $$d clean; done
