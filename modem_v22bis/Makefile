CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -O2 -D_POSIX_C_SOURCE=200809L
LDFLAGS = -lm -lpthread

TARGET  = modem_v22bis
SRCS    = modem_v22bis.c v22bis_tx.c v22bis_rx.c
OBJS    = $(SRCS:.c=.o)

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

modem_v22bis.o: modem_v22bis.c v22bis.h
v22bis_tx.o:    v22bis_tx.c v22bis.h
v22bis_rx.o:    v22bis_rx.c v22bis.h

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Interop/BER verification harness. Links libspandsp (the reference V.22bis
# modem) and our core; built only when you run `make test`. Our public API
# mirrors spandsp's names, so for the test our symbols are aliased to mine_*
# (matching the #define block in test_v22bis_interop.c).
RENAMES = -Dv22bis_init=mine_v22bis_init -Dv22bis_tx=mine_v22bis_tx \
          -Dv22bis_rx=mine_v22bis_rx \
          -Dv22bis_rx_restart=mine_v22bis_rx_restart

test: test_v22bis_interop
	./test_v22bis_interop

v22bis_tx_r.o: v22bis_tx.c v22bis.h v22bis_rrc.h
	$(CC) $(CFLAGS) $(RENAMES) -c -o $@ v22bis_tx.c
v22bis_rx_r.o: v22bis_rx.c v22bis.h v22bis_rrc.h
	$(CC) $(CFLAGS) $(RENAMES) -c -o $@ v22bis_rx.c

test_v22bis_interop: test_v22bis_interop.c v22bis_tx_r.o v22bis_rx_r.o v22bis.h
	$(CC) $(CFLAGS) -o $@ test_v22bis_interop.c v22bis_tx_r.o v22bis_rx_r.o \
	    -lspandsp $(LDFLAGS)

clean:
	rm -f $(OBJS) $(TARGET) test_v22bis_interop v22bis_tx_r.o v22bis_rx_r.o
