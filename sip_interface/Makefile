CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -O2 -D_POSIX_C_SOURCE=200809L
LDFLAGS = -lm -lpthread

TARGET  = sip_interface
SRCS    = sip_interface.c call.c sip_util.c alaw.c g722.c
OBJS    = $(SRCS:.c=.o)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

sip_interface.o: sip_interface.c call.h sip_util.h alaw.h
call.o:          call.c call.h sip_util.h alaw.h g722.h
sip_util.o:      sip_util.c sip_util.h
alaw.o:          alaw.c alaw.h
g722.o:          g722.c g722.h

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)
