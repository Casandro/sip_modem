CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -O2 -D_POSIX_C_SOURCE=200809L
LDFLAGS = -lm -lpthread

TARGET  = line_sim
# alaw.c is shared with sip_interface (the optional G.711 round-trip).
SRCS    = line_sim.c channel.c resample.c filters.c ../sip_interface/alaw.c
OBJS    = line_sim.o channel.o resample.o filters.o alaw.o

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

line_sim.o: line_sim.c channel.h resample.h filters.h
channel.o:  channel.c channel.h resample.h filters.h ../sip_interface/alaw.h
resample.o: resample.c resample.h
filters.o:  filters.c filters.h
alaw.o:     ../sip_interface/alaw.c ../sip_interface/alaw.h
	$(CC) $(CFLAGS) -c -o $@ ../sip_interface/alaw.c

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Standalone DSP unit checks (no sockets): SNR, THD, group delay, frequency
# response, resampler quality, clock-slip bound, lockstep sample-count.
test: test_dsp
	./test_dsp

test_dsp: test_dsp.c channel.o resample.o filters.o alaw.o channel.h resample.h filters.h
	$(CC) $(CFLAGS) -o $@ test_dsp.c channel.o resample.o filters.o alaw.o $(LDFLAGS)

clean:
	rm -f $(OBJS) $(TARGET) test_dsp
