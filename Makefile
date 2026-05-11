CC ?= gcc
CFLAGS ?= -O2 -g -Wall -Wextra -std=c11
LDFLAGS ?=

TARGET := pci_bar_bench
SRC := pci_bar_bench.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)
