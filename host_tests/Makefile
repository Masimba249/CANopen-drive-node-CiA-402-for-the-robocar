# Host build of the portable firmware components + the test suite.
# No ESP-IDF required: everything under canopen/, cia402/ and robocar_od/ is
# plain C99.

CC      ?= gcc
ROOT    := ..
FW      := $(ROOT)/firmware/components

INCLUDES := -I$(FW)/canopen/include -I$(FW)/cia402/include -I$(FW)/robocar_od/include -I.

CFLAGS  ?= -std=c99 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter \
           -fno-omit-frame-pointer
SANFLAGS ?= -fsanitize=address,undefined -fno-sanitize-recover=all

SRCS := \
	$(FW)/canopen/co_od.c \
	$(FW)/canopen/co_sdo.c \
	$(FW)/canopen/co_pdo.c \
	$(FW)/canopen/co_emcy.c \
	$(FW)/canopen/co_node.c \
	$(FW)/canopen/co_crc.c \
	$(FW)/cia402/cia402.c \
	$(FW)/robocar_od/od_data.c \
	test_util.c test_od.c test_sdo.c test_cia402.c test_pdo.c test_nmt_emcy.c main.c

BUILD := build
BIN   := $(BUILD)/run_tests

.PHONY: all run clean nosan od_dump sim

all: $(BIN)

$(BIN): $(SRCS) test_util.h | $(BUILD)
	$(CC) $(CFLAGS) $(SANFLAGS) $(INCLUDES) $(SRCS) -o $@

$(BUILD):
	mkdir -p $(BUILD)

run: $(BIN)
	./$(BIN)

# Same build without the sanitizers, for toolchains that lack them.
nosan: | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRCS) -o $(BUILD)/run_tests_nosan
	./$(BUILD)/run_tests_nosan

# Dumps the real object dictionary as CSV; master/tools/check_eds.py diffs
# it against the hand-written EDS.
od_dump: $(BUILD)/od_dump

$(BUILD)/od_dump: od_dump.c $(FW)/canopen/co_od.c $(FW)/robocar_od/od_data.c \
                  $(FW)/canopen/co_pdo.c $(FW)/canopen/co_sdo.c \
                  $(FW)/canopen/co_emcy.c $(FW)/canopen/co_node.c \
                  $(FW)/canopen/co_crc.c $(FW)/cia402/cia402.c | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@

# Shared library that lets a real CANopen master drive the real stack with no
# hardware: see master/tools/sim_node.py.
CORE_SRCS := \
	$(FW)/canopen/co_od.c $(FW)/canopen/co_sdo.c $(FW)/canopen/co_pdo.c \
	$(FW)/canopen/co_emcy.c $(FW)/canopen/co_node.c $(FW)/canopen/co_crc.c \
	$(FW)/cia402/cia402.c $(FW)/robocar_od/od_data.c

sim: $(BUILD)/libcanopen_sim.so

$(BUILD)/libcanopen_sim.so: sim_node.c $(CORE_SRCS) | $(BUILD)
	$(CC) $(CFLAGS) -fPIC -shared $(INCLUDES) $^ -o $@

clean:
	rm -rf $(BUILD)
