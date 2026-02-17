# Project Setup
CORE_SRC_DIR := src
EXAMPLE_SRC_DIR := examples
OBJ_DIR := .build
BIN_DIR := bin

# Find all core framework source files
CORE_SRC := $(wildcard $(CORE_SRC_DIR)/*.c)
CORE_OBJ := $(CORE_SRC:$(CORE_SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
CORE_DEP := $(CORE_OBJ:.o=.d)

# Find all example source files
EXAMPLE_SRC := $(wildcard $(EXAMPLE_SRC_DIR)/*.c)
# Binaries named after the source files
EXAMPLE_BINS := $(EXAMPLE_SRC:$(EXAMPLE_SRC_DIR)/%.c=$(BIN_DIR)/%)

# Compiler and Flags
CC := gcc
CFLAGS := -Wfatal-errors -pedantic -Wall -Wextra -Werror -std=c99
CFLAGS += -I ./include -I ../ode/include/ -I ../raylib/src -DPLATFORM_DESKTOP
LDFLAGS := -L./lib -L../raylib/src -L ../ode/ode/src/.libs/ 
LDFLAGS += -lraylib -lode -lX11 -ldl -pthread -lm -lstdc++

# Instrumentation (asan, etc)
INSTR := -fsanitize=address,leak,undefined -fno-omit-frame-pointer

# Default target
all: release

# Build all examples
examples: $(EXAMPLE_BINS)

# Link example binaries
$(BIN_DIR)/%: $(EXAMPLE_SRC_DIR)/%.c $(CORE_OBJ)
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) $< $(CORE_OBJ) -o $@ $(LDFLAGS)

# Compile core source files with dependency generation
$(OBJ_DIR)/%.o: $(CORE_SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Include dependencies
-include $(CORE_DEP)

# Modes
debug: CFLAGS += -g
debug: examples

release: CFLAGS += -O2
release: examples

inst: CFLAGS += $(INSTR) -g
inst: LDFLAGS += $(INSTR)
inst: examples

docs:
	doxygen docs/Doxyfile
	
read: docs
	firefox docs/doxygen/html/index.html

# Clean
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

.PHONY: all clean debug release inst examples docs read
