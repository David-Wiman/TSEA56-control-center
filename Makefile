# Makefile with support for recursive subdirectories
#
# Note that this files makes a some  assumtions about the recursive makefiles:
# they must have a "base" target which does not make the main() function, they
# must have the same definition of SRC_DIR and OBJ_DIR.

# Folders
SRC_DIR = src
TEST_DIR := tests
OBJ_DIR := build

# Compiler
CCC = g++

# Compiling flags
DEBUG_FLAGS := -Og
OPTIM_FLAGS := -O3
CCFLAGS += -Wno-deprecated-declarations -Wall -Wextra -pedantic -Weffc++ -Wold-style-cast -Woverloaded-virtual -fmax-errors=3
CCFLAGS += -std=c++17 -MMD $(OPTIM_FLAGS)

# Linking flags
#LDFLAGS += -lsfml-graphics -lsfml-audio -lsfml-window -lsfml-system

# File which contains the main function
MAINFILE := main.cpp

# Name of output
OUTNAME := main.out
TEST_OUTNAME := test.out

MAINOBJ := main.o
SOURCE := $(shell find $(SRC_DIR) -name '*.cpp' ! -name $(MAINFILE))
TEST_SOURCE := $(shell find $(TEST_DIR) -name '*.cpp')
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SOURCE))
TEST_OBJS := $(patsubst $(TEST_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(TEST_SOURCE))
ALL_OBJS := $(OBJS) $(TEST_OBJS) $(OBJ_DIR)/$(MAINOBJ)
DEPS := $(patsubst %.o, %.d, $(ALL_OBJS))

# For handling recursive directories
SUBDIRS := logger
CCFLAGS += $(foreach d, $(SUBDIRS), -I$(d)/src)
SUBDIR_OBJS = $(wildcard $(foreach d, $(SUBDIRS), $(d)/$(OBJ_DIR)/*.o))

# Main objetice - created with 'make' or 'make main'.
main: subdirs base $(OBJ_DIR)/$(MAINOBJ)
	@ echo Linking main file
	@ $(CCC) $(CCFLAGS) -o $(OUTNAME) \
		$(OBJS) $(OBJ_DIR)/$(MAINOBJ) $(SUBDIR_OBJS) $(LDFLAGS)
	@ echo ""

# Test objetice
tests: subdirs base $(TEST_OBJS)
	@ echo Linking test file
	@ $(CCC) $(CCFLAGS) -I$(SRC_DIR) -o $(TEST_OUTNAME) \
		$(OBJS) $(TEST_OBJS) $(SUBDIR_OBJS) $(LDFLAGS)
	@ echo ""

# Recursive make of subdirectories

.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	@ rm -f $@/$(OBJ_DIR)/$(MAINOBJ)
	$(MAKE) base -C $@

# Compile everything except mainfile
base: $(OBJ_DIR) $(OBJS) Makefile

# Main program objects
$(OBJS) $(OBJ_DIR)/$(MAINOBJ): $(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@ echo Compiling $<
	@ $(CCC) $(CCFLAGS) -c $< -o $@

# Test program objects
$(TEST_OBJS): $(OBJ_DIR)/%.o: $(TEST_DIR)/%.cpp
	@ echo Compiling $<
	@ $(CCC) -I$(SRC_DIR) $(CCFLAGS) -c $< -o $@

$(OBJ_DIR):
	@ mkdir -p $(OBJ_DIR)

# Run output file (and compile it if needed)
run: main
	@ ./$(OUTNAME)

check: tests
	@ ./$(TEST_OUTNAME)

check-leaktest: tests
	@ valgrind --leak-check=full --suppressions=./suppressions.txt ./test

run-leaktest: main
	@ valgrind --leak-check=full ./$(OUTNAME)

# 'make clean' removes object files and memory dumps.
.PHONY: clean
clean:
	@ \rm -rf $(foreach d, $(SUBDIRS) ., $(d)/$(OBJ_DIR)) *.gch core

# 'make zap' also removes the executable and backup files.
zap: clean
	@ \rm -rf $(OUTNAME) $(OUTNAME) *~

-include $(DEPS)
