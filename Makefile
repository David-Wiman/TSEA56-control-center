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
TESTOUTNAME := check.out

MAINOBJ := main.o
SOURCE := $(shell find $(SRC_DIR) -name '*.cpp' ! -name $(MAINFILE))
TEST_SOURCE := $(shell find $(TEST_DIR) -name '*.cpp')
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SOURCE))
TEST_OBJS := $(patsubst $(TEST_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(TEST_SOURCE))
ALL_OBJS := $(OBJS) $(TEST_OBJS) $(OBJ_DIR)/$(MAINOBJ)
DEPS := $(patsubst %.o, %.d, $(ALL_OBJS))

# Main objetice - created with 'make' or 'make main'.
main: base $(OBJ_DIR)/$(MAINOBJ)
	@ echo Linking main file
	@ $(CCC) $(CCFLAGS) -o $(OUTNAME) $(OBJS) $(OBJ_DIR)/$(MAINOBJ) $(LDFLAGS)
	@ echo ""

# Test objetice
tests: base $(TEST_OBJS)
	@ echo Linking test file
	@ $(CCC) $(CCFLAGS) -o $(TESTOUTNAME) $(OBJS) $(TEST_OBJS) $(LDFLAGS)
	@ echo ""

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
	@ ./$(TESTOUTNAME)

check-leaktest: tests
	@ valgrind --leak-check=full ./$(TESTOUTNAME)

run-leaktest: main
	@ valgrind --leak-check=full ./$(OUTNAME)

# 'make clean' removes object files and memory dumps.
clean:
	@ \rm -rf $(OBJ_DIR) *.gch core

# 'make zap' also removes the executable and backup files.
zap: clean
	@ \rm -rf $(OUTNAME) test *~

-include $(DEPS)
