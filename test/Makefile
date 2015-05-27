# Makefile for all Controls Code Testing

CXX = g++
LD = g++
OPTFLAG = -O1
CFLAGS = -std=c++11 -pedantic $(WFLAGS) $(OPTFLAG)
WFLAGS = -Wall -Wno-unused-parameter -Wno-unused-function -Wno-format-zero-length \
         -Werror -Wconversion -Wvla

ROOT_DIR = ~/maav-controls
PROJ_DIR = $(ROOT_DIR)/ccs_projects/maav
INC_DIR = $(PROJ_DIR)/include
SRC_DIR = $(PROJ_DIR)/src
TEST_DIR = $(ROOT_DIR)/test
BIN_DIR = $(TEST_DIR)/bin
BUILD_DIR = $(TEST_DIR)/build

LIBS = -lboost_unit_test_framework

all:DofTest SampleTest


sampleTest: sampleTest.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CFLAGS) $(BUILD_DIR)/sampleTest.o $(LIBS) -o $(BIN_DIR)/sampleTest

sampleTest.o: sampleTest.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) -c $(CFLAGS) sampleTest.cpp -o $(BUILD_DIR)/sampleTest.o


DofTest: DofTest.o Dof.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CFLAGS) $(BUILD_DIR)/DofTest.o $(BUILD_DIR)/Dof.o $(LIBS) -o $(BIN_DIR)/DofTest

DofTest.o: DofTest.cpp $(INC_DIR)/Dof.hpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) -c $(CFLAGS) DofTest.cpp -o $(BIN_DIR)/DofTest.c
	
Dof.o: $(SRC_DIR)/Dof.cpp $(INC_DIR)/Dof.hpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) -c $(CFLAGS) Dof.cpp -o $(BUILD_DIR)/Dof.o



clean:
	rm -f $(BUILD_DIR)/*.o $(BIN_DIR)/sampleTest $(BIN_DIR)/DofTest

run: all
	$(BIN_DIR)/sampleTest
	$(BIN_DIR)/DofTest
