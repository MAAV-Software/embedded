# Makefile for all Controls Code Testing

CXX = g++
LD = g++
CFLAGS = -std=c++03 $(WFLAGS)
WFLAGS = -Wall -Wno-unused-parameter -Wno-unused-function -Wextra \
		 -Wno-format-zero-length -Werror -Wconversion -Wvla -pedantic

ROOT_DIR = ~/maav-controls
PROJ_DIR = $(ROOT_DIR)/maav
INC_DIR = $(ROOT_DIR)/include
SRC_DIR = $(ROOT_DIR)/src
TEST_DIR = $(ROOT_DIR)/test
BIN_DIR = $(TEST_DIR)/bin
BUILD_DIR = $(TEST_DIR)/build

LIBS = -lboost_unit_test_framework

all: setup test run

setup:
	mkdir -p $(BIN_DIR)
	mkdir -p $(BUILD_DIR)

test: pidTest dofTest sampleTest

sampleTest: sampleTest.o
	$(CXX) $(CFLAGS) $(BUILD_DIR)/sampleTest.o $(LIBS) -o $(BIN_DIR)/sampleTest

sampleTest.o: sampleTest.cpp
	$(CXX) -c $(CFLAGS) sampleTest.cpp -o $(BUILD_DIR)/sampleTest.o

pidTest: Pid.o pidTest.o
	$(CXX) $(CFLAGS) $(BUILD_DIR)/pidTest.o $(BUILD_DIR)/Pid.o $(LIBS) -o $(BIN_DIR)/pidTest

pidTest.o: $(INC_DIR)/Pid.hpp pidTest.cpp
	$(CXX) -c $(CFLAGS) pidTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/pidTest.o

Pid.o: $(SRC_DIR)/Pid.cpp $(INC_DIR)/Pid.hpp
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/Pid.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Pid.o

dofTest: Pid.o Dof.o dofTest.o
	$(CXX) $(CFLAGS) $(BUILD_DIR)/dofTest.o $(BUILD_DIR)/Dof.o $(BUILD_DIR)/Pid.o $(LIBS) -o $(BIN_DIR)/dofTest

dofTest.o: $(INC_DIR)/Dof.hpp dofTest.cpp
	$(CXX) -c $(CFLAGS) dofTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/dofTest.o

Dof.o: $(SRC_DIR)/Dof.cpp $(INC_DIR)/Dof.hpp $(INC_DIR)/Pid.hpp
	$(CXX) -c $(CLFAGS) $(SRC_DIR)/Dof.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Dof.o

clean:
	rm -rf $(BUILD_DIR)/*.o $(BIN_DIR)/*

run:  
	$(BIN_DIR)/pidTest
	$(BIN_DIR)/dofTest
	$(BIN_DIR)/sampleTest


