# Makefile for all Controls Code Testing

CXX = g++
LD = g++
CFLAGS = -std=c++03 $(WFLAGS) -g
WFLAGS = -Wall -Wno-unused-parameter -Wno-unused-function -Wextra \
		 -Wno-format-zero-length -Werror -Wconversion -Wvla -pedantic \
		 -Wno-unused-variable

ROOT_DIR = ../.
PROJ_DIR = $(ROOT_DIR)/maav
INC_DIR = $(ROOT_DIR)/include
SRC_DIR = $(ROOT_DIR)/src
TEST_DIR = $(ROOT_DIR)/test
BIN_DIR = $(TEST_DIR)/bin
BUILD_DIR = $(TEST_DIR)/build

CMEIGEN_DIR = $(ROOT_DIR)/research/cmeigen

LIBS = -lm -lboost_unit_test_framework
LIBS_NO_BOOST = -lm

ekf: setup KalmanTest
all: setup test

setup:
	@echo ""
	@echo "Setting Up Build"
	mkdir -p $(BIN_DIR)
	mkdir -p $(BUILD_DIR)
	@echo "Setup Done, Building Code"

#test: LidarSwTest Px4SwTest LowPassTest pidTest dofTest sampleTest
test: BatteryTest LidarSwTest Px4SwTest LowPassTest pidTest dofTest


ExtendedKalmanFilter.o: $(SRC_DIR)/kalman/ExtendedKalmanFilter.cpp $(INC_DIR)/kalman/ExtendedKalmanFilter.hpp
	@echo ""
	@echo "Building ExtendedKalmanFilter"
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/kalman/ExtendedKalmanFilter.cpp -I $(INC_DIR) -I $(CMEIGEN_DIR) -DLINUX=1 -o $(BUILD_DIR)/ExtendedKalmanFilter.o

KalmanFunctions.o: $(SRC_DIR)/kalman/KalmanFunctions.cpp $(INC_DIR)/kalman/KalmanFunctions.hpp
	@echo ""
	@echo "Building KalmanFunctions"
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/kalman/KalmanFunctions.cpp -I $(INC_DIR) -I $(CMEIGEN_DIR) -DLINUX=1 -o $(BUILD_DIR)/KalmanFunctions.o

KalmanTest.o: KalmanTest.cpp $(INC_DIR)/kalman/ExtendedKalmanFilter.hpp $(INC_DIR)/kalman/KalmanFunctions.hpp
	@echo ""
	@echo "Building KalmanTest"
	$(CXX) -c $(CFLAGS) KalmanTest.cpp -I $(INC_DIR) -I $(CMEIGEN_DIR) -DLINUX=1 -o $(BUILD_DIR)/KalmanTest.o

KalmanTest: ExtendedKalmanFilter.o KalmanFunctions.o KalmanTest.o
	@echo ""
	@echo "Liking KalmanTest"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/ExtendedKalmanFilter.o $(BUILD_DIR)/KalmanFunctions.o $(BUILD_DIR)/KalmanTest.o $(CMEIGEN_DIR)/cmeigen.o $(LIBS_NO_BOOST) -o $(BIN_DIR)/KalmanTest

sampleTest: sampleTest.o
	@echo ""
	@echo "Building Sample Test"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/sampleTest.o $(LIBS) -o $(BIN_DIR)/sampleTest

sampleTest.o: sampleTest.cpp
	$(CXX) -c $(CFLAGS) sampleTest.cpp -o $(BUILD_DIR)/sampleTest.o

LowPassTest: LowPass.o LowPassTest.o
	@echo ""
	@echo "Building LowPass Test"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/LowPassTest.o $(BUILD_DIR)/LowPass.o $(LIBS) -o $(BIN_DIR)/LowPassTest

LowPassTest.o: $(INC_DIR)/LowPass.hpp LowPassTest.cpp
	$(CXX) -c $(CFLAGS) LowPassTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/LowPassTest.o

LowPass.o: $(SRC_DIR)/LowPass.cpp $(INC_DIR)/LowPass.hpp
	@echo ""
	@echo "Building LowPass"
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/LowPass.cpp -I $(INC_DIR) -o $(BUILD_DIR)/LowPass.o

pidTest: Pid.o pidTest.o LowPass.o
	@echo ""
	@echo "Building PID Test"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/pidTest.o $(BUILD_DIR)/Pid.o $(BUILD_DIR)/LowPass.o $(LIBS) -o $(BIN_DIR)/pidTest

pidTest.o: $(INC_DIR)/Pid.hpp pidTest.cpp
	$(CXX) -c $(CFLAGS) pidTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/pidTest.o

Pid.o: $(SRC_DIR)/Pid.cpp $(INC_DIR)/Pid.hpp $(INC_DIR)/LowPass.hpp
	@echo ""
	@echo "Building Pid"
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/Pid.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Pid.o

dofTest: LowPass.o Pid.o Dof.o dofTest.o
	@echo ""
	@echo "Building Dof Test"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/dofTest.o $(BUILD_DIR)/Dof.o $(BUILD_DIR)/Pid.o $(BUILD_DIR)/LowPass.o $(LIBS) -o $(BIN_DIR)/dofTest

dofTest.o: $(INC_DIR)/Dof.hpp dofTest.cpp
	$(CXX) -c $(CFLAGS) dofTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/dofTest.o

Dof.o: $(SRC_DIR)/Dof.cpp $(INC_DIR)/Dof.hpp $(INC_DIR)/Pid.hpp
	@echo ""
	@echo "Building Dof"
	$(CXX) -c $(CLFAGS) $(SRC_DIR)/Dof.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Dof.o

Px4SwTest: Px4.o Px4SwTest.o
	@echo ""
	@echo "Building Px4 Sw Test"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/Px4.o $(BUILD_DIR)/Px4SwTest.o $(LIBS) -o $(BIN_DIR)/Px4SwTest

Px4SwTest.o: $(INC_DIR)/Px4.hpp $(INC_DIR)/Px4Defines.hpp Px4SwTest.cpp
	$(CXX) -c $(CFLAGS) Px4SwTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Px4SwTest.o

Px4.o: $(SRC_DIR)/Px4.cpp $(INC_DIR)/Px4.hpp $(INC_DIR)/Px4Defines.hpp
	@echo ""
	@echo "Building Px4"
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/Px4.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Px4.o

LidarSwTest: Lidar.o LidarSwTest.o
	@echo ""
	@echo "Building Lidar Sw Test"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/Lidar.o $(BUILD_DIR)/LidarSwTest.o $(LIBS) -o $(BIN_DIR)/LidarSwTest

LidarSwTest.o: $(INC_DIR)/Lidar.hpp $(INC_DIR)/LidarDefines.hpp LidarSwTest.cpp
	$(CXX) -c $(CFLAGS) LidarSwTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/LidarSwTest.o

Lidar.o: $(SRC_DIR)/Lidar.cpp $(INC_DIR)/Lidar.hpp $(INC_DIR)/LidarDefines.hpp
	@echo ""
	@echo "Building Lidar"
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/Lidar.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Lidar.o

BatteryTest: Battery.o BatteryTest.o
	@echo ""
	@echo "Building Battery Test"
	$(CXX) $(CFLAGS) $(BUILD_DIR)/BatteryTest.o $(BUILD_DIR)/Battery.o $(LIBS) -o $(BIN_DIR)/BatteryTest

BatteryTest.o: BatteryTest.cpp $(INC_DIR)/Battery.hpp
	$(CXX) -c $(CFLAGS) BatteryTest.cpp -I $(INC_DIR) -o $(BUILD_DIR)/BatteryTest.o

Battery.o: $(SRC_DIR)/Battery.cpp $(INC_DIR)/Battery.hpp
	@echo ""
	@echo "Building Battery"
	$(CXX) -c $(CFLAGS) $(SRC_DIR)/Battery.cpp -I $(INC_DIR) -o $(BUILD_DIR)/Battery.o

clean:
	@echo
	@echo "Removing Build Files"
	rm -rf $(BUILD_DIR)/*.o $(BIN_DIR)/*

run:
	@echo ""
	@echo "Running Tests"
	@echo ""
	$(BIN_DIR)/BatteryTest
	@echo ""
	$(BIN_DIR)/LowPassTest
	@echo ""
	$(BIN_DIR)/LidarSwTest
	@echo ""
	$(BIN_DIR)/Px4SwTest
	@echo ""
	$(BIN_DIR)/pidTest
	@echo ""
	$(BIN_DIR)/dofTest
	@echo ""
#$(BIN_DIR)/sampleTest
