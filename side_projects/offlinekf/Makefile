CXX := g++
CFLAGS := --std=c++11 -Wall -Werror -pedantic -g -I../../include -ICMSISINCLOC

offlinekf:
	$(CXX) $(CFLAGS) offlinekf.cpp ../../src/kalman/ExtendedKalmanFilter.cpp ../../src/kalman/KalmanFunctions.cpp -o offlinekf

clean:
	rm -vfr offlinekf *.exe *.out
