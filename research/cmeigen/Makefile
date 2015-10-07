EIGEN_INC = /usr/include/eigen3/
CFLAGS = -std=c++03 -pedantic -Werror -Wall

all: test

cmeigen.o: cmeigen.cpp cmeigen.hpp
	g++ -c ${CFLAGS} cmeigen.cpp -I ${EIGEN_INC}

test.o: test.cpp
	g++ -c ${CFLASG} test.cpp -I ${EIGEN_INC}

test: cmeigen.o test.o
	g++ ${CFLAGS} test.o cmeigen.o -o test

clean:
	rm -rf test *.o *~

#vim_cleanup:
#	rm -rvf *~
