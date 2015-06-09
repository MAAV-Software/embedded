Test Cases

Herein lies the automated test cases for controls code. The testing framework 
requires Boost's Unit Test library.

To build and run test cases, you MUST have the following two directories in this
folder: "bin" and "build". To set these up, run:

	make setup

or you can just add the two folders yourselves. To build and run all tests, run:

	make

This will build the .o files in "build" and the excutables in "bin" and also run
all tests.
