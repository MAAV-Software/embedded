MAAV CONTROLS REPOSITORY

by Sajan Patel, Clark Zhang, Zhengjie Cui, Sasawat Prankprakma
--------------------------------------------------------------

This is the repository for MAAV's controls subteam.
All controls code, documentation, and other important products/resources will go here.


Installation Instructions (for test cases for hardware-independent code)
------------------------------------------------------------------------
If working on CAEN, please followed the specialized CAEN instructions below

Ensure that you have cmake, eigen3, lcm, and boost installed. Make a build 
directory:

	mkdir build

Then go into the build directory:

	cd build

Run cmake to initialize the build environment: 

	cmake ../

and address and correct any errors it might tell you (e.g. missing packages or 
files are in incorrect places...HINT: use Google to help you out). Now you are 
ready to build the hardware-independent controller code, cmeigen, and test 
cases:
	
	make -j7

Note that the -j7 is optional to enable faster, parallelized build, and that you 
can choose any number (but we suggest 7 or 8 usually).

To run the test cases automatically:

	make test

To build the HTML documentation:
    
    make doc

If you need to clean anything up or remove the built files:

	make clean

Making clean will not remove documentation files (limitation of cmake). If you 
need to clean up the doxygen documentation:

	make doc_clean

Any other questions should first be addressed to Google (if it's about make, 
cmake, or Linux terminal commands) or the team leads (anything and everything).

CAEN Installation Instructions (for test cases for hardware-independent code)
------------------------------------------------------------------------
If working on CAEN, please followed the specialized CAEN instructions below

Starting in the repository directory (by default "ctrl"), go up one level:

	cd ..

Download and unzip eigen3 into "Eigen"

Download and unzip boost into "boost" 

Go into the boost directory

	cd boost

Install boost (this will take some time)

	./bootstrap.sh

	./b2

Check that everything is in the right place. From the dir that contains the
cloned respository, run:

	ls

If it is set up right, and the repository is in a folder named ctrl, 
the output should look somewhat like:
	
	> boost ctrl Eigen

Go into the repository:
	
	cd ctrl

Make a build directory:

	mkdir build

Then go into the build directory:

	cd build

Run cmake to initialize the build environment: 

	cmake -D CAEN ../

and address and correct any errors it might tell you (e.g. missing packages or 
files are in incorrect places...HINT: use Google to help you out). Now you are 
ready to build the hardware-independent controller code, cmeigen, and test 
cases:
	
	make -j7

Note that the -j7 is optional to enable faster, parallelized build, and that you 
can choose any number (but we suggest 7 or 8 usually).

To run the test cases automatically:

	make test

If you need to clean anything up or remove the built files:

	make clean

Any other questions should first be addressed to Google (if it's about make, 
cmake, or Linux terminal commands) or the team leads (anything and everything).



maav CCS Project Instructions
-----------------------------
You will have to copy maav/.project.bak to maav/.project before importing the
project. Likewise, you will have to copy maav/.cproject.bak to maav/.cproject 
before importing the CCS project.

***If your .cproject changes at any time and git wants you to commit it, restore
your .cproject file to the orignal .cproject.bak file.***


Once the project is imported, the following paths need to be set up in order to
open the "maav" project in CCS. Refer to docs/ccs_build.tex for how to setup 
these paths.

REPO_LOC: this is the path to the maav-controls/repo on your computer
TIVAWARE_INSTALL: this is the path to your installation of Ti's TivaWare software library

Repo Structure
--------------
maav		This is the main Code Composer Studio project for the flight controller running on the Tiva. 
		main.cpp, interrupt vector table, and Tiva build files live here in this directory.

src		This folder holds all of the source files for the custome flight controller code used in
		the maav project. This is only for the flight controller project but
		can be used in other projects.

test		This folder holds all of the Boost Unit Test Framework automated test cases for non-Tiva-dependent
		code. Included in this directory is a bin and build folder as well as a Makefile for building
		these test cases separately from the maav project with g++ rather than Ti's compiler (since 
		we're using Boost for the test cases and non-hardware dependent code should function in the
		same way regardless of compilation).

cmeigen		This directory contains the eigen implementation of the CMSIS-DspLib
		matrix math and other functions used in our code. Note this is not a complete
		re-implementation of the entire CMSIS-DspLib; it is just the bare necessity for 
		us to run test cases in Linux for code that requires these functions. CMSIS itself 
		is designed only for ARM microcontrollers, and the original DspLib is used on the 
		Tiva.

research	This directory contains all research for control theory, state estimation, and data processing of 
		flight and sensor logs. Any simulation code in C/C++/Matlab/Python/etc (not full CCS projects) and
		all flight logs go here in their appropriate subdirectories. All data processing code also goes here
		as well as the results/output files from such code.

side_projects	This directory contains all Code Composer Studio projects used for testing sensors or playing around with
		the Tiva. Any project that is not the main maav project goes here. Projects here can access headers from
		the include directory but may not change any code or add new code to include or src.

doc		All documentation or Doxygen config files for MAAV Controls (infrastructure, main flight controller, side projects, etc) goes here.

cmake 	Contains the cmake pakage searching scripts.

lcmtypes	All lcm message specification files and auto-generated code goes here.


Contact Sajan Patel at sajanptl@umich.edu for more information.
