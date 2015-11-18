MAAV CONTROLS REPO

by Sajan Patel, Clark Zhang, Zhengjie Cui, Sasawat Prapanka
-----------------------------------------------------------

This is the repository for MAAV's controls subteam.
All controls code, documentation, and other important products/resources will go here.



Update: Added SD Card Failure Handling by Sasawati


maav CCS Project Instructions
-----------------------------
The following paths need to be set up in order to open the "maav" project in CCS.

REPO_LOC: this is the path to the maav-controls/repo on your computer
TIVAWARE_INSTALL: this is the path to your installation of Ti's TivaWare software library

In the Include Options, you need to add ${REPO_LOC}/include and ${REPO_LOC}/src in order to 
use maav's custom code in the project.



Repo Structure
--------------
maav		This is the main Code Composer Studio project for the flight controller running on the Tiva. 
		main.cpp, interrupt vector table, and Tiva build files live here in this directory.

include		This folder holds all of the header files for the custom flight controller code used in 
		the maav project. This is only for the flight controller, but other side projects may 
		include headers from here (but may not add non-maav project code).

src		This folder holds all of the source files for the custome flight controller code used in
		the maav project. As with include, this is also only for the flight controller project but
		can be used in other projects.

test		This folder holds all of the Boost Unit Test Framework automated test cases for non-Tiva-dependent
		code. Included in this directory is a bin and build folder as well as a Makefile for building
		these test cases separately from the maav project with g++ rather than Ti's compiler (since 
		we're using Boost for the test cases and non-hardware dependent code should function in the
		same way regardless of compilation).

research	This directory contains all research for control theory, state estimation, and data processing of 
		flight and sensor logs. Any simulation code in C/C++/Matlab/Python/etc (not full CCS projects) and
		all flight logs go here in their appropriate subdirectories. All data processing code also goes here
		as well as the results/output files from such code.

side_projects	This directory contains all Code Composer Studio projects used for testing sensors or playing around with
		the Tiva. Any project that is not the main maav project goes here. Projects here can access headers from
		the include directory but may not change any code or add new code to include or src.

docs		All documentation for MAAV Controls (infrastructure, main flight controller, side projects, etc) goes here.

lcmtypes	All lcm message specification files and auto-generated code goes here.


Contact Sajan Patel at sajanptl@umich.edu for more information.
