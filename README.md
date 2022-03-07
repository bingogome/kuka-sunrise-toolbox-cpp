# kuka-sunrise-toolbox-cpp
## Introduction

**kuka-sunrise-toolbox-cpp** is a package developed for easy setup of Kuka IBR iiwa robot using C++. It is a packaged based on KST-Kuka-Sunrise-Toolbox (https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox), which is intended for Matlab users. The packaged has two parts: the Sunrise cabinet part, and the computer part. The Sunrise cabinet part is based on the original package, but has some modifications and additions. The computer side is based on C++. 

This package can be used 

## Dependencies

## Instruction

### Kuka Sunrise Cabinet side
- Connect ethernet cable from Sunrise Cabinet port X66 to computer
- Check Sunrise Cabinet IP (default should be 172.31.1.147). 
- Build and install the project under directory sunrise-cabinet-side as a Sunrise Project in Sunrise Workbench

### Computer side
- The easiest way is to copy the cpp and hpp files to your C++ project directly, setup your own CMake file (or equivalent).
- The subroutine to start the iiwa connection to your problem should contain: 
	1. instantiate a KstServoing class, 
	2. call KSTServoing::KstServoing::NetEstablishConnection(), 
	3. use the provided methods to command the robot.