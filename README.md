# kuka-sunrise-toolbox-cpp
![alt text](https://github.com/bingogome/documents/blob/main/kuka-sunrise-toolbox-cpp/system.drawio.png)
## Introduction

**kuka-sunrise-toolbox-cpp** is a package developed for easy setup of Kuka IBR iiwa robot using C++. It is a packaged based on KST-Kuka-Sunrise-Toolbox (https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox), which is intended for Matlab users. The packaged has two parts: the Sunrise cabinet part, and the computer part. The Sunrise cabinet part is based on the original package, but has some modifications and additions. The computer side is based on C++. 

This package can be used as a submodule of your project. Simply include the cpp and hpp files. An example is being developed based on ROS/ROS2 in https://github.com/bingogome/kuka-sunrise-toolbox-cpp-ros. 

It can potentially used in other languages if a built C++ library can be called. Or, simply use other language and follow the same implementation method of this C++ package or the original Matlab package.

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
	2. call KstServoing::NetEstablishConnection(), 
	3. use the provided methods to command the robot, such as PTPJointSpace, PTPLineEEF. 
- When you are ready to start the connection, go to the teaching pad and start ToolboxServer. Within 30 second, call KstServoing::NetEstablishConnection().
- Be sure to call KstServoing::NetTurnoffServer() when finish using the robot. Otherwise, the Cabinet side will continue to run and the only way to stop is to restart the cabinet.

### Note
- Be sure to read and understand the code before any operation. The developers are not liable for any damage to property and personel for misuse.
- When using PTPJointSpace and PTPLineEEF, make sure to pass in a proper relative velocity or absolute velocity. The relative velocity is a proportional number relVel, such that the robot will move at the relVel multiplied by maximum capacity. Absolution velocity vel is mm/sec.
- KstServoing::ServoDirectCartesianStart() is the method to start direct servoing. The frequency of this method is 200-1000hz. If your application needs something higher than 1000hz, Kuka FRI may be a better choice.
- KstServoing::ServoSmartCartesianStart() is the method to start smart servoing. The frequency is 50-200hz.