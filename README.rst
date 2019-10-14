eBikeSpdController
==================

This repo contains the source files for the design, implementation and testing of a cruise control system for an instrumented electric bicycle.  

Controller Design Source Code
-----------------------------
The design folder contains MATLAB code that can be used to replicate the results of the controller design process outlined in this blog post here: https://mechmotum.github.io/blog/ebike-controller-design.html 

To replicate the controller design process, download the .m files in the design folder and run them in 
the following order: SystemID.m -> ControllerTuning.m -> ControllerRobustnessTesting.m -> ControllerEval.m. 
For more information, see the blog post mentioned above.

Implementation Source Code 
--------------------------
The Implementation folder contains the .ino file used to implement the derived contoller on an Arduino nano augmented to the powertrain of an electric bicycle. Please see this blog post for more information on how this software fits into the overall system: 
https://mechmotum.github.io/blog/ebike-controller-implementation.html

Testing Source Code 
-------------------
The Testing folder contains the source code used in the evaluation and diagnostics of the cruise control system implementation. The boxplot figure codes contain the code used to create the boxplot figures shown in the paper and poster presented at the 2019 Bicycle and Motorcycle Dynamics Conference. These materials can be found here: https://mechmotum.github.io/blog/ebike-controller-bmd-presentation.html 

Also included in the Testing folder is a utility function for converting bits to volts and the testAnalyzer.m file used to plot and evaluate the performance of runs collected from testing. 

Please see each file for more information on what's included in them. These files are meant to be used with the comma delimited diagnostics files produced by the TopLevel.ino file found in the Implementation folder of this repo.

Prerequisites
-------------

Software

You will need MATLAB R2018b or above with the Control System and Robust Control Toolboxes installed. Also needed is the Arduino IDE with the 
PID_v1 and PinChangeInterrupt libraries. 

Hardware 

Hardware prerequisites can be found on the implementation blog post mentioned above.

Authors
-------

Trevor Metz 
UC Davis Mechanical Engineering 2019
tzmetz@ucdavis.edu

Acknowledgments
---------------
Jason K. Moore for support and guidance 

Nicholas Chan for help with the Arduino code 

Brett Beauregard for developing the PID Arduino Library heavily used in the Implementation code 

Grey Gnome for developing the pin change interrupt library used in the Implementation code

