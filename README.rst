eBikeSpdController
==================

This repo contains the source files for the design and implementation of a cruise control system for an instrumented ebike. 
See the following blog posts for more information on how to use this source code 

<insert link to design blog post> 

(Controller implementation blog post coming soon)

Getting Started
---------------

Controller Design Source Code
To replicate the results of the controller design all you will need to do is download the .m files in the design folder and run them in 
the following order: SystemID.m -> ControllerTuning.m -> ControllerRobustnessTesting.m -> ControllerEval.m. See the blog post mentioned 
above for more information.

Implementation Source Code 
To implement the control system simply download it and upload it to an Arduino nano. For more information see the blog post 
mentioned above.

Prerequisites
-------------

Software

You will need the MATLAB R2018b with the Control System and Robust Control Toolboxes installed. Also needed is the Arduino IDE with the 
PID_v1 and PinChangeInterrupt libraries. 

Hardware 

Hardware prerequisites can be found on the implementation blog post mentioned above.

Authors
-------

Trevor Metz

Acknowledgments
---------------
Jason K. Moore for support and guidance 

Nicholas Chan for help with the Arduino code

