# 3D_robotic_positioning_Cisco_CSIG_Labs
This repository is for a project I completed at CSIG Labs, Cisco Systems (San Jose). Using Posyx 3D positioning sensors configured and set up in a test room, I wrote up code for robotic navigation.


![setup](https://github.com/shahjaidev/3D_robotic_positioning_Cisco_CSIG_Labs/blob/master/Pozyx%203D%20gyro.png)

Steps
(1)	Establishes connection with 4 Pozyx Tags and finds initial coordinates
(2)	Moves forward for 2 seconds in the initial direction then stops at (x1,y1)
(3)	Finds current coordinates and then finds the positive acute angle using tan inverse function
(4)	Calculate positive slope angle between the current (x1,y1) and destination (x2,y2)
(5)	Using 1 of 4 cases, calculate ‘x’ (Reqd angle difference)
(6)	Add ‘x’ to initial magnetometer reading to get required angle
(7)	Execute ‘turn’ command till robot is correctly oriented towards destination
(8)	When the current coordinates match with destination coordinates, tell motors to stop

Functions
(1)	Void Orient()
Description;
Calculates required angle and turns motors till robot is properly oriented

(2)	Void forward()
Executes ‘forward’ command to motors and constantly checks if it has reached destination grid. Stops when it reaches destination

(3)	void printRawSensorData()
Prints current Euler Angles

(4)	void send2slave(byte c)
Sends a character command to the Slave Arduino which controls motors

(5)	float get_heading()
Gets orientation angle between 0 and 360 from the magnetometer

(6)	void printCalibrationResult()
Prints Successful or unsuccessful calibration of Anchors


(7)	Pozyx.doPositioning(&coor, dimension, height, algorithm)
Stores current coordinates of Tag in Object coor (Which is passed by reference)

