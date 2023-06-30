# ForCEs-Exoskeleton-Project
Internal Project for the development of a lower-limb active exoskeleton at the Deggendorf Institute of Technology

## Description
The goal of this project is the design and development of an exoskeleton that can be worn with higher levels of acceptance
from patients. In particular, the actuator has a unique four-bar-linkage design, with a conventional belt-and-pulley drive
that allows it to be back driveable. 
The Control System of the exoskeleton relies on the operation of multiple interfaces, mainly a core running on a central processor, 
in this case, a portable Raspberry Pi 4 model B (4 Gb). Arduino Microcontrollers were used for Data Acquisition and ADC.
No specific HMI exists for the use and operation of the exoskeleton. Since an ODrive v3.4 controller is used for the motor control,
most testing and interactions have been achieved through CLI and basic python scripts.
The support structure is created with topology optimization and a prototype was assembled with powder printed parts.
The EMG and Motion Capture Data can be visualized with Software such as EMGandMotionTools by Cometa.

## Structure
- python scripts for:
-- the computation with four-bar-linkage 
-- basic odrive control
- arduino codes for sensor input and control
- cad files for the actuator construction details
- EMG and Motion Capture data 

## Goal of this Repository
To complement the publication titled "Design and Development of a knee rehabilitation exoskeleton with four-bar linkage actuation", so that the experimental trials, as well as results are made available and can be reproduced independently. This work has been completed by several researchers at the DIT and crediting us for any use of the material here would be very much appreciated.
For the image/build file for the Raspberry Pi and additional work with AI Gait recognition, please contact us.


Further organisation information under: https://zaf.th-deg.de/public/project/201
