<div align="center">
<img src="/Images/logo_png.png" width="100" height="100">
</div>

# APRI0007: Major Project in Electronics: V-Luxe
## Team members
* Julien Boniver
* Guillaume Eyen
* Victoria Filée
* Matthieu Gillard
* Arnaud Innaurato

## Project Description
  This project has been carried by a team of 5 students in the scope of their first year in MSc of Electrical Engineering. The goal of this whole year project is to develop from scratch a working electronics device with a software development in assembly on PIC microcontroller.  
  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;This project aims to design and build a scale model of a tilting window equipped with an electronically controlled blind. The system will support two modes of operation: automatic and manual.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In automatic mode, the window and blind will open or close based on sensor data and user-defined threshold values. The goal is to automatically regulate parameters such as temperature or brightness to achieve the desired indoor conditions.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In manual mode, the user has full control over the window and blind, allowing them to be adjusted as desired. In this mode, the system will not respond to changes in the environment.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;A control system has been implemented to have a blind which is not sensitive to external disturbances (in both modes).  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The system will feature a control panel equipped with an LCD screen and buttons. This interface will allow the user to select the operating mode and customize settings as wanted.  

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;This repository contains the [embedded software](https://github.com/Arnaudinnau/APRI0007_VLuxe/blob/main/V_Luxe_Code.asm) developed for the V-Luxe project. The software is written entirely in assembly language for the PIC18F4331 microcontroller.   
Demonstration videos validating the system’s functionality and complete project report are also included in this repository.  

<div align="center">
  <img src="/Images/Final_Structure.jpeg" width="250" height="250">
</div>

## Project Report
The project report outlines the whole project components including management, electronics, mechanics, software and control system. Technicals choices that have been done during the whole project elaboration are justified. Some improvement paths are also developed at the end of this report.

## Video descriptions
### [Video 1 - Automatic Window Opening (Humidity)](https://github.com/Arnaudinnau/APRI0007_VLuxe/blob/main/Videos/1.mp4)
This video demonstrates the automatic opening of the window in response to increased indoor relative humidity.

### [Video 2 - Blind Control System](https://github.com/Arnaudinnau/APRI0007_VLuxe/blob/main/Videos/2.mp4)
This video illustrates the blind control system in action. When external disturbances attempt to shift the blind from its set position, the control system actively counteracts the disturbance to maintain its target.

### [Video 3 - Control Panel in Automatic Mode](https://github.com/Arnaudinnau/APRI0007_VLuxe/blob/main/Videos/3.mp4)
This video showcases the system operating in automatic mode. The user sets a desired indoor temperature, which is displayed on the LCD. The system automatically adjusts conditions to maintain that temperature.

### [Video 4 - Automatic Window Opening (Temperature)](https://github.com/Arnaudinnau/APRI0007_VLuxe/blob/main/Videos/4.mp4)
This video demonstrates the automatic opening of the window due to elevated indoor temperature. When the measured temperature exceeds the user-defined threshold, the window is automatically opened to regulate the environment.

### [Video 5 - Automatic Blind Closing (Luminosity)](https://github.com/Arnaudinnau/APRI0007_VLuxe/blob/main/Videos/5.mp4)
This video shows the automatic closing of the blind in response to high outdoor luminosity. The system reacts by closing the blind to reduce incoming light intensity.

### [Video 6 - Manual Control Mode](https://github.com/Arnaudinnau/APRI0007_VLuxe/blob/main/Videos/6.mp4)
This video displays the manual control mode for both the window and the blind. In this mode, the user can operate both devices independently of environmental conditions.

