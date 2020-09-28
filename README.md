<br />
<p align="center">
  <h3 align="center">UV Bot - Controller</h3>
  <p align="center">
    A robot that disinfects surfaces of public facilities with UV light radiation. 
    <br />
  </p>
</p>

<!-- TABLE OF CONTENTS -->
## Table of Contents
* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
* [Usage](#usage)
* [Roadmap](#roadmap)
* [Contact](#contact)

<!-- ABOUT THE PROJECT -->
## About The Project
As our communities begin to reopen, effective indoor virus disinfection measures must be implemented to reduce the spread of COVID-19. We developed a low-cost ultraviolet lighting robot (UVBOT) that can autonomously navigate spaces such as houses, classrooms, corridors, shops, and buses to disinfect surfaces and the air with ultraviolet light. Existing products in the market can cost up to fifty-thousand US dollars while being extremely large and heavy. This renders them accessible only by facilities with adequate financial resources such as larger hospitals. Our UVBOT is built mostly with off-the-shelf materials, uses cheap manufacturing methods, and carries custom-developed navigation software. It can disinfect a variety of indoor environments with a development cost of less than one thousand US dollars. 

The UVBOT is built upon the iRobot Create2 with four UV lamps mounted perpendicular to the iRobot Create2’s surface. The UV lamps reach 3.5 feet above the ground to disinfect walls, tables, counter-tops, door handles, and other surfaces. The robot navigation is controlled by a Raspberry Pi and lidar sensors. The lamps and Raspberry Pi are powered by external power banks. All hardware components are mounted on top of the iRobot Create2 using 3D printed and laser cut structures.   

### Built With
Hardware:
-Solidworks
-3D printer
-Laser cutter

Software:
-Flutter (Dart)
-Python
-Raspberry Pi (Raspbian)
-Firebase
-Flask

<!-- GETTING STARTED -->
## Getting Started
This is an example of how you may give instructions on setting up your project locally. What does a developer need to install to get working on this project? 
To get a local copy up and running follow these simple example steps. 

Hardware:
Download CURA, Creality Slicer, or another 3D printing slicing software. 

Software:
Download and install Raspbian on Raspberry Pi. Install Create2 api and firebase onto Raspberry Pi. Add firebase-sdk.json to same directory as your python files on the raspberry pi.
Download and install flutter on laptop. Create firebase real time database and follow the given directions for connecting it to your flutter app for Android (the code in this gitub isn't set up for IOS yet)

<!-- USAGE EXAMPLES -->
## Usage
Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

<!-- CONTACT -->
## Contact
* Harris Nisar - Simulation Engineer - nisar2@illinois.edu
* Yao Li - yaoli0508@gmail.com
* Fanxin
* Peter Chien - Intern - peterc3@illinois.edu
* Kesh - kesh@illinois.edu

Project Link: [https://github-dev.cs.illinois.edu/nisar2/self-tracking-server-provider](https://github-dev.cs.illinois.edu/nisar2/self-tracking-server-provider)

# robot-sanitizer
## library and bug-fix
irobot library: https://pypi.org/project/pycreate2/, https://github.com/MomsFriendlyRobotCompany/pycreate2

Sensor reading bug fix (updated june 23,2020):
> update /home/pi/.local/lib/python3.7/site-packages/pycreate2/create2api.py with our create2api.py

lidar library: https://github.com/SkoltechRobotics/rplidar

slam library: https://github.com/simondlevy/BreezySLAM

slam visualization: https://github.com/simondlevy/PyRoboViz

