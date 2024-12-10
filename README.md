# 2024-25 UTD Raytheon UAX Drone Competition: Scout Drone

<p>The following README has instructions to setup, run, and simulate the code within this repository with the jetson</p>

<p>This repository is meant to be loaded onto the Nvidia Jetson Orin Nano related with autonomous drone flight, simulation, and visual processing.</p>

<p>

To gain full understanding of the code/repository **PLEASE FOLLOW 
THE STEPS IN ORDER**

Please read the documentation fully and thoroughly (your questions will probably be answered later in the documentation) before you ask for help, this project is research heavy you will spend hours on forums and outdated documentation especially for things regarding drone and ground vehicle movement and commands. Good Luck :smiley:

Also we dont know if sim works for mac :frowning_face:

</p>

<h2> Drone/Jetson</h2>

<h2> SITL Simulation</h2>

<p>

**NOTE:** dronekit-sitl is outdated and no longer supported along with dronekit, however it is a simple and easy to use interface and if necessary you can interact using pyMavlink commands exposed by the dronkit/dronekit-sitl interface

</p>

<p>

This can be used to simulate both the drone and ground vehicle through arducopter, it is suggested that if you are changing any flight procedures by adding, removing or editing dronekit/Mavlink commmands that we are sending to the cube pilot that you test thoroughly using SITL</p>

<h3>SITL Local Setup:</h3>

1. from drone local repository use command in terminal: 

        pip install -r requirements.txt

2. Install [Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html) 

3. Navigate to DroneCode folder and try to run ```DroneSimTest.py```

> - If you see the following issue: 

```
PS C:\Users\rushi\OneDrive\Desktop\School\CS4485\drone\DroneCode> python DroneSimTest.py
Traceback (most recent call last):
  File "C:\Users\rushi\OneDrive\Desktop\School\CS4485\drone\DroneCode\DroneSimTest.py", line 4, in <module>
    from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
  File "C:\Users\rushi\AppData\Local\Programs\Python\Python313\Lib\site-packages\dronekit\__init__.py", line 2689, in <module>
    class Parameters(collections.MutableMapping, HasObservers):
                     ^^^^^^^^^^^^^^^^^^^^^^^^^^
AttributeError: module 'collections' has no attribute 'MutableMapping'
```

> - Change ```collections.MutableMapping``` to ```collections.abc.MutableMapping```
> - For further information regarding fix: https://github.com/dronekit/dronekit-python/issues/1132

5. Once you are able to run DroneSimTest.py successfully, you should see an output similar to the following where you should see "The drone is not in guided mode yet" repeating after some time.

```
PS C:\Users\rushi\OneDrive\Desktop\School\CS4485\drone\DroneCode> python DroneSimTest.py
Starting copter simulator (SITL)
SITL already Downloaded and Extracted.
Ready to boot.
CRITICAL:autopilot:APM:Copter V3.3 (d6053245)
CRITICAL:autopilot:Frame: QUAD
CRITICAL:autopilot:Calibrating barometer
CRITICAL:autopilot:Initialising APM...
CRITICAL:autopilot:barometer calibration complete
CRITICAL:autopilot:GROUND START
Starting Location: , (32.9201928, -96.9483109)
Heading:  353
CRITICAL:root:ARMING DRONE
Waiting for drone to become armable
Waiting for drone to become armable
Waiting for drone to become armable
Waiting for drone to become armable
Waiting for drone to become armable
The drone is now armable
The drone is not in guided mode yet
The drone is not in guided mode yet
The drone is not in guided mode yet
```

6. You have successfully completed setup


<h3>Running Simulation:</h3>

1. Open Mission Planner & have DroneSimTest.py running

2. Click Connect on Top Right Corner on Mission Planner (set connection type to tcp and make sure baudrate (number to the right of it) is set to 115200)

3. Set Hostname to 127.0.0.1 and Port to default port 5763 and connect

4. In Mission Planner once the parameters are loaded you should see an actions on the left hand side under the attitude indicator, under the actions tab you should see a  Set Mode in the first column of buttons. In the drop down selection to the left of the set mode button, change the mode to guided click the set mode button

5. You should now see the drone start flying in the football field doing the 30x30yd square search pattern

6. If you want to set waypoints that are not the calculation based 30x30 yd continue reading (This is likely how we will set it on competition day)

<h3> SITL continued:</h3>

7. If simulation is still running terminate the DroneSimTest.py program and restart it. 

8. Connect Mission Planner to sim as before and go to the Plan tab on the top left navigation panel

10. Click the Read button on the right hand side

11. Set waypoints 1-4 from the program in a clockwise manner where 1 is top left corner of box

12. Set waypoint 5 for landing zone can be anywhere.

13. Click Save File if you want for future use

14. Click Write and you should see it populate the waypoints in the data tab 

> **NOTE:** if this fails just click read and clear the points and load the file or place points again and write. 

15. Change mode to guided and it should work according to the set waypoints

<h3>SITL Resources</h3>

- [SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

- [dronekit-sitl](https://github.com/dronekit/dronekit-sitl)

- [pyMavlink docs](https://mavlink.io/en/mavgen_python/)

- [Videos on how to use pyMavlink](https://www.youtube.com/watch?v=kecnaxlUiTY&list=PLy9nLDKxDN68cwdt5EznyAul6R8mUSNou)



<h2> Camera Simulation</h2>

<p>

**NOTE:** To run ```cameratest.py``` you do not need Zed SDK as it should use the default camera on your system

</p>

1. To use the Zed Camera you will need the Zed SDK (this is already on the jetson, however if you want to run and test with that specific camera on a personal computer you will need this)

2. Use a 6x6 ArUco marker to test 

<h3>Resources</h3>

- [ArUco Docs](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

- [Zed Api Docs](https://www.stereolabs.com/docs)


<h2>Running Real Drone</h2>

<p>When running code on live drone be sure to take necessary precautions, have a FAA licenced drone pilot at all times, even when operating autonomously. Be sure to test thouroughly before using on the real drone.</p>

1. Power drone and connect both Mission Planner and Remote to drone

2. Setup a mission on mission planner, create an inclusion zone aka "GeoFence" on mission planner along with the 5 waypoints setup same as step 7 under **SITL continued**.

3. VNC/SSH into Jetson

4. Run The following command in DroneCode Directory

        python IntegratedUAVcode.py --livedrone

<h3>Resources</h3>

- [Inclusion Zone Setup](https://www.youtube.com/watch?v=yhNrtTERnJk)

<h2>ESP32 Communication System</h2>

- Someone please add info

<h2>General Resrouces/Tips/Notes</h2>

- Read all forms of documentation thoroughly

- Ask ChatGPT as last resource unless you are making some new functionality and don't have to provide it a lot of context


<h3>USEFUL LINKS: </h3>

- [ArduPilot Docs](https://ardupilot.org/dev/index.html)

- [Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html)

- [Zed Api Docs](https://www.stereolabs.com/docs)

- [ArUco Docs](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

- [SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

- [dronekit-sitl](https://github.com/dronekit/dronekit-sitl)

- [pyMavlink docs](https://mavlink.io/en/mavgen_python/)
