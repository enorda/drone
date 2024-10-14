# 2024-25 UTD Raytheon UAX Drone Competition: Scout Drone

<p>The following README has instructions to setup, run, and simulate the code within this repository with the jetson</p>
<p>This repository is meant to be loaded onto the NVidia Jetson Orin Nano related with autonomous drone flight, simulation, and visual processing.</p>

<h2> Drone/Jetson</h2>

<h2> SITL Simulation</h2>

1. from drone local repository use command in terminal: 

        pip install -r requirements.txt
2. install mission planner https://github.com/dronekit/dronekit-python/issues/1132 (collections.abc.MutableMapping Change in dronekit/__init__.py)
3. Run Test
4. Click Connect on Top Right Corner on Mission Planner (set connection type to tcp)
5. Set Hostname to 127.0.0.1 and Port to default port 5763
