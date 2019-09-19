# Multi-Robot-Mission-Planner

## Description
In this project we have created a custom mission planner / ground control station for multi robot systems. In an earlier project a basic mission planner was created which has a web interface where the user can specify line tasks for multiple robots. The goal of this project was to extend this mission planner to allow users to specify more different tasks and configure the robots by adding parameters and constraints. The mission planner should also take into account more variables when deciding what robots should do which task.

Ardupilot is the autopilot software used for controlling the robots, and needs to be running on all the robots if using real robots. For simulation we use SITL(Software In The Loop) which is a simulation tool for Ardupilot and is used to simulate different vehicles.



<br />

## Setup
This section describes how to setup the tool on windows. All dependent software should be compatible with linux and os x, but will require different installation steps.

Copy the script below to a powershell window to download and install MavProxy, Cygwin, SITL, Ardupilot and Python packages. If the script fails the setup tutorial on the official SITL website needs to be followed instead.
http://ardupilot.org/dev/docs/sitl-native-on-windows.html 

```
Import-Module BitsTransfer

Write-Output "Starting Downloads"

Write-Output "Downloading MAVProxy (1/7)"
Start-BitsTransfer -Source "http://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe" -Destination "$PSScriptRoot\MAVProxySetup-latest.exe"

Write-Output "Downloading Cygwin x64 (2/7)"
Start-BitsTransfer -Source "https://cygwin.com/setup-x86_64.exe" -Destination "$PSScriptRoot\setup-x86_64.exe"

Write-Output "Installing Cygwin x64 (3/7)"
Start-Process -wait -FilePath $PSScriptRoot\setup-x86_64.exe -ArgumentList "--root=C:\cygwin --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site=http://cygwin.mirror.constant.com --packages autoconf,automake,ccache,gcc-g++,git,libtool,make,gawk,libexpat-devel,libxml2-devel,python2,python2-future,python2-libxml2,python2-pip,libxslt-devel,python2-devel,procps-ng,zip,gdb,ddd --quiet-mode"

Write-Output "Copying JSBSim and APM install script to Cygwin (4/7)"
Start-BitsTransfer -Source "https://github.com/ArduPilot/ardupilot/raw/master/Tools/autotest/win_sitl/jsbsimAPM_install.sh" -Destination "C:\cygwin\home\jsbsimAPM_install.sh"

Write-Output "Downloading extra Python packages (5/7)"
Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c 'pip2 install empy'"

Write-Output "Downloading and installing JSBSim, then downloading APM source (6/7)"
Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c ../jsbsimAPM_install.sh"

Write-Output "Installing MAVProxy (7/7)"
& $PSScriptRoot\MAVProxySetup-latest.exe /SILENT | Out-Null

Write-Host "Finished. Press any key to continue ..."
$x = $host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")


```


<br /> 
After the download is complete you need to setup path variable to ardupilot

1. Open and close Cygwin from the desktop to create initialisation files
2. Navigate to the .bashrc file on your computer (e.g. C:\cygwin\home\user_name\.bashrc
3. Add the following line to the end of .bashrc

```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
```


<br /> 
Then you need to build the code.
In a new Cygwin terminal navigate to the ArduCopter directory and run make.

```
cd ~/ardupilot/ArduCopter
make sitl -j4
```


<br /> 

## How to use
This section describes how to use the mission planner.


### Add vehicles
To add a new simulated vehicle click the add drone button, then the copy button and paste the command into cygwin and click connect. 

![alt text](https://github.com/95danlos/Multi-Robot-Mission-Planner/blob/master/Images/MissionPlanner_img_1.png)

<br /> 


Alternatively you can startup the vehicles manually by navigating to ArduCopter in Cygwin and start the simulation.

Cygwin terminal 1
```
cd ~/ardupilot/ArduCopter  
sim_vehicle.py -L KSFO -I 0
```

Cygwin terminal 2
```
cd ~/ardupilot/ArduCopter  
sim_vehicle.py -L KSFO -I 1
```

Then start the Mission Planner with the number of vehicles you have
```
python Mission_Planner.py 2
```


<br /> 

### Define Tasks
The panel at the right side of the screen lets you define tasks. So far there are three types of tasks. Line task makes a vehicle fly from one point to another. Pickup task is the same as line but with an associated weight. Search task makes the vehicle move over the defined area.

Obstacles can also be defined. The vehicles will try to find the shortest path around the obstacles. The algorithm is not optimized yet, and to many obstacles will result in a crash due to python max recursion depth error when calculating the path.

![alt text](https://github.com/95danlos/Multi-Robot-Mission-Planner/blob/master/Images/MissionPlanner_img_5.png)

<br /> 


### Flight Data and Vehicle Parameters
In the flight data window sensor data for each vehicle is displayed. It is also possible to change the parameters for each vehicle, like flight speed.

![alt text](https://github.com/95danlos/Multi-Robot-Mission-Planner/blob/master/Images/MissionPlanner_img_2.png)
![alt text](https://github.com/95danlos/Multi-Robot-Mission-Planner/blob/master/Images/MissionPlanner_img_3.png)


<br /> 

## Future work
As for future work on the mission planner, more types of tasks can be add, and more parameters which can be used when deciding which vehicle should take which task.
To be able to decide which vehicle can complete a specific task fastest, speed and acceleration should also be taken into account, which can be difficult for search tasks where there is a lot of turning. The obstacle avoidance algorithm also needs some adjustments.

Adaptive planning can also be implemented, where tasks are reallocated between the vehicles if something happens or new tasks or vehicles are added in the middle of a mission. Collision avoidance between the vehicles should also be taken into account when planning the path. Precedence and temporal constraints can also be added, where one can specify which tasks needs to be  completed before other tasks or within a time limit.

Testing with more types of different vehicle should be done. This should be easy since Ardupilot supports a number of different vehicles. And testing the mission planner with real vehicles should also be done.

