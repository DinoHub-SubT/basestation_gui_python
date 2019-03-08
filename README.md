# Overview
A GUI for sending command to the robots and receiving information other than map data. Based on John's state machine gui package.



# How to install
Install dependencies
```bash
sudo apt-get install ros-melodic-mavros
```

Make new workspace
```bash
mkdir -p basestation_ws/src
cd basestation_ws/src
```

First, you'll need Graeme's DARPA command post simulator 
```bash
git clone git@bitbucket.org:cmusubt/darpa_command_post.git
```

Install gui repo
```bash
git clone git@bitbucket.org:cmusubt/basestation_gui_python.git
cd ..
catkin build
source devel/setup.bash
```

# How to run
(Optional) If you want to simulate the darpa command post, run:
```bash
rosrun darpa_command_post CommandPostScoring.py
```

In another sourced terminal, run:
```bash
roslaunch basestation_gui_python gui.launch simulating_command_post:=false simulating_artifact_detections:=true
```

The ```simulating_darpa_command_post``` argument is a boolean one to designate whether the simulated darpa command post is running. The default is "false". If you make it true, you must launch the CommandPostScoring first.
The ```simulating_artifact_detections``` argument is a boolean one to designate whether to publish fake artifact detections (as radio messages). The default is "false".

In the "Plugins" menu at the top "Basestation Gui" will be listed and when you click it, it gets added to rqt. If its not an option, close the window and run:
```bash
rqt --force-discover
```

Basestation Gui should now appear as an option in the Plugins dropdown. If it doesn't, ensure you're running the launch command in the same terminal as the one that has been sourced.  

You may get some red text involving "TypeError: coercing to Unicode: need string or buffer, NoneType found". Ignore it. 

The first time you may have to click the "Open Config..." button in the GUI and select the config/gui_params.yaml file. This will get saved as the default when you run it in the future.



# Depricated: How to install and run old simple 4-button estop gui
## Install old 4-button gui
```bash
sudo apt-get install ros-kinetic-mavros
sudo apt-get install python-tk
cd
mkdir basestation_ws
cd basestation_ws
mkdir src
cd src
git clone git@bitbucket.org:cmusubt/basestation_gui_python.git
cd basestation_gui_python/scripts
chmod u+x gui_simple_estop.py
roscd
cd ..
catkin_make
source devel/setup.bash
```

## Run old 4-button gui
```bash
rosrun basestation_gui_python gui_simple_estop.py
```

## This line ends the depricated section


# Who to contact
Bob DeBortoli: debortor@oregonstate.edu
