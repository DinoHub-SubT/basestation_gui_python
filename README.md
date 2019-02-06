# Overview
A GUI for sending command to the robots and receiving information other than map data. Based on John's state machine gui package.

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


# How to install
install dependencies
```bash
sudo apt-get install python-requests
sudo apt-get install ros-melodic-mavros
```

Make new workspace
```bash
mkdir -p basestation_ws/src
cd basestation_ws/src
```

Install repo
```bash
git clone git@bitbucket.org:cmusubt/basestation_gui_python.git
cd basestation_gui_python
git checkout bob-initial-branch
cd ../..
catkin_make
source devel/setup.bash
```

# How to run
In another terminal, run:
```bash
roscore
```

In ther terminal that has sourced the basestation_ws, run:
```bash
rqt
```

In the "Plugins" menu at the top "Basestation Gui" will be listed and when you click it, it gets added to rqt. If its not an option, close the window and run:
```bash
rqt --force-discover
```

Basestaion Gui should now appear as an option in the Plugins dropdown. Select it. 

You may get some red text involving "TypeError: coercing to Unicode: need string or buffer, NoneType found". Ignore it. 

The first time you may have to click the "Open Config..." button in the GUI and select the config/gui_params.yaml file. This will get saved as the default when you run it in the future.





# Who to contact
Bob DeBortoli: debortor@oregonstate.edu
