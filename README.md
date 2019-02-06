# Overview
A GUI for sending command to the robots and receiving information other than map data. Based on John's state machine gui package.

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
Run 
```bash
roscore
rqt
```

In the "Plugins" menu at the top "Basestation Gui" will be listed and when you click it, it gets added to rqt. If its not an option, close the window and run:
```bash
rqt --force-discover
```

Basestation Gui should now appear as an option in the Plugins dropdown. Select it. 

The first time you may have to click the "Open Config..." button in the GUI and select the config/gui_params.yaml file. This will get saved as the default when you run it in the future.

The first time you load the GUI it may not be large enough to see everything. Feel free to resize (standard click and drag on the window corners) it should remember the window size from now on. 




# Who to contact
Bob DeBortoli: debortor@oregonstate.edu
