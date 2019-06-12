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

You will also need Ji's entrance_calib package. 
```bash
git clone git@bitbucket.org:cmusubt/entrance_calib.git
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

If you wish to simulate artifact detections, run the fake publisher first in a sourced terminal:
```bash
rosrun basestation_gui_python fake_artifact_detections.py 
```

In another sourced terminal, run:
```bash
roslaunch basestation_gui_python gui.launch connect_to_command_post:=false simulating_artifact_detections:=true
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

# Artifact Refinement

To do artifact refinement, two markers in RViz need to be subscribed to:

```bash
/basic_controls/update
```
which is an interactive marker for updating the artifact location. And

```bash
/refinement_marker_orig_pos
```

which is a MarkerArray of two markers. A green sphere to designate the original location of the artifact and a red arrow pointing to the artifact location. 

Once these are subscribed to, back in the GUI, select an artifact to display in the artifact panel (by double-clicking on the artifact in the queue). Then, in the middle of the GUI there is a Show Refinement Marker button, click it. A large red arrow should appear in RViz, which points down to indicate the location of the refinement. Below it will be the orange interactive marker and hidden beneath the orange sphere is a static green sphere which is the original position marker.


# Defining waypoints

To define a waypoint, you need to add the interatice marker topic
```bash
/define_waypoint/update
```
to RViz. This marker will become visible upon selecting 1+ of the "Define waypoint" buttons in the gui. Its starting location is set as the robot pose. This marker then can be moved around to its final location by clicking and dragging in RViz. Once its final location is set, de-select the button in the gui, this de-selection will publish a RadioMsg waypoint to the robot. 

# To add a new plugin

Generate a .py file following the convention of the Gui.py format (inherit from the Plugin class, etc.). An barebones example is in src/SkeletonPlugin.py

In the basestation_gui_python/plugin.xml file add the plugin as a new class. Keep it in the same group as the other plugins (Basestation Gui).

In package.xml, add the following line within the existing export tag:
```bash
<rqt_gui plugin="${prefix}/plugin.xml"/>
```

Launch the gui using the normal launch command. If the pluging does not show up in the dropdown:
	-Close the gui
	-Run rqt --force-discover. 
	-It should now show up in the drop-down for plugins. 

To ensure that its displayed everytime the gui is roslaunched, add it to the perspective. To do this:
	-Position all of the plugins how you want them in the rqt window
	-On the top bar there is a Perspective tab. Click it
	-Export the perspective
	-Save it as whatever the gui.launch file calls. Probably /config/subt_test_perspective or something like that. 

# Known issues / TODO

* (D+V+B) Take out /ugv1/real_artifact_imgs hardcoded topic in the ArtifactHandler and think of some naming convention. More generally, defining a naming convention for topic types w.r.t. artifact detection. 

* (D) Saving/loading of gui from CSV. In case the gui crashes, we want to be able to load from a saved .csv or something like that. Either load from button press or tab on top 

* (D) Inherit from the base class for the interactiveMarkerProcessing.py file?

* (D) Integrating of darpa transform into gui

* (D) Convert functionality for artifact refinement marker from old gui to modularized gui

* (D) Artifacts, upon pressing "submit" are deleted from the queue. This could be a problem if, for example, the artifact tries to be submitted and the connection to the command post breaks. This artifact is then effectively "lost". It sits in the ArtifactHandler archived. It might be best to only delete from the table once the confirmation from darpa comes back? If the unique_id is still persisitent. Otherwise some other solution. 

** This is, depending on how you look at it, an effect of a bigger problem: if you close the artifact queue, that info will be out of sync with the artifact handler. some way to refresh with artifacthandler info? Either by button press or maybe automatically at 1Hz or so

* (D) Pull out radio and wifi messages into some other more lightweight and ubiquitous package in order to all them to have the messag without installing the entire basestation package

* (D+V) Handling bluetooth/audio detections

* (Unassigned) The Signal/Slot triggers should be intialized in the init function? Not outside of it. 

* (Unassigned) Clean up branches on basestation_gui_python package

* (Unassigned) Fully utilizing tabs in rqt. It looks like you can just drag plugins on top of each other and it will automatically generate tabs 

* (Unassigned) Don't deepcopy immutable objects in ArtifactHandler, like when new GuiArtifact objects are generated. 

* (Unassigned) In the manipulation plugin, in updateArtifactPose callback, Might be less code to just have every manual update of the position update all 3 coordinates (e.g. have the same callback for all 3 coordinate boxes).

=======
# Development

Basestation_gui is developed using rospy and thus requires Python2.7.  However, for
consistency of development amongst several developers we use
[black](https://github.com/python/black) for all formatting of Python source code which
requires Python3.6.  Playing with different Python installations isn't fun so the
following instructions will make it easy to format the source code:

* Install Python3 and pip3 if it's not already installed:
  `sudo apt-get install python3 python3-pip`
* Install _black_ with Python3's pip: `pip3 install black`
* (Optional) Ensure Python3's site-packages directory is on your path if you can't run
  _black_ after installing (see last bullet).  Running Ubuntu you should have a _.profile_
  file in your home directory.  In that file, somewhere at the bottom you should add the
  line: `export PATH="$HOME/.local/lib/python3.6/site-packages:$PATH"`
* If you modified your _.profile_ then re-source it: `source ~/.profile`
* Now you can run _black_ on your code: `black my_python_file.py`

Note that we use _black's_ defaults for formatting thus allowing for zero configuration
and keeping things simple.  If you wish to integrate _black_ within your editor workflow
then check out its project page from the link above for setup with your editor of choice.


=======
# Troubleshooting
For some reason, sometimes it wants to launch twice (you'll see 2 messages about an HTTP server being launched). To fix this error for now, run 
```bash
rqt --clear-config 
```
and re-launch the gui


# Quick Start Guide

##Window 1



```bash
cd home/name/workspaces/gui/
source devel/setup.bash
rosrun darpa_command_post CommandPostScoring.py
```


##Window 2

```bash
cd home/name/workspaces/gui/
source devel/setup.bash
roslaunch basestation_gui_python gui.launch connect_to_command_post:=false simulating_artifact_detections:=true
```


# Who to contact
Bob DeBortoli: debortor@oregonstate.edu
