Overview
========

Primary application for the Basestation operator to interact with the SubT robots.


Installation
============

Create a workspace to host the Basestation repository and its required dependencies:
```bash
mkdir -p basestation_ws/src
cd basestation_ws/src
```

Then clone the repository:
`git clone git@bitbucket.org:cmusubt/basestation_gui_python.git`


ROS Dependencies
----------------

- Standard ROS packages:
  `sudo apt-get install ros-melodic-mavros`

- Basestation messages package to communicate with robots:
  `git clone git@bitbucket.org:cmusubt/basestation_msgs.git`

- (Opitonal) DARPA command post simulator for GUI testing:
  `git clone git@bitbucket.org:cmusubt/darpa_command_post.git`

- The *entrance_calib* package is required to perform robot calibrations:
  `git clone git@bitbucket.org:cmusubt/entrance_calib.git`


Python2.7 Dependencies
----------------------

- *pyserial*:  For DARPA E-Stop commands
  `pip2 install pyserial`

- *requests*:  To communicate with the DARPA command post
  `pip2 install requests`

- *shapely*:  To coordinate robot exploration areas
  `pip2 install shapely`


Development Dependencies
------------------------

Basestation is developed using rospy and thus requires Python2.7.  However, for
consistency of development amongst several developers we use
[black](https://github.com/python/black) for formatting of Python source code which
requires Python3.6.  The following will help with this and allow the two to co-exist:

- Install Python3 and pip3 if it's not already installed:
  `sudo apt-get install python3 python3-pip`

- Install _black_ with Python3's pip: `pip3 install black`

- (Optional) Ensure Python3's site-packages directory is on your path if you can't run
  _black_ after installing (see last bullet).  Running Ubuntu you should have a _.profile_
  file in your home directory.  In that file, somewhere at the bottom you should add the
  line: `export PATH="$HOME/.local/lib/python3.6/site-packages:$PATH"`

- If you modified your _.profile_ then re-source it: `source ~/.profile`

- Now you can run _black_ on your code: `black my_python_file.py`

Note that we use _black's_ defaults for formatting thus allowing for zero configuration
and keeping things simple.  If you wish to integrate _black_ within your editor workflow
then check out its project page from the link above for setup with your editor of choice.


Building
========

From the workspace that was created in the step above:
```bash
catkin build
source devel/setup.bash
```


Quick Start
===========

The following demonstrates how to run the Basestation application along with the DARPA
command post simulator.  This assumes you created `basestation_ws/src` workspace
directory listed in the _install_ step above.

Window 1
--------

```bash
cd ~/basestation_ws
source devel/setup.bash
rosrun darpa_command_post CommandPostScoring.py
```

Window 2
--------

```bash
cd ~/basestation_ws
source devel/setup.bash
roslaunch basestation_gui_python gui.launch connect_to_command_post:=true
```


Running
=======

(Optional) To simulate the DARPA command post:
`rosrun darpa_command_post CommandPostScoring.py`

To simulate artifact detections in another sourced terminal:
`rosrun basestation_gui_python fake_artifact_detections.py`

For the Basestation Application:
`roslaunch basestation_gui_python gui.launch connect_to_command_post:=false simulating_artifact_detections:=true`

The `simulating_darpa_command_post` argument designates whether the simulated DARPA
command post is running.  The default is *false* but when set to *true* the command post
should be run first.

The `simulating_artifact_detections` argument designates whether to publish fake artifact
detections (as radio messages).  The default is *false*.

The _Plugins_ menu of the running application contains a _Basestation_ sub-menu that
contains all the Basestation specific plugins, which are all loaded by default.  If it is
not listed then run `bash rqt --force-discover` and restart the application.


RViz Interaction
================

The Basestation application offers a couple of features that can interact with a running
RViz instance.  For this to work correctly the *interactiveMarkerProcessing* node needs to
have its *reference_frame* match the one that is listed in RViz.  This is inside the
*gui.launch* file for the Basestation application.  With that set, within the Basestation
application, one can define a waypoint to order a robot to move to a specific position, or
can change the location of a discovered artifact.

Defining Waypoints
------------------

To define a waypoint add the `/define_waypoint/update` topic inside of RViz.  When the
*Define Waypoint* for a specific robot is pressed a pop-up will inform the user to move
the interactive marker in RViz.  Move the marker inside RViz and once the desired position
is set then click _Ok_ from the pop-up of the Basestation application.  The robot will now
be ordered to move to that application if possible.  If one wishes to cancel the marker
placement simply press the _Cancel_ button inside of the pop-up.

Artifact Refinement
-------------------

To change the location of a discovered artifact add the `/artifact_refinement/update`
topic inside of RViz.  Then for any artifact click its _Refine_ button to bring up a
pop-up that allows the changing of a location manually or by using RViz.  To use RViz
click the _Show RViz Marker_ button, move the marker inside RViz to the desired location,
and click the same button again.  Upon clicking the button a second time the text fields
inside the pop-up will be populated with the selected location.  Click _Ok_ to accept the
new position or _Cancel_ to keep the original position.

Robot Coordination
------------------

The robot coordination plugin allows the coordination of exploration between several
robots.  The following controls allow one to resize and position polygons within the
plugin:

* Click and drag inside a polygon to translate it.
* Click and drag a polygon vertex to move it.
* Click and drag a polygon edge to insert a vertex and move it.
* Right click a polygon vertex to remove it.
* Scroll wheel to zoom in and out.
* Click and drag outside a polygon to translate view.
* Click robot button to enable/disable the movement of the associated polygon.
* Click reset view to reset the center and the zoom.
* Click reset robots to reset polygons to initial configuration.


Adding New Plugins
==================

Generate a .py file following the convention of the Gui.py format (inherit from the Plugin
class, etc.).  An barebones example is in src/SkeletonPlugin.py or see one of the existing
plugins.

In the basestation_gui_python/plugin.xml file add the plugin as a new class. Keep it in
the same group as the other plugins (i.e. *Basestation*).

Launch the application using the normal launch command.  If the pluging does not show up in
the dropdown:

- Close the application.
- Run `rqt --force-discover`.
- Restart the Basestation application.

To have the plugin loaded with the default Basestation application plugin layout:

- Position all of the plugins how you want them in the rqt window.
- Under the _Perspectives_ menu select *Export*
- Name the perspective file and save.  The default perspective file is
  *subt_tests_gui.perspective*, which is under the _config_ directory of the Basestation
  repository, if you wish to override the default.


Known Issues / TODO
===================

* (D+V) Handling bluetooth/audio detections
* (GB) Overlay map (downsampled point cloud)
* (GB) Overlay robot locations
* (GB) Transofrm polygons into map frame
* (GB) Publish polygons to robots (in map frame)


Contact
=======

Bob DeBortoli: debortor@oregonstate.edu
Graem Best: bestg@oregonstate.edu
