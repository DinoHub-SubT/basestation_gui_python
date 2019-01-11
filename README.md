# how to install
install using the following (note this creates a new workspace basestation_ws):
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
chmod u+x gui.py
roscd
cd ..
catkin_make
source devel/setup.bash
```

# how to run
run using the following:
```bash
rosrun basestation_gui_python main.py
```

