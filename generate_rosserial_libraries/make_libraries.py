#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##################
# EDITS BY GRAEME
# this has been copied from the rosserial_arduino library
# all i've done is added the custom basestation_gui_python::RadioMsg message type
#
# compile the .h files for arduino by running:
# rm -rf <<arduino_sketchbook>>/libraries/ros_lib (if that directory already exists)
# source <<basestation_ws>>/devel/setup.bash
# rosrun basestation_gui_python make_libraries.py <<arduino_sketchbook>>/libraries
#
# make sure your arduino program has it's sketchbook pointing at the <<arduino_sketchbook>>
# then you should be able to compile the TeensyRosserialRadio stuff without any complaints of "missing .h file"
#################

THIS_PACKAGE = "rosserial_arduino"

__usage__ = """
make_libraries.py generates the Arduino rosserial library files.  It 
requires the location of your Arduino sketchbook/libraries folder.

rosrun rosserial_arduino make_libraries.py <output_path>
"""

import rospkg
import rosserial_client
from rosserial_client.make_library import *

# for copying files
import shutil

ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          8, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('float',             4, AVR_Float64DataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header']),
'Radio':   ('basestation_msgs::Radio',  0, MessageDataType, ['basestation_msgs/Radio'])
}

# need correct inputs
if (len(sys.argv) < 2):
    print __usage__
    exit()
    
# get output path
path = sys.argv[1]
if path[-1] == "/":
    path = path[0:-1]
print "\nExporting to %s" % path

rospack = rospkg.RosPack()

# copy ros_lib stuff in
rosserial_arduino_dir = rospack.get_path(THIS_PACKAGE)
shutil.copytree(rosserial_arduino_dir+"/src/ros_lib", path+"/ros_lib")
rosserial_client_copy_files(rospack, path+"/ros_lib/")

# generate messages
rosserial_generate(rospack, path+"/ros_lib", ROS_TO_EMBEDDED_TYPES)

