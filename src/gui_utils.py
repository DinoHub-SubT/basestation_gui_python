#!/usr/bin/env python

'''
Contains variosu functions that may be useful by multiple plugins in the GUI
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

def displaySeconds(seconds):
	'''
	Function to convert seconds float into a min:sec string

	seconds is a float
	'''
	return "%02d:%02d" % (seconds / 60, seconds % 60)