#!/usr/bin/env python

from std_msgs.msg import Int32



def pub_estop(number, pubs):
    '''
    Function to publish estop message (an int) after a button press
    '''
    print pubs
    #TODO: add window pop-up to this method to check if they really want to hard e-stop
    print(number)
    msg_int = Int32()
    msg_int.data = number
    pubs.publish(msg_int)
    