#!/usr/bin/env python2
# FILE      : subject_3_3_roll.py
# AUTHOR    : K.Supasan
# CREATE ON : 2020, Febuary 16
# MAINTAINER: K.Supasan

# README

# REFERENCE

import rospy

from std_msgs.msg import Bool

from zeabus.ros import message as nm

from zeabus.math.general import bound_radian , equal , same_sign

from zeabus.connvertion.control import ControlHandle

from zeabus_utility.srv import SendBool, SendBoolResponse

class Subject33Roll:

    def __init__( self ):
        rospy.init_node( "project33" )

        self.ch = ControlHandle( "project33" )
    
        self.state = False

        self.server_state = rospy.Service( "/project/activate" ,
                SendBool,
                self.callback_state )

    def activate( self ):
        self.ch.activate( False )
        while self.ch.ok() : # Loop activate node
            rospy.sleep( 0.25 )
            reset_state = True
            while self.state : # Lopp working

                if reset_state:
                    self.ch.activate( False )
                    self.ch.reset_all( False ) # Reset only orientation
                    self.ch.absolute_pitch( 0 )
                    self.ch.absolute_roll( 0 )
                    self.ch.absolute_depth( -1 )
                    self.ch.set_mask()
                    reset_state = False

                self.ch.pub( "Waiting z ok position in one time" )
                
                while self.ch.check_error( z = 0.05 ) and self.ch.ok():
                    self.ch.sleep()


                self.ch.set_mask( roll = False )

                self.ch.pub( "Command addition force positive roll" )
                start_time = rospy.get_rostime()
                while self.ch.ok() and ( rospy.get_rostime() - start_time ).to_sec() < 15:
                    self.ch.add_force( roll = 2 )
                    self.ch.sleep()

                self.ch.pub( "Command addition force negative roll" )
                start_time = rospy.get_rostime()
                while self.ch.ok() and ( rospy.get_rostime() - start_time ).to_sec() < 15:
                    self.ch.add_force( roll = -2 )
                    self.ch.sleep()

                self.ch.absolute_depth( -0.15 )
                self.ch.set_mask()
                while self.ch.check_error( z = 0.2 ) and self.ch.ok():
                    self.ch.sleep()
                self.ch.activate( False ) 
                self.ch.pub( "Finish stop process" )
                self.state = False

                if self.ch.ok():
                    print( "Stop process by ros shutdon" )
                    break

    def callback_state( self , request ):
        self.state = request.data
        return SendBoolResponse()

if __name__ == "__main__":
    mission = Subject33Roll()
    mission.activate()
