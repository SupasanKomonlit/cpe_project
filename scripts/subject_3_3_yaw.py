#!/usr/bin/env python2
# FILE      : subject_3_3_yaw.py
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

class Subject33Yaw:

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
                    reset_state = False

                self.ch.pub( "Waiting z ok position in one time" )
                
                while self.ch.check_error( z = 0.05 ) and self.ch.ok():
                    self.ch.sleep()

                count_round = 0
                save_state = self.ch.get_target()[1][0]
                self.ch.pub( "Start posiive rotation yaw on " + str( save_state ) )
                while count_round < 3 and self.ch.ok():
                    
                    while self.ch.ok():
                        self.ch.sleep()
                        self.ch.target_velocity( 0.6 )
                        if abs( bound_radian( save_state - self.ch[5] ) ) > 1:
                            break
                    
                    while self.ch.ok():
                        self.ch.sleep()
                        self.ch.target_velocity( 0.6 )
                        if abs( bound_radian( save_state - self.ch[ 5] ) ) < 0.5:
                            count_round += 1
                            self.ch.pub( "positive rotation " + str( count_round ) + " round" )
                            break
                                
                count_round = 0
                self.ch.pub( "Start negative rotation yaw on " + str( save_state ) )
                while count_round < 3 and self.ch.ok():
                    
                    while self.ch.ok():
                        self.ch.sleep()
                        self.ch.target_velocity( -0.6 )
                        if abs( bound_radian( save_state - self.ch[5] ) ) > 1:
                            break
                    
                    while self.ch.ok():
                        self.ch.sleep()
                        self.ch.target_velocity( -0.6 )
                        if abs( bound_radian( save_state - self.ch[ 5] ) ) < 0.5:
                            count_round += 1
                            self.ch.pub( "negative rotation " + str( count_round ) + " round" )
                            break

                self.ch.absolute_depth( -0.15 )
                self.ch.absolute_yaw( save_state )
                while self.ch.check_error( yaw = 0.1 ) and self.ch.ok():
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
    mission = Subject33Yaw()
    mission.activate()
