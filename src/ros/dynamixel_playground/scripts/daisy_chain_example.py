#!/usr/bin/env python
import time

import roslib;
roslib.load_manifest('dynamixel_msgs')

import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

# A map from joint name to the last received joint state
JOINT_STATES = {}

# What states w can be in
class State:
    START   = 'START'       # Start state
    WAITING = 'WAITING'     # Waiting for all controllers to broadcast one state
    UP      = 'UP'          # Have recieved at least one state from each joint

# What state we're in.
STATE = State.START

# Our command publishers
PUBS = []

def transition(new_state):
    """Called to move us from one state to another."""
    # We need to use global here because we are modifying the STATE variable.
    global STATE

    old_state = STATE
    rospy.loginfo('Transitioning from {0} to {1}'.format(old_state, new_state))

    STATE = new_state

    # Were we waiting to be up?
    if old_state == State.WAITING and new_state == State.UP:
        rospy.loginfo('All controllers are now up.')
        reset_all()

        # Sleep for a bit
        r = rospy.Rate(0.5) # Hz
        r.sleep()

        # Set each controller
        r = rospy.Rate(2) # Hz
        for p in PUBS:
            if rospy.is_shutdown():
                continue
            p.publish(-0.5)
            r.sleep()

        # Sleep for a bit
        r = rospy.Rate(0.5) # Hz
        r.sleep()

        # Set each controller
        r = rospy.Rate(2) # Hz
        for p in PUBS[::-1]:
            if rospy.is_shutdown():
                continue
            p.publish(0.5)
            r.sleep()

        # Sleep for a bit
        r = rospy.Rate(0.5) # Hz
        r.sleep()

        reset_all()

def reset_all():
    # Reset all actuators to centre
    rospy.loginfo('Resetting all actuators...')
    r = rospy.Rate(20) # Hz
    for p in PUBS:
        if rospy.is_shutdown():
            continue
        p.publish(0)
        r.sleep()

def joint_state_cb(data):
    # We do not need to use global here because we are modifying the *value* of PUBS
    # and not the variable itself.
    JOINT_STATES[data.name] = data

    # Are we waiting for each controller?
    if len(JOINT_STATES) == 7 and STATE == State.WAITING:
        transition(State.UP)

def main():
    rospy.init_node('daisy_chain_example', anonymous=True)

    # Create a publisher for each id 1, 2, ..., 7
    # We do not need to use global here because we are modifying the *value* of PUBS
    # and not the variable itself.
    PUBS.extend([rospy.Publisher('a{0}/command'.format(id_), Float64, queue_size=10)
                for id_ in range(1, 8)])
    rospy.loginfo('Created {0} publishers'.format(len(PUBS)))

    # Create a subscriber for each id 1, 2, ..., 7
    subs = [rospy.Subscriber('a{0}/state'.format(id_), JointState, joint_state_cb)
            for id_ in range(1, 8)]
    rospy.loginfo('Created {0} subscribers'.format(len(subs)))

    transition(State.WAITING)

    # Wait for messages
    rospy.spin()

if __name__ == '__main__':
    main()
