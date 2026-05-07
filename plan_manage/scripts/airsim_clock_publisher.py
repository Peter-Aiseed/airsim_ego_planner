#!/usr/bin/env python
import rospy
import time
from rosgraph_msgs.msg import Clock
import airsim

def publish_airsim_clock():
    rospy.init_node('airsim_clock_publisher')
    pub = rospy.Publisher('/clock', Clock, queue_size=10)
    
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    rospy.loginfo("Clock publisher connected to AirSim. Publishing simulation time to /clock topic.")

    # We loop at Wall Time frequency
    clock_frequency = 100.0  # Hz
    sleep_time = 1.0 / clock_frequency
    
    while not rospy.is_shutdown():
        # Get simulation time from AirSim (in nanoseconds)
        sim_time_ns = client.getMultirotorState().timestamp
        
        # Convert to ROS Clock message
        ros_clock = Clock()
        ros_clock.clock.secs = int(sim_time_ns / 1e9)
        ros_clock.clock.nsecs = int(sim_time_ns % 1e9)
        
        pub.publish(ros_clock)
        time.sleep(sleep_time)

if __name__ == '__main__':
    try:
        publish_airsim_clock()
    except rospy.ROSInterruptException:
        pass