#!/usr/bin/env python3
import rospy
import airsim
import threading
import numpy as np
import math
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf.transformations as tf

class AirSimBridge:
    def __init__(self):
        rospy.init_node('ego_to_airsim_bridge')

        # Register Shutdown Hook (The Safety Net)
        # This will run even if you Ctrl+C the terminal
        rospy.on_shutdown(self.emergency_landing)

        # 1. AirSim Connection
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, "drone_0")
        self.client.armDisarm(True, "drone_0")

        rospy.loginfo("Taking off...")
        self.client.takeoffAsync(vehicle_name="drone_0").join()

        rospy.loginfo("Climbing to safe altitude (5.0m)...")
        self.client.moveToZAsync(-5.0, velocity=1, vehicle_name="drone_0").join()

        # 2. Shared State Variables (Updated by Callback)
        self.target_pos = np.array([0.0, 0.0, -5.0]) # Start at 5m altitude
        self.target_vel = np.array([0.0, 0.0, 0.0])
        self.target_yaw = 0.0
        self.cmd_received = False
        self.running = True
        self.lock = threading.Lock()
        
        # 3. Subscriber and Publisher
        rospy.Subscriber("/drone_0_planning/pos_cmd", PositionCommand, self.callback)
        rospy.Subscriber("/airsim_node/drone_0/odom_local_ned", Odometry, self.odomCallback, queue_size=10)

        self.odom_pub = rospy.Publisher("/airsim_node/drone_0/odom_local_enu", Odometry, queue_size=10)

        # 4. Start the Control Thread (Fixed 50Hz)
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        rospy.loginfo("Ego-planner to Airsim Bridge Started.")

    def callback(self, msg):
        with self.lock:
            # ENU (ROS) to NED (AirSim) Transformation
            self.target_pos[0] = msg.position.y
            self.target_pos[1] = msg.position.x
            self.target_pos[2] = -msg.position.z
            
            self.target_vel[0] = msg.velocity.y
            self.target_vel[1] = msg.velocity.x
            self.target_vel[2] = -msg.velocity.z
            
            # Convert Yaw: ROS Rad (CCW) -> AirSim Deg (CW)
            self.target_yaw = -msg.yaw * 180.0 / math.pi
            
            self.cmd_received = True
    
    def odomCallback(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "world_enu"
        out.child_frame_id = "drone_0/odom_local_enu"

        # -------------------------
        # POSITION: NED -> ENU
        # -------------------------
        out.pose.pose.position.x = msg.pose.pose.position.y
        out.pose.pose.position.y = msg.pose.pose.position.x
        out.pose.pose.position.z = -msg.pose.pose.position.z

        # -------------------------
        # VELOCITY: NED -> ENU
        # -------------------------
        out.twist.twist.linear.x = msg.twist.twist.linear.y
        out.twist.twist.linear.y = msg.twist.twist.linear.x
        out.twist.twist.linear.z = -msg.twist.twist.linear.z

        # -------------------------
        # ORIENTATION (quaternion)
        # NED -> ENU rotation fix
        # -------------------------
        q = msg.pose.pose.orientation
        quat_ned = [q.x, q.y, q.z, q.w]

        # Rotation: NED → ENU = 180° rotation around X-axis
        q_fix = tf.quaternion_from_euler(math.pi, 0, 0)

        quat_enu = tf.quaternion_multiply(q_fix, quat_ned)

        out.pose.pose.orientation = Quaternion(
            x=quat_enu[0],
            y=quat_enu[1],
            z=quat_enu[2],
            w=quat_enu[3]
        )

        self.odom_pub.publish(out)

    def control_loop(self):
        rate = rospy.Rate(50) # 50Hz control loop
        dt = 1.0 / 50.0
        
        # Gain for position correction (prevents drifting away from planner path)
        K_pos = 0.6
        Kv_xy = 0.2
        Kv_z  = 0.3

        # Set the clamp for max speed
        MAX_SPEED_XY = 6.0
        MAX_VZ = 1.0
        MAX_ACC = 2.0

        # --- Previous velocity (for acceleration limiting) ---
        prev_vx, prev_vy, prev_vz = 0.0, 0.0, 0.0

        def clamp(v, max_v):
            return max(min(v, max_v), -max_v)

        while not rospy.is_shutdown() and self.running:
            if self.cmd_received:
                with self.lock:
                    pos = self.target_pos
                    vel = self.target_vel
                    yaw = self.target_yaw

                # Get current drone state for feedback
                state = self.client.getMultirotorState(vehicle_name="drone_0")
                curr_pos = state.kinematics_estimated.position
                curr_vel = state.kinematics_estimated.linear_velocity

                vx = curr_vel.x_val
                vy = curr_vel.y_val
                vz = curr_vel.z_val

                speed_xy = math.sqrt(vx * vx + vy * vy)
                speed_z = abs(vz)

                rospy.loginfo_throttle(0.2, f"speed_xy: {speed_xy:.2f} m/s | speed_z: {speed_z:.2f} m/s")
                
                # Combine Feedforward (Planner Vel) + Feedback (Position Error)
                cmd_vx = vel[0] + K_pos * (pos[0] - curr_pos.x_val) - Kv_xy * curr_vel.x_val
                cmd_vy = vel[1] + K_pos * (pos[1] - curr_pos.y_val) - Kv_xy * curr_vel.y_val
                cmd_vz = vel[2] + K_pos * (pos[2] - curr_pos.z_val) - Kv_z * curr_vel.z_val

                speed_xy = math.sqrt(cmd_vx**2 + cmd_vy**2)
                if speed_xy > MAX_SPEED_XY:
                    scale = MAX_SPEED_XY / speed_xy
                    cmd_vx *= scale
                    cmd_vy *= scale

                cmd_vz = clamp(cmd_vz, MAX_VZ)

                # Send Command with Yaw
                # Duration is set small (0.02) to match loop frequency
                self.client.moveByVelocityAsync(
                    cmd_vx, cmd_vy, cmd_vz, 
                    duration=0.02, 
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(False, yaw), # False = use absolute angle
                    vehicle_name="drone_0"
                )

            rate.sleep()
        
    def emergency_landing(self):
        rospy.loginfo("Shutting down bridge... Emergency landing initiated.")

        self.running = False
        rospy.sleep(0.1)  # let thread exit cleanly
        
        try:
            # 2. Create a NEW client (important!)
            client = airsim.MultirotorClient()
            client.confirmConnection()

            # 3. Stop motion
            client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name="drone_0")

            # 4. Land
            rospy.loginfo("Landing...")
            client.landAsync(vehicle_name="drone_0").join()

            # 5. Disarm
            client.armDisarm(False, vehicle_name="drone_0")
            client.enableApiControl(False, vehicle_name="drone_0")

        except Exception as e:
            rospy.logwarn(f"Emergency landing failed: {e}")

        rospy.loginfo("Drone safe. Connection closed.")

if __name__ == '__main__':
    try:
        bridge = AirSimBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass