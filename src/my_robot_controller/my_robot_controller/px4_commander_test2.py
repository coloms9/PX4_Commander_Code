#!/usr/bin/env python3


# PX4 ROS2 setup guide:
#   https://docs.px4.io/main/en/ros/ros2_comm.html

# CODE REQUIRES the following ROS2 workspaces to operate:
#   Micro-XRCE-DDS-Agent
#   PX4-Autopilot
#   ws_sensor_combined

# NEED TO SOURCE THE FOLLOWING:
#   source /opt/ros/foxy/setup.bash
#   source ~/ws_sensor_combined/install/setup.bash

# MAKE SURE THAT Gazebo IS RUNNING
#   If it isn't, navigate to "~/PX4-Autopilot" and run "make px4_sitl gazebo" in separate terminal

# MAKE SURE THAT MicroXRCE Agent IS RUNNING!
#   If it isn't, run "MicroXRCEAgent udp4 -p 8888" in separate terminal

import rclpy
from rclpy.node import Node
from rclpy import qos
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode

class PX4_Commander_Test2(Node):

    offboard_setpoint_counter = 0 # This is used in the timercallback function


    def __init__(self):

        super().__init__("px4_commander_test2")
        self.get_logger().info("PX4 Commander Test 2 started.")

        #self.get_logger().info("a") #  DEBUGING LINE

        # Creating the publishers
        self.px4_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos.qos_profile_sensor_data)
        self.px4_control_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos.qos_profile_sensor_data)
        self.px4_trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos.qos_profile_sensor_data)

        #self.get_logger().info("b") #  DEBUGING LINE

        # The line below is not nessessary to run the drone, but it seems to provide a more stable start
        self.publish_vehicle_command(176, 0.0, 0.0) 
        # VehicleCommand.VEHICLE_CMD_DO_SET_MODE is set to 176 according to:
        # https://docs.px4.io/main/en/msg_docs/VehicleCommand.html

        self.timer = self.create_timer(0.1, self.timercallback) # Calls the timercallback function every 100ms

    
    def timercallback(self): # Function that runs the drone

        #self.get_logger().info("c") #  DEBUGING LINE

        if self.offboard_setpoint_counter == 10:

            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) 
            # Unsure what the line above does exactly, it is not nessessary, but I will keep it in for now
            self.arm() # Arms the drone

        #self.get_logger().info(str(self.offboard_setpoint_counter)) # DEBUGING LINE

        self.publish_offboard_control_mode() # This function needs to be run a least 1 second before if arms into offboard mode
        self.publish_trajectory_setpoint() # In offboard mode, the postion need to be constantly updated

        if self.offboard_setpoint_counter < 11:

            self.offboard_setpoint_counter = self.offboard_setpoint_counter + 1

            #self.get_logger().info(str(self.offboard_setpoint_counter)) #  DEBUGING LINE
    

    def arm(self): # Function that arms drone

        #self.get_logger().info("d") #  DEBUGING LINE

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 1.0)
        # ^ Runs the publish_vehicle_command function with param1 and param2 set to 1.0; unsure of
        # VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, see lines 50-51 for more info
        # Both param1 AND param2 need to equal 1.0 to arm the drone successfully
        self.get_logger().info("Arm command send")


    def disarm(self): # Function that disarms drone

        #self.get_logger().info("e") #  DEBUGING LINE

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        # ^ Runs the publish_vehicle_command function with param1 and param2 set to 0.0; unsure of
        # VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, see lines 50-51 for more info
        self.get_logger().info("Disrm command send")


    def publish_offboard_control_mode(self): # Function that puts drone into offboard control mode

        #self.get_logger().info("f") #  DEBUGING LINE

        msg = OffboardControlMode()
        msg.position = True # Unknown significance
        msg.velocity = False # Unknown significance
        msg.acceleration = False # Unknown significance
        msg.attitude = False # Unknown significance
        msg.body_rate = False # Unknown significance
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # msg.timestamp requires an integer
        self.px4_control_mode_pub.publish(msg)


    def publish_trajectory_setpoint(self): # Function that tell the drone where to go

        #self.get_logger().info("g1") #  DEBUGING LINE
        
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -5.0] # position drone will fly to in [x, y, z] meters
        # movement follows right hand rule, with z axis in the negative direction
        msg.yaw = -3.14 # Unsure why this needs to be pi
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # msg.timestamp requires an integer
        self.px4_trajectory_setpoint_pub.publish(msg)

        #self.get_logger().info("g2") #  DEBUGING LINE


    def publish_vehicle_command(self, command, param1, param2): # Function that initializes drone

        #self.get_logger().info("h1") #  DEBUGING LINE

        msg = VehicleCommand()
        msg.param1 = param1 # Unknown significance
        msg.param2 = param2 # Unknown significance
        msg.command = command # See lines 50-51 for more info
        msg.target_system = 1 # Unknown significance
        msg.target_component = 1 # Unknown significance
        msg.source_system = 1 # Unknown significance
        msg.source_component = 1 # Unknown significance
        msg.from_external = True # Unknown significance
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # msg.timestamp requires an integer
        self.px4_command_pub.publish(msg)

        #self.get_logger().info("h2") #  DEBUGING LINE



def main(args=None):

    rclpy.init(args=args)
    node = PX4_Commander_Test2()
    rclpy.spin(node)
    rclpy.shutdown()
