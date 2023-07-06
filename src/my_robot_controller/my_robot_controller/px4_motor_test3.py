#!/usr/bin/env python3

# CODE REQUIRES the following ROS2 workspaces to operate:
#   Micro-XRCE-DDS-Agent
#   PX4-Autopilot
#   ws_sensor_combined

# MAKE SURE THAT Gazebo IS RUNNING
#   If it isn't, navigate to "~/PX4-Autopilot" and run "make px4_sitl gazebo" in separate terminal

# MAKE SURE THAT MicroXRCE Agent IS RUNNING!
#   If it isn't, run "MicroXRCEAgent udp4 -p 8888" in separate terminal

# Add "<depend>px4_msgs</depend>" to package.xml

# Add "px4_motor_test3 = YOUR_PACKAGE_NAME.px4_motor_test3:main" to setup.py

import rclpy
from rclpy.node import Node
from rclpy import qos
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleRatesSetpoint

class PX4_Motor_Test3(Node):

    offboard_setpoint_counter = 0

    def __init__(self):

        super().__init__("px4_commander_test3")
        self.get_logger().info("PX4 Commander Test 3 started.")

        self.px4_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos.qos_profile_sensor_data)
        self.px4_control_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos.qos_profile_sensor_data)
        self.px4_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos.qos_profile_sensor_data)

        self.publish_vehicle_command(176, 0.0, 0.0)

        self.timer = self.create_timer(0.1, self.timercallback)


    def timercallback(self):

        if self.offboard_setpoint_counter == 10:

            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.arm()

        self.publish_offboard_control_mode()

        self.publish_rates_setpoint()

        if self.offboard_setpoint_counter <11:

            self.offboard_setpoint_counter = self.offboard_setpoint_counter + 1


    def arm(self):

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 1.0)
        self.get_logger().info("Arm command send")


    def disarm(self):

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        self.get_logger().info("Disarm command send")


    def publish_offboard_control_mode(self): # https://docs.px4.io/main/en/flight_modes/offboard.html

        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False 
        msg.acceleration = False 
        msg.attitude = False 
        msg.body_rate = True
        msg.actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) 
        self.px4_control_mode_pub.publish(msg)

    
    def publish_rates_setpoint(self): # https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleRatesSetpoint.msg
        
        msg = VehicleRatesSetpoint()
        # msg.roll = float(1.0)
        # msg.pitch = float(2.0)
        # msg.yaw = float(4.0)
        msg.thrust_body = [float(0), float(0), float(-1.0)]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_rates_setpoint_pub.publish(msg)


    def publish_vehicle_command(self, command, param1, param2):

        msg = VehicleCommand()
        msg.param1 = param1 
        msg.param2 = param2 
        msg.command = command 
        msg.target_system = 1 
        msg.target_component = 1 
        msg.source_system = 1 
        msg.source_component = 1 
        msg.from_external = True 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.px4_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4_Motor_Test3()
    rclpy.spin(node)
    rclpy.shutdown()
