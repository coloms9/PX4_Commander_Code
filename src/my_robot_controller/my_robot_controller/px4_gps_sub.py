#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorGps
from rclpy import qos


class PX4_GPS_Subscriber(Node):

    def __init__(self):
        super().__init__("px4_gps_sub")
        self.get_logger().info("PX4 GPS Subscriber started.")
        self.get_logger().info(str(SensorGps.alt))
        self.px4_gps_sub = self.create_subscription(
            SensorGps, "/fmu/out/vehicle_gps_position", self.gps_pose_callback, qos.qos_profile_sensor_data)

    def gps_pose_callback(self, data: SensorGps):
        gps_alt = data.alt
        gps_heading = data.heading
        self.get_logger().info(str(gps_alt) + "     " + str(gps_heading))

def main(args=None):
    rclpy.init(args=args)
    node = PX4_GPS_Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()