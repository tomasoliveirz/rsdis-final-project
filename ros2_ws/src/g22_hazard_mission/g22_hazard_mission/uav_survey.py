"""UAV mission: take off from the pad and fly a lawnmower route over the yard.

The X3 controller takes velocity commands in the body frame, so this node runs
a P controller on the position from odometry (world frame).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from g22_hazard_mission.search import lawnmower


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class UavSurvey(Node):

    def __init__(self):
        super().__init__('uav_survey')
        self.altitude = self.declare_parameter('altitude', 6.0).value
        self.max_speed = self.declare_parameter('max_speed', 2.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.4).value
        # x_min, x_max, y_min, y_max of the survey
        area = self.declare_parameter('area', [-13.0, 13.0, -10.0, 10.0]).value
        spacing = self.declare_parameter('lane_spacing', 6.0).value

        self.route = lawnmower(*area, spacing)
        self.waypoint = 0

        self.pose = None
        self.home = None
        self.state = 'takeoff'

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, 'enable', 10)
        self.create_timer(0.1, self.tick)

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        if self.home is None:
            self.home = (self.pose.position.x, self.pose.position.y)

    def fly_to(self, x, y, z):
        """Send one velocity command towards (x, y, z). True once it is there."""
        p = self.pose.position
        ex, ey, ez = x - p.x, y - p.y, z - p.z
        vx, vy = ex, ey
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx, vy = vx * self.max_speed / speed, vy * self.max_speed / speed
        yaw = yaw_of(self.pose.orientation)
        cmd = Twist()
        cmd.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        cmd.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        cmd.linear.z = max(-1.0, min(1.0, ez))
        cmd.angular.z = max(-1.0, min(1.0, -yaw))  # keep facing +x
        self.cmd_pub.publish(cmd)
        return math.hypot(ex, ey) < self.tolerance and abs(ez) < self.tolerance

    def tick(self):
        if self.pose is None:
            return
        if self.state == 'takeoff':
            self.enable_pub.publish(Bool(data=True))
            if self.fly_to(*self.home, self.altitude):
                self.get_logger().info(f'at {self.altitude} m, starting the survey')
                self.state = 'survey'
        elif self.state == 'survey':
            if self.fly_to(*self.route[self.waypoint], self.altitude):
                self.waypoint += 1
                if self.waypoint == len(self.route):
                    self.get_logger().info('survey done')
                    self.state = 'hover'
        elif self.state == 'hover':
            self.fly_to(*self.route[-1], self.altitude)


def main():
    rclpy.init()
    node = UavSurvey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
