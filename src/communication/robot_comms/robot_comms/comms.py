"""UT Austin SPL communication module between robots and gamecontroller"""
# pylint: disable=too-many-instance-attributes

import socket
import math
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


from gc_spl_interfaces.msg import RCGCD15 as RCGCD
from gc_spl_interfaces.msg import RCGCRD4 as RCGCRD


class CommNode(Node):
    """Node that runs on the robot to communicate with SPL Game Controller
    and communicates with other robots."""
    def __init__(self, node_name="robot_comms_node", **kwargs):
        super().__init__(node_name, **kwargs)

        # Declare parameters
        self.declare_parameter("player_num", 5)
        self.declare_parameter("team_num", 1)
        self.declare_parameter("team_port", 10001)
        self.declare_parameter("team_broadcast_ip", "localhost")

        # Comm related state variables
        self._msg_budget = 0

        # Mutex for thread safety
        self._mutex = Lock()

        # Publisher for GC Return
        self._gcr_pub = self.create_publisher(RCGCRD, "gc/return_data", 10)
        self._gcr_timer = self.create_timer(2.0, self.gcr_callback)

        # Publisher for Team Communication
        self._team_pub = self.create_publisher(String, "comms/recv_tp", 10)

        # Subscribers for localization
        self._agent_pose = (0, 0, 0)
        self._agent_subscriber = self.create_subscription(
            Odometry, "localization/agent_pose", self.agent_pos_listener, 10)
        self._ball_pose = (0, 0)
        self._ball_subscriber_ = self.create_subscription(
            Odometry, "localization/ball_pose", self.ball_pos_listener, 10)

        self._has_fallen = False
        self._has_fallen_subscriber_ = self.create_subscription(
            Bool, "has_fallen", self.has_fallen_listener, 10)

        self.ball_detection_timestamp = 0
        self._ball_age = -1
        self._last_ball_detection_timestamp_subscriber_ = self.create_subscription(
            Header, "/last_ball_detection_timestamp",
            self.last_ball_detection_timestamp_listener, 10)

        # Subscriber for GC Data
        self._gc_subscriber = self.create_subscription(
            RCGCD, "gc/data", self.gc_callback, 10)

        # Subscriber for Team Communication
        self._tp_subscriber = self.create_subscription(
            String, "comms/send_data", self.send_tp, 10)

        self._tp_listener_thread = Thread(target=self.tp_listener)
        self._tp_listener_thread.start()

    def agent_pos_listener(self, msg):
        """Callback for agent position"""
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y**2 + orientation.z**2))
        with self._mutex:
            self._agent_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def ball_pos_listener(self, msg):
        """Callback for ball position"""
        with self._mutex:
            self._ball_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)


    def last_ball_detection_timestamp_listener(self, msg):
        """Callback for listening to ball detection timestamp"""
        with self._mutex:
            self.ball_detection_timestamp = Time.from_msg(msg.stamp).nanoseconds

    def has_fallen_listener(self, msg):
        """Callback for detecting fall"""
        with self._mutex:
            self._has_fallen = msg.data


    def tp_listener(self):
        """Thread for listening to team communication"""
        client = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        client.bind(("", self.get_parameter("team_port").value))

        while rclpy.ok():
            try:
                data, addr = client.recvfrom(128)
                self.get_logger().debug(f'received message from {addr}')
                msg = String()
                msg.data = data.decode("utf-8")
                self._team_pub.publish(msg)
            except TimeoutError:
                pass

    def send_tp(self, msg):
        """Callback for sending team communication"""
        if self._msg_budget < 5:
            return
        server = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        server.settimeout(0.2)
        server.sendto(msg.data.encode("utf-8"),
                      (self.get_parameter("team_broadcast_ip").value,
                       self.get_parameter("team_port").value))

    def gcr_callback(self):
        """Send return data to game controller"""
        with self._mutex:
            if self.ball_detection_timestamp != 0:
                self._ball_age = (self.get_clock().now().nanoseconds -
                                  self.ball_detection_timestamp)/(10**9)


            msg = RCGCRD()
            msg.player_num = self.get_parameter("player_num").value
            msg.team_num = self.get_parameter("team_num").value
            msg.fallen = self._has_fallen
            msg.pose[0] = self._agent_pose[0]
            msg.pose[1] = self._agent_pose[1]
            msg.pose[2] = self._agent_pose[2]
            msg.ball_age = float(self._ball_age)
            msg.ball[0] = self._ball_pose[0]
            msg.ball[1] = self._ball_pose[1]


            #print ("SENDING DATA TO GAME CONTROLLER")

            self._gcr_pub.publish(msg)

    def gc_callback(self, msg):
        """Callback for receiving data from game controller"""
        for team in msg.teams:
            if team.team_number == self.get_parameter("team_num").value:
                self._msg_budget = team.message_budget


def main(args=None):
    """Main function for running the node"""
    rclpy.init(args=args)
    comm_node = CommNode()
    rclpy.spin(comm_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
