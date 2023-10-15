"""Basic goal scoring behavior"""
#pylint: disable=too-many-instance-attributes
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from nao_command_msgs.msg import JointIndexes, JointPositions, JointStiffnesses, ChestLed
from nao_sensor_msgs.msg import JointPositions as SensorPositions
from nao_sensor_msgs.msg import Buttons, Touch
from walk_msg.msg import Walk
from gc_spl_interfaces.msg import RCGCD15 as RCGCD

from behavior.state_machine import State, StateMachineRoot
from behavior.resources import SubscriptionResource, PublisherResource
from behavior.basic_states import *  #pylint: disable=wildcard-import
from behavior.basic_events import SubscriptionValueEquals, FallingEdgeEvent

#MSG_FREQ = 83
#MAX_HEAD_ANGLE = 1.1
#SCAN_PERIOD = 10.0 # sec


class Soccer(Node):
    """Main node"""
    def __init__(self):
        super().__init__('soccer')
        # Publishers
        self.GAME_STATE = "INITIAL"

        self.chest_led_pub_ = self.create_publisher(ChestLed,
                                             'effectors/chest_led', 10)

        self.pos_pub_ = self.create_publisher(JointPositions,
                                             'effectors/joint_positions', 10)
        self.stiff_pub_ = self.create_publisher(JointStiffnesses,
                                               'effectors/joint_stiffnesses', 10)
        self.walk_msg_pub_ = self.create_publisher(Walk, '/motion/walk', 10)
        self.spawn_pub_ = self.create_publisher(Pose, 'localization/spawn', 10)
        self.timer_ = self.create_timer(1.0/MSG_FREQ, self.timer_callback)
        # Subscribers
        self.ball_pose = (0, 0)
        self.ball_rel_pose = (0, 0)
        self.ball_cov = 100
        self.ball_rel_cov = 100

        self.buttons_sub_ = self.create_subscription(
            Buttons,
            'sensors/buttons',
            self.button_sub_listener,
            10)
        self.head_buttons_sub_ = self.create_subscription(
            Touch,
            'sensors/touch',
            self.head_button_sub_listener,
            10)

        self.rel_ball_subscriber_ = self.create_subscription(
           PoseWithCovariance,
           'localization/ball_rel_pose',
           self.rel_ball_pos_listener,
           10)
        self.ball_subscriber_ =  self.create_subscription(
           Odometry,
           'localization/ball_pose',
           self.ball_pos_listener,
           10)
        self.agent_pose = (0, 0)
        self.agent_cov = 10
        self.agent_subscriber = self.create_subscription(
           Odometry,
           'localization/agent_pose',
           self.agent_pos_listener,
           10)
        self.head_yaw = 0
        self.joint_subscriber = self.create_subscription(
           SensorPositions,
           'sensors/joint_positions',
           self.joint_pos_listener,
           10)
        # Other variables
        self.current_state = 'init'
        self.start_time = self.get_clock().now()
        self.last_head_move = self.start_time
        self.current_state_start = self.start_time
        self.current_head_angle = 0.0
        self.target_head_angle = 0.0

        # Spawn agent
        spawn_msg = Pose()
        spawn_msg.position.x = -2000.0
        spawn_msg.position.y = -3000.0
        spawn_msg.position.z = 0.0
        spawn_msg.orientation.x = 0.0
        spawn_msg.orientation.y = 0.0
        spawn_msg.orientation.z = 0.5**0.5
        spawn_msg.orientation.w = 0.5**0.5
        self.spawn_pub_.publish(spawn_msg)


    def button_sub_listener (self, msg):
        if msg.chest == True:
           if self.GAME_STATE == "INITIAL":
               self.GAME_STATE = "PENALIZED"
           elif self.GAME_STATE == "PENALIZED":
               self.GAME_STATE = "PLAYING"
           elif self.GAME_STATE == "PLAYING":
               self.GAME_STATE = "PENALIZED"

    def head_button_sub_listener (self, msg):
       pass


    def timer_callback(self):
        """Main behavior callback"""
        pmsg = JointPositions()
        smsg = JointStiffnesses()
        pmsg.indexes = smsg.indexes = [JointIndexes.HEADYAW, JointIndexes.HEADPITCH]
        pmsg.positions = [0.0, 0.3]
        smsg.stiffnesses = [1.0] * 2
        wmsg = self.make_walk_msg()

        button_color_msg = ChestLed()
        if self.GAME_STATE == "INITIAL":
           button_color_msg.color.r = 0
           button_color_msg.color.g = 0
           button_color_msg.color.b = 0
        elif self.GAME_STATE == "PENALIZED":
           button_color_msg.color.r = 255
           button_color_msg.color.g = 0
           button_color_msg.color.b = 0
        elif self.GAME_STATE == "PLAYING":
           button_color_msg.color.r = 0
           button_color_msg.color.g = 255
           button_color_msg.color.b = 0
        self.self.chest_led_pub_.publish(button_color_msg)

        #
        if self.current_state == 'init':
            # Transitions
            if (self.get_clock().now() - self.start_time >
                rclpy.duration.Duration(seconds=5)):
                self.transition_to_state('scan')

        elif self.current_state == 'scan':
            head_angle = math.sin((self.get_clock().now() - self.current_state_start)
                                  .nanoseconds / 1e9 * math.pi * 2 / SCAN_PERIOD
                                  )* MAX_HEAD_ANGLE
            pmsg.positions[0] = head_angle
            # Transitions
            if self.ball_cov < 1000:
                self.transition_to_state('approach')
            if (self.get_clock().now() - self.current_state_start >
                rclpy.duration.Duration(seconds=SCAN_PERIOD)):
                self.transition_to_state('spin')

        elif self.current_state == 'spin':
            wmsg = self.make_walk_msg(60.0, 0.0, 0.5)
            # Transitions
            if (self.get_clock().now() - self.current_state_start >
                rclpy.duration.Duration(seconds=SCAN_PERIOD)):
                self.transition_to_state('scan')

        elif self.current_state == 'approach':
            ball_angle = math.atan2(self.ball_pose[1]-self.agent_pose[1],
                    self.ball_pose[0]-self.agent_pose[0]) - self.agent_pose[2]
            ball_distance = math.sqrt((self.ball_pose[0]-self.agent_pose[0])**2 +
                                      (self.ball_pose[1]-self.agent_pose[1])**2)
            # clamp angle because it will be blocked by shoulders after a point
            desired_angle = min(max(ball_angle, -MAX_HEAD_ANGLE), MAX_HEAD_ANGLE)
            if (self.get_clock().now() - self.last_head_move >
                    rclpy.duration.Duration(seconds=2)):
                self.last_head_move = self.get_clock().now()
                self.target_head_angle = desired_angle
            self.current_head_angle += (self.target_head_angle - self.current_head_angle) * 0.1
            pmsg.positions = [self.current_head_angle, 0.3]
            self.get_logger().info(f"ball_angle: {ball_angle},\n" +
                                   f"desired_angle: {self.current_head_angle}")
            if abs(ball_angle) < math.pi/6:
                wmsg = self.make_walk_msg(100.0)
            else:
                wmsg = self.make_walk_msg(60.0, 0.0, 0.5 if ball_angle > 0 else -0.5)
            # Transitions
            if ball_distance < 300:
                self.transition_to_state('rotate_around_ball')
            if self.ball_cov > 2000:
                self.transition_to_state('scan')

        elif self.current_state == 'rotate_around_ball':
            goal_pos = (4000, 0)
            goal_angle = math.atan2(goal_pos[1] - self.agent_pose[1],
                                    goal_pos[0] - self.agent_pose[0])
            turn_angle = goal_angle - self.agent_pose[2]
            if turn_angle > math.pi:
                turn_angle -= math.pi * 2
            elif turn_angle < -math.pi:
                turn_angle += math.pi * 2
            self.get_logger().info(f"agent_pose: {self.agent_pose},\n" +
                                   f"turn_angle: {turn_angle}\n" +
                                   f"goal_angle: {goal_angle}")
            turn_radius = 250.0
            turn_speed = 80.0
            wmsg = self.make_walk_msg(0.0, turn_speed if turn_angle < 0 else -turn_speed,
                                      turn_speed/turn_radius * (1 if turn_angle > 0 else -1))
            # Transitions
            if abs(turn_angle) < 0.1:
                self.transition_to_state('kick')

        elif self.current_state == 'kick':
            wmsg = self.make_walk_msg(300.0)
            # Transitions
            if (self.get_clock().now() - self.current_state_start >
                    rclpy.duration.Duration(seconds=1.5)):
                self.transition_to_state('scan')

        self.pos_pub_.publish(pmsg)
        self.stiff_pub_.publish(smsg)
        if self.current_state != 'init':
            self.walk_msg_pub_.publish(wmsg)

    def transition_to_state(self, state):
        """Transitions to a state"""
        self.current_state = state
        self.current_state_start = self.get_clock().now()
        self.get_logger().info(f"Transitioning to state {state}")

    def make_walk_msg(self, forward=0.0, left=0.0, turn=0.0):
        """Returns a walk message"""
        msg = Walk()
        msg.forward = forward
        msg.left = left
        msg.turn = turn
        msg.kick = 0
        msg.speed = 1.0
        msg.bend = 1.0
        msg.power = 1.0
        return msg


    def rel_ball_pos_listener(self, msg):
        """Callback for close ball position"""
        ball_x = msg.pose.pose.position.x-self.agent_pose[0]
        ball_y = msg.pose.pose.position.y-self.agent_pose[1]
        rot = -self.agent_pose[2]
        self.ball_rel_pose = (ball_x*math.cos(rot) + ball_y*math.sin(rot),
                             -ball_x*math.sin(rot) + ball_y*math.cos(rot))
        self.ball_rel_cov = msg.covariance[0]

    def ball_pos_listener(self, msg):
        """Callback for ball position"""
        self.ball_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.ball_cov = msg.pose.covariance[0]

    def agent_pos_listener(self, msg):
        """Callback for agent position"""
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(2*(orientation.w*orientation.z + orientation.x*orientation.y),
                        1 - 2*(orientation.y**2 + orientation.z**2))
        self.agent_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.agent_cov = msg.pose.covariance[0]

    def joint_pos_listener(self, msg):
        """Callback for joint positions"""
        self.head_yaw = msg.positions[JointIndexes.HEADYAW]


def main(args=None):
    """Creates node"""
    rclpy.init(args=args)
    soccer = Soccer()
    rclpy.spin(soccer)
    soccer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
