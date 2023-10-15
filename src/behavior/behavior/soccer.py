"""Basic goal scoring behavior"""
#pylint: disable=too-many-instance-attributes
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from nao_command_msgs.msg import JointIndexes, JointPositions, JointStiffnesses, ChestLed
from nao_sensor_msgs.msg import JointPositions as SensorPositions
from nao_sensor_msgs.msg import Buttons, Touch, Angle
from walk_msg.msg import Walk
from gc_spl_interfaces.msg import RCGCD15 as RCGCD

from behavior.state_machine import State, StateMachineRoot
from behavior.resources import SubscriptionResource, PublisherResource, MotionPublisherResource
from behavior.basic_states import *  #pylint: disable=wildcard-import
from behavior.basic_events import SubscriptionValueEquals, FallingEdgeEvent, RisingEdgeEvent, TimeEvent

#MSG_FREQ = 83
#MAX_HEAD_ANGLE = 1.1
#SCAN_PERIOD = 10.0 # sec


class Soccer(Node):
    """Main node"""
    def __init__(self):
        super().__init__('soccer')

        # Declare Parameters
        self.declare_parameter('frequency', 30)
        self.declare_parameter('team_number', 1)
        self.declare_parameter('player_number', 4)

        # Main state machine
        fsm = StateMachineRoot(self)

        # Subscriber Resources
        fsm.new_resource(SubscriptionResource(
            self, '/gc/data', RCGCD, 'rcgcd'))
        fsm.new_resource(SubscriptionResource(
            self, '/sensors/buttons', Buttons, 'buttons'))
        fsm.new_resource(SubscriptionResource(
            self, '/sensors/touch', Touch, 'head_buttons'))
        fsm.new_resource(SubscriptionResource(
            self, '/gc/data', RCGCD, 'gc_data'))
        fsm.new_resource(SubscriptionResource(
            self, '/whistle', Bool, 'whistle'))
        fsm.new_resource(SubscriptionResource(
            self, 'localization/ball_pose', Odometry, 'ball_pose'))
        fsm.new_resource(SubscriptionResource(
            self, 'localization/ball_rel_pose', Odometry, 'ball_rel_pose'))
        fsm.new_resource(SubscriptionResource(
            self, 'localization/agent_pose', Odometry, 'agent_pose'))
        fsm.new_resource(SubscriptionResource(
            self, '/has_fallen', Bool, 'has_fallen'))
        fsm.new_resource(SubscriptionResource(
            self, '/sensors/angle', Angle, 'angle'))
        fsm.new_resource(SubscriptionResource(
            self, '/keyframe_motions/is_done', Bool, 'kf_motion_done'))

        # Publisher Resources
        fsm.new_resource(MotionPublisherResource(self))
        fsm.new_resource(PublisherResource(
            self, '/effectors/chest_led', ChestLed, 'chest_led'))
        fsm.new_resource(PublisherResource(
            self, '/localization/spawn', Pose, 'spawn'))

        # States
        fsm.add_state(Initial())
        fsm.add_state(Ready())
        fsm.add_state(Set())
        fsm.add_state(Playing())
        fsm.add_state(Finished())
        fsm.add_state(Penalized())
        #fsm.add_state(Calibration())
        fsm.add_state(Unstiff())

        # Events
        chest_button = SubscriptionValueEquals('chest_button', 'buttons', 'chest')
        front_head_button = SubscriptionValueEquals(
            'front_head_button', 'head_buttons', 'head_front')
        middle_head_button = SubscriptionValueEquals(
            'middle_head_button', 'head_buttons', 'head_middle')
        rear_head_button = SubscriptionValueEquals(
            'rear_head_button', 'head_buttons', 'head_rear')
        chest_button_press = FallingEdgeEvent(chest_button)
        all_head_button_press = DelayEvent(front_head_button &
                                           middle_head_button &
                                           rear_head_button, 1.0)
        gc_ready = SubscriptionValueEquals(
            'gc_ready', 'gc_data', 'state', RCGCD.STATE_READY)
        gc_set = SubscriptionValueEquals(
            'gc_set', 'gc_data', 'state', RCGCD.STATE_SET)
        gc_playing = SubscriptionValueEquals(
            'gc_playing', 'gc_data', 'state', RCGCD.STATE_PLAYING)
        gc_finished = SubscriptionValueEquals(
            'gc_finished', 'gc_data', 'state', RCGCD.STATE_FINISHED)
        team_num = self.get_parameter('team_number').value
        player_num = self.get_parameter('player_number').value
        def penalty_type(msg):
            for team in msg.teams:
                if team.team_number == team_num:
                    return team.players[player_num-1].penalty
            return -1
        gc_penalized = SubscriptionValueAtLeast(
            'gc_penalized', 'gc_data', penalty_type, 1)
        gc_unpenalized = SubscriptionValueEquals(
            'gc_unpenalized', 'gc_data', penalty_type, 0)

        whistle = SubscriptionValueEquals(
            'whistle', 'whistle', 'data')
        whistle = whistle & TimeEvent(1.0)

        # Transitions
        fsm.add_transition('INITIAL', chest_button_press, 'PENALIZED')
        fsm.add_transition('PENALIZED', chest_button_press, 'PLAYING')
        fsm.add_transition('PLAYING', chest_button_press, 'PENALIZED')

        fsm.add_transition('INITIAL', all_head_button_press, 'UNSTIFF')
        for event in fsm._transitions['INITIAL']:
            print(event)
        fsm.add_transition('READY', all_head_button_press, 'UNSTIFF')
        fsm.add_transition('SET', all_head_button_press, 'UNSTIFF')
        fsm.add_transition('PLAYING', all_head_button_press, 'UNSTIFF')
        fsm.add_transition('FINISHED', all_head_button_press, 'UNSTIFF')
        fsm.add_transition('PENALIZED', all_head_button_press, 'UNSTIFF')
        fsm.add_transition('UNSTIFF', chest_button_press, 'INITIAL')

        fsm.add_transition('INITIAL', gc_ready, 'READY')
        fsm.add_transition('READY', gc_set, 'SET')
        fsm.add_transition('SET', gc_playing, 'PLAYING')
        fsm.add_transition('PLAYING', gc_finished, 'FINISHED')
        fsm.add_transition('PLAYING', gc_ready, 'READY')
        fsm.add_transition('READY', gc_penalized, 'PENALIZED')
        fsm.add_transition('SET', gc_penalized, 'PENALIZED')
        fsm.add_transition('PLAYING', gc_penalized, 'PENALIZED')
        fsm.add_transition('PENALIZED', gc_ready & gc_unpenalized, 'READY')
        fsm.add_transition('PENALIZED', gc_set & gc_unpenalized, 'SET')
        fsm.add_transition('PENALIZED', ~gc_ready & ~gc_set & gc_unpenalized,
                           'PLAYING')

        fsm.add_transition('SET', whistle, 'PLAYING')
        fsm.add_transition('PLAYING', whistle, 'READY')

        # Validate after all states and transitions have been added
        fsm.validate()
        self._state_machine = fsm
        self._state_machine.start()
        self._timer = self.create_timer(1.0/self.get_parameter('frequency').value,
                                        self.timer_callback)

    def timer_callback(self):
        """Main behavior callback"""
        self._state_machine.step()
        print(self._state_machine._current_state.get_name())

def main(args=None):
    """Creates node"""
    rclpy.init(args=args)
    soccer = Soccer()
    rclpy.spin(soccer)
    soccer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
