"""State definitions for state machine"""

import math
from rclpy.node import Node
from rclpy.duration import Duration
from nao_command_msgs.msg import ChestLed
from behavior.state_machine import State, StateMachine, Event
from geometry_msgs.msg import Pose
from behavior.basic_events import TimeEvent, SubscriptionValueEquals, SubscriptionValueAtLeast, DelayEvent

class Initial(State):
    """Initial state"""
    def __init__(self):
        super().__init__('INITIAL')
        self.add_required_resource(['chest_led'])
        self.add_required_resource(['spawn'])
    def step(self):
        super().step()
        msg = ChestLed()
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        self.get_resource('chest_led').publish(msg)

        spawn_msg = Pose()
        spawn_msg.position.x = -3000.0
        spawn_msg.position.y = 3000.0
        spawn_msg.position.z = 0.0
        spawn_msg.orientation.x = 0.0
        spawn_msg.orientation.y = 0.0
        spawn_msg.orientation.z = 0.5**0.5
        spawn_msg.orientation.w = 0.5**0.5
        self.get_resource('spawn').publish(spawn_msg)

class Ready(StateMachine):
    """Ready state"""
    def __init__(self):
        super().__init__('READY')
        self.add_required_resource(['chest_led', 'motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('stand')
    def step(self):
        super().step()
        msg = ChestLed()
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        self.get_resource('chest_led').publish(msg)

class Set(StateMachine):
    """Set state"""
    def __init__(self):
        super().__init__('SET')
        self.add_required_resource(['chest_led', 'motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('stand')
    def step(self):
        super().step()
        msg = ChestLed()
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        self.get_resource('chest_led').publish(msg)

class Playing(StateMachine):
    """Playing state"""
    def __init__(self):
        super().__init__('PLAYING')
        self.add_required_resource(['chest_led', 'ball_pose', 'ball_rel_pose'])
        self.add_required_resource(['spawn'])

        self.add_state(Stand())
        self.add_state(Search())
        self.add_state(Approach())
        self.add_state(RotateAroundBall())
        self.add_state(GetUpFront())
        self.add_state(GetUpBack())
        self.add_state(Walk(forward=300.0, name='kick'))
        self.add_state(Walk(forward=150.0, name='recover'))

        ball_found = ~SubscriptionValueAtLeast(
            'ball_found', 'ball_rel_pose',
            lambda msg: msg.pose.covariance[0],
            10000)
        ball_outside_field = SubscriptionValueAtLeast(
            'ball_outside_x', 'ball_pose',
            lambda msg: abs(msg.pose.pose.position.x),
            4500) | SubscriptionValueAtLeast(
            'ball_outside_y', 'ball_pose',
            lambda msg: abs(msg.pose.pose.position.y),
            3000)
        ball_lost = DelayEvent(~ball_found, 5.0)

        fallen_forward= SubscriptionValueAtLeast(
            "fallen_forward", "angle", "y", 1.5)
        fallen_backward= ~SubscriptionValueAtLeast(
            "fallen_backward", "angle", "y", -1.4)
        is_kf_motion_done = SubscriptionValueEquals(
            "is_kf_done", "kf_motion_done", "data")


        self.add_transition('stand', TimeEvent(1.0), 'search')
        self.add_transition('search', ball_found & ~ball_outside_field, 'approach')
        self.add_transition('approach', BallCloseEvent(300), 'rotate_around_ball')
        self.add_transition('rotate_around_ball', GoodKickAngle(0.1), 'kick')
        self.add_transition('kick', TimeEvent(1.5), 'recover')
        self.add_transition('recover', TimeEvent(0.5), 'search')

        self.add_transition('approach', ball_lost, 'search')
        self.add_transition('rotate_around_ball', ball_lost, 'search')

        self.add_transition('rotate_around_ball', ~BallCloseEvent(1000), 'approach')

        for state in ['stand', 'search', 'approach', 'rotate_around_ball', 'kick', 'recover']:
            self.add_transition(state, fallen_forward, 'getup_front')
            self.add_transition(state, fallen_backward, 'getup_back')

        self.add_transition('getup_front', is_kf_motion_done , 'search')
        self.add_transition('getup_back', is_kf_motion_done , 'search')

    def start(self):
        super().start()
        spawn_msg = Pose()
        spawn_msg.position.x = -3000.0
        spawn_msg.position.y = 3000.0
        spawn_msg.position.z = 0.0
        spawn_msg.orientation.x = 0.0
        spawn_msg.orientation.y = 0.0
        spawn_msg.orientation.z = 0.5**0.5
        spawn_msg.orientation.w = 0.5**0.5
        self.get_resource('spawn').publish(spawn_msg)

    def step(self):
        super().step()
        msg = ChestLed()
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        self.get_resource('chest_led').publish(msg)

class GetUpFront(State):
    """Get up front state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'getup_front')
        self.add_required_resource(['motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('getup_front')

class GetUpBack(State):
    """Get up back state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'getup_back')
        self.add_required_resource(['motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('getup_back')

class BallCloseEvent(Event):
    """Event that is true when the ball is close to the agent"""
    def __init__(self, threshold, name=None):
        super().__init__(name or 'ball_close')
        self.add_required_resource(['ball_pose', 'agent_pose'])
        self._threshold = threshold
    def update(self):
        super().update()
        bmsg = self._state.get_resource('ball_pose').get()
        amsg = self._state.get_resource('agent_pose').get()
        if bmsg is None or amsg is None:
            return False
        ball_pose = (bmsg.pose.pose.position.x, bmsg.pose.pose.position.y)
        orientation = amsg.pose.pose.orientation
        yaw = math.atan2(2*(orientation.w*orientation.z + orientation.x*orientation.y),
                         1 - 2*(orientation.y**2 + orientation.z**2))
        agent_pose = (amsg.pose.pose.position.x, amsg.pose.pose.position.y, yaw)
        self._status = ((ball_pose[0] - agent_pose[0])**2 +
                        (ball_pose[1] - agent_pose[1])**2) < self._threshold**2
        return self._status

class GoodKickAngle(Event):
    """Event that is true when the robot is angled well to kick the ball into the goal."""
    def __init__(self, threshold, name=None):
        super().__init__(name or 'good_kick_angle')
        self.add_required_resource(['agent_pose'])
        self._threshold = threshold
    def update(self):
        super().update()
        amsg = self._state.get_resource('agent_pose').get()
        if amsg is None:
            return False
        orientation = amsg.pose.pose.orientation
        yaw = math.atan2(2*(orientation.w*orientation.z + orientation.x*orientation.y),
                         1 - 2*(orientation.y**2 + orientation.z**2))
        agent_pose = (amsg.pose.pose.position.x, amsg.pose.pose.position.y, yaw)
        goal_pose = (4500, 0)
        goal_angle = math.atan2(goal_pose[1] - agent_pose[1],
                                goal_pose[0] - agent_pose[0])
        turn_angle = goal_angle - agent_pose[2]
        if turn_angle > math.pi:
            turn_angle -= math.pi * 2
        elif turn_angle < -math.pi:
            turn_angle += math.pi * 2
        self._status = turn_angle < self._threshold
        return self._status

class Stand(State):
    """Stand state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'stand')
        self.add_required_resource(['motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('stand')

class Scan(State):
    """Scan state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'scan')
        self.add_required_resource(['motion'])
    def step(self):
        super().step()
        scan_period = 5.0 # seconds
        max_head_angle = 1.1 # radians
        head_angle = math.sin(self.get_elapsed_time().nanoseconds / 1e9 *
                              math.pi * 2 / scan_period)* max_head_angle
        self.get_resource('motion').walk(0.0)
        self.get_resource('motion').head_position(head_angle, 0.3)

class Spin(State):
    """Spin state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'spin')
        self.add_required_resource(['motion'])
    def step(self):
        super().step()
        self.get_resource('motion').walk(60.0, turn=0.5)

class Search(StateMachine):
    """Search state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'search')
        self.add_required_resource(['motion'])
        self.add_state(Scan())
        self.add_state(Spin())
        self.add_transition('scan', TimeEvent(5.0), 'spin')
        self.add_transition('spin', TimeEvent(3.0), 'scan')

class Approach(State):
    """Approach state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'approach')
        self.add_required_resource(['motion', 'ball_pose', 'agent_pose'])
        self.last_head_move = None
        self.current_head_angle = None
        self.target_head_angle = None
    def start(self):
        super().start()
        self.last_head_move = self.get_start_time()
        self.current_head_angle = 0.0
        self.target_head_angle = 0.0
    def step(self):
        super().step()
        bmsg = self.get_resource('ball_pose').get()
        amsg = self.get_resource('agent_pose').get()
        if bmsg is None or amsg is None:
            return
        ball_pose = (bmsg.pose.pose.position.x, bmsg.pose.pose.position.y)
        orientation = amsg.pose.pose.orientation
        yaw = math.atan2(2*(orientation.w*orientation.z + orientation.x*orientation.y),
                         1 - 2*(orientation.y**2 + orientation.z**2))
        agent_pose = (amsg.pose.pose.position.x, amsg.pose.pose.position.y, yaw)
        ball_angle = math.atan2(ball_pose[1] - agent_pose[1],
                                ball_pose[0] - agent_pose[0]) - agent_pose[2]
        # clamp angle because it will be blocked by shoulders after a point
        max_head_angle = 1.1
        desired_angle = min(max(ball_angle, -max_head_angle), max_head_angle)
        if self._node.get_clock().now() - self.last_head_move > Duration(seconds=2):
            self.last_head_move = self._node.get_clock().now()
            self.target_head_angle = desired_angle
        self.current_head_angle += (self.target_head_angle - self.current_head_angle) * 0.1
        self.get_resource('motion').head_position(self.current_head_angle, 0.3)
        if abs(ball_angle) < math.pi/6:
            self.get_resource('motion').walk(200.0)
        else:
            self.get_resource('motion').walk(60.0, turn=0.5 if ball_angle > 0 else -0.5)

class RotateAroundBall(State):
    """RotateAroundBall state"""
    def __init__(self, name:str=None):
        super().__init__(name or 'rotate_around_ball')
        self.add_required_resource(['motion', 'ball_pose', 'agent_pose'])
    def step(self):
        super().step()
        bmsg = self.get_resource('ball_pose').get()
        amsg = self.get_resource('agent_pose').get()
        if bmsg is None or amsg is None:
            return
        orientation = amsg.pose.pose.orientation
        yaw = math.atan2(2*(orientation.w*orientation.z + orientation.x*orientation.y),
                         1 - 2*(orientation.y**2 + orientation.z**2))
        agent_pose = (amsg.pose.pose.position.x, amsg.pose.pose.position.y, yaw)

        goal_pose = (4000, 0)
        goal_angle = math.atan2(goal_pose[1] - agent_pose[1],
                                goal_pose[0] - agent_pose[0])
        turn_angle = goal_angle - agent_pose[2]
        if turn_angle > math.pi:
            turn_angle -= math.pi * 2
        elif turn_angle < -math.pi:
            turn_angle += math.pi * 2
        turn_radius = 250.0
        turn_speed = 80.0
        self.get_resource('motion').walk(0.0,
            turn_speed/turn_radius * (1 if turn_angle > 0 else -1),
            turn_speed if turn_angle < 0 else -turn_speed)

class Walk(State):
    """WalkForward state"""
    def __init__(self, forward=0.0, left=0.0, turn=0.0, name:str=None):
        super().__init__(name or 'walk')
        self.add_required_resource(['motion'])
        self._forward = forward
        self._left = left
        self._turn = turn
    def step(self):
        super().step()
        self.get_resource('motion').walk(self._forward, self._left, self._turn)

class Finished(State):
    """Finished state"""
    def __init__(self):
        super().__init__('FINISHED')
        self.add_required_resource(['chest_led', 'motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('stand')
    def step(self):
        super().step()
        msg = ChestLed()
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        self.get_resource('chest_led').publish(msg)

class Penalized(State):
    """Penalized state"""
    def __init__(self):
        super().__init__('PENALIZED')
        self.add_required_resource(['chest_led', 'motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('stand')
    def step(self):
        super().step()
        msg = ChestLed()
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        self.get_resource('chest_led').publish(msg)

class Calibration(StateMachine):
    """Calibration state"""
    def __init__(self):
        super().__init__('CALIBRATION')
        self.add_required_resource(['chest_led'])
    def step(self):
        super().step()
        msg = ChestLed()
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        self.get_resource('chest_led').publish(msg)

class Unstiff(State):
    """Unstiff state"""
    def __init__(self):
        super().__init__('UNSTIFF')
        self.add_required_resource(['chest_led', 'motion'])
    def start(self):
        super().start()
        self.get_resource('motion').keyframe_motion('unstiff')
    def step(self):
        super().step()
        msg = ChestLed()
        time = (self._node.get_clock().now() - self._start_time).nanoseconds*1e-9
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.5 + 0.5*math.sin(2*math.pi*time)
        self.get_resource('chest_led').publish(msg)
