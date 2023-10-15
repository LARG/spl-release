"""Resources used by the state machine.

In practical terms, resources provide the interfaces between the state machine
and the ros framework. State machines aquire resources when they are needed.
Accessing the resources through this interface makes it easy to investigate
which states are accessing particular resources, which is useful for
debugging."""

from rclpy.node import Node
from walk_msg.msg import Walk
from std_msgs.msg import Bool
from keyframe_motion_msg.msg import Motion
from nao_command_msgs.msg import JointPositions, JointStiffnesses, JointIndexes
from behavior.state_machine import Resource, UniqueResource

class SubscriptionResource(Resource):
    """A resource representing a ros subscription."""
    def __init__(self, node:Node, topic:str, msg_type, name=None):
        super().__init__(name or topic)
        self._node = node
        self._topic = topic
        self._msg_type = msg_type
        self._subscription = self._node.create_subscription(
                self._msg_type, self._topic, self._callback, 1)
        self._msg = None
        self._event_callbacks = []

    def _callback(self, msg):
        """A callback for the subscription."""
        self._msg = msg
        for callback in self._event_callbacks:
            callback(msg)

    def get(self):
        """Get the most recent message."""
        return self._msg or self._msg_type()

    def add_event_callback(self, callback):
        """Add a callback to be called when the subscription receives a message."""
        self._event_callbacks.append(callback)

    def remove_event_callback(self, callback):
        """Remove a callback from the list of callbacks to be called when the
        subscription receives a message."""
        self._event_callbacks.remove(callback)

class PublisherResource(UniqueResource):
    """A resource representing a ros publisher."""
    def __init__(self, node, topic, msg_type, name=None):
        super().__init__(name or topic)
        self._node = node
        self._topic = topic
        self._msg_type = msg_type
        self._publisher = self._node.create_publisher(self._msg_type, self._topic, 1)

    def publish(self, msg):
        """Publish a message."""
        self._publisher.publish(msg)

class MotionPublisherResource(UniqueResource):
    """The publisher for the robot motion."""
    def __init__(self, node:Node):
        super().__init__("motion")
        self._walk_publisher = node.create_publisher(Walk, "/motion/walk", 1)
        self._keyframe_publisher = node.create_publisher(
            Motion, "/motion/keyframe_motions", 1)
        self._head_pos_publisher = node.create_publisher(
            JointPositions, "effectors/joint_positions", 1)
        self._head_stiff_publisher = node.create_publisher(
            JointStiffnesses, "effectors/joint_stiffnesses", 1)
        self._kf_done = True
        self._is_walking = False
        self._kf_done_subcription = node.create_subscription(
            Bool, "/keyframe_motions/is_done", self._kf_done_callback, 1)

    def _kf_done_callback(self, msg):
        """A callback for the keyframe done subscription."""
        self._kf_done = msg.data

    def walk(self, forward=0.0, left=0.0, turn=0.0):
        """Publish a walk message."""
        if not self._kf_done:
            kf_stop_msg = Motion()
            kf_stop_msg.motion = "stop"
            self._keyframe_publisher.publish(kf_stop_msg)
        msg = Walk()
        msg.forward = forward
        msg.left = left
        msg.turn = turn
        msg.kick = 0
        msg.speed = 1.0
        msg.bend = 1.0
        msg.power = 1.0
        msg.enabled = True
        self._walk_publisher.publish(msg)
        self._is_walking = True

    def keyframe_motion(self, motion:str):
        """Publish a keyframe motion message."""
        if  self._is_walking:
            walk_msg = Walk()
            walk_msg.enabled = False
            self._walk_publisher.publish(walk_msg)
            self._is_walking = False
        direction = ""
        if motion == "getup_front":
            motion = "getup"
            direction = "FRONT"
        elif motion == "getup_back":
            motion = "getup"
            direction = "BACK"
        msg = Motion()
        msg.speed = "MODERATE"
        msg.motion = motion
        msg.direction = direction
        self._keyframe_publisher.publish(msg)

    def head_position(self, yaw=0.0, pitch=0.0):
        """Publish a head position message."""
        # Don't send head position if keyframe is running
        if not self._kf_done:
#            assert False
            return
        pmsg = JointPositions()
        pmsg.indexes = [JointIndexes.HEADYAW, JointIndexes.HEADPITCH]
        pmsg.positions = [yaw, pitch]
        self._head_pos_publisher.publish(pmsg)
        smsg = JointStiffnesses()
        smsg.indexes = [JointIndexes.HEADYAW, JointIndexes.HEADPITCH]
        smsg.stiffnesses = [1.0, 1.0]
        self._head_stiff_publisher.publish(smsg)
