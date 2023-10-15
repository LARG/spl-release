"""Basic event types for the state machine."""
# pylint: disable=too-many-arguments

from typing import Any, Callable, Union
from rclpy.time import Duration
from behavior.state_machine import State, Event, CompositeEvent

class StateValueAtLeast(Event):
    """Event that fires when a state value is above a certain threshold."""
    def __init__(self, name:str, value_fn:Callable[[State],Any], threshold:Any,
                 lower_threshold:Any=None):
        super().__init__(name)
        self._value_fn = value_fn
        self._threshold = threshold
        if lower_threshold:
            if lower_threshold > threshold:
                raise ValueError("Lower threshold must be less than upper threshold.")
        self._lower_threshold = lower_threshold
        self.initialized = False

    def reset(self):
        super().reset()
        self.initialized = False

    def update(self):
        """Checks if the value is above the threshold."""
        value = self._value_fn(self._state)
        if value is None:
            return False
        if value >=self._threshold:
            self._status = True
        elif self._lower_threshold and value <=self._lower_threshold:
            self._status = False
        self.initialized = True
        return self._status

class StateValueEquals(Event):
    """Event that fires when a state value is equal to a certain value."""
    def __init__(self, name:str, value_fn:Callable[[State],Any], value:Any=True):
        super().__init__(name)
        self._value_fn = value_fn
        self._target_value = value
        self.initialized = False

    def reset(self):
        super().reset()
        self.initialized = False

    def update(self):
        """Checks if the value is equal to the threshold."""
        value = self._value_fn(self._state)
        if value is None:
            return False
        self._status = value == self._target_value
        self.initialized = True
        return self._status

class SubscriptionValueAtLeast(Event):
    """Event that fires when a subscription value is above a certain threshold.

    @param name: Name of the event.
    @param sub_res: name of SubscriptionResource
    @param msg_attrib: name of attribute in message to check
    @param threshold: threshold value
    @param lower_threshold: lower threshold value (optional)
    @param state: State the state that the event is called from
    """
    def __init__(self, name:str, sub_res:str, msg_attrib:Union[str,Callable[[Any],Any]],
                 threshold:Any, lower_threshold:Any=None):
        super().__init__(name)
        self._sub_res = sub_res
        self.add_required_resource(sub_res)
        if isinstance(msg_attrib, str):
            self._value_fn = lambda msg: getattr(msg, msg_attrib)
        else:
            self._value_fn = msg_attrib
        self._threshold = threshold
        if lower_threshold:
            if lower_threshold >=threshold:
                raise ValueError("Lower threshold must be less than upper threshold.")
        self._lower_threshold = lower_threshold
        self._value = None
        self.initialized = False

    def msg_callback(self, msg):
        """Callback for the subscription."""
        self._value = self._value_fn(msg)
        self.initialized = True

    def reset(self):
        self._value = None
        super().reset()
        self.initialized = False

    def update(self):
        """Checks if the value is above the threshold."""
        if self._value is None:
            return False
        if self._value >= self._threshold:
            self._status = True
        elif self._lower_threshold and self._value <=self._lower_threshold:
            self._status = False
        return self._status

    def enable(self, state:State):
        """Enables the subscription."""
        super().enable(state)
        res = None
        res = state.get_resource(self._sub_res)
        res.add_event_callback(self.msg_callback)

    def disable(self):
        """Disables the subscription."""
        if not self.is_enabled():
            return
        res = self._state.get_resource(self._sub_res)
        res.remove_event_callback(self.msg_callback)
        super().disable()

class SubscriptionValueEquals(Event):
    """Event that fires when a subscription value is equal to a certain value.

    @param name: Name of the event.
    @param sub_res: name of SubscriptionResource
    @param msg_attrib: name of attribute in message to check
    @param value: value to check
    """
    def __init__(self, name:str, sub_res:str, msg_attrib:Union[str,Callable[[Any],Any]],
                 value:Any=True):
        super().__init__(name)
        self._sub_res = sub_res
        self.add_required_resource(sub_res)
        if isinstance(msg_attrib, str):
            self._value_fn = lambda msg: getattr(msg, msg_attrib)
        else:
            self._value_fn = msg_attrib
        self._target_value = value
        self._value = None
        self.initialized = False

    def reset(self):
        self._value = None
        super().reset()
        self.initialized = False

    def msg_callback(self, msg):
        """Callback for the subscription."""
        self._value = self._value_fn(msg)
        self.initialized = True

    def update(self):
        """Checks if the value is equal to the threshold."""
        if self._value is None:
            return False
        self._status = self._value == self._target_value
        print(f"{self._name}: {self._value}, {self._status}")
        return self._status

    def enable(self, state:State):
        """Enables the subscription."""
        super().enable(state)
        res = state.get_resource(self._sub_res)
        res.add_event_callback(self.msg_callback)

    def disable(self):
        """Disables the subscription."""
        if not self.is_enabled():
            return
        res = self._state.get_resource(self._sub_res)
        res.remove_event_callback(self.msg_callback)
        super().disable()

class TimeEvent(StateValueAtLeast):
    """Event that fires after a certain amount of time has passed."""
    def __init__(self, duration:Union[float,Duration]):
        if isinstance(duration, float):
            duration = Duration(seconds=duration)
        super().__init__(f"Timer_{duration.nanoseconds*1e-9:.3f}s",
                         lambda s: s.get_elapsed_time(), duration)

class DelayEvent(CompositeEvent):
    """Event wrapper that fires after base event has been true for a certain
    amount of time. A kind of low pass filter.
    """
    def __init__(self, event:Event, duration:Union[float,Duration],
                 reset_duration:Union[float,Duration]=None,
                 name:str=None):
        if isinstance(duration, float):
            duration = Duration(seconds=duration)
        if reset_duration and isinstance(reset_duration, float):
            reset_duration = Duration(seconds=reset_duration)
        super().__init__(name if name else
                         f"{event.get_name()}_{duration.nanoseconds*1e-9:.3f}s")
        self.add_event(event)
        self._wrapped_event = event
        print(f"DelayEvent: {self._name} from {event.get_name()}")
        self._duration = duration
        self._reset_duration = reset_duration if reset_duration else duration
        self._flip_time = None

    def update(self):
        """Checks if the event has been true for a certain amount of time."""
        super().update()
        if self._wrapped_event.get_status():
            if self._status:
                self._flip_time = None
            else:
                if self._flip_time:
                    if (self._state.get_current_time() -
                            self._flip_time) > self._reset_duration:
                        self._status = True
                else:
                    self._flip_time = self._state.get_current_time()
        else:  # wrapped event is false
            if self._status:
                if self._flip_time:
                    if (self._state.get_current_time() -
                           self._flip_time) > self._duration:
                        self._status = False
                else:
                    self._flip_time = self._state.get_current_time()
            else:
                self._flip_time = None

        return self._status

    def reset(self):
        """Resets the event counter."""
        self._flip_time = None
        super().reset()

class FallingEdgeEvent(CompositeEvent):
    """Event wrapper that fires when the base event goes from true to false."""
    def __init__(self, event:Event, name:str=None):
        super().__init__(name if name else f"FallingEdge_{event.get_name()}")
        self.add_event(event)
        self._wrapped_event = event
        self._prev_status = None

    def update(self):
        """Checks if the event has gone from true to false."""
        super().update()
        if self._wrapped_event.get_status():
            self._prev_status = True
        elif self._prev_status:
            self._prev_status = False
            self._status = True
        else:
            self._status = False
        return self._status

    def reset(self):
        """Resets the event counter."""
        self._prev_status = None
        super().reset()

class RisingEdgeEvent(FallingEdgeEvent):
    """Event wrapper that fires when the base event goes from false to true."""
    def __init__(self, event:Event, name:str=None):
        super().__init__(~event, name if name else f"RisingEdge_{event.get_name()}")
