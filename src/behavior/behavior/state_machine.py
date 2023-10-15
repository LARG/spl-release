"""Basic state machine implementation."""
# pylint: disable=unnecessary-pass, too-many-instance-attributes

from typing import Iterable, Union
from rclpy import logging
from rclpy.node import Node
from rclpy.time import Time, Duration

LOGGER = logging.get_logger("behavior")

class Resource:
    """The resource class is used to track access to shared resources.

    When a state starts, it attempts to acquire all the resources it needs.
    Resources are automatically freed when the state stops. By default,
    multiple states can access the same resource, but a UniqueResource will
    throw an error if a state tries to acquire a resource that is already owned
    by another state.

    The registry is a class variable that is used to keep track of all the
    resources and their owners.
    """
    _registry = {}

    def __init__(self, name: str):
        self._name = name
        if name in Resource._registry:
            LOGGER.error(f"Resource {name} already exists. "
                "Different resources must have unique names.")
        Resource._registry[name] = set()

    def __del__(self):
        Resource._registry.pop(self._name)

    def get_name(self) -> str:
        """Get the name of the resource"""
        return self._name

    def get_owner(self):
        """Get the owner of the resource"""
        return Resource._registry[self._name]

    def is_free(self) -> bool:
        """Check if the resource is free"""
        return len(Resource._registry[self._name]) == 0

    def assign(self, owner: 'State') -> 'Resource':
        """Assign the resource to an owner."""
        if owner in Resource._registry[self._name]:
            LOGGER.error(f"Resource {self._name} already owned by {owner}.")
        Resource._registry[self._name].add(owner)
        return self

    def transfer(self, old_owner: 'State', new_owner: 'State') -> 'Resource':
        """Transfer ownership of the resource"""
        if old_owner not in Resource._registry[self._name]:
            LOGGER.error(f"Resource {self._name} not owned by {old_owner}.")
        Resource._registry[self._name].remove(old_owner)
        Resource._registry[self._name].add(new_owner)
        return self

    def free(self, owner: 'State'):
        """Free the resource"""
        Resource._registry[self._name].remove(owner)

    @staticmethod
    def validate(*resources: str):
        """Check if a set of resource names is valid"""
        for resource in resources:
            if resource not in Resource._registry:
                LOGGER.error(f"Resource {resource} does not exist. Invalid name.")

class UniqueResource(Resource):
    """A resource that can only be owned by one state at a time"""
    def assign(self, owner: 'State') -> Resource:
        """Assign the resource to an owner."""
        if not self.is_free():
            LOGGER.error(f"Resource {self._name} already owned by "
                         f"{Resource._registry[self._name]}.")
        Resource._registry[self._name].add(owner)
        return self

class State:
    """Superclass for states"""
    def __init__(self, name,
                 node:Node = None,
                 resources:Iterable[str] = None,
                 parent:'StateMachine' = None):
        self._base_name = name
        self._name = name
        self._node = None
        self._running = False
        self._start_time = None
        self._required_resources = set() if resources is None else set(resources)
        self._parent = None
        self._resources = {}
        if node is not None:
            self._node = node
        if parent is not None:
            self.set_parent(parent)

    def set_node(self, node:Node):
        """Set the node for the state"""
        self._node = node

    def set_parent(self, parent:'StateMachine'):
        """Set the parent state machine"""
        if self._parent is not None:
            LOGGER.error(f"State {self._name} already has parent.")
        if self._running:
            LOGGER.error(f"State {self._name} already running. "
                         "Cannot change parent.")
        self._parent = parent
        self._name = f"{parent.get_name()}/{self._base_name}"

    def get_base_name(self):
        """Get the base name of the state"""
        return self._base_name

    def get_name(self):
        """Get the name of the state"""
        return self._name

    def get_required_resources(self) -> Iterable[str]:
        """Get the set of required resources"""
        return self._required_resources

    def add_required_resource(self, resource: Union[str, Iterable[str]]):
        """Add a required resource"""
        if self._running:
            LOGGER.error(f"State {self._name} already running. "
                         "Cannot add required resources.")
        if isinstance(resource, str):
            self._required_resources.add(resource)
        else:
            for res in resource:
                self._required_resources.add(res)
        if self._parent is not None:
            self._parent.add_required_resource(resource)

    def get_start_time(self) -> Time:
        """Get the time the state started"""
        return self._start_time

    def get_current_time(self) -> Time:
        """Get the current time"""
        return self._node.get_clock().now()

    def get_elapsed_time(self) -> Duration:
        """Get the time the state has been running"""
        return self._node.get_clock().now() - self._start_time

    def get_node(self) -> Node:
        """Get the node for the state"""
        return self._node

    def start(self):
        """Start the state"""
        if self._node is None:
            LOGGER.error(f"State {self._name} must be run within a ros2 node. "
                         "Use set_node() to set the node.")
        if self._running:
            LOGGER.warning(f"State {self._name} already running")
        for resource in self._required_resources:
            self._acquire_resource(resource)
        self._running = True
        self._start_time = self._node.get_clock().now()

    def stop(self):
        """Stop the state"""
        if not self._running:
            LOGGER.warning(f"State {self._name} not running")
            return
        self._release_all_resources()
        self._running = False

    def step(self):
        """Do one step of the state"""
        if not self._running:
            LOGGER.error(f"State {self._name} has not been started. Cannot step.")

    def get_resource(self, resource: str) -> Resource:
        """Get a resource reference"""
        return self._resources[resource]

    def _acquire_resource(self, resource_name: str):
        """Acquire a resource"""
        if resource_name in self._resources:
            LOGGER.warning(f"Resource {resource_name} already acquired")
        resource = self._parent.get_resource(resource_name)
        if isinstance(resource, UniqueResource):
            self._resources[resource_name] = resource.transfer(self._parent, self)
        else:
            self._resources[resource_name] = resource.assign(self)

    def _release_resource(self, resource_name: str):
        """Release a resource back to parent"""
        if resource_name not in self._resources:
            LOGGER.error(f"Resource {resource_name} not in resource list for "
                         f"{self.get_name()}.")
        resource = self._resources[resource_name]
        if isinstance(resource, UniqueResource):
            resource.transfer(self, self._parent)
        else:
            resource.free(self)
        self._resources[resource].transfer(self, self._parent)
        self._resources.pop(resource)

    def _release_all_resources(self):
        """Release all resources"""
        for resource in self._resources.values():
            if isinstance(resource, UniqueResource):
                resource.transfer(self, self._parent)
            else:
                resource.free(self)
        self._resources.clear()

    def validate(self):
        """Validate the state"""
        if self._parent is None:
            # Only need to validate resource names at global level
            Resource.validate(*self._required_resources)

class Event:
    """An event that signals a transtition between states"""

    def __init__(self, name):
        self._name = name
        self._status = False
        self._enabled = False
        self._state = None
        self._required_resources = set()
        self.initialized = True # Some events might need additional info before
                                # being initialized such as subscriber events

    def get_name(self) -> str:
        """Get the name of the event"""
        return self._name

    def reset(self):
        """Reset the event"""
        self._status = False

    def get_status(self) -> bool:
        """Get the status of the event without updating"""
        return self._status

    def update(self) -> bool:
        """Update the event and get status"""
        return self._status

    def add_required_resource(self, resource: Union[str, Iterable[str]]):
        """Add a required resource"""
        if isinstance(resource, str):
            self._required_resources.add(resource)
        else:
            for res in resource:
                self._required_resources.add(res)

    def get_required_resources(self) -> Iterable[str]:
        """Get the set of required resources"""
        return self._required_resources

    def enable(self, state:State):
        """Called when corresponding state is started"""
        if self._state is not None:
            LOGGER.error(f"Event {self._name} already enabled for state "
                         f"{self._state.get_name()}")
        self._state = state
        self._enabled = True

    def is_enabled(self) -> bool:
        """Check if event is enabled"""
        return self._enabled

    def disable(self):
        """Called when corresponding state is stopped"""
        self.reset()
        self._state = None
        self._enabled = False

    def __invert__(self) -> 'Event':
        if isinstance(self, NotEvent):
            return self._e1
        return NotEvent(self)

    def __and__(self, other:'Event') -> 'Event':
        return AndEvent(self, other)

    def __or__(self, other:'Event') -> 'Event':
        return OrEvent(self, other)


class CompositeEvent(Event):
    """An event that is a combination of other events."""
    def __init__(self, name):
        super().__init__(name)
        self._events = set()

    def get_base_events(self) -> Iterable[Event]:
        """Returns the base (non-composite) events"""
        return self._events

    def add_event(self, event:'Event'):
        """Add an event to the set"""
        if isinstance(event, self.__class__):
            self._events |= event.get_base_events()
        self._events.add(event)
        self._required_resources |= event.get_required_resources()

    def reset(self):
        for event in self._events:
            event.reset()
        self._status = False

    def update(self) -> bool:
        for event in self._events:
            event.update()
        return self._status

    def enable(self, state:State):
        super().enable(state)
        for event in self._events:
            if not event.is_enabled():
                event.enable(state)

class NotEvent(CompositeEvent):
    """An event that is true if its base event is false"""
    def __init__(self, event, name=None):
        super().__init__(f"not_{event.get_name()}" if name is None else name)
        self.add_event(event)
        self._e1 = event

    def update(self) -> bool:
        if not self._e1.initialized:
            return False
        self._status = not self._e1.update()
        return self._status

class AndEvent(CompositeEvent):
    """An event that is true if both of two events is true"""
    def __init__(self, event1, event2, name=None):
        super().__init__(f"{event1.get_name()}_and_{event2.get_name()}"
                         if name is None else name)
        self.add_event(event1)
        self.add_event(event2)
        self._e1 = event1
        self._e2 = event2

    def update(self) -> bool:
        super().update()
        self._status = self._e1.get_status() and self._e2.get_status()
        return self._status

class OrEvent(CompositeEvent):
    """An event that is true if at least one of two events is true"""
    def __init__(self, event1, event2, name=None):
        super().__init__(f"{event1.get_name()}_or_{event2.get_name()}"
                         if name is None else name)
        self.add_event(event1)
        self.add_event(event2)
        self._e1 = event1
        self._e2 = event2

    def update(self) -> bool:
        super().update()
        self._status = self._e1.get_status() or self._e2.get_status()
        return self._status

class StateMachine(State):
    """A basic state machine"""
    def __init__(self, name, node:Node = None):
        super().__init__(name, node=node)
        self._states = {} # type: Dict[str,State]
        self._events = {} # type: Dict[str,Event]
        self._transitions = {} # type: Dict[str,Dict[str,str]] # from -> event -> to
        self._current_state = None
        self._start_state = None

    def set_node(self, node:Node):
        super().set_node(node)
        for state in self._states.values():
            state.set_node(node)

    def add_state(self, state:State):
        """Add a state to the state machine"""
        if state.get_base_name() in self._states:
            LOGGER.warning(f"State {state.get_base_name()} already in state "
                            "machine. Skipping.")
            return
        state.set_node(self._node)
        state.set_parent(self)
        self._states[state.get_base_name()] = state
        self._required_resources |= state.get_required_resources()
        # By default, the start state is the first state added
        if self._start_state is None:
            self._start_state = state.get_base_name()

    def add_transition(self,
                       from_state:Union[str,State],
                       event:Event,
                       to_state:Union[str,State]):
        """Add a transition to the state machine"""
        if isinstance(from_state, State):
            from_state = from_state.get_base_name()
        if isinstance(to_state, State):
            to_state = to_state.get_base_name()
        if from_state not in self._states:
            LOGGER.error(f"State {from_state} not in state machine. Invalid transition.")
        if to_state not in self._states:
            LOGGER.error(f"State {to_state} not in state machine. Invalid transition.")
        if from_state not in self._events:
            self._events[from_state] = {}
        elif event in self._events[from_state]:
            LOGGER.warning(f"Transition from {from_state} on event {event} already exists. "
                           "Overwriting.")
        self._events[event.get_name()] = event
        if from_state not in self._transitions:
            self._transitions[from_state] = {}
        self._transitions[from_state][event.get_name()] = to_state
        self._states[from_state].add_required_resource(event.get_required_resources())


    def set_start(self, state:Union[str,State]):
        """Set the start state"""
        if isinstance(state, State):
            state = state.get_base_name()
        if state not in self._states:
            LOGGER.error(f"State {state} not in state machine. Invalid start state.")
        self._start_state = state

    def _transition(self, state:Union[str,State]):
        """Transition to a new state"""
        if isinstance(state, State):
            state = state.get_base_name()
        if state not in self._states:
            LOGGER.error(f"State {state} not in state machine. Invalid transition.")
        if self._current_state is not None:
            for event in self.get_all_base_events():
                event.disable()
            self._current_state.stop()
        self._current_state = self._states[state]
        self._current_state.start()
        if self._current_state.get_base_name() in self._transitions:
            for event_name in self._transitions[self._current_state.get_base_name()]:
                event = self._events[event_name]
                event.enable(self._current_state)
        LOGGER.debug(f"Transitioned to state {state}")
        print (f"======Transitioned to state {state}======")

    def get_all_base_events(self, state=None) -> Iterable[Event]:
        """Recursively gets a set of all base events for a state"""
        if state is None:
            state = self._current_state
        events = set()
        if state.get_base_name() not in self._transitions:
            return events
        for event_name in self._transitions[state.get_base_name()]:
            event = self._events[event_name]
            if isinstance(event, CompositeEvent):
                events |= self._events[event_name].get_base_events()
            events.add(event)
        return events

    def start(self):
        """Start the state machine"""
        super().start()
        if self._start_state is None:
            return
        self._transition(self._start_state)
        LOGGER.debug(f"Started state machine {self._name}")

    def stop(self):
        """Stop the state machine"""
        if self._current_state is not None:
            for event in self.get_all_base_events():
                event.disable()
            self._current_state.stop()
        super().stop()
        LOGGER.debug(f"Stopped state machine {self._name}")

    def step(self):
        """Do one step of the state machine"""
        if self._current_state is None:
            return
        if self._current_state.get_base_name() in self._transitions:
            for event_name, to_state in self._transitions[
                    self._current_state.get_base_name()].items():
                if self._events[event_name].update():
                    self._node.get_logger().info(
                        f"{event_name}: {self._current_state.get_base_name()} -> {to_state}")
                    self._transition(to_state)
                    break
        self._current_state.step()

    def _check_reachability(self, start_state:str, reachable:set):
        """Recursively check reachability of states"""
        if start_state in reachable:
            return
        reachable.add(start_state)
        if start_state not in self._transitions:
            return
        for to_state in self._transitions[start_state].values():
            self._check_reachability(to_state, reachable)

    def validate(self):
        """Validates State machine consturction to check for errors at initialization"""
        super().validate()
        if self._start_state is None:
            return
        # Recursively validate child state machines
        for state in self._states.values():
            state.validate()
        # Check reachability
        reachable = set()
        self._check_reachability(self._start_state, reachable)
        unreachable = set(self._states.keys()) - reachable
        if len(unreachable) > 0:
            LOGGER.warning(f"State machine {self._name} has unreachable states: {unreachable}")
        # Check resource sharing
        for state in self._states.values():
            missing_resources = state.get_required_resources() - self._required_resources
            if len(missing_resources) > 0:
                LOGGER.error(f"State {state.get_name()} requires resources {missing_resources} "
                             f"that are not provided by state machine {self._name}")
        for state, events in self._transitions.items():
            for event in events:
                missing_resources = (self._events[event].get_required_resources() -
                                     self._states[state].get_required_resources())
                if len(missing_resources) > 0:
                    LOGGER.error(f"Event {event} requires resources {missing_resources} "
                                 f"that are not provided by state {state.get_name()}")

class StateMachineRoot(StateMachine):
    """A State Machine with no parent"""
    def __init__(self, node:Node):
        super().__init__("/", node)

    def new_resource(self, resource:Resource):
        """Add a new resource to the state machine"""
        self.add_required_resource(resource.get_name())
        self._resources[resource.get_name()] = resource.assign(self)

    def set_parent(self, parent):
        LOGGER.error("Cannot set parent of root state machine")
    def _acquire_resource(self, resource_name):
        pass
    def _release_resource(self, resource_name):
        pass
    def _release_all_resources(self):
        pass

class ParallelState(State):
    """A state that runs multiple states in parallel"""
    def __init__(self, name, node:Node = None, states:Iterable[State] = None):
        super().__init__(name, node)
        self._states = []
        if states is not None:
            for state in states:
                self.add_state(state)

    def add_state(self, state):
        """Add a state to the parallel state"""
        self._states.append(state)
        state.set_node(self._node)
        state.set_parent(self._resources)
        self._required_resources |= state.get_required_resources()
        if self._running:
            state.start()

    def start(self):
        """Start the parallel state"""
        super().start()
        for state in self._states:
            state.start()

    def stop(self):
        """Stop the parallel state"""
        for state in self._states:
            state.stop()
        super().stop()

    def step(self):
        """Do one step of the parallel state"""
        super().step()
        for state in self._states:
            state.step()

    def validate(self):
        super().validate()
        # Validate child states
        for state in self._states:
            state.validate()
        # Check resource sharing
        resource_counts = {}
        for state in self._states:
            for resource in state.get_required_resources():
                if resource not in resource_counts:
                    resource_counts[resource] = 0
                resource_counts[resource] += 1
        for resource, count in resource_counts.items():
            if count > 1 and isinstance(resource, UniqueResource):
                LOGGER.warning(f"UniqueResource {resource} is shared by {count} states in"
                               f"parallel state {self._name}. This may cause runtime "
                                "errors or unexpected behavior.")
