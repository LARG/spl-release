#!/usr/bin/env python

import core
from core import DEG_T_RAD, RAD_T_DEG
from task import Task, MultiTask
import pose, head, kicks, skills, body, nodes, field
import commands, cfgstiff, cfgcam, cfgpose
import strategy
from state_machine import *
import roles, keeper
from memory import *

lastState = None
currentState = None
currentTask = None

def processFrame():
  global currentState, lastState, currentTask
  commands.setHeadTilt()
  lastState = currentState
  currentState = game_state.state()


  if currentState == core.INITIAL:
    audio_processing.state_ = core.AudioProcessingBlock.Detecting
  elif currentState == core.SET:
    behavior_mem.ballLeftCenter = False
    whistleFrame = max(audio_processing.whistle_heard_frame_, audio_processing.teammate_heard_frame_)
    tilt = sensors.getValue(core.angleY)
    roll = sensors.getValue(core.angleX)
    if util.gettingUp() or abs(tilt) > 10 * DEG_T_RAD or abs(roll) > 10 * DEG_T_RAD:
      audio_processing.state_ = core.AudioProcessingBlock.Off
    else:
      audio_processing.state_ = core.AudioProcessingBlock.Detecting
    # if we are not a keeper or a defender and we've heard the whistle recently, override the current game state
    if robot_state.WO_SELF > 1 and vision_frame_info.frame_id - whistleFrame <= 30: # Game controller will keep sending the set state in the start of play
      game_state.whistleOverride()
      currentState = game_state.state()
      behavior_mem.timePlayingStarted = vision_frame_info.seconds_since_start
  else:
    ball = world_objects.getObjPtr(core.WO_BALL).loc
    if ball.getMagnitude() > core.CIRCLE_RADIUS + 300:
      if not behavior_mem.ballLeftCenter:
        print "ball left the center!"
      behavior_mem.ballLeftCenter = True
    audio_processing.state_ = core.AudioProcessingBlock.Off


  if util.currentFrame() % 30 == 0: util.checkTemperatures()
  if util.currentFrame() % 30 == 0: util.checkCommunication()

  strategy.selectSetPlay()

  if odometry.getting_up_side_ != core.Getup.NONE:
    audio_processing.state_ = core.AudioProcessingBlock.Off
    util.setFallCounter()
    UTdebug.tlog(20, "odom getting up, set no walk or kick")
    walk_request.noWalk()
    kick_request.abortKick()
    currentState = core.FALLING
  
  if lastState == core.FALLING and currentState != core.FALLING:
    # we're not doing a getup from the keeper dive for sure now
    walk_request.getup_from_keeper_dive_ = False

  if areDistinct(currentState, lastState):
    if currentTask: currentTask.finish()
    currentTask = createStateTask(currentState)

  if util.gettingUp():
    util.setFallenCounter()
    UTdebug.tlog(20, "odom getting up, set no walk or kick")
    walk_request.noWalk()
    kick_request.abortKick()
    return

  if util.checkFallen() and robot_state.WO_SELF != core.WO_TEAM_COACH and not game_state.isPenaltyKick:
    commands.setStiffness(cfgstiff.Zero)
    kick_request.abortKick()
    walk_request.setFalling()
    return
  

  #UTdebug.stimer("ctask")
  currentTask.processFrame()
  #UTdebug.etimer("ctask")

def areDistinct(state1, state2):
  if state1 == core.INITIAL and state2 == core.FINISHED: return False
  if state1 == core.FINISHED and state2 == core.INITIAL: return False
  if state1 == state2: return False
  return True

def createStateTask(state):
  if UTdebug.TRACE:
    states = ['undef', 'initial', 'ready', 'set', 'playing', 'testing', 'penalised', 'finished', 'falling', 'bottom', 'top', 'test', "manual"]
    print "Starting state:",states[state]
  if robot_state.WO_SELF == core.WO_TEAM_COACH:
    if state in [core.READY, core.SET, core.PLAYING, core.TESTING]:
      return roles.CoachPlaying()
    else:
      return roles.CoachInitial()
  
  if state == core.INITIAL: return Initial()
  if state == core.FINISHED: return Finished()
  if state == core.READY: return Ready()
  if state == core.PLAYING: return Playing()
  if state == core.TESTING: return Testing()
  if state == core.PENALISED: return Penalised()
  if state == core.SET: return Set()
  if state == core.FALLING: return Falling()
  if state == core.MANUAL_CONTROL: return ManualControl()
  raise Exception("Invalid state: %i" % state)

class Initial(pose.Sit): pass
class Ready(pose.Sit): pass
class Set(pose.Sit): pass
class Playing(pose.Sit): pass
class Penalised(pose.StandStraight): pass
class Finished(pose.Sit): pass
class Set(head.TrackBall): pass
class Falling(Task): pass

def load(bvr):
  import importlib
  m = importlib.import_module('behaviors.' + bvr)
  global Ready, Set, Playing, Testing
  if hasattr(m, 'Ready'): Ready = m.Ready
  if hasattr(m, 'Set'): Set = m.Set
  if hasattr(m, 'Playing'): Playing = m.Playing
  if hasattr(m, 'Testing'): Testing = m.Testing

class ManualControl(Task):
  def reset(self):
    self.otimer = util.Timer()
    self.stance = -1
    self.head = head.Scan() #LookForBall()
    super(ManualControl, self).reset()
    self.tracker = nodes.RotationTracker()

  def run(self):
    self.tracker.check()
    commands.setHeadTilt()
    commands.setStiffness()
    if behavior_mem.test_odom_new:
      self.otimer.reset()
      behavior_mem.test_odom_new = False
    if self.otimer.elapsed() > behavior_mem.test_odom_walk_time:
      if self.stance != core.Poses.SITTING:
        self.stance = core.Poses.SITTING
        return pose.Sit()
      return

    velX = behavior_mem.test_odom_fwd
    velY = behavior_mem.test_odom_side
    velTheta = behavior_mem.test_odom_turn
    reqstance = behavior_mem.test_stance
    if reqstance != self.stance:
      self.stance = reqstance
      if reqstance == core.Poses.SITTING:
        return pose.Sit()
      elif not util.isStanding():
        return pose.Stand()
    if self.stance == core.Poses.STANDING:
      commands.setWalkVelocity(velX, velY, velTheta)
      self.head.processFrame()
