#!/usr/bin/env python
import core
core.init() # this has to be run before anything else can be imported
import memory, mem_objects
import sys, os, traceback
import config

def init():
  global firstFrame
  firstFrame = True
  initMemory()
  initNonMemory()
  print "Python initialized"

def initMemory():
  memory.init()
  mem_objects.init()
  cfgwalk.initWalk()

def initNonMemory(initLoc=True):
  core.CONFIG_ID = cfgmap.getConfigId()
  cfgwalk.initWalk()
  config.configLocalization()
  if initLoc:
    core.localizationC.reInit()
  core.opponentsC.reInit()
  memory.localization_mem.blueSide = core.Sides.Undefined

  kicks.checkKicks() # must be before setKickAngles
  

  cfgpositions.initializePositions()

def processFrame():
  try:
    global firstFrame
    if firstFrame:
      memory.world_objects.init(memory.robot_state.team_)
      core.visionC.initSpecificModule()
      #core.localizationC.initSpecificModule()
      initNonMemory(initLoc=False)
      memory.speech.say("Vision")
      runBehavior("soccer")
      firstFrame = False

    core.visionC.processFrame()
    core.localizationC.processFrame()
    core.opponentsC.processFrame()
    processBehaviorFrame()
    core.instance.publishData()
  except:
    handle()

def processBehaviorFrame():
  try:
    if memory.robot_state.WO_SELF != core.WO_TEAM_COACH:
      mem_objects.update()
    if core.TOOL:
      initMemory()
    camOffsets = cfgcam.getCamOffsets()
    core.localizationC.filterCloseBallPosition(camOffsets.rel_ball_fwd,camOffsets.rel_ball_side,camOffsets.scale_side)
    core.behaviorC.processFieldCalculations()
    memory.behavior_mem.setAllUnevaluated()
    core.pythonC.updatePercepts()
  except: 
    handle()

def handle():
  lines = traceback.format_exception(*(sys.exc_info()))
  message = ''.join('!! ' + line for line in lines)
  if core.instance.type_ == core.CORE_TOOLSIM:
    print message
  memory.speech.say("python")
  core.pythonC.is_ok_ = False
  with open(os.path.expanduser('~/error.txt'), 'w') as f:
    f.write(message)

