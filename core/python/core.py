import sys
import pythonswig_module
import math

DEG_T_RAD = math.pi / 180.0
RAD_T_DEG = 180.0 / math.pi

def init():
  global instance, swig
  global visionC, pythonC, ledsC, localizationC, opponentsC, behaviorC
  global text_logger
  global sensor_values, joint_values, joint_stiffness
  global RANDOM_SEED
  global TOOL
  global CONFIG_ID
  global ENABLE_SONAR
  global OPTIMIZE, DEBUG

  swig = pythonswig_module
  instance = pythonswig_module.PythonInterface().CORE_INSTANCE
  visionC = instance.vision_
  pythonC = instance.interpreter_
  behaviorC = instance.behavior_
  ledsC = instance.leds_
  localizationC = instance.localization_
  opponentsC = instance.opponents_
  text_logger = instance.textlog()
  TOOL = (instance.type_ == swig.CORE_TOOL)
  ENABLE_SONAR = instance.rconfig().sonar_enabled
  OPTIMIZE = instance.EnableOptimizations()
  DEBUG = not OPTIMIZE

  joint_values = pythonC.joint_values_
  sensor_values = pythonC.sensor_values_
  joint_stiffness = pythonC.joint_stiffness_

  this = sys.modules[__name__]
  for item in dir(swig):
    if item.startswith("__"): continue
    setattr(this, item, getattr(swig, item))
  RANDOM_SEED = swig.Random.SEED
  CONFIG_ID = -1
