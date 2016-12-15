import memory, geometry
import core

def init():
  global world_objects, abs_parts, rel_parts, opponent_models
  world_objects = ObjectAccessor(memory.world_objects.getObjPtr, core.NUM_WORLD_OBJS)
  abs_parts = ObjectAccessor(memory.body_model.getAbsPartPtr, core.BodyPart.NUM_PARTS)
  rel_parts = ObjectAccessor(memory.body_model.getRelPartPtr, core.BodyPart.NUM_PARTS)
  opponent_models = ObjectAccessor(memory.opponent_mem.getModel, core.MAX_OPP_MODELS_IN_MEM)
  update()

def update():
  global close_ball, max_vel
  fball = core.localizationC.filtered_close_ball_
  close_ball = geometry.Point2D(fball.x, fball.y)
  max_vel = geometry.Point2D(memory.walk_param.bh_params_.rsSpeedMax.translation.x,memory.walk_param.bh_params_.rsSpeedMax.translation.y)

class ObjectAccessor:
  def __init__(self, func, max_index):
    self.func = func
    self.max_index = max_index
    self.objects = {i:None for i in range(self.max_index)}

  def __getitem__(self, i):
    obj = self.objects[i]
    if not obj:
      self.objects[i] = obj = self.func(i)
    return obj


