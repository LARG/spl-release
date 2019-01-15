#ifndef VISION_BLOCKS_H
#define VISION_BLOCKS_H

#include <memory/MemoryCache.h>
#include <memory/WorldObjectBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>
#include <memory/ImageBlock.h>
#include <memory/RobotVisionBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/RobotInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/ROIBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/TeamPacketsBlock.h>

/// @ingroup vision
class VisionBlocks {
  public:
    WorldObjectBlock*& world_object;
    BodyModelBlock*& body_model;
    JointBlock*& joint;
    ImageBlock*& image;
    RobotVisionBlock*& robot_vision;
    FrameInfoBlock*& frame_info;
    RobotStateBlock*& robot_state;
    RobotInfoBlock*& robot_info;
    SensorBlock*& sensor;
    GameStateBlock*& game_state;
    ROIBlock*& roi;
    BehaviorBlock*& behavior;
    TeamPacketsBlock*& team_packets;    // Added this so we can filter teammates vs opponents in vision roughly using localization. It's also a dependency now in VisionModule - SN 7/19/17

    VisionBlocks(
        WorldObjectBlock*& world_object_block,
        BodyModelBlock*& body_model_block,
        JointBlock*& joint_block,
        ImageBlock*& image_block,
        RobotVisionBlock*& robot_vision_block,
        FrameInfoBlock*& frame_info_block,
        RobotStateBlock*& robot_state_block,
        RobotInfoBlock*& robot_info_block,
        SensorBlock*& sensor_block,
        GameStateBlock*& game_state_block,
        ROIBlock*& roi_block,
        BehaviorBlock*& behavior_block,
        TeamPacketsBlock*& team_packets_block
      )
      : world_object(world_object_block),
      body_model(body_model_block),
      joint(joint_block),
      image(image_block),
      robot_vision(robot_vision_block),
      frame_info(frame_info_block),
      robot_state(robot_state_block),
      robot_info(robot_info_block),
      sensor(sensor_block),
      game_state(game_state_block),
      roi(roi_block),
      behavior(behavior_block),
      team_packets(team_packets_block)
      { }

    VisionBlocks(MemoryCache& cache)
      : world_object(cache.world_object),
      body_model(cache.body_model),
      joint(cache.joint),
      image(cache.image),
      robot_vision(cache.robot_vision),
      frame_info(cache.frame_info),
      robot_state(cache.robot_state),
      robot_info(cache.robot_info),
      sensor(cache.sensor),
      game_state(cache.game_state),
      roi(cache.roi),
      behavior(cache.behavior),
      team_packets(cache.team_packets)  
      { }
};

#endif
