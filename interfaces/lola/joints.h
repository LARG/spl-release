#pragma once

/* Structures to store values of various joints */

#include <array>

namespace Lola {

	struct Joint {
	    float angle = 0.f;
	    float stiffness = 0.f;
	    float temperature = 0.f;
	    float current = 0.f;
	    int status = 0;
	};

	enum HeadJointNames {
	    HeadYaw = 0,
	    HeadPitch,
	    HEAD_JOINT_SIZE
	};

	enum ArmJointNames {
	    LShoulderPitch = 0,
	    LShoulderRoll,
	    LElbowYaw,
	    LElbowRoll,
	    LWristYaw,
	    LHand,
	    RShoulderPitch,
	    RShoulderRoll,
	    RElbowYaw,
	    RElbowRoll,
	    RWristYaw,
	    RHand,
	    ARM_JOINT_SIZE
	};

	enum LegJointNames {
	    LHipRoll = 0,
	    LHipPitch,
	    LKneePitch,
	    LAnklePitch,
	    LAnkleRoll,
	    HipYawPitch,
	    RHipRoll,
	    RHipPitch,
	    RKneePitch,
	    RAnklePitch,
	    RAnkleRoll,
	    LEG_JOINT_SIZE
	};

	using HeadJoints = std::array<Joint, HEAD_JOINT_SIZE>;
	using ArmJoints = std::array<Joint, ARM_JOINT_SIZE>;
	using LegJoints = std::array<Joint, LEG_JOINT_SIZE>;

	using HeadJointAngles = std::array<float, HEAD_JOINT_SIZE>;
	using ArmJointAngles = std::array<float, ARM_JOINT_SIZE>;
	using LegJointAngles = std::array<float, LEG_JOINT_SIZE>;


	struct Joints {
	    HeadJoints head;
	    ArmJoints arms;
	    LegJoints legs;
	};

	enum class JointCategory {
	    HEAD, ARM, LEG
	};

}

template<std::size_t N>
void angle_interpolation(const std::array<float, N>& from, const std::array<float, N>& to, float val, std::array<Lola::Joint, N>* target) {
    for (size_t i = 0; i < from.size(); i++) {
        (*target)[i].angle = from[i] * (1.f - val) + to[i] * val;
    }
}

template<std::size_t N>
void angle_interpolation(const std::array<float, N>& from, const std::array<float, N>& to, float val, std::array<float, N>* target) {
    for (size_t i = 0; i < from.size(); i++) {
        (*target)[i] = from[i] * (1.f - val) + to[i] * val;
    }
}

template<std::size_t N>
void joint_interpolation(const std::array<Lola::Joint, N>& from, const std::array<Lola::Joint, N>& to, float val, std::array<Lola::Joint, N>* target) {
    for (size_t i = 0; i < from.size(); i++) {
        (*target)[i].angle = from[i].angle * (1.f - val) + to[i].angle * val;
        (*target)[i].stiffness = from[i].stiffness * (1.f - val) + to[i].stiffness * val;
    }
}

inline void joint_interpolation(const Lola::Joints& from, const Lola::Joints& to, float val, Lola::Joints* target) {
    joint_interpolation(from.arms, to.arms, val, &target->arms);
    joint_interpolation(from.legs, to.legs, val, &target->legs);
    joint_interpolation(from.head, to.head, val, &target->head);
}

template<std::size_t N>
std::array<float, N> extract_angles(const std::array<Lola::Joint, N>& joints) {
    std::array<float, N> angles;
    for (size_t i = 0; i < joints.size(); i++) {
        angles[i] = joints[i].angle;
    }
    return angles;
}

template<std::size_t N>
void set_angles(const std::array<float, N>& angles, std::array<Lola::Joint, N>* joints) {
    for (size_t i = 0; i < joints->size(); i++) {
        (*joints)[i].angle = angles[i];
    }
}

template<typename Joints>
void set_stiffness(float val, Joints* joints) {
    for (size_t i = 0; i < joints->size(); i++) {
        (*joints)[i].stiffness = val;
    }
}

inline void mirrorLtoR(Lola::ArmJoints* joints) {
    (*joints)[Lola::RShoulderPitch] = (*joints)[Lola::LShoulderPitch];
    (*joints)[Lola::RShoulderRoll] = {-(*joints)[Lola::LShoulderRoll].angle, (*joints)[Lola::LShoulderRoll].stiffness};
    (*joints)[Lola::RElbowYaw] = {-(*joints)[Lola::LElbowYaw].angle, (*joints)[Lola::LElbowYaw].stiffness};
    (*joints)[Lola::RElbowRoll] = {-(*joints)[Lola::LElbowRoll].angle, (*joints)[Lola::LElbowRoll].stiffness};
    (*joints)[Lola::RWristYaw] = {-(*joints)[Lola::LWristYaw].angle, (*joints)[Lola::LWristYaw].stiffness};
    (*joints)[Lola::RHand] = (*joints)[Lola::LHand];
}

struct WalkRequest {
    float dx, dy, da;
};
