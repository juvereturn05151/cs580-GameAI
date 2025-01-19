#pragma once
#ifndef ANIMATIONCOMPONENT_H
#define ANIMATIONCOMPONENT_H

#include "Component.h"
#include "../Misc/NiceTypes.h" 
#include <vector>

struct KeyFrame {
    float timeStamp;           // Time at which this keyframe occurs
    Vec3 position;        // Position at this keyframe
    Vec3 rotation;       // Rotation (Euler angles in degrees) at this keyframe
    Vec3 scale;           // Scale at this keyframe
};

// Animation structure
struct AnimationController {
    std::vector<KeyFrame> keyframes; // All keyframes for the animation
    float duration;                  // Total duration of the animation
    float ticksPerSecond;            // Speed of the animation
};

class AnimationComponent : public Component
{
private:
    AnimationController animation; // Holds animation data (keyframes, duration, etc.)
    float currentTime;   // Current time in the animation timeline

public:
    AnimationComponent(Agent& _owner, const AnimationController& anim);

    void update(float deltaTime);
};

#endif