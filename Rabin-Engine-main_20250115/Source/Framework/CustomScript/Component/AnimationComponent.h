#pragma once
#ifndef ANIMATIONCOMPONENT_H
#define ANIMATIONCOMPONENT_H

#include "Component.h"
#include "../Misc/NiceTypes.h" 
#include <vector>

struct KeyFrame {
    Vec3 position;        // Position at this keyframe
    Vec3 rotation;       // Rotation (Euler angles in degrees) at this keyframe
    Vec3 scale;           // Scale at this keyframe
};

// Animation structure
struct Animation 
{
    std::vector<KeyFrame> keyframes; // All keyframes for the animation
    float frameSpeed;                  // Total duration of the animation
};

class AnimationComponent : public Component
{
private:
    Animation animation; // Holds animation data (keyframes, duration, etc.)
    float currentTime;   // Current time in the animation timeline
    int currentFrame = 0;      // Index of the current frame being displayed
    float elapsedTime = 0.0f;  // Accumulated time since the last frame update

    bool isPlaying;
    bool isLoop;

public:
    AnimationComponent(Agent& _owner);

    void update(float deltaTime);
    void add_anim(const Animation& anim, bool _isLoop = false);
    void play();
    void stop();
    void reset();
    int getCurrentFrame();
};

#endif