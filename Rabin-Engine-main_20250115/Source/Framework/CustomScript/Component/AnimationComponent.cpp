#include "pch.h"
#include "AnimationComponent.h"

AnimationComponent::AnimationComponent(Agent& _owner, const AnimationController& anim)
    :  Component(_owner), animation(anim), currentTime(0.0f) {}

void AnimationComponent::update(float deltaTime) {
    // Advance the animation time, looping if necessary
    currentTime += deltaTime * animation.ticksPerSecond;

    if (currentTime > animation.duration) {
        currentTime = fmod(currentTime, animation.duration);
    }
}