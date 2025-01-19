#include "pch.h"
#include "AnimationComponent.h"

AnimationComponent::AnimationComponent(Agent& _owner)
    :  Component(_owner), currentTime(0.0f), isPlaying(false) {}

void AnimationComponent::update(float deltaTime) {
    if (isPlaying)
    {
        // Advance the animation time, looping if necessary
        currentTime += deltaTime * animation.ticksPerSecond;

        if (currentTime > animation.duration) {
            currentTime = fmod(currentTime, animation.duration);
        }

        if (animation.keyframes.empty()) {
            return;
        }

        // Find the current and next keyframe
        const KeyFrame* currentFrame = nullptr;
        const KeyFrame* nextFrame = nullptr;

        for (size_t i = 0; i < animation.keyframes.size() - 1; ++i) {
            if (currentTime >= animation.keyframes[i].timeStamp &&
                currentTime < animation.keyframes[i + 1].timeStamp) {
                currentFrame = &animation.keyframes[i];
                nextFrame = &animation.keyframes[i + 1];
                break;
            }
        }

        // If no valid frames found, return identity matrix
        if (!currentFrame || !nextFrame) {
            return;
        }

        // Calculate the interpolation factor
        float deltaTime = nextFrame->timeStamp - currentFrame->timeStamp;
        float factor = (currentTime - currentFrame->timeStamp) / deltaTime;

        // Manually interpolate position
        Vec3 interpolatedPosition(
            currentFrame->position.x + (nextFrame->position.x - currentFrame->position.x) * factor,
            currentFrame->position.y + (nextFrame->position.y - currentFrame->position.y) * factor,
            currentFrame->position.z + (nextFrame->position.z - currentFrame->position.z) * factor
        );

        // Manually interpolate rotation (Euler angles)
        Vec3 interpolatedRotation(
            currentFrame->rotation.x + (nextFrame->rotation.x - currentFrame->rotation.x) * factor,
            currentFrame->rotation.y + (nextFrame->rotation.y - currentFrame->rotation.y) * factor,
            currentFrame->rotation.z + (nextFrame->rotation.z - currentFrame->rotation.z) * factor
        );

        // Manually interpolate scale
        Vec3 interpolatedScale(
            currentFrame->scale.x + (nextFrame->scale.x - currentFrame->scale.x) * factor,
            currentFrame->scale.y + (nextFrame->scale.y - currentFrame->scale.y) * factor,
            currentFrame->scale.z + (nextFrame->scale.z - currentFrame->scale.z) * factor
        );

        owner->set_position(interpolatedPosition);
        owner->set_pitch(interpolatedRotation.x);
        owner->set_scaling(interpolatedScale);

    }

}

void AnimationComponent::add_anim(const AnimationController& anim)
{
    animation = anim;
}

void AnimationComponent::play()
{
    isPlaying = true;
}

void AnimationComponent::stop()
{
    isPlaying = false;
}