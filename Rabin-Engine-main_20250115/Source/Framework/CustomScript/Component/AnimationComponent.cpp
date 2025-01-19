#include "pch.h"
#include "AnimationComponent.h"

AnimationComponent::AnimationComponent(Agent& _owner)
    :  Component(_owner), currentTime(0.0f), isPlaying(false) {}

void AnimationComponent::update(float deltaTime) {
    if (isPlaying)
    {
        elapsedTime += deltaTime;
        if (elapsedTime >= animation.frameSpeed)
        {
            currentFrame = (currentFrame + 1) % animation.keyframes.capacity();

            elapsedTime -= animation.frameSpeed;
        }

        owner->set_position(animation.keyframes[currentFrame].position);
        owner->set_pitch(animation.keyframes[currentFrame].rotation.x);
        owner->set_scaling(animation.keyframes[currentFrame].scale);

    }

}

void AnimationComponent::add_anim(const Animation& anim, bool _isLoop)
{
    animation = anim;
    isLoop = _isLoop;
}

void AnimationComponent::play()
{
    isPlaying = true;
}

void AnimationComponent::stop()
{
    isPlaying = false;
}

void AnimationComponent::reset()
{
    currentFrame = 0;
}

int AnimationComponent::getCurrentFrame()
{
    return currentFrame;
}