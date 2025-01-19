#pragma once
#include "BehaviorNode.h"
#include "CustomScript/Component/AnimationComponent.h"
class L_KnockTheDoor :public BaseNode<L_KnockTheDoor>
{
public:
    L_KnockTheDoor();

    float original_pitch;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    virtual void on_exit();
    Animation createSampleAnimation();
};

