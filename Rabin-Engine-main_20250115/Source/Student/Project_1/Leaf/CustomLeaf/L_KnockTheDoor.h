#pragma once
#include "BehaviorNode.h"
class L_KnockTheDoor :public BaseNode<L_KnockTheDoor>
{
public:
    L_KnockTheDoor();

protected:
    float timer;
    float knockState;
    float knockTimer;
    float knockDuration;
    float original_pitch;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    virtual void on_exit();
};

