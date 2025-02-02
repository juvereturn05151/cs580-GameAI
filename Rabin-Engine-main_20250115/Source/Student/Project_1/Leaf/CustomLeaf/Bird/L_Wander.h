#pragma once
#include "BehaviorNode.h"
class L_Wander : public BaseNode<L_Wander>
{
public:
    L_Wander();

protected:
    float timer;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};

