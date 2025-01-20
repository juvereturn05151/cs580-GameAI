#pragma once
#include "BehaviorNode.h"
class L_FireUpward : public BaseNode<L_FireUpward>
{
public:
    L_FireUpward();

protected:
    float timer;
    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};

