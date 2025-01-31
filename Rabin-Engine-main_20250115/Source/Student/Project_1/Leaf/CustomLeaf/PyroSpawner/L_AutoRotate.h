#pragma once
#include "BehaviorNode.h"
class L_AutoRotate : public BaseNode<L_AutoRotate>
{
public:
    L_AutoRotate();

protected:
    float timer;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};

