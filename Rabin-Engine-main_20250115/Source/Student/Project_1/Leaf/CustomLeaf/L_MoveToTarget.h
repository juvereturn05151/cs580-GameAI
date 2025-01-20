#pragma once
#include "BehaviorNode.h"
class L_MoveToTarget :
    public BaseNode<L_MoveToTarget>
{
protected:
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
};

