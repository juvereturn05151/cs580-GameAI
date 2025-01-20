#pragma once
#include "BehaviorNode.h"
class L_MoveToOrigin : public BaseNode<L_MoveToOrigin>
{
protected:
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
};

