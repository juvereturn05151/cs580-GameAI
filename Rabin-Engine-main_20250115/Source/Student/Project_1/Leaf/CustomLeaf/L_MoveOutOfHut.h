#pragma once
#include "BehaviorNode.h"
class L_MoveOutOfHut : public BaseNode<L_MoveOutOfHut>
{
public:
    L_MoveOutOfHut();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
};

