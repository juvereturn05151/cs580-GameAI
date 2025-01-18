#pragma once
#include "BehaviorNode.h"
class L_MoveToHut : public BaseNode<L_MoveToHut>
{
public:
    L_MoveToHut();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
};

