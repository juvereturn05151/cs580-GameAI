#pragma once
#include "BehaviorNode.h"
class L_CheckGoldbergIsStandby : public BaseNode<L_CheckGoldbergIsStandby>
{
protected:
    virtual void on_update(float dt);
};

