#pragma once
#include "BehaviorNode.h"
class L_Seek : public BaseNode<L_Seek>
{
public:
    L_Seek();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};

