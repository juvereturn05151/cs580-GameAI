#pragma once
#include "BehaviorNode.h"
class L_SpawnPyro : public BaseNode<L_SpawnPyro>
{
public:
    L_SpawnPyro();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};

