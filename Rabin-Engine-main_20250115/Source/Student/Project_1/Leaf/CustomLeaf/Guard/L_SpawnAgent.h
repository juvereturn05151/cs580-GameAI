#pragma once
#include "BehaviorNode.h"
class L_SpawnAgent :public BaseNode<L_SpawnAgent>
{
public:
    L_SpawnAgent();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    virtual void on_exit() override;
};

