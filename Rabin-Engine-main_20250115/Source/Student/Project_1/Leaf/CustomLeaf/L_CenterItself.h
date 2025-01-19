#pragma once
#include "BehaviorNode.h"
class L_CenterItself : public BaseNode<L_CenterItself>
{
public:
    L_CenterItself();

protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    virtual void on_exit() override;

private:
    float timer;
};

